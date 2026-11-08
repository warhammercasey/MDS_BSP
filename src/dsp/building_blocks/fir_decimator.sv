// Single-multiplier multichannel complex FIR decimator
// Each TID is filtered independantly
// dout has no backpressure: a result that isn't taken by the time the next one is done is lost
module fir_decimator #(
    parameter WIDTH = 18,
    parameter TAPS = 32,
    parameter DECIMATION = 4,
    parameter TID_WIDTH = $clog2(8),
    parameter INIT_FILE = "",
    parameter INIT_HEX = 1
) (
    input  logic clk,
    input  logic rstn,

    axis_tid_if.slv din,
    axis_tid_if.mst dout
);

    localparam CHANNELS = (1 << TID_WIDTH);
    localparam BUFFER_AWIDTH = $clog2(TAPS*CHANNELS);

    // Start address of each channel's TAPS samples in the input buffer
    function automatic logic [BUFFER_AWIDTH-1:0] buffer_addr(int channel);
        return BUFFER_AWIDTH'(channel*TAPS);
    endfunction

    typedef enum logic [1:0] {
        IDLE,
        FILTERING
    } state_t;
    state_t state;
    logic convolution_done;
    logic [$clog2(CHANNELS)-1:0] current_channel;

    logic [BUFFER_AWIDTH-1:0] input_offsets[CHANNELS];

    logic input_data_ready[CHANNELS];
    logic [$clog2(DECIMATION)-1:0] input_decim_count[CHANNELS];

    logic [BUFFER_AWIDTH-1:0] data_index;
    logic [WIDTH*2-1:0] bram_data_out;

    // Buffer up samples in input BRAM
    simple_dual_port_bram #(
        .WIDTH(WIDTH*2), // *2 for complex samples
        .AWIDTH(BUFFER_AWIDTH)
    ) input_buffer (
        .clka(clk),
        .wea(din.tvalid && din.tready),
        .ena(1'b1),
        .addra(buffer_addr(din.tid) + input_offsets[din.tid]),
        .dina(din.tdata),
    
        .clkb(clk),
        .enb(1'b1),
        .addrb(data_index),
        .doutb(bram_data_out)
    );

    // Tready should only be asserted if we are not waiting for the current channel to be processed
    assign din.tready = din.tvalid && ~input_data_ready[din.tid];

    // Increment input offsets as data comes in
    always_ff @(posedge clk) begin
        if(~rstn) begin
            input_offsets <= '{default: '0};

        end else begin
            if(din.tvalid && din.tready) begin
                if(input_offsets[din.tid] == TAPS - 1) begin
                    input_offsets[din.tid] <= '0;
                end else begin
                    input_offsets[din.tid] <= input_offsets[din.tid] + 1;
                end
            end
        end
    end

    // Count up the input decimation counter for each channel
    // When this overflows, we have enough samples to filter and output a sample
    always_ff @(posedge clk) begin
        if(~rstn) begin
            input_decim_count <= '{default: '0};
        end else begin
            if(din.tvalid && din.tready) begin
                if(input_decim_count[din.tid] == DECIMATION - 1) begin
                    input_decim_count[din.tid] <= '0;
                end else begin
                    input_decim_count[din.tid] <= input_decim_count[din.tid] + 1;
                end
            end
        end
    end

    // When the decimation counter overflows, we have enough samples to filter and output a sample
    // One channel can become ready in the same cycle another finishes, so the clear and set are
    // independent. They're never the same channel: a ready channel's input is held off
    always_ff @(posedge clk) begin
        if(~rstn) begin
            input_data_ready <= '{default: 1'b0};
        end else begin
            if(state == FILTERING && convolution_done) begin
                input_data_ready[current_channel] <= 1'b0;
            end
            if(din.tvalid && din.tready && input_decim_count[din.tid] == DECIMATION - 1) begin
                input_data_ready[din.tid] <= 1'b1;
            end
        end
    end



    // Taps ROM
    logic [$clog2(TAPS)-1:0] tap_index;
    logic [WIDTH-1:0] bram_tap_out;
    rom #(
        .WIDTH(WIDTH),
        .AWIDTH($clog2(TAPS)),
        .INIT_FILE(INIT_FILE),
        .INIT_HEX(INIT_HEX)
    ) taps_rom (
        .clk(clk),
        .en(1'b1),
        .addr($clog2(TAPS)'(TAPS - 1) - tap_index), // Samples are read oldest first, so the taps go last first
        .dout(bram_tap_out)
    );

    logic any_input_ready;
    always_comb begin
        any_input_ready = 1'b0;
        for(int i = 0; i < CHANNELS; i++) begin
            if(input_data_ready[i]) begin
                any_input_ready = 1'b1;
            end
        end
    end


    always_ff @(posedge clk) begin
        if(~rstn) begin
            state <= IDLE;
        end else begin
            case(state)
                IDLE: begin
                    if(any_input_ready) begin
                        state <= FILTERING;
                    end
                end
                FILTERING: begin
                    if(convolution_done) begin
                        state <= IDLE;
                    end
                end
            endcase
        end
    end


    // Find an available channel to process and latch it until the filtering is done
    logic [$clog2(CHANNELS)-1:0] next_channel;
    always_comb begin
        next_channel = 0;
        for(int i = 0; i < CHANNELS; i++) begin
            if(input_data_ready[i]) begin
                next_channel = i;
            end
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            current_channel <= '0;
        end else begin
            if(state == IDLE && any_input_ready) begin
                current_channel <= next_channel;
            end
        end
    end


    // Increment the tap index as we filter the samples
    assign convolution_done = (tap_index == TAPS - 1);
    always_ff @(posedge clk) begin
        if(~rstn) begin
            tap_index <= '0;
        end else begin
            if(state == FILTERING) begin
                tap_index <= tap_index + 1;
            end else begin
                tap_index <= '0;
            end
        end
    end

    // Increment the data index as we filter the samples
    always_ff @(posedge clk) begin
        if(~rstn) begin
            data_index <= '0;
        end else begin
            if(state == IDLE && any_input_ready) begin
                // The next write position holds the oldest sample
                data_index <= buffer_addr(next_channel) + input_offsets[next_channel];
            end else if(state == FILTERING) begin
                if(data_index == buffer_addr(current_channel) + BUFFER_AWIDTH'(TAPS - 1)) begin
                    data_index <= buffer_addr(current_channel);
                end else begin
                    data_index <= data_index + 1;
                end
            end else begin
                data_index <= '0;
            end
        end
    end

    // Register the output of brams for timing purposes
    logic signed [WIDTH-1:0] input_sample_i;
    logic signed [WIDTH-1:0] input_sample_q;
    logic signed [WIDTH-1:0] tap_sample;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            input_sample_i <= '0;
            input_sample_q <= '0;
            tap_sample <= '0;
        end else begin
            input_sample_i <= bram_data_out[WIDTH*2-1:WIDTH];
            input_sample_q <= bram_data_out[WIDTH-1:0];
            tap_sample     <= bram_tap_out;
        end
    end

    // Multiply
    logic signed [WIDTH*2-1:0] product_i;
    logic signed [WIDTH*2-1:0] product_q;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            product_i <= '0;
            product_q <= '0;
        end else begin
            product_i <= input_sample_i * tap_sample;
            product_q <= input_sample_q * tap_sample;
        end
    end

    // Accumulate the products for the final output
    logic signed [WIDTH*2-1:0] accumulated_i;
    logic signed [WIDTH*2-1:0] accumulated_q;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            accumulated_i <= '0;
            accumulated_q <= '0;
        end else begin
            if($past(state, 3) == FILTERING) begin // pipeline should have latency of 3
                accumulated_i <= accumulated_i + product_i;
                accumulated_q <= accumulated_q + product_q;
            end else begin
                accumulated_i <= '0;
                accumulated_q <= '0;
            end
        end
    end

    // Output the accumulated results
    always_ff @(posedge clk) begin
        if(~rstn) begin
            dout.tdata <= '0;
            dout.tvalid <= 1'b0;

        end else begin
            if($past(convolution_done, 4)) begin // At this point the pipeline should have a latency of 4 cycles
                dout.tdata <= {accumulated_i[WIDTH*2-1:WIDTH], accumulated_q[WIDTH*2-1:WIDTH]};
                dout.tid <= $past(current_channel, 4);
                dout.tvalid <= 1'b1;
            end else if(dout.tready) begin
                dout.tvalid <= 1'b0;
            end

        end
    end
endmodule