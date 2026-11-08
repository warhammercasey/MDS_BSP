// Single-multiplier multichannel complex FIR decimator
// Each TID is filtered independantly
// dout has no backpressure: a result that isn't taken by the time the next one is done is lost
// din takes at most one beat every 3 clocks
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
    logic [$clog2(TAPS)-1:0] tap_index;
    logic [CHANNELS-1:0] clearing; // The channel whose ready flag clears this clock
    logic [$clog2(CHANNELS)-1:0] current_channel;

    // Next write position in each channel's TAPS samples, as a buffer address (it holds the
    // channel's oldest sample)
    logic [BUFFER_AWIDTH-1:0] input_ptrs[CHANNELS];

    logic input_data_ready[CHANNELS];
    logic [$clog2(DECIMATION)-1:0] input_decim_count[CHANNELS];

    logic [BUFFER_AWIDTH-1:0] data_index;
    logic [WIDTH*2-1:0] bram_data_out;

    // Input register, for timing: tready is just "empty", so it doesn't depend on tid. Whether
    // the beat's channel is waiting to be processed (and so held off) is looked up a clock
    // later into a register, so a beat takes 3 clocks at least, far more than the input rate
    // needs. The lookup can only be stale the safe way: a channel stops waiting, never starts,
    // except by taking a beat, which is this one.
    logic                    in_valid;
    logic                    in_checked; // in_blocked is for the held beat
    logic                    in_blocked;
    logic [TID_WIDTH-1:0]    in_tid;
    logic [WIDTH*2-1:0]      in_data;
    logic [CHANNELS-1:0]     din_accept; // The held beat goes into channel c's buffer

    assign din.tready = ~in_valid;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            in_valid   <= 1'b0;
            in_checked <= 1'b0;
        end else if(din.tvalid && din.tready) begin
            in_valid   <= 1'b1;
            in_checked <= 1'b0;
        end else if(|din_accept) begin
            in_valid   <= 1'b0;
            in_checked <= 1'b0;
        end else if(in_valid) begin
            in_checked <= 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if(din.tvalid && din.tready) begin
            in_tid  <= din.tid;
            in_data <= din.tdata;
        end
        in_blocked <= input_data_ready[in_tid];
    end

    // Decode per channel, so the per channel state doesn't sit behind a tid mux
    always_comb begin
        for(int c = 0; c < CHANNELS; c++) begin
            din_accept[c] = in_valid && in_checked && !in_blocked && in_tid == c;
        end
    end

    // The buffer write is registered for timing. It lands a clock after the handshake, before
    // the sample can be read: it's the newest, read last
    logic                     write_en;
    logic [BUFFER_AWIDTH-1:0] write_addr;
    logic [WIDTH*2-1:0]       write_data;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            write_en <= 1'b0;
        end else begin
            write_en <= |din_accept;
        end
    end

    always_ff @(posedge clk) begin
        write_addr <= input_ptrs[in_tid];
        write_data <= in_data;
    end

    // Buffer up samples in input BRAM
    simple_dual_port_bram #(
        .WIDTH(WIDTH*2), // *2 for complex samples
        .AWIDTH(BUFFER_AWIDTH)
    ) input_buffer (
        .clka(clk),
        .wea(write_en),
        .ena(1'b1),
        .addra(write_addr),
        .dina(write_data),

        .clkb(clk),
        .enb(1'b1),
        .addrb(data_index),
        .doutb(bram_data_out)
    );

    // Move each channel's write position on as data comes in
    always_ff @(posedge clk) begin
        if(~rstn) begin
            for(int c = 0; c < CHANNELS; c++) begin
                input_ptrs[c] <= buffer_addr(c);
            end
        end else begin
            for(int c = 0; c < CHANNELS; c++) begin
                if(din_accept[c]) begin
                    if(input_ptrs[c] == buffer_addr(c) + BUFFER_AWIDTH'(TAPS - 1)) begin
                        input_ptrs[c] <= buffer_addr(c);
                    end else begin
                        input_ptrs[c] <= input_ptrs[c] + 1'b1;
                    end
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
            for(int c = 0; c < CHANNELS; c++) begin
                if(din_accept[c]) begin
                    if(input_decim_count[c] == DECIMATION - 1) begin
                        input_decim_count[c] <= '0;
                    end else begin
                        input_decim_count[c] <= input_decim_count[c] + 1;
                    end
                end
            end
        end
    end

    // When the decimation counter overflows, we have enough samples to filter and output a sample
    // One channel can become ready in the same cycle another finishes, so the clear and set are
    // independent. They're never the same channel: a ready channel's input is held off
    logic input_data_ready_next[CHANNELS];
    always_comb begin
        for(int c = 0; c < CHANNELS; c++) begin
            input_data_ready_next[c] = input_data_ready[c];
            if(clearing[c]) begin
                input_data_ready_next[c] = 1'b0;
            end
            if(din_accept[c] && input_decim_count[c] == DECIMATION - 1) begin
                input_data_ready_next[c] = 1'b1;
            end
        end
    end

    // Find an available channel to process, into registers. From the ready flags less the one
    // being cleared: one being set is seen a clock later.
    // clearing: the channel whose ready flag clears this clock, registered with convolution_done
    logic [CHANNELS-1:0] ready_avail;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            clearing <= '0;
        end else begin
            for(int c = 0; c < CHANNELS; c++) begin
                clearing[c] <= state == FILTERING && !convolution_done && tap_index == $clog2(TAPS)'(TAPS - 2)
                               && current_channel == c;
            end
        end
    end

    always_comb begin
        for(int c = 0; c < CHANNELS; c++) begin
            ready_avail[c] = input_data_ready[c] && !clearing[c];
        end
    end

    logic any_input_ready;
    logic [$clog2(CHANNELS)-1:0] next_channel;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            input_data_ready <= '{default: 1'b0};
            any_input_ready  <= 1'b0;
            next_channel     <= '0;
        end else begin
            input_data_ready <= input_data_ready_next;
            any_input_ready  <= |ready_avail;
            for(int c = 0; c < CHANNELS; c++) begin
                if(ready_avail[c]) begin
                    next_channel <= c;
                end
            end
        end
    end



    // Taps ROM
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


    // Latch the channel to process until the filtering is done

    always_ff @(posedge clk) begin
        if(~rstn) begin
            current_channel <= '0;
        end else begin
            if(state == IDLE && any_input_ready) begin
                current_channel <= next_channel;
            end
        end
    end


    // Increment the tap index as we filter the samples. convolution_done is tap_index == TAPS-1
    // while filtering, worked out a clock ahead into a register.
    always_ff @(posedge clk) begin
        if(~rstn) begin
            convolution_done <= 1'b0;
        end else begin
            convolution_done <= state == FILTERING && !convolution_done && tap_index == $clog2(TAPS)'(TAPS - 2);
        end
    end

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

    // The channel's samples in the buffer, registered with the channel
    logic [BUFFER_AWIDTH-1:0] region_first, region_last;
    always_ff @(posedge clk) begin
        if(state == IDLE && any_input_ready) begin
            region_first <= buffer_addr(next_channel);
            region_last  <= buffer_addr(next_channel) + BUFFER_AWIDTH'(TAPS - 1);
        end
    end

    // Increment the data index as we filter the samples
    always_ff @(posedge clk) begin
        if(~rstn) begin
            data_index <= '0;
        end else begin
            if(state == IDLE && any_input_ready) begin
                // The next write position holds the oldest sample
                data_index <= input_ptrs[next_channel];
            end else if(state == FILTERING) begin
                if(data_index == region_last) begin
                    data_index <= region_first;
                end else begin
                    data_index <= data_index + 1;
                end
            end else begin
                data_index <= '0;
            end
        end
    end

    // Register the output of brams for timing purposes. No enable and no reset, so synthesis
    // packs these into the RAM blocks' output registers.
    logic signed [WIDTH-1:0] input_sample_i;
    logic signed [WIDTH-1:0] input_sample_q;
    logic signed [WIDTH-1:0] tap_sample;
    always_ff @(posedge clk) begin
        input_sample_i <= bram_data_out[WIDTH*2-1:WIDTH];
        input_sample_q <= bram_data_out[WIDTH-1:0];
        tap_sample     <= bram_tap_out;
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