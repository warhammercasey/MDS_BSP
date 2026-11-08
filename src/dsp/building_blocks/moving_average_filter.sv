module moving_average_filter #(
    parameter LOG2_WINDOW_SIZE = 4,
    parameter DATA_WIDTH = 16,
    parameter NUM_CHANNELS = 3
)(
    input  logic clk,
    input  logic rstn,

    axis_if.slv din,
    axis_if.mst dout
);

    localparam WINDOW_SIZE = 1 << LOG2_WINDOW_SIZE;
    localparam TOTAL_WIDTH = DATA_WIDTH * NUM_CHANNELS;


    logic [LOG2_WINDOW_SIZE-1:0] bram_addr;
    logic [TOTAL_WIDTH-1:0] bram_data;
    logic signed [DATA_WIDTH+LOG2_WINDOW_SIZE-1:0] running_sum[NUM_CHANNELS];
    logic filled; // A whole window has been written since reset, so bram_data is a real sample

    // Pipeline: the handshake cycle registers the input and the bram output (the sample
    // leaving the window), the next cycle updates the sum, which is the output register.
    // Both stages move together and hold while the output is stalled.
    logic advance;
    logic din_hs;
    logic din_valid_q;
    logic [TOTAL_WIDTH-1:0] din_data_q;
    logic [TOTAL_WIDTH-1:0] bram_data_q;
    logic filled_q;

    assign advance    = dout.tready || ~dout.tvalid;
    assign din.tready = advance;
    assign din_hs     = din.tvalid && din.tready;


    // Reads one ahead of the write, so at each handshake bram_data holds the sample from
    // WINDOW_SIZE beats ago (read at the previous handshake, held since by enb)
    simple_dual_port_bram #(
        .WIDTH(TOTAL_WIDTH),
        .AWIDTH(LOG2_WINDOW_SIZE)
    ) bram_i (
        .clka(clk),
        .wea(din_hs),
        .ena(1'b1),
        .addra(bram_addr),
        .dina(din.tdata),

        .clkb(clk),
        .enb(din_hs),
        .addrb(bram_addr + 1'b1),
        .doutb(bram_data)
    );

    always_ff @(posedge clk) begin
        if(~rstn) begin
            bram_addr <= '0;
            filled    <= 1'b0;
        end else if(din_hs) begin
            bram_addr <= bram_addr + 1'b1;
            if(&bram_addr) filled <= 1'b1;
        end
    end


    // Register the input and the bram output, so no logic sits on the bram output port
    always_ff @(posedge clk) begin
        if(~rstn) begin
            din_valid_q <= 1'b0;
        end else if(advance) begin
            din_valid_q <= din_hs;
        end
    end

    always_ff @(posedge clk) begin
        if(advance) begin
            din_data_q  <= din.tdata;
            bram_data_q <= bram_data;
            filled_q    <= filled;
        end
    end


    // Running sum: add the new sample, take off the one leaving the window. Until the window
    // has filled since reset, the bram holds stale (or undefined) data, so take off 0 instead.
    always_ff @(posedge clk) begin
        if(~rstn) begin
            running_sum <= '{default: '0};
        end else if(advance && din_valid_q) begin
            for(int i = 0; i < NUM_CHANNELS; i++) begin
                running_sum[i] <= running_sum[i] + $signed(din_data_q[i*DATA_WIDTH +: DATA_WIDTH])
                                - $signed(filled_q ? bram_data_q[i*DATA_WIDTH +: DATA_WIDTH] : DATA_WIDTH'(0));
            end
        end
    end

    always_comb begin
        for(int i = 0; i < NUM_CHANNELS; i++) begin
            dout.tdata[i*DATA_WIDTH +: DATA_WIDTH] = DATA_WIDTH'(running_sum[i] >>> LOG2_WINDOW_SIZE);
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            dout.tvalid <= 1'b0;
        end else if(advance) begin
            dout.tvalid <= din_valid_q;
        end
    end

endmodule
