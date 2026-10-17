module axis_cdc_fifo #(
    parameter WIDTH  = 8, // Data width
    parameter AWIDTH = 4, // Address width. FIFO size is 2^AWIDTH
    parameter CLK_RATIO = 2,
    parameter FAST_TO_SLOW = 0 // Set to 1 for fast to slow CDC
)(
    input  logic clka,
    input  logic rstn, // On clka domain
    
    input  logic clkb,

    axis_if.slv axis_in,
    axis_if.mst axis_out
);

    logic rstnb;
    pulse_cdc #(
        .EN_FAST_TO_SLOW(FAST_TO_SLOW),
        .CLK_RATIO(CLK_RATIO)
    ) pulse_cdc_rstn (
        .clka(clka),
        .clkb(clkb),

        .din(rstn),
        .dout(rstnb)
    );

    logic [AWIDTH-1:0] rd_pointer_a;
    logic [AWIDTH-1:0] rd_pointer_b;
    logic [AWIDTH-1:0] wr_pointer_a;
    logic [AWIDTH-1:0] wr_pointer_b;
    logic [AWIDTH-1:0] count_a;
    logic [AWIDTH-1:0] count_b;

    assign count_a = wr_pointer_a - rd_pointer_a;
    assign count_b = wr_pointer_b - rd_pointer_b;

    assign axis_in.tready = count_a < '1;
    always_ff @(posedge clka) begin
        if(~rstn) begin
            wr_pointer_a <= '0;
        end else begin
            if(axis_in.tvalid && axis_in.tready) begin
                wr_pointer_a <= wr_pointer_a + 1;
            end
        end
    end

    assign axis_out.tvalid = count_b > 0;
    always_ff @(posedge clkb) begin
        if(~rstn) begin
            rd_pointer_b <= '0;
        end else begin
            if(axis_out.tvalid && axis_out.tready) begin
                rd_pointer_b <= rd_pointer_b + 1;
            end
        end 
    end

    ff_sync #(
        .WIDTH(AWIDTH)
    ) ff_sync_rd_pointer (
        .clka(clkb),
        .clkb(clka),

        .din(rd_pointer_b),
        .dout(rd_pointer_a)
    );
    
    ff_sync #(
        .WIDTH(AWIDTH)
    ) ff_sync_wr_pointer (
        .clka(clka),
        .clkb(clkb),

        .din(wr_pointer_a),
        .dout(wr_pointer_b)
    );


    two_port_bram #(
        .WIDTH(WIDTH),
        .AWIDTH(AWIDTH)
    ) two_port_bram_i (
        .clka   (clka),
        .wea    (axis_in.tvalid && axis_in.tready),
        .ena    (1'b1),
        .addra  (wr_pointer_a),
        .dina   (axis_in.tdata),
        .douta  (),
    
        .clkb   (clkb),
        .web    (1'b0),
        .enb    (1'b1),
        .addrb  (rd_pointer_b),
        .dinb   ('0),
        .doutb  (axis_out.tdata)
    );


endmodule