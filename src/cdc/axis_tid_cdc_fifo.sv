module axis_tid_cdc_fifo #(
    parameter WIDTH  = 8, // Data width
    parameter ID_WIDTH = 4,
    parameter AWIDTH = 4 // Address width. FIFO size is 2^AWIDTH
) (
    input  logic clka,
    input  logic rstna, // On clka domain
    
    input  logic clkb,
    input  logic rstnb,

    axis_tid_if.slv axis_in,
    axis_tid_if.mst axis_out
);

    axis_if #(
        .WIDTH(WIDTH + ID_WIDTH)
    ) axis_comb_in ();

    axis_if #(
        .WIDTH(WIDTH + ID_WIDTH)
    ) axis_comb_out ();

    assign axis_comb_in.tdata = {axis_in.tid, axis_in.tdata};
    assign axis_comb_in.tvalid = axis_in.tvalid;
    assign axis_in.tready = axis_comb_in.tready;

    assign axis_out.tdata = axis_comb_out.tdata[0 +: WIDTH];
    assign axis_out.tid = axis_comb_out.tdata[WIDTH +: ID_WIDTH];
    assign axis_out.tvalid = axis_comb_out.tvalid;
    assign axis_comb_out.tready = axis_out.tready;

    axis_cdc_fifo #(
        .WIDTH(WIDTH + ID_WIDTH),
        .AWIDTH(AWIDTH) // Address width. FIFO size is 2^AWIDTH
    ) axis_cdc_fifo_i (
        .clka(clka),
        .rstna(rstna),
        .clkb(clkb),
        .rstnb(rstnb),

        .axis_in(axis_comb_in),
        .axis_out(axis_comb_out)
    );


endmodule