module rmii (
    input  logic       clk,
    input  logic       rstn,

    output logic [1:0] rmii_txd,
    output logic       rmii_txen,
    output logic       rmii_rstn,
    input  logic       rmii_int,
    input  logic       rmii_crs_dv,
    input  logic [1:0] rmii_rxd,
    input  logic       rmii_rxerr,

    // axis_in is layer 2 ethernet frame
    // This module appends preamble, SFD, and IPG
    axis_tlast_if.slv  axis_in,

    axis_tlast_if.mst  axis_out
);
    localparam AXIS_WIDTH = 8;

    assign rmii_rstn = rstn;

    rmii_tx rmii_tx_i (
        .clk(clk),
        .rstn(rstn),
        .rmii_txd(rmii_txd),
        .rmii_txen(rmii_txen),
        .axis_in(axis_in)
    );

    rmii_rx rmii_rx_i (
        .clk(clk),
        .rstn(rstn),
        .rmii_crs_dv(rmii_crs_dv),
        .rmii_rxd(rmii_rxd),
        .rmii_rxerr(rmii_rxerr),
        .axis_out(axis_out)
    );


endmodule