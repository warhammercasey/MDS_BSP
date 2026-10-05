module uart #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
)(
    input  logic clk,
    input  logic rstn,

    // IO
    input  logic rx,
    output logic tx,

    // AXI Stream
    axis_if.slv axis_tx,
    axis_if.mst axis_rx
);

    logic rx_d, tx_d;

    // Latch the incoming and outgoing data
    always @(posedge clk ) begin
        if (!rstn) begin
            rx_d <= 1'b1;

        end else begin
            rx_d <= rx;
            tx <= tx_d;
        end
    end

    uart_rx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD(BAUD)
    ) uart_rx_i (
        .clk(clk),
        .rstn(rstn),
        .rx(rx_d),
        .axis_rx(axis_rx)
    );

    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD(BAUD)
    ) uart_tx_i (
        .clk(clk),
        .rstn(rstn),
        .tx(tx_d),
        .axis_tx(axis_tx)
    );

endmodule