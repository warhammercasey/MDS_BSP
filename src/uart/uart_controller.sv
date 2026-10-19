module uart_controller #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200,
    parameter APB_ADDR_WIDTH = 8,
    parameter APB_SEL_WIDTH = 1
) (
    input  logic clk,
    input  logic rstn,

    input  logic rx,
    output logic tx,

    apb_if.mst apb
);

    localparam APB_DATA_WIDTH = 8;

    // Instantiate uart
    axis_if #(
        .WIDTH(8)
    ) uart_tx_if ();
    axis_if #(
        .WIDTH(8)
    ) uart_rx_if ();
    uart #(
        .CLK_FREQ(CLK_IO_FREQ),
        .BAUD(UART_BAUD)
    ) uart_i (
        .clk(clk_io),
        .rstn(rstn_io),
        .tx(tx),
        .rx(rx),
        .axis_tx(uart_tx_if),
        .axis_rx(uart_rx_if)
    );



    

endmodule