module cain_top (
    input        clk_50,   // 50 MHz clock from board. Shared with ethernet PHY for RMII
    input        clk_core, // Core clock for high speed DSP - as high as timing allows. Currently 200MHz
    input        clk_sram, // SRAM clock. Also as high as timing allows. Currentl 100MHz
    input        clk_locked, // Clock from PLL is locked
    input        clk_io, // 73.5714 MHz for ADC, UART, and I2S interfaces
    input        clk_io_locked, // Second PLL for IO clock is locked
    input        rst_in_n,    // Active low reset

    // RGB LED output
    output [2:0] rgb_led,

    // GPIOs
    output [7:0] gpio_OUT,
    input  [7:0] gpio_IN,
    output [7:0] gpio_OE, // GPIO output enable

    // I2S interface
    // 8 microphones, 2 per data line. Share sck/ws
    output        i2s_sck,
    output        i2s_ws,
    input  [3:0]  i2s_data,

    // ADC SPI interface
    output        adc_spi_sck,
    output        adc_spi_mosi,
    input         adc_spi_miso,
    output        adc_spi_cs_n,

    // FTDI UART
    input         uart_rx,
    output        uart_tx,

    // FTDI Bit-bang
    output [7:0] bb_OUT,
    input  [7:0] bb_IN,
    output [7:0] bb_OE,

    // RMII interface for Ethernet
    output [1:0] rmii_txd,
    output       rmii_txen,
    output       rmii_rstn,
    input        rmii_int,
    input        rmii_crs_dv,
    input  [1:0] rmii_rxd,
    input        rmii_rxerr,
    output       rmii_mdc,
    input        rmii_mdio_IN,
    output       rmii_mdio_OUT,
    output       rmii_mdio_OE,

    // QSPI SRAM interface
    output        qspi_sck,
    input  [3:0]  qspi_sio_IN,
    output [3:0]  qspi_sio_OUT,
    output [3:0]  qspi_sio_OE,
    output        qspi_cs_n
);

    localparam CLK_CORE_FREQ = 200_000_000; // 200 MHz
    localparam CLK_SRAM_FREQ = 100_000_000; // 100 MHz
    localparam CLK_50_FREQ   =  50_000_000; // 50 MHz
    localparam CLK_IO_FREQ   =  73_571_400; // 73.5714 MHz

    localparam UART_BAUD     = 115200; // 115200 baud rate

    localparam ADC_BITS      = 8;
    localparam ADC_SAMPLE_RATE = 200_000; // 100 kHz sample rate

    localparam I2S_CLK_FREQ  = 4_096_000; // 4.096 MHz I2S clock
    localparam I2S_BITS      = 18; // Data format is 24-bit but only 18 bits are used

    logic rstn, rstn_io;
    assign rstn = rst_in_n && clk_locked;
    assign rstn_io = rst_in_n && clk_io_locked;


    // ------------------------------------------------------------------
    // Temporary tie-offs while submodules are in development.
    // Outputs are driven to their inactive level; bidirectional pins are
    // tristated (OE = 0) with OUT parked low.
    // ------------------------------------------------------------------

    // RGB LED - off (assumes active-high drive)
    assign rgb_led       = 3'b000;

    // GPIOs - tristated
    assign gpio_OUT      = 8'h00;
    assign gpio_OE       = 8'h00;

    // I2S
    axis_tid_if #(
        .WIDTH(I2S_BITS),
        .ID_WIDTH($clog2(8))
    ) i2s_axis ();
    i2s #(
        .CLK_FREQ(CLK_IO_FREQ),
        .I2S_CLK(I2S_CLK_FREQ),
        .I2S_BITS(I2S_BITS)
    ) i2s_i (
        .clk(clk_io),
        .rstn(rstn_io),
        .i2s_sck(i2s_sck),
        .i2s_ws(i2s_ws),
        .i2s_data(i2s_data),
        .axis_i2s(i2s_axis)
    );
    assign i2s_axis.tready = 1'b1;


    // ADC SPI
    axis_tid_if #(
        .WIDTH(ADC_BITS),
        .ID_WIDTH($clog2(8))
    ) adc_axis ();
    adc #(
        .CLK_FREQ(CLK_IO_FREQ),
        .SAMPLE_RATE(ADC_SAMPLE_RATE),
        .ADC_BITS(ADC_BITS)
    ) adc_i (
        .clk(clk_io),
        .rstn(rstn_io),
        .adc_spi_sck(adc_spi_sck),
        .adc_spi_mosi(adc_spi_mosi),
        .adc_spi_miso(adc_spi_miso),
        .adc_spi_cs_n(adc_spi_cs_n),
        .adc_axis(adc_axis)
    );
    assign adc_axis.tready = 1'b1;
    

    // UART
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
        .tx(uart_tx),
        .rx(uart_rx),
        .axis_tx(uart_tx_if),
        .axis_rx(uart_rx_if)
    );

    assign uart_tx_if.tvalid = uart_rx_if.tvalid;
    assign uart_tx_if.tdata = uart_rx_if.tdata;
    assign uart_rx_if.tready = uart_tx_if.tready;


    // FTDI bit-bang - tristated
    assign bb_OUT        = 8'h00;
    assign bb_OE         = 8'h00;

    // RMII - transmitter disabled, PHY reset deasserted, MDIO tristated
    assign rmii_txd      = 2'b00;
    assign rmii_txen     = 1'b0;
    assign rmii_rstn     = 1'b1;
    assign rmii_mdc      = 1'b0;
    assign rmii_mdio_OUT = 1'b0;
    assign rmii_mdio_OE  = 1'b0;

    // QSPI SRAM - chip select deasserted, clock idle low, SIO tristated
    assign qspi_sck      = 1'b0;
    assign qspi_sio_OUT  = 4'h0;
    assign qspi_sio_OE   = 4'h0;
    assign qspi_cs_n     = 1'b1;

endmodule