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

    // QSPI SRAM interface (qspi_sck is a clkout GPIO driven by clk_sram in the Interface Designer)
    input  [3:0]  qspi_sio_IN,
    output [3:0]  qspi_sio_OUT,
    output [3:0]  qspi_sio_OE,
    output        qspi_cs_n
);

    // ##################################################################
    // Global parameters
    // ##################################################################

    localparam CLK_CORE_FREQ = 200_000_000; // 200 MHz
    localparam CLK_SRAM_FREQ = 100_000_000; // 100 MHz
    localparam CLK_50_FREQ   =  50_000_000; // 50 MHz
    localparam CLK_IO_FREQ   =  73_571_400; // 73.5714 MHz

    localparam UART_BAUD     = 115200; // 115200 baud rate

    localparam APB_ADDR_WIDTH = 8;
    localparam APB_DATA_WIDTH = 32;     // Narrower peripherals (e.g. 16-bit eth_smi) go behind an apb_width_conv
    localparam APB_NUM_PERIPH = 2;      // Peripherals on the APB bus (psel lines)

    localparam ADC_BITS      = 8;
    localparam ADC_SAMPLE_RATE = 200_000; // 100 kHz sample rate

    localparam I2S_CLK_FREQ  = 4_096_000; // 4.096 MHz I2S clock
    localparam I2S_BITS      = 18; // Data format is 24-bit but only 18 bits are used
    localparam I2S_CHANNELS  = 8;
    localparam ADC_CHANNELS  = 8;

    // Ethernet streaming: ADC and microphone samples go out as two RTP streams, told
    // apart by SSRC. RTP_PAYLOAD_BYTES is the sample bytes per packet; it sizes the
    // packet and Ethernet TX buffers, so it is the main control on their block RAM
    // (see ethernet.sv). A multiple of 24 fills both streams' packets exactly (frames
    // are 8 bytes for the ADC, 24 for the mics).
    localparam RTP_PAYLOAD_BYTES = 1152; // ADC: 144 frames (5.8 ms), mics: 48 frames (0.75 ms)
    localparam logic [31:0] ADC_RTP_SSRC = 32'h4144_4330; // "ADC0"
    localparam logic [31:0] MIC_RTP_SSRC = 32'h4D49_4330; // "MIC0"
    localparam logic [6:0]  ADC_RTP_PT   = 7'd96;         // Dynamic payload types
    localparam logic [6:0]  MIC_RTP_PT   = 7'd97;
    localparam ETH_TX_BUFFER = 0; // 0 drops the Ethernet TX frame buffer (one frame of RAM), see ethernet.sv

    localparam real LED_CLK_FREQ = CLK_50_FREQ;
    localparam real LED_PWM_FREQ = 1000.0; // Hz
    localparam real LED_CUBE_PERIOD = 4.0; // Seconds
    localparam LED_PWM_TIM_WIDTH = $clog2(int'(CLK_50_FREQ / LED_PWM_FREQ));
    localparam LED_INC_TIM_WIDTH = $clog2(int'(LED_CUBE_PERIOD/((1 << LED_PWM_TIM_WIDTH)*2.0/LED_CLK_FREQ)));

    localparam SMI_APB_ADDR_WIDTH = 5;
    localparam SMI_APB_DATA_WIDTH = 16;
    localparam SMI_APB_PERIPH_NUM = 0;

    localparam UDP_APB_PERIPH_NUM = 1;  // udp_stack_regs, register map in udp_stack_regs.sv

    
    // ##################################################################
    // IO Interfaces
    // ##################################################################

    // ------------------------------------------------------------------
    // Reset synchronizers
    //   Syncronize the asynchronous reset to each clock domain. The reset is active low.
    // ------------------------------------------------------------------
    logic rstn_50, rstn_core, rstn_sram, rstn_io;
    reset_sync #(
        .NUM_CLKS(4)
    ) reset_sync_i (
        .clk('{clk_50, clk_core, clk_sram, clk_io}),
        .rst_in_n(rst_in_n),
        .clk_locked(clk_locked && clk_io_locked),
        .rstn('{rstn_50, rstn_core, rstn_sram, rstn_io})
    );


    // ------------------------------------------------------------------
    // USER LED Control
    //  LED is broken - used common anode instead of common cathode
    //  Blue led works with rgb_led[0], it will be the user led. Others are tied off
    // ------------------------------------------------------------------
    logic user_led;
    assign rgb_led[0] = user_led;
    assign rgb_led[1] = 1'b1;
    assign rgb_led[2] = 1'b1;
    led_cube #(
        .PWM_TIM_WIDTH(LED_PWM_TIM_WIDTH),
        .INC_TIM_WIDTH(LED_INC_TIM_WIDTH)
    ) led_cube_i (
        .clk(clk_50),
        .rstn(rstn_50),
        .led_out(user_led)
    );


    // ------------------------------------------------------------------
    // GPIO
    //   Unused for now
    // ------------------------------------------------------------------
    assign gpio_OUT      = 8'h00;
    assign gpio_OE       = 8'h00;

    // ------------------------------------------------------------------
    // Microphone I2S Interface
    //   8 Microphones, identified by tid channel of axi-stream
    //   i2s sends the channels out of order (0 2 4 6 1 3 5 7), so they are put back in
    //   order before going anywhere
    // ------------------------------------------------------------------
    axis_tid_if #(
        .WIDTH(I2S_BITS),
        .ID_WIDTH($clog2(I2S_CHANNELS))
    ) i2s_axis_raw ();
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
        .axis_i2s(i2s_axis_raw)
    );

    axis_tid_if #(
        .WIDTH(I2S_BITS),
        .ID_WIDTH($clog2(I2S_CHANNELS))
    ) i2s_axis ();
    axis_tid_reorder #(
        .WIDTH(I2S_BITS),
        .ID_WIDTH($clog2(I2S_CHANNELS)),
        .CHANNELS(I2S_CHANNELS)
    ) i2s_reorder (
        .clk(clk_io),
        .rstn(rstn_io),
        .axis_in(i2s_axis_raw),
        .axis_out(i2s_axis)
    );

    // Copies of the ordered samples, for the core and Ethernet (see AXI-Stream Routing)
    axis_tid_if #(
        .WIDTH(I2S_BITS),
        .ID_WIDTH($clog2(I2S_CHANNELS))
    ) i2s_axis_copies [2] ();
    localparam I2S_COPY_CORE = 0;
    localparam I2S_COPY_ETH  = 1;


    // ------------------------------------------------------------------
    // ADC Interface
    //   8 ADC channels, identified by tid channel of axi-stream. 
    //   The ADC is configured to sample at ~200 kHz
    // ------------------------------------------------------------------
    axis_tid_if #(
        .WIDTH(ADC_BITS),
        .ID_WIDTH($clog2(ADC_CHANNELS))
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


    
    // ------------------------------------------------------------------
    // UART console, 
    //   Master of the APB bus. Registers are read and written from the
    //     console with "r <sel> <addr>" and "w <sel> <addr> <data>" (see uart_controller.sv)
    // ------------------------------------------------------------------
    apb_if #(
        .ADDR_WIDTH(APB_ADDR_WIDTH),
        .DATA_WIDTH(APB_DATA_WIDTH),
        .SEL_WIDTH(APB_NUM_PERIPH)
    ) apb_bus ();
    uart_controller #(
        .CLK_FREQ(CLK_IO_FREQ),
        .BAUD(UART_BAUD),
        .APB_ADDR_WIDTH(APB_ADDR_WIDTH),
        .APB_DATA_WIDTH(APB_DATA_WIDTH),
        .APB_SEL_WIDTH(APB_NUM_PERIPH)
    ) uart_controller_i (
        .clk(clk_io),
        .rstn(rstn_io),
        .rx(uart_rx),
        .tx(uart_tx),
        .apb(apb_bus)
    );


    
    // ------------------------------------------------------------------
    // FTDI Bit-bang interface
    //   Unused for now
    // ------------------------------------------------------------------
    assign bb_OUT        = 8'h00;
    assign bb_OE         = 8'h00;

    
    // ------------------------------------------------------------------
    // Ethernet serial management interface (SMI)
    //   The SMI is driven by the APB bus
    //   APB transactions are converted directly to SMI frames
    // ------------------------------------------------------------------
    apb_if #(
        .ADDR_WIDTH(SMI_APB_ADDR_WIDTH),
        .DATA_WIDTH(SMI_APB_DATA_WIDTH),
        .SEL_WIDTH(1)
    ) eth_smi_apb ();
    eth_smi #(
        .DATA_WIDTH(SMI_APB_DATA_WIDTH),
        .ADDR_WIDTH(SMI_APB_ADDR_WIDTH)
    ) eth_smi_i (
        .clk       (clk_50),
        .rstn      (rstn_50),
        .apb       (eth_smi_apb),
        .mdc       (rmii_mdc),
        .mdio_IN   (rmii_mdio_IN),
        .mdio_OUT  (rmii_mdio_OUT),
        .mdio_OE   (rmii_mdio_OE)
    );


    
    // ------------------------------------------------------------------
    // Ethernet
    //   ADC and microphone samples stream out as RTP over UDP, through the RMII PHY.
    //   The UDP/IP settings are registers on the APB bus (map in udp_stack_regs.sv).
    // ------------------------------------------------------------------
    apb_if #(
        .ADDR_WIDTH(APB_ADDR_WIDTH),
        .DATA_WIDTH(APB_DATA_WIDTH),
        .SEL_WIDTH(1)
    ) udp_apb ();
    ethernet #(
        .RTP_PAYLOAD_BYTES(RTP_PAYLOAD_BYTES),
        .ADC_BITS(ADC_BITS),
        .ADC_CHANNELS(ADC_CHANNELS),
        .ADC_ID_WIDTH($clog2(ADC_CHANNELS)),
        .ADC_SSRC(ADC_RTP_SSRC),
        .ADC_PAYLOAD_TYPE(ADC_RTP_PT),
        .MIC_BITS(I2S_BITS),
        .MIC_CHANNELS(I2S_CHANNELS),
        .MIC_ID_WIDTH($clog2(I2S_CHANNELS)),
        .MIC_SSRC(MIC_RTP_SSRC),
        .MIC_PAYLOAD_TYPE(MIC_RTP_PT),
        .TX_BUFFER(ETH_TX_BUFFER)
    ) ethernet_i (
        .clk        (clk_50),
        .rstn       (rstn_50),
        .clk_io     (clk_io),
        .rstn_io    (rstn_io),
        .rmii_txd   (rmii_txd),
        .rmii_txen  (rmii_txen),
        .rmii_rstn  (rmii_rstn),
        .rmii_int   (rmii_int),
        .rmii_crs_dv(rmii_crs_dv),
        .rmii_rxd   (rmii_rxd),
        .rmii_rxerr (rmii_rxerr),
        .apb        (udp_apb),
        .adc_axis   (adc_axis),
        .mic_axis   (i2s_axis_copies[I2S_COPY_ETH])
    );


    
    // ------------------------------------------------------------------
    // QSPI SRAM
    //   Instantiated but currently unused
    // ------------------------------------------------------------------
    sram_controller sram_controller_i(
        .clk(clk_sram),
        .rstn(rstn_sram),

        .qspi_sio_IN(qspi_sio_IN),
        .qspi_sio_OUT(qspi_sio_OUT),
        .qspi_sio_OE(qspi_sio_OE),
        .qspi_cs_n(qspi_cs_n),

        .wr_addr('0),
        .wr_addr_valid(1'b0),
        .wr_data('0),
        .wr_valid(1'b0),
        .wr_ready(),
        .wr_last(1'b0),

        .rd_addr('0),
        .rd_addr_valid(1'b0),
        .rd_data(),
        .rd_valid(),
        .rd_ready(1'b0),
        .rd_last(),
        .rd_burst_len('0),

        .busy()
    );



    
    // ##################################################################
    // APB Bus Routing
    // ##################################################################

    // One APB bus per peripheral, indexed by <sel>. All on clk_io.
    apb_if #(
        .ADDR_WIDTH(APB_ADDR_WIDTH),
        .DATA_WIDTH(APB_DATA_WIDTH),
        .SEL_WIDTH(1)
    ) apb_periph [APB_NUM_PERIPH] ();
    apb_splitter #(
        .DATA_WIDTH(APB_DATA_WIDTH),
        .N(APB_NUM_PERIPH)
    ) apb_splitter_i (
        .apb_in(apb_bus),
        .apb_out(apb_periph)
    );


    // Convert width for ethernet SMI and CDC
    apb_if #(
        .ADDR_WIDTH(SMI_APB_ADDR_WIDTH),
        .DATA_WIDTH(SMI_APB_DATA_WIDTH),
        .SEL_WIDTH(1)
    ) eth_smi_apb_io ();
    apb_width_conv #(
        .IN_ADDR_WIDTH(APB_ADDR_WIDTH),
        .IN_DATA_WIDTH(APB_DATA_WIDTH),
        .OUT_ADDR_WIDTH(SMI_APB_ADDR_WIDTH),
        .OUT_DATA_WIDTH(SMI_APB_DATA_WIDTH)
    ) eth_smi_apb_width_conv (
        .apb_in(apb_periph[SMI_APB_PERIPH_NUM]),
        .apb_out(eth_smi_apb_io)
    );

    apb_cdc #(
        .ADDR_WIDTH(SMI_APB_ADDR_WIDTH),
        .DATA_WIDTH(SMI_APB_DATA_WIDTH)
    ) eth_smi_apb_cdc (
        .clka(clk_io),
        .rstna(rstn_io),
        .clkb(clk_50),
        .rstnb(rstn_50),
        .apb_a(eth_smi_apb_io),
        .apb_b(eth_smi_apb)
    );


    // UDP stack registers, on clk_50 with the stack
    apb_cdc #(
        .ADDR_WIDTH(APB_ADDR_WIDTH),
        .DATA_WIDTH(APB_DATA_WIDTH)
    ) udp_apb_cdc (
        .clka(clk_io),
        .rstna(rstn_io),
        .clkb(clk_50),
        .rstnb(rstn_50),
        .apb_a(apb_periph[UDP_APB_PERIPH_NUM]),
        .apb_b(udp_apb)
    );
    



    
    
    // ##################################################################
    // AXI-Stream Routing
    // ##################################################################

    // Ordered microphone samples go to both the core and Ethernet
    axis_tid_broadcast #(
        .N(2)
    ) i2s_broadcast (
        .axis_in(i2s_axis),
        .axis_out(i2s_axis_copies)
    );

    // CDC microphone data to core clock
    axis_tid_if #(
        .WIDTH(I2S_BITS),
        .ID_WIDTH($clog2(I2S_CHANNELS))
    ) i2s_axis_core();
    axis_tid_cdc_fifo #(
        .WIDTH(I2S_BITS), // Data width
        .ID_WIDTH($clog2(I2S_CHANNELS)),
        .AWIDTH(4)
    ) axis_tid_cdc_fifo_adc_data (
        .clka(clk_io),
        .rstna(rstn_io),
        .clkb(clk_core),
        .rstnb(rstn_core),

        .axis_in(i2s_axis_copies[I2S_COPY_CORE]),
        .axis_out(i2s_axis_core)
    );
    assign i2s_axis_core.tready = 1'b1;

endmodule