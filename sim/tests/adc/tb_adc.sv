`timescale 1ns/1ps
`include "tb_macros.svh"

// ADC sanity test: the DUT continuously converts IN0..IN7 in order and outputs
// each result on AXI-Stream with tid = channel.
//
// The SPI side is driven by a datasheet-based ADC088S022 model, which records
// what it actually converted. Each AXIS sample is checked for:
//   - tid following the sequence 0,1,..,7,0,..
//   - tid matching the channel the ADC converted (i.e. the address sent on DIN)
//   - tdata matching the converted value
// It also checks AXIS stability under backpressure and the conversion rate.
module tb_adc;
    import tb_pkg::*;

    localparam int  CLK_FREQ      = 73_571_400; // cain_top CLK_IO_FREQ
    localparam int  SAMPLE_RATE   = 100_000;
    localparam int  ADC_BITS      = 8;
    localparam real CLK_PERIOD_NS = 1.0e9 / CLK_FREQ;

    localparam int ROUNDS_FREE = 3; // rounds of all 8 channels with tready held high
    localparam int ROUNDS_BP   = 2; // rounds with random backpressure

    `TB_INIT
    `TB_WATCHDOG(5_000_000) // 5 ms

    // ------------------------------------------------------------------
    // DUT + ADC model
    // ------------------------------------------------------------------
    logic clk  = 1'b0;
    logic rstn = 1'b0;

    always #(CLK_PERIOD_NS/2) clk = ~clk;

    logic adc_spi_sck;
    logic adc_spi_mosi;
    logic adc_spi_cs_n;
    wire  adc_spi_miso;

    axis_tid_if #(.WIDTH(ADC_BITS), .ID_WIDTH(3)) adc_axis ();

    adc #(
        .CLK_FREQ(CLK_FREQ),
        .SAMPLE_RATE(SAMPLE_RATE),
        .ADC_BITS(ADC_BITS)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .adc_spi_sck(adc_spi_sck),
        .adc_spi_mosi(adc_spi_mosi),
        .adc_spi_miso(adc_spi_miso),
        .adc_spi_cs_n(adc_spi_cs_n),
        .adc_axis(adc_axis)
    );

    // Input level per channel, as the ideal output code. Distinct bit patterns first.
    logic [ADC_BITS-1:0] analog_in [8] = '{8'h00, 8'hFF, 8'h55, 8'hAA, 8'h01, 8'h80, 8'h3C, 8'hC3};

    adc088s022_model #(.BITS(ADC_BITS)) adc_chip (
        .cs_n(adc_spi_cs_n),
        .sclk(adc_spi_sck),
        .din(adc_spi_mosi),
        .dout(adc_spi_miso),
        .analog_in(analog_in)
    );

    // Once enabled, keep changing the inputs so every conversion sees new values
    bit vary_inputs = 1'b0;
    initial forever begin
        #2345;
        if (vary_inputs)
            analog_in[$urandom_range(7)] = ADC_BITS'($urandom);
    end

    // ------------------------------------------------------------------
    // AXI-Stream sink
    // tready changes on posedge, so at negedge tvalid/tready show the next handshake
    // ------------------------------------------------------------------
    bit backpressure = 1'b0;
    always @(posedge clk)
        adc_axis.tready <= !backpressure || ($urandom_range(3) == 0);

    int unsigned n_samples = 0;
    logic [2:0]  exp_tid   = 3'd0;
    realtime     t_first, t_last;

    always @(negedge clk) begin : axis_monitor
        if (rstn && adc_axis.tvalid && adc_axis.tready) begin
            `CHECK_EQ(adc_axis.tid, exp_tid, $sformatf("sample %0d: tid sequence", n_samples))

            if (adc_chip.conversions.size() == 0) begin
                tb_error($sformatf("sample %0d: DUT output tid=%0d tdata=0x%02h, but the ADC has not completed a conversion",
                                   n_samples, adc_axis.tid, adc_axis.tdata));
            end else begin
                automatic int unsigned         conv_ch  = adc_chip.conversions[0].channel;
                automatic logic [ADC_BITS-1:0] conv_val = adc_chip.conversions[0].value;
                void'(adc_chip.conversions.pop_front());
                `CHECK_EQ(adc_axis.tid, 3'(conv_ch),
                          $sformatf("sample %0d: tid vs channel the ADC converted (IN%0d)", n_samples, conv_ch))
                `CHECK_EQ(adc_axis.tdata, conv_val,
                          $sformatf("sample %0d: tdata for IN%0d", n_samples, conv_ch))
            end

            if (n_samples == 0)
                t_first = $realtime;
            t_last = $realtime;
            exp_tid++;
            n_samples++;
        end
    end

    // AXIS rule: while tvalid && !tready, tvalid/tdata/tid must hold
    logic                stalled = 1'b0;
    logic [ADC_BITS-1:0] stalled_data;
    logic [2:0]          stalled_id;
    int unsigned         stability_errors = 0;

    always @(negedge clk) begin : axis_stability
        if (stalled && (!adc_axis.tvalid || adc_axis.tdata !== stalled_data || adc_axis.tid !== stalled_id)) begin
            if (stability_errors++ < 5)
                tb_error($sformatf("AXIS changed while stalled: was tid=%0d tdata=0x%02h, now tvalid=%b tid=%0d tdata=0x%02h",
                                   stalled_id, stalled_data, adc_axis.tvalid, adc_axis.tid, adc_axis.tdata));
        end
        stalled      = rstn && adc_axis.tvalid && !adc_axis.tready;
        stalled_data = adc_axis.tdata;
        stalled_id   = adc_axis.tid;
    end

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    task automatic wait_samples(int unsigned n);
        while (n_samples < n)
            @(posedge clk);
    endtask

    initial begin
        real rate;

        repeat (20) @(posedge clk);
        @(negedge clk) rstn = 1'b1;
        repeat (5) @(posedge clk);
        `CHECK_EQ(adc_spi_cs_n, 1'b0, "CS (active low) asserted after reset")

        tb_info("Phase 1: one round with fixed input patterns");
        wait_samples(8);

        tb_info("Phase 2: changing inputs");
        vary_inputs = 1'b1;
        wait_samples(8 * ROUNDS_FREE);

        rate = 1.0e9 * (n_samples - 1) / (t_last - t_first);
        tb_info($sformatf("Conversion rate %.2f ksps (%.2f ksps per channel)", rate / 1e3, rate / 8e3));
        `CHECK(rate > 0.95 * SAMPLE_RATE && rate < 1.05 * SAMPLE_RATE,
               $sformatf("conversion rate %.2f ksps is not within 5%% of SAMPLE_RATE %0d sps", rate / 1e3, SAMPLE_RATE))

        tb_info("Phase 3: random backpressure");
        backpressure = 1'b1;
        wait_samples(8 * (ROUNDS_FREE + ROUNDS_BP));
        backpressure = 1'b0;

        adc_chip.report();
        tb_finish();
    end

endmodule
