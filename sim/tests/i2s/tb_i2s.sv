`timescale 1ns/1ps
`include "tb_macros.svh"

// I2S microphone receiver test: 8 SPH0645 mics, 2 per data line, sharing BCLK/WS.
//
// Mic m sits on data line m/2 with SELECT = m%2, so its AXIS tid should be m:
// tid[2:1] = data line, tid[0] = channel (WS level). Each mic model records the
// words it transmits; every AXIS sample is checked against the next word from
// the mic its tid points to. Also checks AXIS stability under backpressure,
// shared-line contention, and (in the models) the datasheet's I2S timing.
module tb_i2s;
    import tb_pkg::*;

    // The I2S clock is not set here: the DUT's I2S_CLK default is used, so the test
    // always runs at the rate the design is configured for.
    localparam int  CLK_FREQ      = 73_571_400; // cain_top CLK_IO_FREQ
    localparam int  I2S_BITS      = 18;
    localparam real CLK_PERIOD_NS = 1.0e9 / CLK_FREQ;

    localparam int FRAMES_FIXED = 3;  // frames (one word per mic) with fixed patterns
    localparam int FRAMES_SINE  = 40; // frames of sine waves
    localparam int FRAMES_BP    = 6;  // frames with random backpressure

    `TB_INIT
    `TB_WATCHDOG(20_000_000) // 20 ms

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    logic clk  = 1'b0;
    logic rstn = 1'b0;

    always #(CLK_PERIOD_NS/2) clk = ~clk;

    logic       i2s_sck;
    logic       i2s_ws;
    logic [3:0] i2s_data;

    axis_tid_if #(.WIDTH(I2S_BITS), .ID_WIDTH(3)) axis_i2s ();

    i2s #(
        .CLK_FREQ(CLK_FREQ),
        .I2S_BITS(I2S_BITS)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .i2s_sck(i2s_sck),
        .i2s_ws(i2s_ws),
        .i2s_data(i2s_data),
        .axis_i2s(axis_i2s)
    );

    // ------------------------------------------------------------------
    // Microphones
    // ------------------------------------------------------------------
    logic signed [17:0] mic_in    [8];  // acoustic input per mic (ideal code)
    logic               mic_data  [8];
    logic               mic_oe    [8];
    logic signed [17:0] mic_word  [8];
    int unsigned        mic_words [8];

    logic signed [17:0] expected  [8][$]; // words sent by each mic, not yet seen on AXIS

    for (genvar m = 0; m < 8; m++) begin : g_mic
        sph0645_model #(.SEL(1'(m % 2))) mic (
            .bclk(i2s_sck),
            .ws(i2s_ws),
            .data(mic_data[m]),
            .data_oe(mic_oe[m]),
            .sample_in(mic_in[m]),
            .word(mic_word[m]),
            .word_count(mic_words[m])
        );

        always @(mic_words[m])
            if (mic_words[m] != 0)
                expected[m].push_back(mic_word[m]);
    end

    // Shared data lines: whichever mic drives, else the 100k pull-down
    for (genvar l = 0; l < 4; l++) begin : g_line
        assign i2s_data[l] = mic_oe[2*l]   ? mic_data[2*l]
                           : mic_oe[2*l+1] ? mic_data[2*l+1]
                           : 1'b0;

        always @(mic_oe[2*l] or mic_oe[2*l+1])
            if (mic_oe[2*l] && mic_oe[2*l+1])
                tb_error($sformatf("Bus contention: both mics on data line %0d are driving", l));
    end

    // Input signals. Fixed: distinct full-scale/edge codes per mic. Sine: a different tone per mic.
    localparam logic signed [17:0] FIXED_CODES [8] = '{
        18'sh1FFFF,  // +full scale
        18'sh20000,  // -full scale
        18'sh3FFFF,  // -1
        18'sh00001,  // +1
        18'sh2AAAA,
        18'sh15555,
        18'sh0F0F0,
        18'sh30F0F
    };
    bit sine_inputs = 1'b0;

    initial forever begin
        for (int m = 0; m < 8; m++) begin
            if (sine_inputs) begin
                automatic real freq = 500.0 * (m + 1);
                mic_in[m] = 18'($rtoi(0.9 * 131071.0 * $sin(2.0 * 3.14159265358979 * freq * $realtime * 1.0e-9)));
            end else begin
                mic_in[m] = FIXED_CODES[m];
            end
        end
        #1000;
    end

    // ------------------------------------------------------------------
    // AXI-Stream sink
    // tready changes on posedge, so at negedge tvalid/tready show the next handshake
    // ------------------------------------------------------------------
    bit backpressure = 1'b0;
    always @(posedge clk)
        axis_i2s.tready <= !backpressure || ($urandom_range(3) == 0);

    logic signed [17:0] rx_audio [8];    // last sample received per mic (view as analog in waves)
    int unsigned        rx_count [8] = '{default: 0};
    int unsigned        discarded = 0;
    realtime            t_first_rx, t_last_rx;

    always @(negedge clk) begin : axis_monitor
        if (rstn && axis_i2s.tvalid && axis_i2s.tready) begin
            automatic int unsigned m = int'(axis_i2s.tid);

            if (g_mic_words_total() == 0) begin
                // Before any mic has sent a word (start-up frame), there is nothing valid to check
                discarded++;
            end else if (expected[m].size() == 0) begin
                tb_error($sformatf("tid %0d (line %0d, ch %0d): sample 0x%05h but that mic has not sent a word",
                                   m, m / 2, m % 2, axis_i2s.tdata));
            end else begin
                automatic logic signed [17:0] exp = expected[m].pop_front();
                `CHECK_EQ(axis_i2s.tdata, exp,
                          $sformatf("tid %0d (line %0d, ch %0d) sample %0d", m, m / 2, m % 2, rx_count[m]))
                rx_audio[m] = axis_i2s.tdata;
                if (m == 0) begin
                    if (rx_count[0] == 0)
                        t_first_rx = $realtime;
                    t_last_rx = $realtime;
                end
                rx_count[m]++;
            end
        end
    end

    function automatic int unsigned g_mic_words_total();
        int unsigned n = 0;
        foreach (mic_words[m])
            n += mic_words[m];
        return n;
    endfunction

    // AXIS rule: while tvalid && !tready, tvalid/tdata/tid must hold
    logic        stalled = 1'b0;
    logic [17:0] stalled_data;
    logic [2:0]  stalled_id;
    int unsigned stability_errors = 0;

    always @(negedge clk) begin : axis_stability
        if (stalled && (!axis_i2s.tvalid || axis_i2s.tdata !== stalled_data || axis_i2s.tid !== stalled_id)) begin
            if (stability_errors++ < 5)
                tb_error($sformatf("AXIS changed while stalled: was tid=%0d tdata=0x%05h, now tvalid=%b tid=%0d tdata=0x%05h",
                                   stalled_id, stalled_data, axis_i2s.tvalid, axis_i2s.tid, axis_i2s.tdata));
        end
        stalled      = rstn && axis_i2s.tvalid && !axis_i2s.tready;
        stalled_data = axis_i2s.tdata;
        stalled_id   = axis_i2s.tid;
    end

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    function automatic int unsigned min_rx_count();
        int unsigned n = rx_count[0];
        foreach (rx_count[m])
            if (rx_count[m] < n)
                n = rx_count[m];
        return n;
    endfunction

    task automatic wait_frames(int unsigned n);
        while (min_rx_count() < n)
            @(posedge clk);
    endtask

    initial begin
        real rate;

        repeat (20) @(posedge clk);
        @(negedge clk) rstn = 1'b1;

        tb_info($sformatf("Phase 1: fixed input codes (I2S_CLK = %0d Hz, actual BCLK %.4f MHz)",
                          dut.I2S_CLK, CLK_FREQ / (1.0e6 * 2 * dut.I2S_CLK_PERIOD_2)));
        wait_frames(FRAMES_FIXED);

        tb_info("Phase 2: sine inputs");
        sine_inputs = 1'b1;
        wait_frames(FRAMES_FIXED + FRAMES_SINE);

        rate = 1.0e9 * (rx_count[0] - 1) / (t_last_rx - t_first_rx);
        tb_info($sformatf("Per-mic sample rate %.3f kHz", rate / 1e3));

        tb_info("Phase 3: random backpressure");
        backpressure = 1'b1;
        wait_frames(FRAMES_FIXED + FRAMES_SINE + FRAMES_BP);
        backpressure = 1'b0;

        // Every mic delivered, nothing was dropped (at most the word in flight is pending)
        foreach (expected[m])
            `CHECK(expected[m].size() <= 1,
                   $sformatf("mic %0d has %0d words that never appeared on AXIS", m, expected[m].size()))
        `CHECK(discarded <= 4, $sformatf("%0d samples output before any mic sent data (expected at most 4)", discarded))
        tb_info($sformatf("Discarded %0d start-up samples", discarded));

        tb_info($sformatf("Mics sent %0d words each (mic 0 mode: %s)", mic_words[0],
                          g_mic[0].mic.active ? "active" : "sleep"));
        tb_finish();
    end

endmodule
