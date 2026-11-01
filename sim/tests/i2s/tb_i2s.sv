`timescale 1ns/1ps
`include "tb_macros.svh"

// I2S microphone receiver test: 8 INMP441 mics, 2 per data line, sharing SCK/WS.
//
// Mic m sits on data line m/2 with L/R = m%2, so its AXIS tid should be m:
// tid[2:1] = data line, tid[0] = channel (WS level). Each mic model records the
// 24-bit words it transmits; every AXIS sample is checked against the top I2S_BITS
// of the next word from the mic its tid points to. Also checks AXIS stability under
// backpressure, shared-line contention, and (in the models) the datasheet's I2S
// timing. The mics' startup delay is shortened so the zero words it outputs are
// still seen.
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
    localparam int STARTUP_SCK  = 256; // mic zero-output startup (2^18 on the real part)

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
    logic signed [23:0]         mic_in    [8];  // acoustic input per mic (ideal code)
    logic                       mic_data  [8];
    logic                       mic_oe    [8];
    logic signed [23:0]         mic_word  [8];
    int unsigned                mic_words [8];

    logic signed [I2S_BITS-1:0] expected  [8][$]; // words sent by each mic (top bits), not yet seen on AXIS
    int unsigned                zero_words = 0;   // startup words (all mics)

    for (genvar m = 0; m < 8; m++) begin : g_mic
        inmp441_model #(.LR(1'(m % 2)), .STARTUP_SCK(STARTUP_SCK)) mic (
            .sck(i2s_sck),
            .ws(i2s_ws),
            .data(mic_data[m]),
            .data_oe(mic_oe[m]),
            .sample_in(mic_in[m]),
            .word(mic_word[m]),
            .word_count(mic_words[m])
        );

        always @(mic_words[m])
            if (mic_words[m] != 0) begin
                expected[m].push_back(mic_word[m][23 -: I2S_BITS]);
                if (mic_word[m] == 0)
                    zero_words++;
            end
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

    // Input signals. Fixed: distinct full-scale/edge codes per mic, with the bits below
    // I2S_BITS set so dropping them is checked. Sine: a different tone per mic.
    localparam logic signed [23:0] FIXED_CODES [8] = '{
        24'sh7FFFFF,  // +full scale
        24'sh800000,  // -full scale
        24'shFFFFFF,  // -1
        24'sh00007F,  // +1 at 18 bits, plus low bits
        24'shAAAAAA,
        24'sh555555,
        24'sh00003F,  // only bits below 18: 0 at 18 bits
        24'shF0F0F0
    };
    bit sine_inputs = 1'b0;

    initial forever begin
        for (int m = 0; m < 8; m++) begin
            if (sine_inputs) begin
                automatic real freq = 500.0 * (m + 1);
                mic_in[m] = 24'($rtoi(0.9 * 8388607.0 * $sin(2.0 * 3.14159265358979 * freq * $realtime * 1.0e-9)));
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

    logic signed [I2S_BITS-1:0] rx_audio [8]; // last sample received per mic (view as analog in waves)
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
                automatic logic signed [I2S_BITS-1:0] exp = expected[m].pop_front();
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
    logic [I2S_BITS-1:0] stalled_data;
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

        // The startup zeros came through like any other word (STARTUP_SCK covers ~2 frames)
        `CHECK(zero_words >= 8, $sformatf("only %0d zero startup words from the mics", zero_words))
        tb_info($sformatf("Mics sent %0d words each, %0d startup zeros in all", mic_words[0], zero_words));
        tb_finish();
    end

endmodule
