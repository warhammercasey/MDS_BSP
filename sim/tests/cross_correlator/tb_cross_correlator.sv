`timescale 1ns/1ps
`include "tb_macros.svh"

// cross_correlator test: every pair of channels correlated over a sliding window, checked
// bit-exact against a model of MATLAB's xcorr on each window, plus an independent check that
// the correlation peaks where it should.
//
// Model, per window k (ending at sample e = (k+1)N - 1 of every channel, skipping windows that
// would start before sample 0), per pair a < b in order, per lag m = -D..D:
//   R[m] = sum over n = 0..L-1 with 0 <= n+m < L of x_b[s+n+m] * conj(x_a[s+n]),  s = e-L+1
//   beat {re, im} = (R >>> OUT_SHIFT) saturated to OUT_WIDTH, tid = pair, tlast on m = D
// Checked throughout: every beat (tid, data, tlast) in order, nothing extra or missing, and the
// output holds while stalled.
// Phases:
//   delays     every channel is the same complex noise, delayed by d_c samples (0..D). Each
//              pair's |R| must peak at m = d_b - d_a. The input comes at a fixed rate just
//              slow enough for the correlator to keep up, by the clocks-per-window formula in
//              cross_correlator.sv: it must never be held off, and every window must take
//              exactly that many clocks
//   noise      full-scale random samples, shuffled channel order, back to back, the output
//              ready ~60% of the time with occasional long stalls: the correlator has to
//              hold off the input, and the output must still be exact
module cross_correlator_inst #(
    parameter int CHANNELS  = 4,
    parameter int WIDTH     = 18,
    parameter int L         = 64,
    parameter int N         = 16,
    parameter int D         = 5,
    parameter int MACS      = 3,
    parameter int OUT_WIDTH = 32,
    parameter int OUT_SHIFT = (2*WIDTH + 1 + $clog2(L) > OUT_WIDTH) ? 2*WIDTH + 1 + $clog2(L) - OUT_WIDTH : 0
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int CH_W      = $clog2(CHANNELS);
    localparam int PAIRS     = CHANNELS * (CHANNELS - 1) / 2;
    localparam int PAIR_W    = PAIRS > 1 ? $clog2(PAIRS) : 1;
    localparam int LAGS      = 2 * D + 1;
    localparam int PASSES    = (LAGS + MACS - 1) / MACS;
    localparam int SKIP      = (L + N - 1) / N - 1;
    localparam int RUN_CLKS  = PAIRS * PASSES * (L + MACS - 1);  // per window, from the DUT's header
    localparam int BEAT_W    = PAIR_W + 2 * OUT_WIDTH + 1;      // {tid, re, im, tlast}

    logic rstn = 1'b0;

    axis_tid_if       #(.WIDTH(2 * WIDTH),     .ID_WIDTH(CH_W))   din ();
    axis_tid_tlast_if #(.WIDTH(2 * OUT_WIDTH), .ID_WIDTH(PAIR_W)) dout ();

    cross_correlator #(
        .CHANNELS(CHANNELS),
        .WIDTH(WIDTH),
        .WINDOW_LEN(L),
        .WINDOW_STEP(N),
        .MAX_LAG(D),
        .MACS(MACS),
        .OUT_WIDTH(OUT_WIDTH),
        .OUT_SHIFT(OUT_SHIFT)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .din(din),
        .dout(dout)
    );

    string NAME = $sformatf("%0d ch, L %0d, N %0d, +/-%0d, %0d MACs", CHANNELS, L, N, D, MACS);

    // ------------------------------------------------------------------
    // Source: beats from a queue, {tid, I, Q}, either at a fixed interval or with random gaps
    // ------------------------------------------------------------------
    logic [CH_W+2*WIDTH-1:0] src_q[$];
    int unsigned             interval = 0;  // clocks between beats (0: random gaps)
    int unsigned             gap_pct  = 0;
    int unsigned             wait_left = 0;
    int unsigned             in_stalls = 0;

    always @(posedge clk) begin
        bit hs;
        if (!rstn) begin
            din.tvalid <= 1'b0;
            din.tdata  <= '0;
            din.tid    <= '0;
        end else begin
            hs = din.tvalid && din.tready;
            if (din.tvalid && !din.tready) in_stalls++;
            if (hs) void'(src_q.pop_front());
            if (wait_left > 0) wait_left--;
            if (!din.tvalid || hs) begin
                if (src_q.size() > 0 && wait_left == 0 && (interval > 0 || $urandom_range(99) >= gap_pct)) begin
                    din.tvalid <= 1'b1;
                    {din.tid, din.tdata} <= src_q[0];
                    wait_left = interval;
                end else begin
                    din.tvalid <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Reference model, fed with every sample the DUT takes
    // ------------------------------------------------------------------
    // Everything each channel was sent. Sized to a power of 2: Verilator 5.020 generates C++
    // that doesn't compile when indexing an array of queues of another size
    logic [2*WIDTH-1:0] samples [1 << CH_W][$];
    logic [BEAT_W-1:0]  expected[$];
    int                 peak_q[$];              // per packet: expected peak lag, or a big value for none
    int unsigned        next_window = SKIP;
    int unsigned        n_windows = 0;
    int unsigned        delay_samples = 0;      // samples per channel in the delays phase
    int                 delay [CHANNELS];

    localparam longint OUT_MAX = (longint'(1) <<< (OUT_WIDTH - 1)) - 1;
    localparam longint OUT_MIN = -(longint'(1) <<< (OUT_WIDTH - 1));

    function automatic logic [OUT_WIDTH-1:0] scale(longint v);
        longint s = v >>> OUT_SHIFT;
        return OUT_WIDTH'(s > OUT_MAX ? OUT_MAX : s < OUT_MIN ? OUT_MIN : s);
    endfunction

    function automatic void model_window(int unsigned k);
        int unsigned e = (k + 1) * N - 1;
        int unsigned s = e - L + 1;
        int unsigned p = 0;
        for (int a = 0; a < CHANNELS - 1; a++)
            for (int b = a + 1; b < CHANNELS; b++) begin
                for (int m = -D; m <= D; m++) begin
                    longint re = 0, im = 0;
                    for (int n = 0; n < L; n++)
                        if (n + m >= 0 && n + m < L) begin
                            automatic logic signed [WIDTH-1:0] ar, ai, br, bi;
                            {ar, ai} = samples[a][s + n];
                            {br, bi} = samples[b][s + n + m];
                            re += longint'(br) * ar + longint'(bi) * ai;
                            im += longint'(bi) * ar - longint'(br) * ai;
                        end
                    expected.push_back({PAIR_W'(p), scale(re), scale(im), m == D});
                end
                peak_q.push_back(e < delay_samples ? delay[b] - delay[a] : 1 << 30);
                p++;
            end
        n_windows++;
    endfunction

    always @(posedge clk)
        if (rstn && din.tvalid && din.tready) begin
            automatic bit ready = 1'b1;
            samples[din.tid].push_back(din.tdata);
            for (int c = 0; c < CHANNELS; c++)
                if (samples[c].size() < (next_window + 1) * N) ready = 1'b0;
            if (ready) model_window(next_window++);
        end

    // ------------------------------------------------------------------
    // Sink: beats, stability, and the peak of each delays-phase packet
    // ------------------------------------------------------------------
    int unsigned       sink_rate = 100;
    bit                long_stalls = 1'b0;
    int unsigned       stall_left = 0;
    int unsigned       n_beats = 0, n_packets = 0, n_peaks = 0;
    bit                stalled = 1'b0;
    logic [BEAT_W-1:0] stalled_beat;
    real               packet_mag [LAGS];
    int unsigned       lag_index = 0;

    always @(posedge clk) begin
        if (rstn) begin
            automatic logic [BEAT_W-1:0] beat = {dout.tid, dout.tdata, dout.tlast};
            if (stalled) begin
                tb_checks++;
                if (dout.tvalid !== 1'b1 || beat !== stalled_beat)
                    tb_error_once({NAME, " stability"}, $sformatf("%s: output changed while stalled", NAME));
            end

            if (dout.tvalid && dout.tready) begin
                tb_checks++;
                if (expected.size() == 0) begin
                    tb_error_once({NAME, " extra"}, $sformatf("%s: beat with nothing due (tid %0d)", NAME, dout.tid));
                end else begin
                    automatic logic [BEAT_W-1:0] exp = expected.pop_front();
                    if (beat !== exp)
                        tb_error_once({NAME, " data"},
                            $sformatf("%s: beat %0d is tid %0d re %0d im %0d last %b, expected tid %0d re %0d im %0d last %b",
                                      NAME, n_beats, dout.tid,
                                      $signed(dout.tdata[2*OUT_WIDTH-1:OUT_WIDTH]), $signed(dout.tdata[OUT_WIDTH-1:0]), dout.tlast,
                                      exp[BEAT_W-1 -: PAIR_W], $signed(exp[2*OUT_WIDTH:OUT_WIDTH+1]), $signed(exp[OUT_WIDTH:1]), exp[0]));
                end

                if (lag_index < LAGS)
                    packet_mag[lag_index] = real'($signed(dout.tdata[2*OUT_WIDTH-1:OUT_WIDTH])) ** 2
                                          + real'($signed(dout.tdata[OUT_WIDTH-1:0])) ** 2;
                lag_index++;
                if (dout.tlast) begin
                    if (peak_q.size() > 0) begin
                        automatic int want = peak_q.pop_front();
                        if (want != 1 << 30) begin
                            automatic int best = 0;
                            for (int i = 1; i < LAGS; i++)
                                if (packet_mag[i] > packet_mag[best]) best = i;
                            tb_checks++;
                            if (best - D != want)
                                tb_error_once({NAME, " peak"},
                                    $sformatf("%s: pair %0d peaks at lag %0d, expected %0d", NAME, dout.tid, best - D, want));
                            n_peaks++;
                        end
                    end
                    lag_index = 0;
                    n_packets++;
                end
                n_beats++;
            end

            stalled      = dout.tvalid && !dout.tready;
            stalled_beat = beat;
        end

        if (stall_left > 0) begin
            stall_left--;
            dout.tready <= 1'b0;
        end else if (long_stalls && $urandom_range(299) == 0) begin
            stall_left = $urandom_range(400, 50);
            dout.tready <= 1'b0;
        end else begin
            dout.tready <= $urandom_range(99) < sink_rate;
        end
    end

    // Clocks each window run takes (no output stalls in the delays phase, so exactly RUN_CLKS)
    int unsigned run_clks = 0, runs = 0, bad_runs = 0;
    bit          timing_runs = 1'b0;
    always @(posedge clk) begin
        if (dut.ctrl0.valid) begin  // state == RUN
            run_clks++;
        end else if (run_clks > 0) begin
            if (timing_runs) begin
                runs++;
                if (run_clks != RUN_CLKS) begin
                    if (bad_runs == 0)
                        tb_error($sformatf("%s: a window took %0d clocks, expected %0d", NAME, run_clks, RUN_CLKS));
                    bad_runs++;
                end
            end
            run_clks = 0;
        end
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic finish_phase(string name);
        int unsigned timeout = 0;
        while ((src_q.size() > 0 || expected.size() > 0) && timeout < 4_000_000) begin
            @(posedge clk);
            timeout++;
        end
        `CHECK(src_q.size() == 0, $sformatf("%s, %s: %0d samples never went in", NAME, name, src_q.size()))
        `CHECK(expected.size() == 0, $sformatf("%s, %s: %0d beats never came out", NAME, name, expected.size()))
        tb_info($sformatf("%s: %s done, %0d windows, %0d packets", NAME, name, n_windows, n_packets));
    endtask

    function automatic logic [WIDTH-1:0] rand_sample(int bits);
        return WIDTH'($signed(WIDTH'($urandom)) >>> (WIDTH - bits));
    endfunction

    task automatic phase_delays();
        int unsigned frames = (SKIP + 1) * N + (6 * N > 40 ? 6 * N : 40);
        int          max_d = 0;
        logic [2*WIDTH-1:0] s [];
        foreach (delay[c]) begin
            delay[c] = (c * 7 + 3) % (D + 1);
            if (delay[c] > max_d) max_d = delay[c];
        end
        s = new[frames + max_d];
        foreach (s[i]) s[i] = {rand_sample(WIDTH - 3), rand_sample(WIDTH - 3)};  // 1/8 full scale
        delay_samples = frames;
        // Just slow enough to keep up: a window's clocks spread over N frames, plus margin
        interval  = (RUN_CLKS + 20) * 5 / 4 / (N * CHANNELS) + 1;
        sink_rate = 100;
        in_stalls = 0;
        timing_runs = 1'b1;
        for (int n = 0; n < frames; n++)
            for (int c = 0; c < CHANNELS; c++)
                src_q.push_back({CH_W'(c), s[n + max_d - delay[c]]});
        finish_phase("delays");
        timing_runs = 1'b0;
        `CHECK(in_stalls == 0, $sformatf("%s: input held off %0d clocks at a rate it should keep up with", NAME, in_stalls))
        `CHECK(runs > 0 && bad_runs == 0, $sformatf("%s: %0d of %0d windows took other than %0d clocks", NAME, bad_runs, runs, RUN_CLKS))
        `CHECK(n_peaks > 0, $sformatf("%s: no packets were peak checked", NAME))
        tb_info($sformatf("%s: %0d clocks per window, %0d packets peak checked", NAME, RUN_CLKS, n_peaks));
    endtask

    task automatic phase_noise();
        int unsigned frames = 6 * N > 40 ? 6 * N : 40;
        int order[$];
        interval    = 0;
        gap_pct     = 0;
        sink_rate   = 60;
        long_stalls = 1'b1;
        repeat (frames) begin
            order.delete();
            for (int c = 0; c < CHANNELS; c++) order.push_back(c);
            order.shuffle();
            foreach (order[i]) begin
                automatic int unsigned r = $urandom_range(9);
                automatic logic [WIDTH-1:0] re = r == 0 ? {1'b1, {(WIDTH-1){1'b0}}} : rand_sample(WIDTH);
                automatic logic [WIDTH-1:0] im = r == 1 ? {1'b1, {(WIDTH-1){1'b0}}} : rand_sample(WIDTH);
                src_q.push_back({CH_W'(order[i]), re, im});
            end
        end
        finish_phase("noise");
        sink_rate   = 100;
        long_stalls = 1'b0;
        tb_info($sformatf("%s: input held off %0d clocks", NAME, in_stalls));
    endtask

    initial begin
        done = 1'b0;
        dout.tready = 1'b1;
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        @(posedge clk);
        `CHECK_EQ(dout.tvalid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        phase_delays();
        phase_noise();

        repeat (50) @(posedge clk);
        `CHECK(expected.size() == 0, $sformatf("%s: %0d beats never came out", NAME, expected.size()))
        `CHECK(n_packets == n_windows * PAIRS, $sformatf("%s: %0d packets for %0d windows", NAME, n_packets, n_windows))
        `CHECK_EQ(dout.tvalid, 1'b0, $sformatf("%s: tvalid low when idle", NAME))
        done = 1'b1;
    end

endmodule


module tb_cross_correlator;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(50_000_000) // 50 ms

    localparam real CLK_PERIOD_NS = 1.0e9 / 200.0e6; // clk_core, where the DSP lives

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    logic [3:0] done;

    // Several lanes, the last lane group partly used (11 lags in groups of 3)
    cross_correlator_inst #(.CHANNELS(4), .WIDTH(18), .L(64), .N(16), .D(5), .MACS(3))
        g_4ch (.clk(clk), .done(done[0]));

    // One lane, non-overlapping windows (like the MATLAB model), narrow output that saturates
    cross_correlator_inst #(.CHANNELS(3), .WIDTH(12), .L(20), .N(20), .D(4), .MACS(1), .OUT_WIDTH(16), .OUT_SHIFT(8))
        g_3ch (.clk(clk), .done(done[1]));

    // 8 channels (28 pairs), a window every sample, all lags in one group
    cross_correlator_inst #(.CHANNELS(8), .WIDTH(18), .L(32), .N(1), .D(3), .MACS(7))
        g_8ch (.clk(clk), .done(done[2]));

    // 2 channels (1 pair), L not a multiple of N, 15 lags in groups of 4
    cross_correlator_inst #(.CHANNELS(2), .WIDTH(16), .L(50), .N(16), .D(7), .MACS(4))
        g_2ch (.clk(clk), .done(done[3]));

    initial begin
        tb_info("Testing cross_correlator in 4 configurations");
        wait (&done);
        tb_finish();
    end

endmodule
