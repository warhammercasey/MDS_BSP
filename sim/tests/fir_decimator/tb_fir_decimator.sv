`timescale 1ns/1ps
`include "tb_macros.svh"

// fir_decimator test: several channels of complex samples, told apart by tid, each
// filtered on its own with the same taps and decimated, checked bit-exact against a
// reference model.
//
// Reference, per channel c, with x[n] the channel's n-th sample (zeros before the
// first) and h[k] the taps read from the DUT's own init file:
//   after every DECIMATION-th sample n:  y = sum_{k=0}^{TAPS-1} h[k] x[n-k]
//   output {I, Q} = the top WIDTH bits of the 2*WIDTH bit sums (floor(y / 2^WIDTH)),
//   I in the upper half of tdata, tid c.
// The model also checks the stimulus can't overflow a 2*WIDTH bit accumulator, so a
// mismatch is the DUT's, not the test's.
//
// Checked throughout:
//   - each channel's outputs come out in order and match the model, and nothing
//     comes out that the model didn't expect
//   - the output holds tvalid/tdata/tid while stalled
// Phases (inputs are sent a frame at a time: one sample per channel):
//   impulses       one full-scale impulse per channel (I positive, Q negative) at a
//                  different decimation phase per channel, zeros otherwise: the
//                  outputs are the taps, so ordering mistakes are easy to read
//   noise          random full-scale I/Q, channels in order, input below the DUT's
//                  throughput, output always ready
//   tones          a complex tone per channel (view rx_i / rx_q as analog in waves)
//   full rate      noise sent back to back in a shuffled channel order every frame:
//                  the DUT has to throttle the input with tready
//   slow sink      noise with the output ready ~30% of the time, but never stalled
//                  for more than TAPS/2 cycles: fir_decimator has no output
//                  backpressure, so each result has to be taken before the next is
//                  done (TAPS + 1 cycles at the fastest)
module fir_decimator_inst #(
    parameter int    WIDTH      = 18,
    parameter int    TAPS       = 32,
    parameter int    DECIMATION = 4,
    parameter int    TID_WIDTH  = 3,
    // Untyped, like the DUT's INIT_FILE: a `string` doesn't survive being passed
    // through untyped parameters in Verilator (the ROM's $readmemh can't find the file)
    parameter        TAPS_FILE  = "",
    parameter int    N_FRAMES   = 64  // input frames per phase
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int CHANNELS  = 1 << TID_WIDTH;
    localparam int ACC_WIDTH = 2 * WIDTH;

    logic rstn = 1'b0;

    axis_tid_if #(.WIDTH(2 * WIDTH), .ID_WIDTH(TID_WIDTH)) din ();
    axis_tid_if #(.WIDTH(2 * WIDTH), .ID_WIDTH(TID_WIDTH)) dout ();

    fir_decimator #(
        .WIDTH(WIDTH),
        .TAPS(TAPS),
        .DECIMATION(DECIMATION),
        .TID_WIDTH(TID_WIDTH),
        .INIT_FILE(TAPS_FILE),
        .INIT_HEX(1)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .din(din),
        .dout(dout)
    );

    string NAME = $sformatf("%0d taps, /%0d, %0d ch", TAPS, DECIMATION, CHANNELS);

    // The taps, from the same file as the DUT's ROM
    logic signed [WIDTH-1:0] taps [TAPS];
    initial $readmemh(TAPS_FILE, taps);

    // ------------------------------------------------------------------
    // Source: beats from a queue, {tid, I, Q}
    // ------------------------------------------------------------------
    logic [TID_WIDTH+2*WIDTH-1:0] src_q[$];
    int unsigned                  gap_pct = 0;

    always @(posedge clk) begin
        bit hs;
        if (!rstn) begin
            din.tvalid <= 1'b0;
            din.tdata  <= '0;
            din.tid    <= '0;
        end else begin
            hs = din.tvalid && din.tready;
            if (hs) void'(src_q.pop_front());
            if (!din.tvalid || hs) begin
                if (src_q.size() > 0 && $urandom_range(99) >= gap_pct) begin
                    din.tvalid <= 1'b1;
                    {din.tid, din.tdata} <= src_q[0];
                end else begin
                    din.tvalid <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Reference model, fed with every sample the DUT takes
    // ------------------------------------------------------------------
    logic signed [WIDTH-1:0] hist_i [CHANNELS][$];  // newest first, TAPS long
    logic signed [WIDTH-1:0] hist_q [CHANNELS][$];
    int unsigned             n_in [CHANNELS] = '{default: 0};
    logic [2*WIDTH-1:0]      expected [CHANNELS][$];
    int unsigned             n_sent = 0;      // samples taken by the DUT
    int unsigned             n_expected = 0;  // outputs the model made

    initial
        for (int c = 0; c < CHANNELS; c++)
            repeat (TAPS) begin
                hist_i[c].push_front('0);
                hist_q[c].push_front('0);
            end

    function automatic bit fits_acc(longint v);
        return v >= -(longint'(1) <<< (ACC_WIDTH - 1)) && v < (longint'(1) <<< (ACC_WIDTH - 1));
    endfunction

    function automatic void model(int c, logic signed [WIDTH-1:0] xi, logic signed [WIDTH-1:0] xq);
        longint acc_i = 0, acc_q = 0;
        hist_i[c].push_front(xi);
        hist_q[c].push_front(xq);
        void'(hist_i[c].pop_back());
        void'(hist_q[c].pop_back());
        if (n_in[c]++ % DECIMATION != DECIMATION - 1)
            return;
        for (int k = 0; k < TAPS; k++) begin
            acc_i += longint'(taps[k]) * longint'(hist_i[c][k]);
            acc_q += longint'(taps[k]) * longint'(hist_q[c][k]);
        end
        if (!fits_acc(acc_i) || !fits_acc(acc_q))
            tb_error_once({NAME, " overflow"},
                $sformatf("%s: test stimulus overflows a %0d bit accumulator (channel %0d)", NAME, ACC_WIDTH, c));
        expected[c].push_back({WIDTH'(acc_i >>> WIDTH), WIDTH'(acc_q >>> WIDTH)});
        n_expected++;
    endfunction

    always @(posedge clk)
        if (rstn && din.tvalid && din.tready) begin
            model(int'(din.tid), din.tdata[2*WIDTH-1:WIDTH], din.tdata[WIDTH-1:0]);
            n_sent++;
        end

    // ------------------------------------------------------------------
    // Sink: order, values and stability
    // ------------------------------------------------------------------
    int unsigned                  sink_rate = 100;  // % of cycles ready
    int unsigned                  stall_cycles = 0; // cycles the current output has waited
    int unsigned                  n_recv = 0;
    int unsigned                  n_rx [CHANNELS] = '{default: 0};
    bit                           stalled = 1'b0;
    logic [TID_WIDTH+2*WIDTH-1:0] stalled_beat;

    logic signed [WIDTH-1:0]      rx_i [CHANNELS];  // last output per channel (view as analog in waves)
    logic signed [WIDTH-1:0]      rx_q [CHANNELS];

    always @(posedge clk) begin
        if (rstn) begin
            if (stalled) begin
                tb_checks++;
                if (dout.tvalid !== 1'b1 || {dout.tid, dout.tdata} !== stalled_beat)
                    tb_error_once({NAME, " stability"}, $sformatf("%s: output changed while stalled", NAME));
            end

            if (dout.tvalid && dout.tready) begin
                automatic int c = int'(dout.tid);
                tb_checks++;
                if (expected[c].size() == 0) begin
                    tb_error_once({NAME, " extra"},
                        $sformatf("%s: channel %0d output 0x%0h with no output due", NAME, c, dout.tdata));
                end else begin
                    automatic logic [2*WIDTH-1:0] exp = expected[c].pop_front();
                    if (dout.tdata !== exp)
                        tb_error_once({NAME, " data"},
                            $sformatf("%s: channel %0d output %0d is I %0d Q %0d, expected I %0d Q %0d", NAME, c, n_rx[c],
                                      $signed(dout.tdata[2*WIDTH-1:WIDTH]), $signed(dout.tdata[WIDTH-1:0]),
                                      $signed(exp[2*WIDTH-1:WIDTH]), $signed(exp[WIDTH-1:0])));
                end
                {rx_i[c], rx_q[c]} = dout.tdata;
                n_rx[c]++;
                n_recv++;
            end

            stalled      = dout.tvalid && !dout.tready;
            stalled_beat = {dout.tid, dout.tdata};
        end

        // Random, but an output never waits more than TAPS/2 cycles (see the slow sink phase)
        stall_cycles = stalled ? stall_cycles + 1 : 0;
        dout.tready <= stall_cycles + 1 >= TAPS / 2 || $urandom_range(99) < sink_rate;
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    localparam logic signed [WIDTH-1:0] MAX = (1 << (WIDTH - 1)) - 1;
    localparam logic signed [WIDTH-1:0] MIN = -(1 << (WIDTH - 1));

    function automatic logic [WIDTH-1:0] noise();
        return WIDTH'($urandom);
    endfunction

    // One frame: a sample per channel, in channel order or shuffled
    typedef logic [2*WIDTH-1:0] frame_t [CHANNELS];
    function automatic void send_frame(frame_t f, bit shuffled);
        int order[$];
        order.delete();  // Otherwise the sim keeps it from the last call
        for (int c = 0; c < CHANNELS; c++) order.push_back(c);
        if (shuffled) order.shuffle();
        foreach (order[i])
            src_q.push_back({TID_WIDTH'(order[i]), f[order[i]]});
    endfunction

    // Wait for everything queued to go in and every output due to come out
    task automatic finish_phase(string name);
        int unsigned timeout = 0;
        while ((src_q.size() > 0 || n_recv < n_expected) && timeout < 400 * N_FRAMES * CHANNELS) begin
            @(posedge clk);
            timeout++;
        end
        `CHECK(src_q.size() == 0, $sformatf("%s, %s: %0d samples never went in", NAME, name, src_q.size()))
        `CHECK(n_recv == n_expected, $sformatf("%s, %s: %0d outputs came out, %0d due", NAME, name, n_recv, n_expected))
        tb_info($sformatf("%s: %s done, %0d outputs so far", NAME, name, n_recv));
    endtask

    task automatic phase_impulses();
        frame_t f;
        gap_pct = 80;
        for (int n = 0; n < CHANNELS * DECIMATION + TAPS + DECIMATION; n++) begin
            for (int c = 0; c < CHANNELS; c++) begin
                // Channel c's I impulse lands on sample c, its Q impulse DECIMATION*c + 1 later
                f[c] = {n == c ? MAX : WIDTH'(0), n == c + DECIMATION * c + 1 ? MIN : WIDTH'(0)};
            end
            send_frame(f, 1'b0);
        end
        finish_phase("impulses");
    endtask

    task automatic phase_noise(string name, int unsigned gaps, bit shuffled, int unsigned sink);
        frame_t f;
        gap_pct   = gaps;
        sink_rate = sink;
        repeat (N_FRAMES) begin
            for (int c = 0; c < CHANNELS; c++) f[c] = {noise(), noise()};
            send_frame(f, shuffled);
        end
        finish_phase(name);
        sink_rate = 100;
    endtask

    task automatic phase_tones();
        frame_t f;
        gap_pct = 90;
        for (int n = 0; n < 4 * N_FRAMES; n++) begin
            for (int c = 0; c < CHANNELS; c++) begin
                // From DC (passband) to 0.35 fs (stopband), in frequency per input sample
                automatic real w     = 2.0 * 3.14159265358979 * 0.35 * c / (CHANNELS > 1 ? CHANNELS - 1 : 1);
                automatic real scale = 0.9 * MAX;
                f[c] = {WIDTH'($rtoi(scale * $cos(w * n))), WIDTH'($rtoi(scale * $sin(w * n)))};
            end
            send_frame(f, 1'b0);
        end
        finish_phase("tones");
    endtask

    initial begin
        done = 1'b0;
        dout.tready = 1'b1;
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        @(posedge clk);
        `CHECK_EQ(dout.tvalid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        phase_impulses();
        phase_noise("noise",     90, 1'b0, 100);
        phase_tones();
        phase_noise("full rate", 0,  1'b1, 100);
        phase_noise("slow sink", 0,  1'b1, 30);

        repeat (4 * TAPS) @(posedge clk);
        `CHECK(n_recv == n_expected, $sformatf("%s: %0d outputs came out, %0d due", NAME, n_recv, n_expected))
        for (int c = 0; c < CHANNELS; c++)
            `CHECK(n_in[c] / DECIMATION == n_rx[c],
                   $sformatf("%s: channel %0d: %0d samples in, %0d outputs (expected %0d)",
                             NAME, c, n_in[c], n_rx[c], n_in[c] / DECIMATION))
        `CHECK_EQ(dout.tvalid, 1'b0, $sformatf("%s: tvalid low when idle", NAME))
        tb_info($sformatf("%s: %0d samples in, %0d outputs", NAME, n_sent, n_recv));
        done = 1'b1;
    end

endmodule


module tb_fir_decimator;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(10_000_000) // 10 ms

    localparam real CLK_PERIOD_NS = 1.0e9 / 200.0e6; // clk_core, where the DSP lives

    // The sim runs in sim/out/<test>, and $readmemh paths are relative to it
    localparam TEST_DIR = "../../tests/fir_decimator/";  // untyped: see TAPS_FILE

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    logic [1:0] done;

    // The example lowpass, sized like the mic path: 8 channels, 32 taps, decimate by 4
    fir_decimator_inst #(
        .TAPS(32), .DECIMATION(4), .TID_WIDTH(3), .TAPS_FILE({TEST_DIR, "taps_lowpass.hex"})
    ) g_lowpass (.clk(clk), .done(done[0]));

    // Asymmetric, all-different taps to catch tap order, with a tap count that isn't a
    // power of 2 and an odd decimation
    fir_decimator_inst #(
        .TAPS(20), .DECIMATION(3), .TID_WIDTH(1), .TAPS_FILE({TEST_DIR, "taps_ramp.hex"})
    ) g_ramp (.clk(clk), .done(done[1]));

    initial begin
        tb_info("Testing fir_decimator: 32 taps / 4 x 8 channels (lowpass), 20 taps / 3 x 2 channels (ramp)");
        wait (&done);
        tb_finish();
    end

endmodule
