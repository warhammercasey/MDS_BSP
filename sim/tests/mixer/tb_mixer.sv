`timescale 1ns/1ps
`include "tb_macros.svh"

// mixer test: several channels of real samples, told apart by tid, mixed to complex
// baseband with the same compile-time tone.
//
// Checked:
//   - the LO table file against ideal cos/sin (within 0.5 LSB), and the DUT's compile-time
//     phase step against round(TONE_FREQ / SAMPLE_RATE * 2^PHASE_BITS)
//   - bit-exact outputs against a model: per channel phase, the table read from the same
//     file, I = round(x*cos / 2^17), Q = round(-x*sin / 2^17), in input order, tid kept
//   - the mixing itself, independently of the model: each channel gets a real tone at
//     LO + d (a different d per channel), where LO is the frequency the phase step gives
//     (TONE_FREQ to within the step's resolution). The output must hold a complex tone at
//     +d with half the input amplitude, and nothing at -d (which is where it would land
//     with the LO's sign wrong)
//   - the output holds while stalled, and no extra or missing outputs
// Phases:
//   tones      the tones, channels in order, back to back, output always ready: must
//              take one sample per clock
//   noise      random samples (some at full scale +/-), shuffled channel order, input
//              gaps, output ready ~50% of the time
module mixer_inst #(
    parameter int WIDTH       = 18,
    parameter int TID_WIDTH   = 3,
    parameter int SAMPLE_RATE = 48_000,
    parameter int TONE_FREQ   = 7_000,
    parameter int DELTA       = 100,   // channel c's test tone is at TONE_FREQ + DELTA*(c+1)
    parameter int N_TONE      = 4800,  // samples per channel; DELTA*N_TONE/SAMPLE_RATE must be whole
    parameter int N_NOISE     = 500,   // frames of noise
    parameter int PHASE_BITS  = 32,
    parameter     LUT_FILE    = ""     // untyped: see tb_fir_decimator
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int  CHANNELS   = 1 << TID_WIDTH;
    localparam int  LO_WIDTH   = 18;
    localparam int  LUT_AWIDTH = 8;
    localparam real PI         = 3.14159265358979;

    logic rstn = 1'b0;

    axis_tid_if #(.WIDTH(WIDTH),     .ID_WIDTH(TID_WIDTH)) din ();
    axis_tid_if #(.WIDTH(2 * WIDTH), .ID_WIDTH(TID_WIDTH)) dout ();

    mixer #(
        .WIDTH(WIDTH),
        .TID_WIDTH(TID_WIDTH),
        .SAMPLE_RATE(SAMPLE_RATE),
        .TONE_FREQ(TONE_FREQ),
        .PHASE_WIDTH(PHASE_BITS),
        .LUT_AWIDTH(LUT_AWIDTH),
        .INIT_FILE(LUT_FILE),
        .INIT_HEX(1)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .din(din),
        .dout(dout)
    );

    string NAME = $sformatf("%0d ch, %0d bit, %0d Hz at %0d Hz", CHANNELS, WIDTH, TONE_FREQ, SAMPLE_RATE);

    localparam logic signed [WIDTH-1:0] MAX = (1 << (WIDTH - 1)) - 1;
    localparam logic signed [WIDTH-1:0] MIN = -(1 << (WIDTH - 1));

    // The LO table, from the same file as the DUT's ROM
    logic [2*LO_WIDTH-1:0] lut [1 << LUT_AWIDTH];
    initial $readmemh(LUT_FILE, lut);

    // Phase step, worked out independently of the DUT
    localparam longint PHASE_INC_FULL = longint'(real'(TONE_FREQ) / real'(SAMPLE_RATE) * 2.0 ** PHASE_BITS);
    localparam logic [PHASE_BITS-1:0] PHASE_INC = PHASE_BITS'(PHASE_INC_FULL);
    // The LO frequency that step gives: TONE_FREQ to within SAMPLE_RATE / 2^(PHASE_BITS+1)
    localparam real LO_FREQ = real'(PHASE_INC_FULL) * real'(SAMPLE_RATE) / 2.0 ** PHASE_BITS;

    // ------------------------------------------------------------------
    // Source: beats from a queue, {tid, x}
    // ------------------------------------------------------------------
    logic [TID_WIDTH+WIDTH-1:0] src_q[$];
    int unsigned                gap_pct = 0;
    int unsigned                in_stalls = 0;  // cycles with tvalid && !tready

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
    logic [PHASE_BITS-1:0]        phase [CHANNELS] = '{default: '0};
    logic [TID_WIDTH+2*WIDTH-1:0] expected[$];  // {tid, I, Q}, in order
    int unsigned                  n_sent = 0;

    function automatic logic [2*WIDTH-1:0] model(int c, logic signed [WIDTH-1:0] x);
        logic [LUT_AWIDTH+1:0]      idx  = phase[c][PHASE_BITS-1 -: LUT_AWIDTH+2];
        logic [2*LO_WIDTH-1:0]      e    = lut[idx[LUT_AWIDTH-1:0]];
        logic signed [LO_WIDTH-1:0] tc   = e[2*LO_WIDTH-1:LO_WIDTH];
        logic signed [LO_WIDTH-1:0] ts   = e[LO_WIDTH-1:0];
        longint                     cosv, sinv, i, q;
        // cos and sin of quadrant*90 degrees + the table's angle
        case (idx[LUT_AWIDTH+1 -: 2])
            2'd0: begin cosv =  longint'(tc); sinv =  longint'(ts); end
            2'd1: begin cosv = -longint'(ts); sinv =  longint'(tc); end
            2'd2: begin cosv = -longint'(tc); sinv = -longint'(ts); end
            default: begin cosv = longint'(ts); sinv = -longint'(tc); end
        endcase
        i = (longint'(x) *  cosv + (longint'(1) <<< (LO_WIDTH - 2))) >>> (LO_WIDTH - 1);
        q = (longint'(x) * -sinv + (longint'(1) <<< (LO_WIDTH - 2))) >>> (LO_WIDTH - 1);
        phase[c] += PHASE_INC;
        return {WIDTH'(i), WIDTH'(q)};
    endfunction

    always @(posedge clk)
        if (rstn && din.tvalid && din.tready) begin
            expected.push_back({din.tid, model(int'(din.tid), din.tdata)});
            n_sent++;
        end

    // ------------------------------------------------------------------
    // Sink: values, order, stability, and the tone measurements
    // ------------------------------------------------------------------
    int unsigned                  sink_rate = 100;
    int unsigned                  n_recv = 0;
    int unsigned                  n_rx [CHANNELS] = '{default: 0};
    bit                           stalled = 1'b0;
    logic [TID_WIDTH+2*WIDTH-1:0] stalled_beat;

    logic signed [WIDTH-1:0]      rx_i [CHANNELS];  // last output per channel (view as analog in waves)
    logic signed [WIDTH-1:0]      rx_q [CHANNELS];

    // Correlation of each channel's first N_TONE outputs with exp(+-j*2*pi*d*n/fs)
    real pos_re [CHANNELS] = '{default: 0.0}, pos_im [CHANNELS] = '{default: 0.0};
    real neg_re [CHANNELS] = '{default: 0.0}, neg_im [CHANNELS] = '{default: 0.0};

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
                if (expected.size() == 0) begin
                    tb_error_once({NAME, " extra"}, $sformatf("%s: output with nothing due (tid %0d)", NAME, c));
                end else begin
                    automatic logic [TID_WIDTH+2*WIDTH-1:0] exp = expected.pop_front();
                    if ({dout.tid, dout.tdata} !== exp)
                        tb_error_once({NAME, " data"},
                            $sformatf("%s: output %0d is tid %0d I %0d Q %0d, expected tid %0d I %0d Q %0d", NAME, n_recv,
                                      dout.tid, $signed(dout.tdata[2*WIDTH-1:WIDTH]), $signed(dout.tdata[WIDTH-1:0]),
                                      exp[TID_WIDTH+2*WIDTH-1 -: TID_WIDTH],
                                      $signed(exp[2*WIDTH-1:WIDTH]), $signed(exp[WIDTH-1:0])));
                end

                {rx_i[c], rx_q[c]} = dout.tdata;
                if (n_rx[c] < N_TONE) begin
                    automatic real w  = 2.0 * PI * DELTA * (c + 1) * n_rx[c] / SAMPLE_RATE;
                    automatic real yi = rx_i[c], yq = rx_q[c];
                    // y * exp(-jw) picks out +d, y * exp(+jw) picks out -d
                    pos_re[c] += yi * $cos(w) + yq * $sin(w);
                    pos_im[c] += yq * $cos(w) - yi * $sin(w);
                    neg_re[c] += yi * $cos(w) - yq * $sin(w);
                    neg_im[c] += yq * $cos(w) + yi * $sin(w);
                end
                n_rx[c]++;
                n_recv++;
            end

            stalled      = dout.tvalid && !dout.tready;
            stalled_beat = {dout.tid, dout.tdata};
        end
        dout.tready <= $urandom_range(99) < sink_rate;
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic finish_phase(string name);
        int unsigned timeout = 0;
        while ((src_q.size() > 0 || n_recv < n_sent) && timeout < 100_000 + 20 * N_TONE * CHANNELS) begin
            @(posedge clk);
            timeout++;
        end
        `CHECK(src_q.size() == 0, $sformatf("%s, %s: %0d samples never went in", NAME, name, src_q.size()))
        `CHECK(n_recv == n_sent, $sformatf("%s, %s: %0d outputs, %0d samples in", NAME, name, n_recv, n_sent))
        tb_info($sformatf("%s: %s done, %0d outputs so far", NAME, name, n_recv));
    endtask

    task automatic check_lut();
        int unsigned bad = 0;
        for (int k = 0; k < (1 << LUT_AWIDTH); k++) begin
            automatic real phi = PI / 2.0 * k / (1 << LUT_AWIDTH);
            automatic real a   = (1 << (LO_WIDTH - 1)) - 1;
            automatic logic signed [LO_WIDTH-1:0] c = lut[k][2*LO_WIDTH-1:LO_WIDTH];
            automatic logic signed [LO_WIDTH-1:0] s = lut[k][LO_WIDTH-1:0];
            if ((c - a * $cos(phi)) ** 2 > 0.25 + 1e-6 || (s - a * $sin(phi)) ** 2 > 0.25 + 1e-6) begin
                if (bad++ < 3)
                    tb_error($sformatf("%s: LO table entry %0d is cos %0d sin %0d, ideal %.2f %.2f",
                                       NAME, k, c, s, a * $cos(phi), a * $sin(phi)));
            end
        end
        `CHECK(bad == 0, $sformatf("%s: %0d LO table entries off by more than 0.5 LSB", NAME, bad))
    endtask

    localparam real AMP = 0.9 * MAX;  // test tone amplitude

    task automatic phase_tones();
        gap_pct   = 0;
        sink_rate = 100;
        in_stalls = 0;
        for (int n = 0; n < N_TONE; n++)
            for (int c = 0; c < CHANNELS; c++) begin
                automatic real f = LO_FREQ + DELTA * (c + 1);
                src_q.push_back({TID_WIDTH'(c), WIDTH'($rtoi(AMP * $cos(2.0 * PI * f * n / SAMPLE_RATE)))});
            end
        finish_phase("tones");
        `CHECK(in_stalls == 0, $sformatf("%s: input stalled %0d cycles with the output always ready", NAME, in_stalls))

        for (int c = 0; c < CHANNELS; c++) begin
            automatic real pos = $sqrt(pos_re[c] ** 2 + pos_im[c] ** 2) / N_TONE;
            automatic real neg = $sqrt(neg_re[c] ** 2 + neg_im[c] ** 2) / N_TONE;
            `CHECK(pos > 0.99 * AMP / 2 && pos < 1.01 * AMP / 2,
                   $sformatf("%s: channel %0d: tone at +%0d Hz is %.1f, expected %.1f", NAME, c, DELTA * (c + 1), pos, AMP / 2))
            `CHECK(neg < 0.001 * AMP,
                   $sformatf("%s: channel %0d: %.1f at -%0d Hz, expected ~0", NAME, c, neg, DELTA * (c + 1)))
            if (c == 0)
                tb_info($sformatf("%s: channel 0: %.1f at +%0d Hz (expected %.1f), %.3f at -%0d Hz",
                                  NAME, pos, DELTA, AMP / 2, neg, DELTA));
        end
    endtask

    task automatic phase_noise();
        int order[$];
        gap_pct   = 30;
        sink_rate = 50;
        repeat (N_NOISE) begin
            order.delete();
            for (int c = 0; c < CHANNELS; c++) order.push_back(c);
            order.shuffle();
            foreach (order[i]) begin
                automatic int unsigned r = $urandom_range(9);
                src_q.push_back({TID_WIDTH'(order[i]), r == 0 ? MAX : r == 1 ? MIN : WIDTH'($urandom)});
            end
        end
        finish_phase("noise");
        sink_rate = 100;
    endtask

    initial begin
        done = 1'b0;
        repeat (5) @(posedge clk);
        `CHECK_EQ(dut.PHASE_INC, PHASE_INC, $sformatf("%s: phase step", NAME))
        check_lut();
        rstn = 1'b1;
        @(posedge clk);
        `CHECK_EQ(dout.tvalid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        phase_tones();
        phase_noise();

        repeat (20) @(posedge clk);
        `CHECK(n_recv == n_sent, $sformatf("%s: %0d outputs, %0d samples in", NAME, n_recv, n_sent))
        `CHECK_EQ(dout.tvalid, 1'b0, $sformatf("%s: tvalid low when idle", NAME))
        tb_info($sformatf("%s: %0d samples", NAME, n_recv));
        done = 1'b1;
    end

endmodule


module tb_mixer;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(5_000_000) // 5 ms

    localparam real CLK_PERIOD_NS = 1.0e9 / 200.0e6; // clk_core, where the DSP lives

    // The sim runs in sim/out/<test>, and $readmemh paths are relative to it
    localparam LUT_FILE = "../../../src/dsp/building_blocks/mixer_lut.hex";

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    logic [2:0] done;

    // 8 18-bit channels, like the mics
    mixer_inst #(
        .WIDTH(18), .TID_WIDTH(3), .SAMPLE_RATE(48_000), .TONE_FREQ(7_000), .N_TONE(4800), .LUT_FILE(LUT_FILE)
    ) g_mics (.clk(clk), .done(done[0]));

    // A negative tone, a narrower sample, 2 channels, and a ratio that isn't a nice binary fraction
    mixer_inst #(
        .WIDTH(12), .TID_WIDTH(1), .SAMPLE_RATE(44_100), .TONE_FREQ(-5_000), .N_TONE(4410), .LUT_FILE(LUT_FILE)
    ) g_neg (.clk(clk), .done(done[1]));

    // cain_top's settings: 5 kHz at 47898 Hz with the shortest phase (the table's 10 bits), so
    // the LO is 5004.9 Hz. 50 Hz steps: 25 whole cycles in 23949 samples
    mixer_inst #(
        .WIDTH(18), .TID_WIDTH(3), .SAMPLE_RATE(47_898), .TONE_FREQ(5_000), .DELTA(50), .N_TONE(23_949),
        .N_NOISE(100), .PHASE_BITS(10), .LUT_FILE(LUT_FILE)
    ) g_top (.clk(clk), .done(done[2]));

    initial begin
        tb_info("Testing mixer: 8 x 18 bit, +7 kHz at 48 kHz; 2 x 12 bit, -5 kHz at 44.1 kHz; 8 x 18 bit, 5 kHz at 47898 Hz, 10 bit phase");
        wait (&done);
        tb_finish();
    end

endmodule
