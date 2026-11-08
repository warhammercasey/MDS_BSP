`timescale 1ns/1ps
`include "tb_macros.svh"

// peak_finder test: packets of complex samples in, the index of each packet's largest
// re^2 + im^2 out (the first on a tie), with the packet's tid. Checked against a model with
// full precision magnitudes.
//
// Packet kinds (mixed at random, random lengths 1..MAX_LEN, random tids):
//   random     random full-range re/im
//   extremes   the most negative value and full scale in re and im, which only the exact
//              magnitude can order
//   near ties  large magnitudes that differ by 1 (re^2 + im^2 vs (re^2 + im^2) + 1), so a
//              truncated or partly wrong magnitude picks the wrong one
//   ties       the same magnitude several times (sign and re/im swaps): the first must win
//   small      values that fit in the low 17-bit piece only
//   cross term two beats that only the 2*lo*hi*2^17 term of |x|^2 puts in the right order
// With random gaps in, and the output ready ~50% of the time with stalls of up to 200 clocks.
module peak_finder_inst #(
    parameter int WIDTH     = 32,
    parameter int MAX_LEN   = 21,
    parameter int TID_WIDTH = 5,
    parameter int PACKETS   = 400
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int IDX_W = MAX_LEN > 1 ? $clog2(MAX_LEN) : 1;
    localparam int MAG_W = 2 * WIDTH + 1;

    logic rstn = 1'b0;

    axis_tid_tlast_if #(.WIDTH(2 * WIDTH), .ID_WIDTH(TID_WIDTH)) din ();
    axis_tid_if       #(.WIDTH(IDX_W),     .ID_WIDTH(TID_WIDTH)) dout ();

    peak_finder #(
        .WIDTH(WIDTH),
        .MAX_LEN(MAX_LEN),
        .TID_WIDTH(TID_WIDTH)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .din(din),
        .dout(dout)
    );

    string NAME = $sformatf("%0d bit, packets up to %0d", WIDTH, MAX_LEN);

    localparam logic signed [WIDTH-1:0] MAX = (1 <<< (WIDTH - 1)) - 1;
    localparam logic signed [WIDTH-1:0] MIN = -(1 <<< (WIDTH - 1));

    // ------------------------------------------------------------------
    // Source: beats from a queue, {tlast, tid, re, im}
    // ------------------------------------------------------------------
    logic [TID_WIDTH+2*WIDTH:0] src_q[$];

    always @(posedge clk) begin
        bit hs;
        if (!rstn) begin
            din.tvalid <= 1'b0;
            din.tdata  <= '0;
            din.tid    <= '0;
            din.tlast  <= 1'b0;
        end else begin
            hs = din.tvalid && din.tready;
            if (hs) void'(src_q.pop_front());
            if (!din.tvalid || hs) begin
                if (src_q.size() > 0 && $urandom_range(99) >= 30) begin
                    din.tvalid <= 1'b1;
                    {din.tlast, din.tid, din.tdata} <= src_q[0];
                end else begin
                    din.tvalid <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Model: queue each packet's answer as it's built
    // ------------------------------------------------------------------
    logic [TID_WIDTH+IDX_W-1:0] expected[$];  // {tid, index}
    int unsigned                n_cross = 0;

    function automatic logic [MAG_W-1:0] mag2(logic signed [WIDTH-1:0] re, logic signed [WIDTH-1:0] im);
        logic signed [MAG_W-1:0] r = re, i = im;
        return r * r + i * i;
    endfunction

    function automatic void send_packet(logic signed [WIDTH-1:0] re[$], logic signed [WIDTH-1:0] im[$], logic [TID_WIDTH-1:0] tid);
        int best = 0;
        foreach (re[n]) begin
            if (mag2(re[n], im[n]) > mag2(re[best], im[best])) best = n;
            src_q.push_back({n == re.size() - 1, tid, re[n], im[n]});
        end
        expected.push_back({tid, IDX_W'(best)});
    endfunction

    function automatic logic signed [WIDTH-1:0] rand_value(int bits);
        logic signed [WIDTH-1:0] v = WIDTH'({$urandom, $urandom});
        return v >>> (WIDTH - bits);
    endfunction

    // A pair of beats only the exact |x|^2 orders: B = (H*2^17 + L, 0) is just bigger than
    // A = (H*2^17, Q*2^17). Without the cross term 2*lo*hi*2^17, B would come out smaller.
    function automatic bit cross_pair(output logic signed [WIDTH-1:0] a_re, a_im, b_re, b_im);
        localparam int H_MAX = WIDTH > 20 ? (1 << (WIDTH - 19)) - 1 : 0;
        if (H_MAX < 16) return 1'b0;
        repeat (20) begin
            automatic longint     h = $urandom_range(H_MAX, 16);
            automatic longint     l = $urandom_range((1 << 17) - 1, 1 << 16);
            automatic real        u = real'(h) * real'(l) / 131072.0;  // h*l / 2^17
            automatic longint     q = longint'($ceil($sqrt(u + 1.0)));
            automatic logic [MAG_W-1:0] true_a, true_b, no_cross_b;
            a_re = WIDTH'(h << 17);
            a_im = WIDTH'(q << 17);
            b_re = WIDTH'((h << 17) + l);
            b_im = '0;
            true_a     = mag2(a_re, a_im);
            true_b     = mag2(b_re, b_im);
            no_cross_b = true_b - MAG_W'(h * l) * (MAG_W'(1) << 17);
            if (q < (1 << 17) && true_a < true_b && true_a > no_cross_b) return 1'b1;
        end
        return 1'b0;
    endfunction

    task automatic make_packet();
        logic signed [WIDTH-1:0] re[$], im[$];
        int len  = $urandom_range(MAX_LEN, 1);
        int kind = $urandom_range(5);
        re.delete();  // Otherwise the sim keeps them from the last call
        im.delete();
        for (int n = 0; n < len; n++) begin
            case (kind)
                0: begin re.push_back(rand_value(WIDTH)); im.push_back(rand_value(WIDTH)); end
                1: begin
                    re.push_back($urandom_range(1) ? MIN : MAX);
                    im.push_back($urandom_range(2) == 0 ? MIN : $urandom_range(1) ? MAX : 0);
                end
                2: begin
                    // Mostly one big value; a couple of beats get +1 in im^2 (im 0 -> 1)
                    re.push_back(MAX - WIDTH'($urandom_range(1)));
                    im.push_back(WIDTH'($urandom_range(1)));
                end
                3: begin
                    // (a, b), (-a, b), (b, a), (-b, -a): all the same magnitude
                    automatic logic signed [WIDTH-1:0] a = rand_value(WIDTH - 1), b = rand_value(WIDTH - 1);
                    case ($urandom_range(3))
                        0: begin re.push_back(a);  im.push_back(b);  end
                        1: begin re.push_back(-a); im.push_back(b);  end
                        2: begin re.push_back(b);  im.push_back(a);  end
                        default: begin re.push_back(-b); im.push_back(-a); end
                    endcase
                end
                4: begin re.push_back(rand_value(WIDTH < 17 ? WIDTH : 17)); im.push_back(rand_value(WIDTH < 17 ? WIDTH : 17)); end
                default: begin re.push_back(rand_value(WIDTH)); im.push_back(rand_value(WIDTH)); end
            endcase
        end
        // Ties: make the magnitude of every beat the same for a third of these packets
        if (kind == 3 && $urandom_range(2) == 0)
            for (int n = 1; n < len; n++) begin
                re[n] = n % 2 ? im[0] : re[0];
                im[n] = n % 2 ? -re[0] : im[0];
            end
        // Cross term: the rest of the packet small, the two close ones in either order
        if (kind == 5 && len >= 2) begin
            logic signed [WIDTH-1:0] a_re, a_im, b_re, b_im;
            if (cross_pair(a_re, a_im, b_re, b_im)) begin
                automatic int ia = $urandom_range(len - 1), ib;
                do ib = $urandom_range(len - 1); while (ib == ia);
                foreach (re[n]) begin
                    re[n] = rand_value(WIDTH / 2);
                    im[n] = rand_value(WIDTH / 2);
                end
                re[ia] = $urandom_range(1) ? a_re : -a_re;
                im[ia] = $urandom_range(1) ? a_im : -a_im;
                re[ib] = $urandom_range(1) ? b_re : -b_re;
                im[ib] = b_im;
                n_cross++;
            end
        end
        send_packet(re, im, TID_WIDTH'($urandom));
    endtask

    // ------------------------------------------------------------------
    // Sink
    // ------------------------------------------------------------------
    int unsigned                n_recv = 0;
    int unsigned                stall_left = 0;
    bit                         stalled = 1'b0;
    logic [TID_WIDTH+IDX_W-1:0] stalled_beat;

    always @(posedge clk) begin
        if (rstn) begin
            automatic logic [TID_WIDTH+IDX_W-1:0] beat = {dout.tid, dout.tdata};
            if (stalled) begin
                tb_checks++;
                if (dout.tvalid !== 1'b1 || beat !== stalled_beat)
                    tb_error_once({NAME, " stability"}, $sformatf("%s: output changed while stalled", NAME));
            end
            if (dout.tvalid && dout.tready) begin
                tb_checks++;
                if (expected.size() == 0) begin
                    tb_error_once({NAME, " extra"}, $sformatf("%s: output with nothing due", NAME));
                end else begin
                    automatic logic [TID_WIDTH+IDX_W-1:0] exp = expected.pop_front();
                    if (beat !== exp)
                        tb_error_once({NAME, " data"},
                            $sformatf("%s: packet %0d: tid %0d index %0d, expected tid %0d index %0d", NAME, n_recv,
                                      dout.tid, dout.tdata, exp[TID_WIDTH+IDX_W-1 -: TID_WIDTH], exp[IDX_W-1:0]));
                end
                n_recv++;
            end
            stalled      = dout.tvalid && !dout.tready;
            stalled_beat = beat;
        end
        // Mostly ~50% ready, with stalls long enough for the next result to have to wait
        if (stall_left > 0) begin
            stall_left--;
            dout.tready <= 1'b0;
        end else if ($urandom_range(99) == 0) begin
            stall_left = $urandom_range(200, 20);
            dout.tready <= 1'b0;
        end else begin
            dout.tready <= $urandom_range(1);
        end
    end

    initial begin
        int unsigned timeout = 0;
        done = 1'b0;
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        @(posedge clk);
        `CHECK_EQ(dout.tvalid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        repeat (PACKETS) make_packet();
        while (n_recv < PACKETS && timeout < 40 * PACKETS * MAX_LEN) begin
            @(posedge clk);
            timeout++;
        end
        repeat (20) @(posedge clk);
        `CHECK(n_recv == PACKETS, $sformatf("%s: %0d of %0d packets came out", NAME, n_recv, PACKETS))
        `CHECK(expected.size() == 0, $sformatf("%s: %0d results never came out", NAME, expected.size()))
        tb_info($sformatf("%s: %0d packets, %0d with a cross-term-only pair", NAME, n_recv, n_cross));
        done = 1'b1;
    end

endmodule


module tb_peak_finder;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(20_000_000) // 20 ms

    logic clk = 1'b0;
    always #2.5 clk = ~clk; // 200 MHz, clk_core

    logic [2:0] done;

    // cross_correlator's default output: 32-bit re/im, 21 lags, 28 pairs
    peak_finder_inst #(.WIDTH(32), .MAX_LEN(21), .TID_WIDTH(5)) g_corr (.clk(clk), .done(done[0]));
    // The widest supported, so the high piece is as big as it gets
    peak_finder_inst #(.WIDTH(34), .MAX_LEN(32), .TID_WIDTH(1)) g_wide (.clk(clk), .done(done[1]));
    // Narrow enough that the high piece is unused
    peak_finder_inst #(.WIDTH(12), .MAX_LEN(5), .TID_WIDTH(2))  g_narrow (.clk(clk), .done(done[2]));

    initial begin
        tb_info("Testing peak_finder: 32 bit x 21, 34 bit x 32, 12 bit x 5");
        wait (&done);
        tb_finish();
    end

endmodule
