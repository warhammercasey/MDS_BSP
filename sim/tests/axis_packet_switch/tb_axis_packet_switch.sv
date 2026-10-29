`timescale 1ns/1ps
`include "tb_macros.svh"

// axis_packet_switch test: N packet streams merged onto one output, for several N.
//
// Each input tags its beats with its index in the top bits of tdata. Checked
// throughout:
//   - every packet comes out whole and in one piece: all its beats from one input,
//     with out_sel naming that input, and no other input's beats in between
//   - each input's packets come out in the order sent, intact, tlast in the right place
//   - no gaps inside a packet: once a packet starts, tvalid stays high until its tlast
//     (what lets rmii_tx run off the switch without starving)
//   - the output holds tvalid/tdata/tlast/out_sel while stalled
// Packets are 1 to 2^AWIDTH beats (a full buffer), inputs and output both stall.
// Phases:
//   spec          gaps between input beats, output always ready
//   backpressure  same inputs, output ready ~30%
//   saturated     inputs back to back, output always ready; every input must get
//                 through its packets (no starving)
module axis_packet_switch_inst #(
    parameter int N         = 2,
    parameter int WIDTH     = 8,
    parameter int AWIDTH    = 6,
    parameter int N_PACKETS = 30  // per input, per phase
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int SRC_W   = N > 1 ? $clog2(N) : 1;  // tdata[WIDTH-1 -: SRC_W] = input index
    localparam int MAX_LEN = 1 << AWIDTH;

    logic rstn = 1'b0;

    axis_tlast_if #(.WIDTH(WIDTH)) axis_in [N] ();
    axis_tlast_if #(.WIDTH(WIDTH)) axis_out ();
    logic [SRC_W-1:0] out_sel;

    axis_packet_switch #(
        .WIDTH(WIDTH),
        .N(N),
        .AWIDTH(AWIDTH)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .axis_in(axis_in),
        .axis_out(axis_out),
        .out_sel(out_sel)
    );

    string NAME = $sformatf("N=%0d", N);

    // ------------------------------------------------------------------
    // Sources: beats {tlast, tdata} from a queue per input
    // ------------------------------------------------------------------
    // Sized so any SRC_W-bit index is in range (Verilator 5.020 generates C++ that
    // doesn't compile for a possibly out-of-range index into an array of queues)
    logic [WIDTH:0]   src_q    [1 << SRC_W][$];
    logic [WIDTH-1:0] expected [1 << SRC_W][$][$];  // Packets, as sent
    int unsigned      gap_pct = 0;

    for (genvar i = 0; i < N; i++) begin : g_src
        always @(posedge clk) begin
            bit hs;
            if (!rstn) begin
                axis_in[i].tvalid <= 1'b0;
                axis_in[i].tdata  <= '0;
                axis_in[i].tlast  <= 1'b0;
            end else begin
                hs = axis_in[i].tvalid && axis_in[i].tready;
                if (hs) void'(src_q[i].pop_front());
                if (!axis_in[i].tvalid || hs) begin
                    if (src_q[i].size() > 0 && $urandom_range(99) >= gap_pct) begin
                        axis_in[i].tvalid <= 1'b1;
                        {axis_in[i].tlast, axis_in[i].tdata} <= src_q[i][0];
                    end else begin
                        axis_in[i].tvalid <= 1'b0;
                    end
                end
            end
        end
    end

    task automatic queue_packet(int i, int unsigned len);
        logic [WIDTH-1:0] p[$];
        p = {};
        for (int b = 0; b < len; b++) begin
            logic [WIDTH-1:0] d = WIDTH'($urandom);
            d[WIDTH-1 -: SRC_W] = SRC_W'(i);
            p.push_back(d);
            src_q[i].push_back({b == len - 1, d});
        end
        expected[i].push_back(p);
    endtask

    // ------------------------------------------------------------------
    // Sink: packets, gaps, stability
    // ------------------------------------------------------------------
    int unsigned      sink_rate = 100;
    int unsigned      n_recv = 0;
    int unsigned      n_recv_from [N] = '{default: 0};
    logic [WIDTH-1:0] cur[$];
    logic [SRC_W-1:0] cur_sel;
    bit               in_packet = 1'b0;   // A packet has started and not finished
    bit               stalled = 1'b0;
    logic [WIDTH+SRC_W:0] stalled_beat;

    function automatic void packet_done();
        logic [WIDTH-1:0] exp[$];
        tb_checks++;
        if (expected[cur_sel].size() == 0) begin
            tb_error_once({NAME, " extra"}, $sformatf("%s: packet from input %0d that was never sent", NAME, cur_sel));
            return;
        end
        exp = expected[cur_sel].pop_front();
        if (exp.size() != cur.size()) begin
            tb_error_once({NAME, " length"},
                $sformatf("%s: packet from input %0d is %0d beats, expected %0d", NAME, cur_sel, cur.size(), exp.size()));
            return;
        end
        foreach (exp[b]) begin
            if (cur[b] !== exp[b]) begin
                tb_error_once({NAME, " data"},
                    $sformatf("%s: packet from input %0d, beat %0d is 0x%0h, expected 0x%0h", NAME, cur_sel, b, cur[b], exp[b]));
                return;
            end
        end
    endfunction

    always @(posedge clk) begin
        if (rstn) begin
            if (stalled) begin
                tb_checks++;
                if (axis_out.tvalid !== 1'b1 || {out_sel, axis_out.tlast, axis_out.tdata} !== stalled_beat)
                    tb_error_once({NAME, " stability"}, $sformatf("%s: output changed while stalled", NAME));
            end
            if (in_packet) begin
                tb_checks++;
                if (axis_out.tvalid !== 1'b1)
                    tb_error_once({NAME, " gap"}, $sformatf("%s: tvalid dropped in the middle of a packet", NAME));
            end

            if (axis_out.tvalid && axis_out.tready) begin
                if (!in_packet) begin
                    cur     = {};
                    cur_sel = out_sel;
                end
                tb_checks++;
                if (out_sel !== cur_sel || axis_out.tdata[WIDTH-1 -: SRC_W] !== cur_sel)
                    tb_error_once({NAME, " mixed"},
                        $sformatf("%s: beat 0x%0h (out_sel %0d) inside a packet from input %0d",
                                  NAME, axis_out.tdata, out_sel, cur_sel));
                cur.push_back(axis_out.tdata);
                in_packet = !axis_out.tlast;
                if (axis_out.tlast) begin
                    packet_done();
                    n_recv++;
                    if (int'(cur_sel) < N) n_recv_from[cur_sel]++;
                end
            end

            stalled      = axis_out.tvalid && !axis_out.tready;
            stalled_beat = {out_sel, axis_out.tlast, axis_out.tdata};
        end
        axis_out.tready <= $urandom_range(99) < sink_rate;
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic phase(string name, int unsigned gaps, int unsigned sink);
        int unsigned target  = n_recv + N_PACKETS * N;
        int unsigned timeout = 0;
        int unsigned from0 [N];
        gap_pct   = gaps;
        sink_rate = sink;
        from0     = n_recv_from;
        for (int i = 0; i < N; i++) begin
            queue_packet(i, 1);
            queue_packet(i, MAX_LEN);
            for (int k = 2; k < N_PACKETS; k++)
                queue_packet(i, $urandom_range(MAX_LEN, 1));
        end
        while (n_recv < target && timeout < 200 * N_PACKETS * N * MAX_LEN) begin
            @(posedge clk);
            timeout++;
        end
        `CHECK(n_recv == target, $sformatf("%s, %s phase: %0d of %0d packets came out", NAME, name, n_recv, target))
        for (int i = 0; i < N; i++)
            `CHECK(n_recv_from[i] - from0[i] == N_PACKETS,
                   $sformatf("%s, %s phase: input %0d got %0d of %0d packets through", NAME, name, i,
                             n_recv_from[i] - from0[i], N_PACKETS))
    endtask

    initial begin
        done = 1'b0;
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        @(posedge clk);
        `CHECK_EQ(axis_out.tvalid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        phase("spec",         40, 100);
        phase("backpressure", 40, 30);
        phase("saturated",    0,  100);

        repeat (5) @(posedge clk);
        for (int i = 0; i < N; i++)
            `CHECK(expected[i].size() == 0, $sformatf("%s: input %0d has %0d packets that never came out", NAME, i, expected[i].size()))
        `CHECK_EQ(axis_out.tvalid, 1'b0, $sformatf("%s: tvalid low when idle", NAME))
        tb_info($sformatf("%s: %0d packets", NAME, n_recv));
        done = 1'b1;
    end

endmodule


module tb_axis_packet_switch;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(20_000_000) // 20 ms

    localparam real CLK_PERIOD_NS = 20.0; // clk_50

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    localparam int NUM_CONFIGS = 3;
    localparam int N_LIST [NUM_CONFIGS] = '{1, 2, 3};

    logic [NUM_CONFIGS-1:0] done;
    for (genvar c = 0; c < NUM_CONFIGS; c++) begin : g_n
        axis_packet_switch_inst #(.N(N_LIST[c])) h (.clk(clk), .done(done[c]));
    end

    initial begin
        tb_info("Testing axis_packet_switch with N = 1, 2, 3");
        wait (&done);
        tb_finish();
    end

endmodule
