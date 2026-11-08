`timescale 1ns/1ps
`include "tb_macros.svh"

// axis_register_slice test: every wrapper (axis_if, axis_tid_if, axis_tlast_if,
// axis_tid_tlast_if), both stage kinds, and 0 to 4 stages.
//
// Each instance checks:
//   - every beat comes out once, in order, with its tid and tlast
//   - the output holds while stalled
//   - with the input always valid and the output always ready, a beat per clock, each
//     exactly STAGES clocks after it went in
//   - REGISTER_READY = 1: in_ready only changes on a clock edge, never with out_ready
// First with random input gaps and the output ready ~50% of the time with stalls of up to
// 50 clocks, then streaming.
module axis_register_slice_inst #(
    parameter int KIND           = 0,  // 0 axis_if, 1 axis_tid_if, 2 axis_tlast_if, 3 axis_tid_tlast_if
    parameter int WIDTH          = 8,
    parameter int ID_WIDTH       = 3,
    parameter int STAGES         = 1,
    parameter int REGISTER_READY = 1,
    parameter int BEATS          = 3000,
    parameter int STREAM_BEATS   = 500
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int  BEAT_W   = 1 + ID_WIDTH + WIDTH;  // {tlast, tid, tdata}
    localparam bit  HAS_ID   = KIND == 1 || KIND == 3;
    localparam bit  HAS_LAST = KIND == 2 || KIND == 3;
    localparam string KIND_S = KIND == 0 ? "axis_if" : KIND == 1 ? "axis_tid_if"
                             : KIND == 2 ? "axis_tlast_if" : "axis_tid_tlast_if";

    string NAME = $sformatf("%s %0d bit, %0d stages, %s", KIND_S, WIDTH, STAGES,
                            REGISTER_READY == 2 ? "2-entry FIFO" : REGISTER_READY ? "registered ready" : "forward only");

    logic rstn = 1'b0;

    logic [WIDTH-1:0]    in_data,  out_data;
    logic [ID_WIDTH-1:0] in_id,    out_id;
    logic                in_last,  out_last;
    logic                in_valid, in_ready, out_valid;
    logic                out_ready = 1'b0;

    // ------------------------------------------------------------------
    // DUT, through the wrapper for this KIND
    // ------------------------------------------------------------------
    if (KIND == 0) begin : g_dut
        axis_if #(.WIDTH(WIDTH)) a ();
        axis_if #(.WIDTH(WIDTH)) b ();
        assign a.tdata  = in_data;
        assign a.tvalid = in_valid;
        assign in_ready = a.tready;
        assign out_data  = b.tdata;
        assign out_valid = b.tvalid;
        assign b.tready  = out_ready;
        assign out_id    = '0;
        assign out_last  = 1'b0;
        axis_register_slice #(.WIDTH(WIDTH), .STAGES(STAGES), .REGISTER_READY(REGISTER_READY))
            dut (.clk(clk), .rstn(rstn), .axis_in(a), .axis_out(b));
    end else if (KIND == 1) begin : g_dut
        axis_tid_if #(.WIDTH(WIDTH), .ID_WIDTH(ID_WIDTH)) a ();
        axis_tid_if #(.WIDTH(WIDTH), .ID_WIDTH(ID_WIDTH)) b ();
        assign a.tdata  = in_data;
        assign a.tid    = in_id;
        assign a.tvalid = in_valid;
        assign in_ready = a.tready;
        assign out_data  = b.tdata;
        assign out_id    = b.tid;
        assign out_valid = b.tvalid;
        assign b.tready  = out_ready;
        assign out_last  = 1'b0;
        axis_tid_register_slice #(.WIDTH(WIDTH), .ID_WIDTH(ID_WIDTH), .STAGES(STAGES), .REGISTER_READY(REGISTER_READY))
            dut (.clk(clk), .rstn(rstn), .axis_in(a), .axis_out(b));
    end else if (KIND == 2) begin : g_dut
        axis_tlast_if #(.WIDTH(WIDTH)) a ();
        axis_tlast_if #(.WIDTH(WIDTH)) b ();
        assign a.tdata  = in_data;
        assign a.tlast  = in_last;
        assign a.tvalid = in_valid;
        assign in_ready = a.tready;
        assign out_data  = b.tdata;
        assign out_last  = b.tlast;
        assign out_valid = b.tvalid;
        assign b.tready  = out_ready;
        assign out_id    = '0;
        axis_tlast_register_slice #(.WIDTH(WIDTH), .STAGES(STAGES), .REGISTER_READY(REGISTER_READY))
            dut (.clk(clk), .rstn(rstn), .axis_in(a), .axis_out(b));
    end else begin : g_dut
        axis_tid_tlast_if #(.WIDTH(WIDTH), .ID_WIDTH(ID_WIDTH)) a ();
        axis_tid_tlast_if #(.WIDTH(WIDTH), .ID_WIDTH(ID_WIDTH)) b ();
        assign a.tdata  = in_data;
        assign a.tid    = in_id;
        assign a.tlast  = in_last;
        assign a.tvalid = in_valid;
        assign in_ready = a.tready;
        assign out_data  = b.tdata;
        assign out_id    = b.tid;
        assign out_last  = b.tlast;
        assign out_valid = b.tvalid;
        assign b.tready  = out_ready;
        axis_tid_tlast_register_slice #(.WIDTH(WIDTH), .ID_WIDTH(ID_WIDTH), .STAGES(STAGES), .REGISTER_READY(REGISTER_READY))
            dut (.clk(clk), .rstn(rstn), .axis_in(a), .axis_out(b));
    end

    // ------------------------------------------------------------------
    // Source: beats from a queue. The fields a KIND doesn't carry are sent as 0.
    // ------------------------------------------------------------------
    logic [BEAT_W-1:0] src_q[$];
    logic [BEAT_W-1:0] expected[$];  // Filled by the monitor below
    longint unsigned   sent_at[$];   // Clock each expected beat was accepted on
    longint unsigned   cycle = 0;
    int unsigned       gap_pct = 30;

    always @(posedge clk) begin
        bit hs;
        if (!rstn) begin
            in_valid <= 1'b0;
            {in_last, in_id, in_data} <= '0;
        end else begin
            hs = in_valid && in_ready;
            if (hs) void'(src_q.pop_front());
            if (!in_valid || hs) begin
                if (src_q.size() > 0 && $urandom_range(99) >= gap_pct) begin
                    in_valid <= 1'b1;
                    {in_last, in_id, in_data} <= src_q[0];
                end else begin
                    in_valid <= 1'b0;
                    {in_last, in_id, in_data} <= BEAT_W'({$urandom, $urandom});  // Junk, ignored
                end
            end
        end
    end

    function automatic void make_beats(int n);
        repeat (n) begin
            logic [WIDTH-1:0]    d = WIDTH'({$urandom, $urandom});
            logic [ID_WIDTH-1:0] i = HAS_ID ? ID_WIDTH'($urandom) : '0;
            logic                l = HAS_LAST ? 1'($urandom) : 1'b0;
            src_q.push_back({l, i, d});
        end
    endfunction

    // ------------------------------------------------------------------
    // Monitor and sink: checks at the clock, drives ready between clocks. The input side is
    // logged in the same block, before the output is checked, so with 0 stages the beat is
    // due before it's checked.
    // ------------------------------------------------------------------
    int unsigned       n_recv = 0;
    bit                streaming = 1'b0;
    bit                stalled = 1'b0;
    logic [BEAT_W-1:0] stalled_beat;
    int unsigned       stall_left = 0;

    always @(posedge clk) begin
        automatic logic [BEAT_W-1:0] beat = {out_last, out_id, out_data};
        cycle++;
        if (rstn && in_valid && in_ready) begin
            expected.push_back({in_last, in_id, in_data});
            sent_at.push_back(cycle);
        end
        if (!rstn) begin
            stalled = 1'b0;
            if (out_valid !== 1'b0)
                tb_error_once({NAME, " reset"}, $sformatf("%s: tvalid high in reset", NAME));
        end else begin
            if (stalled) begin
                tb_checks++;
                if (out_valid !== 1'b1 || beat !== stalled_beat)
                    tb_error_once({NAME, " stability"}, $sformatf("%s: output changed while stalled", NAME));
            end
            if (out_valid && out_ready) begin
                tb_checks++;
                if (expected.size() == 0) begin
                    tb_error_once({NAME, " extra"}, $sformatf("%s: output with nothing due", NAME));
                end else begin
                    automatic logic [BEAT_W-1:0] exp = expected.pop_front();
                    automatic longint unsigned   at  = sent_at.pop_front();
                    if (beat !== exp)
                        tb_error_once({NAME, " data"},
                            $sformatf("%s: beat %0d: tlast %0d tid %0d tdata 0x%0h, expected tlast %0d tid %0d tdata 0x%0h",
                                      NAME, n_recv, out_last, out_id, out_data,
                                      exp[BEAT_W-1], exp[WIDTH +: ID_WIDTH], exp[WIDTH-1:0]));
                    if (streaming) begin
                        tb_checks++;
                        if (cycle - at != STAGES)
                            tb_error_once({NAME, " latency"},
                                $sformatf("%s: beat %0d out %0d clocks after it went in, expected %0d",
                                          NAME, n_recv, cycle - at, STAGES));
                    end
                end
                n_recv++;
            end
            stalled      = out_valid && !out_ready;
            stalled_beat = beat;
        end
    end

    // A registered tready can't change between clock edges, whatever out_ready does
    always @(negedge clk) begin
        automatic logic ready_before = in_ready;
        if (streaming) begin
            out_ready = 1'b1;
        end else if (stall_left > 0) begin
            stall_left--;
            out_ready = 1'b0;
        end else if ($urandom_range(99) == 0) begin
            stall_left = $urandom_range(50, 2);
            out_ready = 1'b0;
        end else begin
            out_ready = 1'($urandom_range(1));
        end
        if (REGISTER_READY && STAGES > 0 && rstn) begin
            #0.1;
            tb_checks++;
            if (in_ready !== ready_before)
                tb_error_once({NAME, " ready path"},
                    $sformatf("%s: in_ready followed out_ready between clocks", NAME));
        end
    end

    task automatic drain(int n, string what);
        int unsigned timeout = 0;
        while ((src_q.size() > 0 || expected.size() > 0) && timeout < 20 * n + 1000) begin
            @(posedge clk);
            timeout++;
        end
        `CHECK(src_q.size() == 0 && expected.size() == 0,
               $sformatf("%s, %s: %0d beats still in the source, %0d never came out", NAME, what,
                         src_q.size(), expected.size()))
    endtask

    initial begin
        longint unsigned start;
        int unsigned     first;
        done = 1'b0;
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        @(posedge clk);
        `CHECK_EQ(out_valid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        // Random gaps and stalls
        make_beats(BEATS);
        drain(BEATS, "random");

        // Streaming: input always valid, output always ready
        @(negedge clk);
        streaming = 1'b1;
        gap_pct   = 0;
        @(posedge clk);
        first = n_recv;
        start = cycle;
        make_beats(STREAM_BEATS);
        drain(STREAM_BEATS, "streaming");
        // The source starts a clock after the beats are queued, then STAGES clocks of latency
        `CHECK(n_recv - first == STREAM_BEATS,
               $sformatf("%s: %0d of %0d streamed beats came out", NAME, n_recv - first, STREAM_BEATS))
        `CHECK(cycle - start <= STREAM_BEATS + STAGES + 2,
               $sformatf("%s: %0d beats took %0d clocks, a beat per clock is %0d", NAME, STREAM_BEATS,
                         cycle - start, STREAM_BEATS + STAGES + 2))

        tb_info($sformatf("%s: %0d beats", NAME, n_recv));
        done = 1'b1;
    end

endmodule


module tb_axis_register_slice;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(20_000_000) // 20 ms

    logic clk = 1'b0;
    always #2.5 clk = ~clk; // 200 MHz, clk_core

    localparam int N = 12;
    logic [N-1:0] done;

    axis_register_slice_inst #(.KIND(0), .WIDTH(8),  .STAGES(0), .REGISTER_READY(1)) g_pass      (.clk(clk), .done(done[0]));
    axis_register_slice_inst #(.KIND(0), .WIDTH(24), .STAGES(1), .REGISTER_READY(0)) g_fwd1      (.clk(clk), .done(done[1]));
    axis_register_slice_inst #(.KIND(0), .WIDTH(24), .STAGES(3), .REGISTER_READY(1)) g_skid3     (.clk(clk), .done(done[2]));
    // The I2S path into the DSP: 24 bit, 8 channels
    axis_register_slice_inst #(.KIND(1), .WIDTH(24), .ID_WIDTH(3), .STAGES(2), .REGISTER_READY(1)) g_tid_skid (.clk(clk), .done(done[3]));
    axis_register_slice_inst #(.KIND(1), .WIDTH(24), .ID_WIDTH(3), .STAGES(2), .REGISTER_READY(0)) g_tid_fwd  (.clk(clk), .done(done[4]));
    axis_register_slice_inst #(.KIND(2), .WIDTH(16), .STAGES(1), .REGISTER_READY(1)) g_tlast     (.clk(clk), .done(done[5]));
    axis_register_slice_inst #(.KIND(2), .WIDTH(16), .STAGES(4), .REGISTER_READY(0)) g_tlast_fwd (.clk(clk), .done(done[6]));
    axis_register_slice_inst #(.KIND(3), .WIDTH(32), .ID_WIDTH(5), .STAGES(4), .REGISTER_READY(1)) g_both_skid (.clk(clk), .done(done[7]));
    axis_register_slice_inst #(.KIND(3), .WIDTH(32), .ID_WIDTH(5), .STAGES(2), .REGISTER_READY(0)) g_both_fwd  (.clk(clk), .done(done[8]));
    // 2-entry FIFO stages
    axis_register_slice_inst #(.KIND(0), .WIDTH(24), .STAGES(1), .REGISTER_READY(2)) g_fifo1     (.clk(clk), .done(done[9]));
    axis_register_slice_inst #(.KIND(1), .WIDTH(36), .ID_WIDTH(3), .STAGES(1), .REGISTER_READY(2)) g_tid_fifo (.clk(clk), .done(done[10]));
    axis_register_slice_inst #(.KIND(3), .WIDTH(16), .ID_WIDTH(2), .STAGES(3), .REGISTER_READY(2)) g_both_fifo (.clk(clk), .done(done[11]));

    initial begin
        tb_info("Testing axis_register_slice: every interface type, both stage kinds, 0 to 4 stages");
        wait (&done);
        tb_finish();
    end

endmodule
