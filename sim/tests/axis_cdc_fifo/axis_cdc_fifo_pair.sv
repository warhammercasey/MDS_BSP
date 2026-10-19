`timescale 1ns/1ps
`include "tb_macros.svh"

// One CDC FIFO between clka and clkb with an AXIS source and sink, shared by the
// axis_cdc_fifo and axis_tid_cdc_fifo tests (USE_TID picks the DUT). The harness
// works on the payload {tid, tdata}, so tid is checked like data.
//
// Phases, each moving N beats with random data:
//   random      source and sink each active ~60% of cycles
//   capacity    sink stalled: the FIFO must accept DEPTH-1 or DEPTH beats, then
//               hold tvalid and its head beat until the sink drains it
//   throughput  source and sink always on: the rate must reach ~min(f_clka, f_clkb)
//   backpressure source always on, sink ~30%
//   reset       both resets asserted mid-stream, released in random order: the FIFO
//               must come back empty and then pass data normally
// Always checked: every beat arrives once, in order, intact; the output holds
// tvalid/payload while stalled; and (ff_sync_checker) synchronized buses change
// at most one bit per clock.
module axis_cdc_fifo_pair #(
    parameter real   FREQ_A   = 100.0e6,
    parameter real   FREQ_B   = 100.0e6,
    parameter string NAME     = "",
    parameter bit    USE_TID  = 1'b0,
    parameter int    WIDTH    = 16,
    parameter int    ID_WIDTH = 3,
    parameter int    AWIDTH   = 4
) (
    input  logic clka,
    input  logic clkb,
    output logic done
);
    import tb_pkg::*;

    localparam int  PW    = USE_TID ? WIDTH + ID_WIDTH : WIDTH;
    localparam int  DEPTH = 1 << AWIDTH;
    localparam real TA    = 1.0e9 / FREQ_A;
    localparam real TB    = 1.0e9 / FREQ_B;
    localparam real T_MAX = TA > TB ? TA : TB;

    logic          rstna = 1'b0;
    logic          rstnb = 1'b0;
    logic          in_valid = 1'b0;
    logic          in_ready;
    logic [PW-1:0] in_payload = '0;
    logic          out_valid;
    logic          out_ready = 1'b0;
    logic [PW-1:0] out_payload;

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    if (USE_TID) begin : g_dut
        axis_tid_if #(.WIDTH(WIDTH), .ID_WIDTH(ID_WIDTH)) axis_in ();
        axis_tid_if #(.WIDTH(WIDTH), .ID_WIDTH(ID_WIDTH)) axis_out ();

        assign axis_in.tvalid             = in_valid;
        assign {axis_in.tid, axis_in.tdata} = in_payload;
        assign in_ready                   = axis_in.tready;
        assign out_valid                  = axis_out.tvalid;
        assign out_payload                = {axis_out.tid, axis_out.tdata};
        assign axis_out.tready            = out_ready;

        axis_tid_cdc_fifo #(
            .WIDTH(WIDTH),
            .ID_WIDTH(ID_WIDTH),
            .AWIDTH(AWIDTH)
        ) dut (
            .clka(clka),
            .rstna(rstna),
            .clkb(clkb),
            .rstnb(rstnb),
            .axis_in(axis_in),
            .axis_out(axis_out)
        );
    end else begin : g_dut
        axis_if #(.WIDTH(WIDTH)) axis_in ();
        axis_if #(.WIDTH(WIDTH)) axis_out ();

        assign axis_in.tvalid  = in_valid;
        assign axis_in.tdata   = in_payload;
        assign in_ready        = axis_in.tready;
        assign out_valid       = axis_out.tvalid;
        assign out_payload     = axis_out.tdata;
        assign axis_out.tready = out_ready;

        axis_cdc_fifo #(
            .WIDTH(WIDTH),
            .AWIDTH(AWIDTH)
        ) dut (
            .clka(clka),
            .rstna(rstna),
            .clkb(clkb),
            .rstnb(rstnb),
            .axis_in(axis_in),
            .axis_out(axis_out)
        );
    end

    // ------------------------------------------------------------------
    // Source (clka)
    // ------------------------------------------------------------------
    int unsigned   src_rate = 0;        // % of cycles a new beat is offered
    int unsigned   n_target = 0;        // total beats to send
    int unsigned   n_issued = 0, n_sent = 0;
    logic [PW-1:0] expected[$];         // sent, not yet received

    always @(posedge clka) begin
        if (!rstna) begin
            in_valid <= 1'b0;
        end else begin
            if (in_valid && in_ready) begin
                expected.push_back(in_payload);
                n_sent++;
            end
            if (!in_valid || in_ready) begin
                if (n_issued < n_target && $urandom_range(99) < src_rate) begin
                    in_valid   <= 1'b1;
                    in_payload <= PW'({$urandom, $urandom});
                    n_issued++;
                end else begin
                    in_valid <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Sink (clkb): order, integrity, stability
    // ------------------------------------------------------------------
    int unsigned   sink_rate = 0;       // % of cycles tready is high
    int unsigned   n_recv = 0;
    bit            in_reset = 1'b1;     // ignore the output while the resets are in progress
    bit            stalled = 1'b0;
    logic [PW-1:0] stalled_payload;
    realtime       t_first, t_last;     // first/last beat received in the current phase
    int unsigned   n_phase = 0;

    always @(posedge clkb) begin
        if (rstnb && !in_reset) begin
            if (stalled) begin
                tb_checks++;
                if (out_valid !== 1'b1 || out_payload !== stalled_payload)
                    tb_error_once({NAME, " stability"},
                        $sformatf("%s: output changed while stalled: tvalid=%b payload 0x%0h -> 0x%0h",
                                  NAME, out_valid, stalled_payload, out_payload));
            end

            if (out_valid && out_ready) begin
                tb_checks++;
                if (expected.size() == 0) begin
                    tb_error_once({NAME, " extra"},
                        $sformatf("%s: beat %0d (0x%0h) received but nothing was sent", NAME, n_recv, out_payload));
                end else begin
                    automatic logic [PW-1:0] exp = expected.pop_front();
                    if (out_payload !== exp)
                        tb_error_once({NAME, " data"},
                            $sformatf("%s: beat %0d is 0x%0h, expected 0x%0h", NAME, n_recv, out_payload, exp));
                end
                if (n_phase == 0)
                    t_first = $realtime;
                t_last = $realtime;
                n_phase++;
                n_recv++;
            end

            stalled         = out_valid && !out_ready;
            stalled_payload = out_payload;
        end else begin
            stalled = 1'b0;
        end
        out_ready <= $urandom_range(99) < sink_rate;
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic wait_cycles(int unsigned n);
        repeat (n) @(posedge clka);
        repeat (n) @(posedge clkb);
    endtask

    // Send n more beats and wait until all of them arrived
    task automatic transfer(string phase, int unsigned n, int unsigned src, int unsigned sink);
        realtime deadline = $realtime + 50.0 * n * T_MAX + 2000.0;
        n_target += n;
        n_phase   = 0;
        src_rate  = src;
        sink_rate = sink;
        while (n_recv < n_target && $realtime < deadline)
            @(posedge clkb);
        `CHECK(n_recv == n_target,
               $sformatf("%s, %s phase: received %0d of %0d beats (sent %0d) before timing out",
                         NAME, phase, n_recv, n_target, n_sent))
    endtask

    task automatic reset_both();
        in_reset = 1'b1;
        fork
            @(negedge clka) rstna = 1'b0;
            @(negedge clkb) rstnb = 1'b0;
        join
        wait_cycles(8);
        expected.delete();
        {n_target, n_issued, n_sent, n_recv} = '0;

        // reset_sync releases each domain independently, so either side may come out first
        if ($urandom_range(1) == 1) begin
            @(negedge clka) rstna = 1'b1;
            repeat ($urandom_range(4)) @(posedge clkb);
            @(negedge clkb) rstnb = 1'b1;
        end else begin
            @(negedge clkb) rstnb = 1'b1;
            repeat ($urandom_range(4)) @(posedge clka);
            @(negedge clka) rstna = 1'b1;
        end
        wait_cycles(6);
        `CHECK_EQ(out_valid, 1'b0, $sformatf("%s: tvalid (empty) after reset", NAME))
        `CHECK_EQ(in_ready,  1'b1, $sformatf("%s: tready (not full) after reset", NAME))
        in_reset = 1'b0;
    endtask

    initial begin
        int unsigned fill;
        real         rate, rate_min;

        done = 1'b0;
        reset_both();

        transfer("random", 300, 60, 60);

        // Capacity: stall the sink and let the source fill the FIFO
        n_target += DEPTH + 4;
        src_rate  = 100;
        sink_rate = 0;
        wait_cycles(8 * DEPTH);
        fill = n_sent - n_recv;
        `CHECK(fill == DEPTH - 1 || fill == DEPTH,
               $sformatf("%s: FIFO accepted %0d beats with the sink stalled (2^AWIDTH = %0d)", NAME, fill, DEPTH))
        `CHECK_EQ(in_ready,  1'b0, $sformatf("%s: tready low when full", NAME))
        `CHECK_EQ(out_valid, 1'b1, $sformatf("%s: tvalid high when full", NAME))
        transfer("capacity", 0, 100, 100);

        transfer("throughput", 400, 100, 100);
        rate     = 1.0e9 * (n_phase - 1) / (t_last - t_first);
        rate_min = FREQ_A < FREQ_B ? FREQ_A : FREQ_B;
        `CHECK(rate >= 0.9 * rate_min,
               $sformatf("%s: throughput %0.2f Mbeat/s is under 90%% of the slower clock (%0.2f MHz)",
                         NAME, rate / 1e6, rate_min / 1e6))

        transfer("backpressure", 200, 100, 30);

        // Reset in the middle of a transfer
        n_target += 1000;
        src_rate  = 70;
        sink_rate = 70;
        wait (n_recv >= n_target - 1000 + 50);
        reset_both();
        transfer("after reset", 300, 60, 60);

        wait_cycles(4 * DEPTH);
        `CHECK(out_valid === 1'b0 && expected.size() == 0,
               $sformatf("%s: drained at the end (tvalid=%b, %0d beats outstanding)", NAME, out_valid, expected.size()))

        tb_info($sformatf("%-20s capacity %0d, throughput %0.2f Mbeat/s (slower clock %0.2f MHz)",
                          NAME, fill, rate / 1e6, rate_min / 1e6));
        done = 1'b1;
    end

endmodule
