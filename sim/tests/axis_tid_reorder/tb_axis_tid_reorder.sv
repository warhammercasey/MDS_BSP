`timescale 1ns/1ps
`include "tb_macros.svh"

// axis_tid_reorder test: samples sent a cycle of channels at a time, each cycle in a
// shuffled order, must come out in channel order, for several channel counts.
//
// Checked throughout:
//   - output tids go 0, 1, ..., CHANNELS-1, 0, ... and each channel's samples come out
//     in the order they were sent, intact
//   - the output holds tvalid/tdata/tid while stalled
//   - tids of CHANNELS and up (sent now and then when CHANNELS isn't a power of 2)
//     never come out
// Phases:
//   i2s           the i2s order (even channels, then odd), output always ready
//   i2s stalls    same, with gaps between input beats and the output ready ~40%
//   shuffled      a random order every cycle, back to back, output always ready
//   shuffled stalls
module axis_tid_reorder_inst #(
    parameter int WIDTH    = 18,
    parameter int ID_WIDTH = 3,
    parameter int CHANNELS = 8,
    parameter int N_CYCLES = 40  // channel cycles per phase
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int NUM_IDS = 1 << ID_WIDTH;

    logic rstn = 1'b0;

    axis_tid_if #(.WIDTH(WIDTH), .ID_WIDTH(ID_WIDTH)) axis_in ();
    axis_tid_if #(.WIDTH(WIDTH), .ID_WIDTH(ID_WIDTH)) axis_out ();

    axis_tid_reorder #(
        .WIDTH(WIDTH),
        .ID_WIDTH(ID_WIDTH),
        .CHANNELS(CHANNELS)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .axis_in(axis_in),
        .axis_out(axis_out)
    );

    string NAME = $sformatf("%0d channels", CHANNELS);

    // ------------------------------------------------------------------
    // Source: beats from a queue, {tid, data}
    // ------------------------------------------------------------------
    logic [ID_WIDTH+WIDTH-1:0] src_q[$];
    int unsigned               gap_pct = 0;

    always @(posedge clk) begin
        bit hs;
        if (!rstn) begin
            axis_in.tvalid <= 1'b0;
            axis_in.tdata  <= '0;
            axis_in.tid    <= '0;
        end else begin
            hs = axis_in.tvalid && axis_in.tready;
            if (hs) void'(src_q.pop_front());
            if (!axis_in.tvalid || hs) begin
                if (src_q.size() > 0 && $urandom_range(99) >= gap_pct) begin
                    axis_in.tvalid <= 1'b1;
                    {axis_in.tid, axis_in.tdata} <= src_q[0];
                end else begin
                    axis_in.tvalid <= 1'b0;
                end
            end
        end
    end

    // Scoreboard: what each channel was sent
    logic [WIDTH-1:0] expected [NUM_IDS][$];
    int unsigned      n_sent = 0;

    always @(posedge clk) begin
        if (rstn && axis_in.tvalid && axis_in.tready && int'(axis_in.tid) < CHANNELS) begin
            expected[axis_in.tid].push_back(axis_in.tdata);
            n_sent++;
        end
    end

    // ------------------------------------------------------------------
    // Sink: order, integrity and stability
    // ------------------------------------------------------------------
    int unsigned               sink_rate = 100;
    int unsigned               n_recv = 0;
    logic [ID_WIDTH-1:0]       next_tid = '0;
    bit                        stalled = 1'b0;
    logic [ID_WIDTH+WIDTH-1:0] stalled_beat;

    always @(posedge clk) begin
        if (rstn) begin
            if (stalled) begin
                tb_checks++;
                if (axis_out.tvalid !== 1'b1 || {axis_out.tid, axis_out.tdata} !== stalled_beat)
                    tb_error_once({NAME, " stability"}, $sformatf("%s: output changed while stalled", NAME));
            end

            if (axis_out.tvalid && axis_out.tready) begin
                tb_checks++;
                if (axis_out.tid !== next_tid) begin
                    tb_error_once({NAME, " order"},
                        $sformatf("%s: output tid %0d, expected %0d", NAME, axis_out.tid, next_tid));
                end else if (expected[axis_out.tid].size() == 0) begin
                    tb_error_once({NAME, " extra"},
                        $sformatf("%s: channel %0d came out with nothing sent", NAME, axis_out.tid));
                end else begin
                    automatic logic [WIDTH-1:0] exp = expected[axis_out.tid].pop_front();
                    if (axis_out.tdata !== exp)
                        tb_error_once({NAME, " data"},
                            $sformatf("%s: channel %0d is 0x%0h, expected 0x%0h", NAME, axis_out.tid, axis_out.tdata, exp));
                end
                next_tid = next_tid == ID_WIDTH'(CHANNELS - 1) ? '0 : next_tid + 1;
                n_recv++;
            end

            stalled      = axis_out.tvalid && !axis_out.tready;
            stalled_beat = {axis_out.tid, axis_out.tdata};
        end
        axis_out.tready <= $urandom_range(99) < sink_rate;
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    // Queue N_CYCLES cycles, in i2s order or shuffled, with stray tids mixed in
    task automatic send_cycles(bit shuffled);
        int order[$];
        repeat (N_CYCLES) begin
            order = {};
            for (int c = 0; c < CHANNELS; c += 2) order.push_back(c);
            for (int c = 1; c < CHANNELS; c += 2) order.push_back(c);
            if (shuffled) order.shuffle();
            foreach (order[i]) begin
                if (CHANNELS < NUM_IDS && $urandom_range(9) == 0)
                    src_q.push_back({ID_WIDTH'($urandom_range(NUM_IDS - 1, CHANNELS)), WIDTH'($urandom)});
                src_q.push_back({ID_WIDTH'(order[i]), WIDTH'($urandom)});
            end
        end
    endtask

    task automatic phase(string name, bit shuffled, int unsigned gaps, int unsigned sink);
        int unsigned target = n_recv + N_CYCLES * CHANNELS;
        int unsigned timeout = 0;
        gap_pct   = gaps;
        sink_rate = sink;
        send_cycles(shuffled);
        while (n_recv < target && timeout < 200 * N_CYCLES * CHANNELS) begin
            @(posedge clk);
            timeout++;
        end
        `CHECK(n_recv == target, $sformatf("%s, %s phase: %0d of %0d samples came out", NAME, name, n_recv, target))
    endtask

    initial begin
        done = 1'b0;
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        @(posedge clk);
        `CHECK_EQ(axis_out.tvalid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        phase("i2s",             1'b0, 0,  100);
        phase("i2s stalls",      1'b0, 50, 40);
        phase("shuffled",        1'b1, 0,  100);
        phase("shuffled stalls", 1'b1, 50, 40);

        repeat (5) @(posedge clk);
        `CHECK(n_sent == n_recv, $sformatf("%s: %0d samples sent, %0d came out", NAME, n_sent, n_recv))
        `CHECK_EQ(axis_out.tvalid, 1'b0, $sformatf("%s: tvalid low when idle", NAME))
        tb_info($sformatf("%s: %0d samples", NAME, n_recv));
        done = 1'b1;
    end

endmodule


module tb_axis_tid_reorder;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(2_000_000) // 2 ms

    localparam real CLK_PERIOD_NS = 1.0e9 / 73.5714e6; // clk_io, where i2s lives

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    logic [2:0] done;
    axis_tid_reorder_inst #(.WIDTH(18), .ID_WIDTH(3), .CHANNELS(8)) g_ch8 (.clk(clk), .done(done[0]));
    axis_tid_reorder_inst #(.WIDTH(8),  .ID_WIDTH(3), .CHANNELS(5)) g_ch5 (.clk(clk), .done(done[1]));
    axis_tid_reorder_inst #(.WIDTH(4),  .ID_WIDTH(1), .CHANNELS(2)) g_ch2 (.clk(clk), .done(done[2]));

    initial begin
        tb_info("Testing axis_tid_reorder with 8, 5 and 2 channels");
        wait (&done);
        tb_finish();
    end

endmodule
