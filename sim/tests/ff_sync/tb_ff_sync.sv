`timescale 1ns/1ps
`include "tb_macros.svh"

// ff_sync test: one synchronizer per ordered pair of the design's clock domains.
//
// Each source drives a Gray counter (the only kind of bus that is safe to pass
// through ff_sync), holding each value for a random number of clka cycles:
//   - slow phase: every value is held long enough to be seen, so dout must step
//     through every value in order
//   - fast phase: values may be shorter than a clkb period, so some can be
//     skipped, but dout must still only show launched values, in order
// Every dout change is also timed: counted in clkb edges after the clka edge
// that registered the value, it must take at least MIN_STAGES edges (one per
// destination flop; fewer means the first flop to see the async signal drives
// the output) and at most MAX_STAGES.
module ff_sync_pair #(
    parameter real   FREQ_A     = 100.0e6,
    parameter real   FREQ_B     = 100.0e6,
    parameter string NAME       = "",
    parameter int    WIDTH      = 4,
    parameter int    MIN_STAGES = 2,
    parameter int    MAX_STAGES = 4,
    parameter int    N_SLOW     = 40,  // values in the slow phase
    parameter int    N_FAST     = 300  // values in the fast phase
) (
    input  logic clka,
    input  logic clkb,
    output logic done
);
    import tb_pkg::*;

    localparam real TA = 1.0e9 / FREQ_A;
    localparam real TB = 1.0e9 / FREQ_B;
    // clka cycles that cover MAX_STAGES + 1 clkb periods: a value held this long is always captured
    localparam int  HOLD_SLOW = int'($ceil((MAX_STAGES + 1) * TB / TA));
    localparam int  HOLD_FAST = int'($ceil(2.0 * TB / TA));

    typedef struct {
        logic [WIDTH-1:0] value;
        realtime          t_launch; // clka edge that registered the value inside ff_sync
    } launch_t;

    logic [WIDTH-1:0] din = '0;
    logic [WIDTH-1:0] dout;

    ff_sync #(.WIDTH(WIDTH)) dut (
        .clka(clka),
        .clkb(clkb),
        .din(din),
        .dout(dout)
    );

    function automatic logic [WIDTH-1:0] to_gray(logic [WIDTH-1:0] b);
        return b ^ (b >> 1);
    endfunction

    // ------------------------------------------------------------------
    // Source (clka): Gray counter with random hold times
    // ------------------------------------------------------------------
    launch_t          launched[$];   // launched, not yet seen on dout
    logic [WIDTH-1:0] count = '0;
    bit               fast = 1'b0;
    bit               pending = 1'b0;
    int unsigned      n_launched = 0;
    int unsigned      n_target = 0;  // stop launching after this many values
    int unsigned      hold_left = 0;

    always @(posedge clka) begin
        // ff_sync's source flop captures last cycle's din on this edge
        if (pending) begin
            launch_t l;
            l.value    = din;
            l.t_launch = $realtime;
            launched.push_back(l);
            pending = 1'b0;
        end

        if (hold_left > 0) begin
            hold_left--;
        end else if (n_launched < n_target) begin
            count      = count + 1'b1;
            din       <= to_gray(count);
            pending    = 1'b1;
            n_launched++;
            hold_left  = fast ? $urandom_range(HOLD_FAST - 1) : HOLD_SLOW + $urandom_range(HOLD_SLOW);
        end
    end

    // ------------------------------------------------------------------
    // Destination (clkb): order, skipped values and latency
    // ------------------------------------------------------------------
    realtime          clkb_edges[$];
    logic [WIDTH-1:0] last_dout = '0;
    int unsigned      n_seen = 0, n_skipped = 0;
    int               lat_min = 1000, lat_max = 0;

    always @(posedge clkb) begin
        clkb_edges.push_back($realtime);
        if (clkb_edges.size() > 4 * MAX_STAGES)
            void'(clkb_edges.pop_front());
    end

    always @(dout) begin
        if (dout !== last_dout) begin
            last_dout = dout;
            // Values launched before this one were skipped (allowed only in the fast phase)
            while (launched.size() > 0 && launched[0].value !== dout) begin
                if (!fast)
                    tb_error($sformatf("%s: value 0x%0h was held %0d+ clka cycles but never appeared on dout",
                                       NAME, launched[0].value, HOLD_SLOW));
                void'(launched.pop_front());
                n_skipped++;
            end

            tb_checks++;
            if (launched.size() == 0) begin
                tb_error($sformatf("%s: dout changed to 0x%0h, which is not a launched value (or is out of order)",
                                   NAME, dout));
            end else begin
                automatic launch_t l = launched.pop_front();
                automatic int edges = 0;
                foreach (clkb_edges[i])
                    if (clkb_edges[i] > l.t_launch)
                        edges++;
                lat_min = edges < lat_min ? edges : lat_min;
                lat_max = edges > lat_max ? edges : lat_max;

                tb_checks++;
                if (edges < MIN_STAGES)
                    tb_error_once("ff_sync too few stages",
                        $sformatf("%s: dout changed %0d clkb edge(s) after the value was registered in clka; a synchronizer needs at least %0d destination flops",
                                  NAME, edges, MIN_STAGES));
                else if (edges > MAX_STAGES)
                    tb_error_once("ff_sync too slow",
                        $sformatf("%s: dout changed %0d clkb edges after the value was registered (expected at most %0d)",
                                  NAME, edges, MAX_STAGES));
                n_seen++;
            end
        end
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic wait_drained();
        int unsigned timeout = 0;
        while ((n_launched < n_target || pending || launched.size() != 0) && timeout < 10 * HOLD_SLOW * N_FAST) begin
            @(posedge clka);
            timeout++;
        end
        `CHECK(launched.size() == 0 && dout === din,
               $sformatf("%s: dout settled at the last value (dout=0x%0h din=0x%0h)", NAME, dout, din))
    endtask

    initial begin
        done = 1'b0;
        repeat (4) @(posedge clka);

        n_target = N_SLOW;
        wait_drained();
        `CHECK_EQ(n_skipped, 0, $sformatf("%s: values skipped in the slow phase", NAME))

        fast     = 1'b1;
        n_target = N_SLOW + N_FAST;
        wait_drained();

        tb_info($sformatf("%-20s %0d values launched, %0d seen, %0d skipped (fast phase); latency %0d-%0d clkb edges",
                          NAME, n_launched, n_seen, n_skipped, lat_min, lat_max));
        done = 1'b1;
    end

endmodule


module tb_ff_sync;
    import tb_pkg::*;
    import cain_clocks_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(2_000_000) // 2 ms

    logic clk [NUM_CLKS];
    for (genvar i = 0; i < NUM_CLKS; i++) begin : g_clk
        clk_gen #(.FREQ_HZ(FREQ_HZ[i])) gen (.clk(clk[i]));
    end

    logic [NUM_PAIRS-1:0] done;
    for (genvar p = 0; p < NUM_PAIRS; p++) begin : g_pair
        ff_sync_pair #(
            .FREQ_A(FREQ_HZ[pair_a(p)]),
            .FREQ_B(FREQ_HZ[pair_b(p)]),
            .NAME(pair_name(p))
        ) h (
            .clka(clk[pair_a(p)]),
            .clkb(clk[pair_b(p)]),
            .done(done[p])
        );
    end

    initial begin
        tb_info($sformatf("Testing ff_sync on %0d clock pairs", NUM_PAIRS));
        wait (&done);
        tb_finish();
    end

endmodule
