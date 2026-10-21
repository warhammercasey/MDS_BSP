`timescale 1ns/1ps
`include "tb_macros.svh"

// axis_switch test: N input streams arbitrated onto one output, for several N.
//
// Each input tags its beats with its index in the top bits of tdata, so every
// output beat can be traced back to its input. Checked throughout:
//   - every beat sent arrives exactly once, intact (order between inputs is free,
//     but each input's beats must stay in order, as the switch never reorders a stream)
//   - the output holds tvalid/tdata while stalled
//   - fairness: while an input waits, the switch accepts at most N-1 beats from
//     other inputs (round robin), and with the output always ready it waits at
//     most N-1 cycles
// Phases:
//   spec          each input idles >= N cycles between beats (the design's rate
//                 assumption), output always ready
//   backpressure  same inputs, output ready ~25% of cycles (like a UART TX)
//   saturated     inputs offer back-to-back beats, beyond the spec. With the
//                 output always ready the switch must pass one beat per cycle;
//                 then the output is throttled to ~50%
module axis_switch_inst #(
    parameter int N        = 2,
    parameter int WIDTH    = 8,
    parameter int N_BEATS  = 60  // beats per input, per phase
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int SRC_W = N > 1 ? $clog2(N) : 1;  // tdata[WIDTH-1 -: SRC_W] = input index

    logic rstn = 1'b0;

    axis_if #(.WIDTH(WIDTH)) axis_in [N] ();
    axis_if #(.WIDTH(WIDTH)) axis_out ();

    axis_switch #(
        .WIDTH(WIDTH),
        .N(N)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .axis_in(axis_in),
        .axis_out(axis_out)
    );

    string NAME = $sformatf("N=%0d", N);

    // ------------------------------------------------------------------
    // Sources
    // ------------------------------------------------------------------
    logic [N-1:0]     in_valid, in_ready;
    logic [WIDTH-1:0] in_data [N];

    int unsigned gap_min = 0, gap_max = 0; // idle cycles between an input's beats
    int unsigned n_target = 0;             // beats per input to send in total

    for (genvar i = 0; i < N; i++) begin : g_src
        int unsigned n_issued = 0;
        int unsigned gap_left = 0;

        assign axis_in[i].tvalid = in_valid[i];
        assign axis_in[i].tdata  = in_data[i];
        assign in_ready[i]       = axis_in[i].tready;

        always @(posedge clk) begin
            if (!rstn) begin
                in_valid[i] <= 1'b0;
                in_data[i]  <= '0;
            end else begin
                if (in_valid[i] && in_ready[i])
                    gap_left = $urandom_range(gap_max, gap_min);
                if (!in_valid[i] || in_ready[i]) begin
                    if (gap_left > 0) begin
                        gap_left--;
                        in_valid[i] <= 1'b0;
                    end else if (n_issued < n_target) begin
                        logic [WIDTH-1:0] d = WIDTH'($urandom);
                        d[WIDTH-1 -: SRC_W] = SRC_W'(i);
                        in_valid[i] <= 1'b1;
                        in_data[i]  <= d;
                        n_issued++;
                    end else begin
                        in_valid[i] <= 1'b0;
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Input monitor: what was accepted, and how long each beat waited
    // ------------------------------------------------------------------
    // Sized so any SRC_W-bit index is in range (Verilator 5.020 generates C++ that
    // doesn't compile for a possibly out-of-range index into an array of queues)
    logic [WIDTH-1:0] expected [1 << SRC_W][$];
    int unsigned      n_sent = 0;           // beats accepted, all inputs
    int unsigned      cycle = 0;
    bit               ready_always = 1'b0;  // output always ready: also check waits in cycles
    bit               waiting  [N] = '{default: 1'b0};
    int unsigned      wait_acc [N];         // n_sent when the beat was first offered
    int unsigned      wait_cyc [N];
    int unsigned      max_others = 0, max_cycles = 0;

    always @(posedge clk) begin
        if (rstn) begin
            for (int i = 0; i < N; i++) begin
                if (in_valid[i] && !waiting[i]) begin
                    waiting[i]  = 1'b1;
                    wait_acc[i] = n_sent;
                    wait_cyc[i] = cycle;
                end
            end
            for (int i = 0; i < N; i++) begin
                if (in_valid[i] && in_ready[i]) begin
                    automatic int unsigned others = n_sent - wait_acc[i];
                    automatic int unsigned cycles = cycle - wait_cyc[i];
                    expected[SRC_W'(i)].push_back(in_data[i]);
                    waiting[i] = 1'b0;
                    max_others = others > max_others ? others : max_others;
                    tb_checks++;
                    if (others > N - 1)
                        tb_error_once({NAME, " fairness"},
                            $sformatf("%s: input %0d waited while %0d beats from other inputs were accepted (round robin allows %0d)",
                                      NAME, i, others, N - 1));
                    if (ready_always) begin
                        max_cycles = cycles > max_cycles ? cycles : max_cycles;
                        tb_checks++;
                        if (cycles > N - 1)
                            tb_error_once({NAME, " wait"},
                                $sformatf("%s: input %0d waited %0d cycles with the output always ready (expected at most %0d)",
                                          NAME, i, cycles, N - 1));
                    end
                end
            end
            n_sent += $countones(in_valid & in_ready);
            tb_checks++;
            if ($countones(in_valid & in_ready) > 1)
                tb_error_once({NAME, " multi"}, $sformatf("%s: more than one input accepted in one cycle", NAME));
        end
        cycle++;
    end

    // ------------------------------------------------------------------
    // Sink: integrity and stability
    // ------------------------------------------------------------------
    int unsigned      sink_rate = 0;
    int unsigned      n_recv = 0;
    bit               stalled = 1'b0;
    logic [WIDTH-1:0] stalled_data;

    always @(posedge clk) begin
        if (rstn) begin
            if (stalled) begin
                tb_checks++;
                if (axis_out.tvalid !== 1'b1 || axis_out.tdata !== stalled_data)
                    tb_error_once({NAME, " stability"},
                        $sformatf("%s: output changed while stalled: tvalid=%b tdata 0x%0h -> 0x%0h",
                                  NAME, axis_out.tvalid, stalled_data, axis_out.tdata));
            end

            if (axis_out.tvalid && axis_out.tready) begin
                automatic logic [SRC_W-1:0] src = axis_out.tdata[WIDTH-1 -: SRC_W];
                tb_checks++;
                if (int'(src) >= N || expected[src].size() == 0) begin
                    tb_error_once({NAME, " extra"},
                        $sformatf("%s: output beat 0x%0h (input %0d) was never sent", NAME, axis_out.tdata, src));
                end else begin
                    automatic logic [WIDTH-1:0] exp = expected[src].pop_front();
                    if (axis_out.tdata !== exp)
                        tb_error_once({NAME, " data"},
                            $sformatf("%s: beat from input %0d is 0x%0h, expected 0x%0h", NAME, src, axis_out.tdata, exp));
                end
                n_recv++;
            end

            stalled      = axis_out.tvalid && !axis_out.tready;
            stalled_data = axis_out.tdata;
        end
        axis_out.tready <= $urandom_range(99) < sink_rate;
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic phase(string name, int unsigned gmin, int unsigned gmax, int unsigned sink);
        int unsigned timeout = 0;
        gap_min      = gmin;
        gap_max      = gmax;
        sink_rate    = sink;
        ready_always = sink == 100;
        n_target    += N_BEATS;
        while (n_recv < n_target * N && timeout < 100 * N_BEATS * N * (gmax + 2)) begin
            @(posedge clk);
            timeout++;
        end
        `CHECK(n_recv == n_target * N,
               $sformatf("%s, %s phase: received %0d of %0d beats", NAME, name, n_recv, n_target * N))
    endtask

    initial begin
        int unsigned c0, r0;
        real         rate;

        done = 1'b0;
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        @(posedge clk);
        `CHECK_EQ(axis_out.tvalid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        phase("spec",         N, 4 * N, 100);
        phase("backpressure", N, 4 * N, 25);

        // Saturated, output always ready: the output must carry a beat every cycle
        c0 = cycle;
        r0 = n_recv;
        phase("saturated",    0, 0, 100);
        rate = real'(n_recv - r0) / real'(cycle - c0);
        `CHECK(rate >= 0.9, $sformatf("%s: %0.2f beats/cycle with all inputs busy and the output always ready", NAME, rate))
        phase("saturated + backpressure", 0, 0, 50);

        repeat (5) @(posedge clk);
        foreach (expected[i])
            `CHECK(expected[i].size() == 0, $sformatf("%s: input %0d has %0d beats that never came out", NAME, i, expected[i].size()))
        `CHECK_EQ(axis_out.tvalid, 1'b0, $sformatf("%s: tvalid low when idle", NAME))

        tb_info($sformatf("%-5s %0d beats; saturated rate %0.2f beats/cycle; worst wait %0d other beats, %0d cycles",
                          NAME, n_recv, rate, max_others, max_cycles));
        done = 1'b1;
    end

endmodule


module tb_axis_switch;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(2_000_000) // 2 ms

    localparam real CLK_PERIOD_NS = 1.0e9 / 73.5714e6; // clk_io, where the UART lives

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    localparam int NUM_CONFIGS = 5;
    localparam int N_LIST [NUM_CONFIGS] = '{1, 2, 3, 4, 8};

    logic [NUM_CONFIGS-1:0] done;
    for (genvar c = 0; c < NUM_CONFIGS; c++) begin : g_n
        axis_switch_inst #(.N(N_LIST[c])) h (.clk(clk), .done(done[c]));
    end

    initial begin
        tb_info("Testing axis_switch with N = 1, 2, 3, 4, 8");
        wait (&done);
        tb_finish();
    end

endmodule
