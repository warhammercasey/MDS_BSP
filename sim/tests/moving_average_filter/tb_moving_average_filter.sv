`timescale 1ns/1ps
`include "tb_macros.svh"

// moving_average_filter test: every input beat gives one output beat, the average of the last
// 2^LOG2_WINDOW_SIZE inputs per channel (the sum shifted down, so rounded towards -inf), with
// inputs before reset counting as 0. Checked against a model that re-sums the window each beat.
//
// Bursts of these (mixed at random, per channel):
//   random     random full-range values
//   max / min  full scale or the most negative value, so a narrow sum overflows
//   alternate  max and min every other beat
//   small      -3..3, where the round-down of negatives shows
//   constant   one value held, so the output settles on it exactly
// With random gaps in, and the output ready ~50% of the time with stalls of up to 200 clocks.
//
// Then, with the window full of non-zero data, a reset and a second run: the history should
// start from zeros again.
module moving_average_filter_inst #(
    parameter int LOG2_WINDOW_SIZE = 4,
    parameter int DATA_WIDTH       = 24,
    parameter int NUM_CHANNELS     = 3,
    parameter int BEATS            = 3000
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int WINDOW = 1 << LOG2_WINDOW_SIZE;
    localparam int TOTAL  = DATA_WIDTH * NUM_CHANNELS;

    logic rstn = 1'b0;

    axis_if #(.WIDTH(TOTAL)) din ();
    axis_if #(.WIDTH(TOTAL)) dout ();

    moving_average_filter #(
        .LOG2_WINDOW_SIZE(LOG2_WINDOW_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .NUM_CHANNELS(NUM_CHANNELS)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .din(din),
        .dout(dout)
    );

    string NAME = $sformatf("%0d x %0d bit, window %0d", NUM_CHANNELS, DATA_WIDTH, WINDOW);
    string phase = "first run";

    localparam logic signed [DATA_WIDTH-1:0] MAX = (1 <<< (DATA_WIDTH - 1)) - 1;
    localparam logic signed [DATA_WIDTH-1:0] MIN = -(1 <<< (DATA_WIDTH - 1));

    function automatic logic signed [DATA_WIDTH-1:0] chan(logic [TOTAL-1:0] beat, int c);
        return beat[c*DATA_WIDTH +: DATA_WIDTH];
    endfunction

    // ------------------------------------------------------------------
    // Model: the window of accepted beats, re-summed for every one
    // ------------------------------------------------------------------
    logic [TOTAL-1:0] window_q[$];
    logic [TOTAL-1:0] expected[$];
    int unsigned      n_sent = 0;

    function automatic void accept(logic [TOTAL-1:0] beat);
        logic [TOTAL-1:0] avg;
        window_q.push_back(beat);
        if (window_q.size() > WINDOW) void'(window_q.pop_front());
        for (int c = 0; c < NUM_CHANNELS; c++) begin
            longint sum = 0;
            foreach (window_q[n]) sum += longint'(chan(window_q[n], c));
            avg[c*DATA_WIDTH +: DATA_WIDTH] = DATA_WIDTH'(sum >>> LOG2_WINDOW_SIZE);
        end
        expected.push_back(avg);
        n_sent++;
    endfunction

    // ------------------------------------------------------------------
    // Source
    // ------------------------------------------------------------------
    logic [TOTAL-1:0] src_q[$];

    always @(posedge clk) begin
        bit hs;
        if (!rstn) begin
            din.tvalid <= 1'b0;
            din.tdata  <= '0;
        end else begin
            hs = din.tvalid && din.tready;
            if (hs) accept(src_q.pop_front());
            if (!din.tvalid || hs) begin
                if (src_q.size() > 0 && $urandom_range(99) >= 30) begin
                    din.tvalid <= 1'b1;
                    din.tdata  <= src_q[0];
                end else begin
                    din.tvalid <= 1'b0;
                    din.tdata  <= TOTAL'({$urandom, $urandom, $urandom, $urandom});  // Junk, ignored
                end
            end
        end
    end

    function automatic logic signed [DATA_WIDTH-1:0] rand_value(int bits);
        logic signed [DATA_WIDTH-1:0] v = DATA_WIDTH'({$urandom, $urandom});
        return v >>> (DATA_WIDTH - bits);
    endfunction

    // Bursts of 1..3 windows, each channel its own kind
    task automatic make_beats(int n);
        while (n > 0) begin
            int len = $urandom_range(3 * WINDOW, 1);
            int kind[NUM_CHANNELS];
            logic signed [DATA_WIDTH-1:0] held[NUM_CHANNELS];
            for (int c = 0; c < NUM_CHANNELS; c++) begin
                kind[c] = $urandom_range(5);
                held[c] = rand_value(DATA_WIDTH);
            end
            for (int k = 0; k < len && n > 0; k++, n--) begin
                logic [TOTAL-1:0] beat;
                for (int c = 0; c < NUM_CHANNELS; c++) begin
                    logic signed [DATA_WIDTH-1:0] v;
                    case (kind[c])
                        0: v = rand_value(DATA_WIDTH);
                        1: v = MAX;
                        2: v = MIN;
                        3: v = k % 2 ? MIN : MAX;
                        4: v = DATA_WIDTH'($signed($urandom_range(6)) - 3);
                        default: v = held[c];
                    endcase
                    beat[c*DATA_WIDTH +: DATA_WIDTH] = v;
                end
                src_q.push_back(beat);
            end
        end
    endtask

    // ------------------------------------------------------------------
    // Sink
    // ------------------------------------------------------------------
    int unsigned      n_recv = 0;
    int unsigned      stall_left = 0;
    bit               stalled = 1'b0;
    logic [TOTAL-1:0] stalled_beat;

    always @(posedge clk) begin
        if (!rstn) begin
            stalled = 1'b0;
            if (dout.tvalid !== 1'b0)
                tb_error_once({NAME, " reset"}, $sformatf("%s: tvalid high in reset", NAME));
        end else begin
            if (stalled) begin
                tb_checks++;
                if (dout.tvalid !== 1'b1 || dout.tdata !== stalled_beat)
                    tb_error_once({NAME, " stability"}, $sformatf("%s: output changed while stalled", NAME));
            end
            if (dout.tvalid && dout.tready) begin
                tb_checks++;
                if (expected.size() == 0) begin
                    tb_error_once({NAME, " extra"}, $sformatf("%s: output with nothing due", NAME));
                end else begin
                    automatic logic [TOTAL-1:0] exp = expected.pop_front();
                    if (dout.tdata !== exp) begin
                        automatic string got_s = "", exp_s = "";
                        for (int c = 0; c < NUM_CHANNELS; c++) begin
                            got_s = {got_s, $sformatf(" %0d", chan(dout.tdata, c))};
                            exp_s = {exp_s, $sformatf(" %0d", chan(exp, c))};
                        end
                        tb_error_once({NAME, " data ", phase},
                            $sformatf("%s, %s: output %0d:%s, expected%s", NAME, phase, n_recv, got_s, exp_s));
                    end
                end
                n_recv++;
            end
            stalled      = dout.tvalid && !dout.tready;
            stalled_beat = dout.tdata;
        end
        // Mostly ~50% ready, with stalls long enough to back the input up
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

    task automatic run(int n);
        int unsigned timeout = 0;
        int unsigned start = n_recv;
        make_beats(n);
        while ((src_q.size() > 0 || n_recv < n_sent) && timeout < 20 * n + 1000) begin
            @(posedge clk);
            timeout++;
        end
        repeat (20) @(posedge clk);
        `CHECK(src_q.size() == 0 && n_recv == n_sent,
               $sformatf("%s, %s: %0d of %0d outputs came out", NAME, phase, n_recv - start, n))
        `CHECK(expected.size() == 0, $sformatf("%s, %s: %0d outputs never came out", NAME, phase, expected.size()))
    endtask

    initial begin
        done = 1'b0;
        repeat (5) @(posedge clk);
        rstn <= 1'b1;
        @(posedge clk);
        `CHECK_EQ(dout.tvalid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        run(BEATS);

        // Reset with a full window of non-zero data in the filter, then go again
        src_q.push_back({NUM_CHANNELS{MAX}});
        repeat (WINDOW - 1) src_q.push_back({NUM_CHANNELS{DATA_WIDTH'(MAX / 2 + 1)}});
        run(0);
        @(posedge clk);
        rstn <= 1'b0;
        repeat (5) @(posedge clk);
        window_q.delete();
        expected.delete();
        n_sent = 0;
        n_recv = 0;
        phase  = "after a reset";
        rstn <= 1'b1;
        @(posedge clk);
        run(BEATS / 4);

        tb_info($sformatf("%s: %0d + %0d beats", NAME, BEATS + WINDOW, n_recv));
        done = 1'b1;
    end

endmodule


module tb_moving_average_filter;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(20_000_000) // 20 ms

    logic clk = 1'b0;
    always #2.5 clk = ~clk; // 200 MHz, clk_core

    logic [2:0] done;

    // The defaults: 3 x 24 bit directions, window 16
    moving_average_filter_inst #(.LOG2_WINDOW_SIZE(4), .DATA_WIDTH(24), .NUM_CHANNELS(3)) g_default (.clk(clk), .done(done[0]));
    // The smallest window, one narrow channel
    moving_average_filter_inst #(.LOG2_WINDOW_SIZE(1), .DATA_WIDTH(8),  .NUM_CHANNELS(1)) g_small   (.clk(clk), .done(done[1]));
    // A long window of 16 bit Q2.14 directions
    moving_average_filter_inst #(.LOG2_WINDOW_SIZE(6), .DATA_WIDTH(16), .NUM_CHANNELS(3)) g_long    (.clk(clk), .done(done[2]));

    initial begin
        tb_info("Testing moving_average_filter: 3 x 24 bit / 16, 1 x 8 bit / 2, 3 x 16 bit / 64");
        wait (&done);
        tb_finish();
    end

endmodule
