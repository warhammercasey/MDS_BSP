// Multichannel complex cross correlator
// Correlates every pair of channels (a < b) over a window of the last WINDOW_LEN (L) samples,
// every WINDOW_STEP (N) samples, at lags -MAX_LAG..MAX_LAG:
//   R_ab[m] = sum over n = 0..L-1, with 0 <= n+m < L, of x_b[n+m] * conj(x_a[n])
// n indexes the window, and samples outside it count as 0. That's MATLAB's xcorr(x_b, x_a) on
// the window, so a peak at m > 0 means channel b lags channel a by m samples.
//
// din:  complex samples {I, Q}, tid = channel. Channels should come in at the same rate, in any
//       order. Windows end on the same sample index for every channel, so a window starts once
//       every channel has its samples. Windows that would start before the first sample are
//       skipped: the first one ends at sample ceil(L/N)*N - 1.
// dout: one packet per pair per window, pairs in order (0,1), (0,2), .. (0,C-1), (1,2), ..,
//       tid = pair index. Each is 2*MAX_LAG+1 beats {re, im}, lag -MAX_LAG first, tlast on the
//       last. Values are (R >>> OUT_SHIFT) saturated to OUT_WIDTH bits; the default OUT_SHIFT
//       keeps the top OUT_WIDTH bits of the full precision sum, which can't saturate.
//
// MACS complex multiply-accumulate lanes (4 18x18 multipliers each) work on MACS adjacent lags
// at once, fed from a delay line, so the input buffers are read once per lane group. A window
// takes PAIRS * ceil((2*MAX_LAG+1)/MACS) * (L+MACS-1) clocks, plus a few. That has to fit in the
// time N samples take to come in: until it's done, din is held off for any channel more than N
// samples past the window (each channel buffers L+N samples). An output packet stalled for long
// stalls the correlation too.
module cross_correlator #(
    parameter CHANNELS    = 8,
    parameter WIDTH       = 18,   // I and Q sample width, up to 18
    parameter WINDOW_LEN  = 2560, // L
    parameter WINDOW_STEP = 32,   // N
    parameter MAX_LAG     = 10,
    parameter MACS        = 3,    // Complex MAC lanes. Divisors of 2*MAX_LAG+1 waste none
    parameter OUT_WIDTH   = 32,   // Width of each of re and im on dout
    parameter OUT_SHIFT   = (2*WIDTH + 1 + $clog2(WINDOW_LEN) > OUT_WIDTH) ? 2*WIDTH + 1 + $clog2(WINDOW_LEN) - OUT_WIDTH : 0
) (
    input  logic clk,
    input  logic rstn,

    axis_tid_if.slv       din,
    axis_tid_tlast_if.mst dout
);

    localparam L         = WINDOW_LEN;
    localparam N         = WINDOW_STEP;
    localparam LAGS      = 2*MAX_LAG + 1;
    localparam PAIRS     = CHANNELS*(CHANNELS-1)/2;
    localparam PASSES    = (LAGS + MACS - 1) / MACS;  // Lane groups per pair
    localparam LAST_MACS = LAGS - (PASSES-1)*MACS;    // Lanes used in the last group
    localparam T_LAST    = L + MACS - 2;              // Each group streams L+MACS-1 samples
    localparam SKIP      = (L + N - 1) / N - 1;       // Windows before the first full one
    localparam ACC_WIDTH = 2*WIDTH + 1 + $clog2(L);

    // Ring buffer per channel: the window being correlated plus room for the next N samples
    localparam DEPTH = L + N;
    localparam AW    = $clog2(DEPTH);
    localparam START = (2*N) % DEPTH; // Start of the first window (ending at sample N-1), mod DEPTH

    localparam CH_W   = $clog2(CHANNELS);
    localparam PAIR_W = PAIRS > 1 ? $clog2(PAIRS) : 1;
    localparam PASS_W = PASSES > 1 ? $clog2(PASSES) : 1;
    localparam T_W    = $clog2(T_LAST + 1);
    localparam M0_W   = $clog2(MAX_LAG + MACS + 1) + 1;            // First lag of a group, signed
    localparam KB_W   = (T_W > M0_W ? T_W : M0_W) + 2;             // Sample index into the window, signed
    localparam FILL_W = $clog2(2*N + 1);
    localparam SKIP_W = $clog2(SKIP + 1) > 0 ? $clog2(SKIP + 1) : 1;

    // Pointer arithmetic before wrapping, in (-DEPTH, 2*DEPTH): signed, and only as wide as that
    // (and m0) needs, so the adders and compares stay short
    localparam PTR_W = (AW + 2 > M0_W + 1) ? AW + 2 : M0_W + 1;
    typedef logic signed [PTR_W-1:0] ptr_sum_t;

    function automatic ptr_sum_t ptr(logic [AW-1:0] addr);
        return ptr_sum_t'({1'b0, addr});
    endfunction

    function automatic logic [AW-1:0] wrap(ptr_sum_t addr);
        if(addr[PTR_W-1]) begin
            return AW'(addr + ptr_sum_t'(DEPTH));
        end else if(addr >= ptr_sum_t'(DEPTH)) begin
            return AW'(addr - ptr_sum_t'(DEPTH));
        end
        return AW'(addr);
    endfunction

    function automatic logic [AW-1:0] inc(logic [AW-1:0] addr);
        return (addr == AW'(DEPTH-1)) ? '0 : addr + 1'b1;
    endfunction


    // ------------------------------------------------------------------
    // Input: write each channel's samples into its ring buffer
    // ------------------------------------------------------------------
    // fill: samples written past the start of the current window's last N, 0..2N. The current
    // window (the one being or next to be correlated) is complete at N, and a channel can't go
    // more than N past it without overwriting it.
    // For timing, the input is registered first (tready is just "empty", so it doesn't depend
    // on tid), and nothing compares fill on the way: full and has_window are registered
    // alongside it. The buffer write is registered too (it lands a clock after the beat is
    // taken, long before the sample is read: it's the window's newest), and fill counts the
    // write, so it's a clock behind. Whether the held beat's channel is full is looked up a clock
    // after it's taken, into a register. That's safe: a beat takes 3 clocks, so the next one sees
    // the count, and the window release is registered too, so full and has_window can only be
    // high a clock too long after it, which holds input off a clock longer, and a new window
    // waits for ptrs_ready anyway.
    logic                in_valid;
    logic [CH_W-1:0]     in_tid;
    logic [WIDTH*2-1:0]  in_data;
    logic [FILL_W-1:0]   fill[CHANNELS];
    logic [CHANNELS-1:0] full;        // fill == 2N: hold this channel off
    logic [CHANNELS-1:0] has_window;  // fill >= N: the current window is complete
    logic [CHANNELS-1:0] accept;      // The held beat goes into channel c
    logic [AW-1:0]       write_ptr[CHANNELS];
    logic [CHANNELS-1:0] write_en;
    logic [WIDTH*2-1:0]  write_data;
    logic                release_window; // Done with the current window
    logic                release_q;
    logic                in_in_range;
    logic                in_checked; // in_full is for the held beat
    logic                in_full;

    assign din.tready  = ~in_valid;

    always_comb begin
        for(int c = 0; c < CHANNELS; c++) begin
            accept[c] = in_valid && in_in_range && in_checked && !in_full && in_tid == c;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            in_valid <= 1'b0;
            in_checked <= 1'b0;
        end else if(din.tvalid && din.tready) begin
            in_valid   <= 1'b1;
            in_checked <= 1'b0;
        end else if(|accept || !in_in_range) begin
            in_valid   <= 1'b0; // Out of range channels are dropped
            in_checked <= 1'b0;
        end else if(in_valid) begin
            in_checked <= 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if(din.tvalid && din.tready) begin
            in_tid      <= CH_W'(din.tid);
            in_in_range <= din.tid < CHANNELS;
            in_data     <= din.tdata;
        end
        in_full <= full[in_tid];
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            release_q <= 1'b0;
        end else begin
            release_q <= release_window;
        end
    end

    logic [FILL_W-1:0] fill_next[CHANNELS];
    always_comb begin
        for(int c = 0; c < CHANNELS; c++) begin
            fill_next[c] = fill[c] + FILL_W'(write_en[c]) - (release_q ? FILL_W'(N) : FILL_W'(0));
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            fill       <= '{default: '0};
            full       <= '0;
            has_window <= '0;
        end else begin
            for(int c = 0; c < CHANNELS; c++) begin
                fill[c]       <= fill_next[c];
                full[c]       <= fill_next[c] >= FILL_W'(2*N);
                has_window[c] <= fill_next[c] >= FILL_W'(N);
            end
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            write_en  <= '0;
            write_ptr <= '{default: '0};
        end else begin
            write_en <= accept;
            for(int c = 0; c < CHANNELS; c++) begin
                if(write_en[c]) begin
                    write_ptr[c] <= inc(write_ptr[c]);
                end
            end
        end
    end

    always_ff @(posedge clk) begin
        write_data <= in_data;
    end

    logic window_ready;
    assign window_ready = &has_window;


    // ------------------------------------------------------------------
    // Sequencing: for each pair, for each group of MACS lags, stream the window through
    // ------------------------------------------------------------------
    typedef enum logic {
        IDLE,
        RUN
    } state_t;
    state_t state;

    // The sequencer moves on. The pipeline behind it runs freely, so holding the sequencer just
    // sends bubbles; it holds a group's last sample until the output hold is free for its sums.
    logic issue;
    logic hold_busy; // The output hold is still sending, or a group's sums are on the way

    logic [AW-1:0]            start_ptr;  // Window start in the ring buffers
    logic [SKIP_W-1:0]        skip;
    logic [T_W-1:0]           t;          // Position in the group's stream
    logic [PASS_W-1:0]        pass;
    logic signed [M0_W-1:0]   m0;         // First lag of the group
    logic [PAIR_W-1:0]        pair;
    logic [CH_W-1:0]          chan_a, chan_b;
    logic [AW-1:0]            a_ptr, b_ptr;

    // t, pass and pair are at their last values: registered, kept in step with them, so the
    // end of a run doesn't need wide compares
    logic t_last, pass_last, pair_last;
    logic skip_left;
    logic run_last;
    logic ptrs_ready;
    assign run_last = state == RUN && t_last && pass_last && pair_last;
    // idle_go: in IDLE with a window ready, worked out a clock ahead into a register, since it
    // starts the whole sequencer. Nothing it depends on can change in between without a release,
    // which lowers ptrs_ready, so it's only acted on with ptrs_ready (still) high.
    logic idle_go, idle_act;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            idle_go <= 1'b0;
        end else begin
            idle_go <= state == IDLE && window_ready && ptrs_ready;
        end
    end
    assign idle_act = state == IDLE && idle_go && ptrs_ready;

    assign release_window = (idle_act && skip_left) || (run_last && issue);

    // The stream start pointers, worked out ahead in registers so the sequencer only selects
    // them: the sum one clock, the wrap the next. They follow start_ptr 2 clocks late, so a run
    // waits for ptrs_ready after it moves. b_next_pass follows m0, which only changes at the
    // end of a pass, T_LAST clocks before the next one ends.
    ptr_sum_t      a_sum, b_sum, b_next_sum;
    logic [AW-1:0] a_first, b_first, b_next_pass;
    always_ff @(posedge clk) begin
        a_sum       <= ptr(start_ptr) - ptr_sum_t'(MACS-1);
        b_sum       <= ptr(start_ptr) - ptr_sum_t'(MAX_LAG);
        b_next_sum  <= ptr(start_ptr) + ptr_sum_t'(m0) + ptr_sum_t'(MACS);
        a_first     <= wrap(a_sum);
        b_first     <= wrap(b_sum);
        b_next_pass <= wrap(b_next_sum);
    end

    logic ptrs_settling;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ptrs_settling <= 1'b1;
            ptrs_ready    <= 1'b0;
        end else begin
            ptrs_settling <= release_window;
            ptrs_ready    <= !release_window && !ptrs_settling;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            t_last    <= T_LAST == 0;
            pass_last <= PASSES == 1;
            pair_last <= PAIRS == 1;
        end else if(state == IDLE) begin
            t_last    <= T_LAST == 0;
            pass_last <= PASSES == 1;
            pair_last <= PAIRS == 1;
        end else if(issue) begin
            if(t_last) begin
                t_last <= T_LAST == 0;
                if(pass_last) begin
                    pass_last <= PASSES == 1;
                    pair_last <= pair == PAIR_W'(PAIRS-2);
                end else begin
                    pass_last <= pass == PASS_W'(PASSES-2);
                end
            end else begin
                t_last <= t == T_W'(T_LAST-1);
            end
        end
    end

    assign issue     = state == RUN && !(t_last && hold_busy);

    // Only the state, the window position and the skip count are reset: the rest is set up when
    // a run starts, so its logic doesn't carry the reset
    always_ff @(posedge clk) begin
        if(~rstn) begin
            state     <= IDLE;
            start_ptr <= AW'(START);
            skip      <= SKIP_W'(SKIP);
            skip_left <= SKIP != 0;
        end else begin
            if(release_window) begin
                start_ptr <= wrap(ptr(start_ptr) + ptr_sum_t'(N));
            end

            case(state)
                IDLE: begin
                    if(idle_act) begin
                        if(skip_left) begin
                            skip      <= skip - 1'b1;
                            skip_left <= skip != SKIP_W'(1);
                        end else begin
                            state <= RUN;
                        end
                    end
                end

                RUN: begin
                    if(issue && run_last) begin
                        state <= IDLE;
                    end
                end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        case(state)
            IDLE: begin
                if(idle_act && !skip_left) begin
                    t      <= '0;
                    pass   <= '0;
                    m0     <= M0_W'(-MAX_LAG);
                    pair   <= '0;
                    chan_a <= '0;
                    chan_b <= CH_W'(1);
                    // x_b streams from the group's first lag, x_a from MACS-1 samples
                    // before the window so it lines up with the end of the delay line
                    b_ptr  <= b_first;
                    a_ptr  <= a_first;
                end
            end

            RUN: if(issue) begin
                if(t_last) begin
                    t     <= '0;
                    a_ptr <= a_first;
                    if(pass_last) begin
                        pass  <= '0;
                        m0    <= M0_W'(-MAX_LAG);
                        b_ptr <= b_first;
                        pair  <= pair + 1'b1;
                        if(chan_b == CH_W'(CHANNELS-1)) begin
                            chan_a <= chan_a + 1'b1;
                            chan_b <= chan_a + CH_W'(2);
                        end else begin
                            chan_b <= chan_b + 1'b1;
                        end
                    end else begin
                        pass  <= pass + 1'b1;
                        m0    <= m0 + M0_W'(MACS);
                        b_ptr <= b_next_pass;
                    end
                end else begin
                    t     <= t + 1'b1;
                    a_ptr <= inc(a_ptr);
                    b_ptr <= inc(b_ptr);
                end
            end
        endcase
    end

    // What each stream position means, carried down the pipeline with the samples
    typedef struct packed {
        logic              valid;
        logic              a_ok;   // x_a is in the window: accumulate
        logic              b_ok;   // x_b is in the window, else it's fed as 0
        logic              first;  // First sample of the window
        logic              last;   // Last sample of the window: the sums are done
        logic [CH_W-1:0]   chan_a;
        logic [CH_W-1:0]   chan_b;
        logic [PAIR_W-1:0] pair;
        logic [PASS_W-1:0] pass;
    } ctrl_t;

    ctrl_t ctrl0, ctrl1, ctrl2, ctrl3, ctrl4, ctrl5, ctrl6, ctrl7;
    logic signed [KB_W-1:0] kb; // x_b's index in the window
    assign kb = $signed({2'b0, t}) + KB_W'(m0);
    always_comb begin
        ctrl0.valid  = issue;
        ctrl0.a_ok   = t >= T_W'(MACS-1);
        ctrl0.b_ok   = kb >= 0 && kb < KB_W'(L);
        ctrl0.first  = t == T_W'(MACS-1);
        ctrl0.last   = t_last;
        ctrl0.chan_a = chan_a;
        ctrl0.chan_b = chan_b;
        ctrl0.pair   = pair;
        ctrl0.pass   = pass;
    end


    // ------------------------------------------------------------------
    // Ring buffers
    // ------------------------------------------------------------------
    // Each buffer's read address, registered (x_b's pointer for chan_b, x_a's for the rest), so
    // the address select isn't on the way into the RAMs
    logic [WIDTH*2-1:0] buffer_out[CHANNELS];
    logic [AW-1:0]      read_addr[CHANNELS];
    ctrl_t              ctrl0q;
    always_ff @(posedge clk) begin
        for(int c = 0; c < CHANNELS; c++) begin
            read_addr[c] <= chan_b == c ? b_ptr : a_ptr;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl0q <= '0;
        end else begin
            ctrl0q <= ctrl0;
        end
    end

    generate
        for(genvar c = 0; c < CHANNELS; c++) begin : g_buffer
            simple_dual_port_bram #(
                .WIDTH(WIDTH*2), // *2 for complex samples
                .AWIDTH(AW),
                .DEPTH(DEPTH)
            ) ring (
                .clka(clk),
                .wea(write_en[c]),
                .ena(1'b1),
                .addra(write_ptr[c]),
                .dina(write_data),

                .clkb(clk),
                .enb(1'b1),
                .addrb(read_addr[c]),
                .doutb(buffer_out[c])
            );
        end
    endgenerate


    // ------------------------------------------------------------------
    // Pipeline
    // ------------------------------------------------------------------
    // The pipeline runs every clock, with ctrl*.valid marking the real samples. Only the delay
    // line and the accumulators hold state, and they only move on valid samples, so bubbles
    // from the sequencer holding pass straight through.
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl1 <= '0;
            ctrl2 <= '0;
        end else begin
            ctrl1 <= ctrl0q;
            ctrl2 <= ctrl1;
        end
    end

    // Register every buffer's output for timing, before the channel select. No enable and no
    // reset, so synthesis packs these into the RAM blocks' output registers.
    logic [WIDTH*2-1:0] buffer_q[CHANNELS];
    always_ff @(posedge clk) begin
        buffer_q <= buffer_out;
    end

    // Pick out the pair's samples
    logic signed [WIDTH-1:0] xa3_re, xa3_im, xb3_re, xb3_im;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl3 <= '0;
        end else begin
            ctrl3 <= ctrl2;
        end
    end

    always_ff @(posedge clk) begin
        {xa3_re, xa3_im} <= buffer_q[ctrl2.chan_a];
        {xb3_re, xb3_im} <= ctrl2.b_ok ? buffer_q[ctrl2.chan_b] : '0;
    end

    // Delay line: lane j works on lag m0+j, which needs x_b MACS-1-j samples back
    logic signed [WIDTH-1:0] line_re[MACS], line_im[MACS];
    logic signed [WIDTH-1:0] xa4_re, xa4_im;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl4 <= '0;
        end else begin
            ctrl4 <= ctrl3;
        end
    end

    always_ff @(posedge clk) begin
        xa4_re <= xa3_re;
        xa4_im <= xa3_im;
        if(ctrl3.valid) begin
            line_re[0] <= xb3_re;
            line_im[0] <= xb3_im;
            for(int d = 1; d < MACS; d++) begin
                line_re[d] <= line_re[d-1];
                line_im[d] <= line_im[d-1];
            end
        end
    end

    // Multiply: x_b * conj(x_a) = (br*ar + bi*ai) + j(bi*ar - br*ai)
    logic signed [WIDTH*2-1:0] p_rr[MACS], p_ii[MACS], p_ir[MACS], p_ri[MACS];
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl5 <= '0;
        end else begin
            ctrl5 <= ctrl4;
        end
    end

    always_ff @(posedge clk) begin
        for(int j = 0; j < MACS; j++) begin
            p_rr[j] <= line_re[MACS-1-j] * xa4_re;
            p_ii[j] <= line_im[MACS-1-j] * xa4_im;
            p_ir[j] <= line_im[MACS-1-j] * xa4_re;
            p_ri[j] <= line_re[MACS-1-j] * xa4_im;
        end
    end

    // Add the products, separately from the accumulator to keep the adders short
    logic signed [WIDTH*2:0] term_re[MACS], term_im[MACS];
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl6 <= '0;
        end else begin
            ctrl6 <= ctrl5;
        end
    end

    always_ff @(posedge clk) begin
        for(int j = 0; j < MACS; j++) begin
            term_re[j] <= p_rr[j] + p_ii[j];
            term_im[j] <= p_ir[j] - p_ri[j];
        end
    end

    // Accumulate. ACC_ZERO is signed so the terms are sign extended ('0 would make the sum unsigned)
    localparam logic signed [ACC_WIDTH-1:0] ACC_ZERO = '0;
    logic signed [ACC_WIDTH-1:0] acc_re[MACS], acc_im[MACS];
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl7 <= '0;
        end else begin
            ctrl7 <= ctrl6;
        end
    end

    always_ff @(posedge clk) begin
        if(ctrl6.valid && ctrl6.a_ok) begin
            for(int j = 0; j < MACS; j++) begin
                acc_re[j] <= (ctrl6.first ? ACC_ZERO : acc_re[j]) + term_re[j];
                acc_im[j] <= (ctrl6.first ? ACC_ZERO : acc_im[j]) + term_im[j];
            end
        end
    end


    // ------------------------------------------------------------------
    // Output: hold each group's sums and send them one lag at a time
    // ------------------------------------------------------------------
    localparam SAT_W = (ACC_WIDTH > OUT_WIDTH ? ACC_WIDTH : OUT_WIDTH) + 1;
    localparam logic signed [SAT_W-1:0] OUT_MAX = (SAT_W'(1) <<< (OUT_WIDTH-1)) - 1;
    localparam logic signed [SAT_W-1:0] OUT_MIN = -(SAT_W'(1) <<< (OUT_WIDTH-1));
    function automatic logic signed [OUT_WIDTH-1:0] scale(logic signed [ACC_WIDTH-1:0] sum);
        logic signed [SAT_W-1:0] shifted = SAT_W'(sum >>> OUT_SHIFT);
        if(shifted > OUT_MAX) begin
            return OUT_MAX[OUT_WIDTH-1:0];
        end else if(shifted < OUT_MIN) begin
            return OUT_MIN[OUT_WIDTH-1:0];
        end
        return shifted[OUT_WIDTH-1:0];
    endfunction

    localparam CNT_W = $clog2(MACS+1);
    logic signed [OUT_WIDTH-1:0] hold_re[MACS], hold_im[MACS];
    logic [CNT_W-1:0]            out_count; // Lags left to send from hold
    logic [CNT_W-1:0]            out_index;
    logic [PAIR_W-1:0]           out_pair;
    logic                        out_last_group;

    // A group's sums are done: they go into hold, which the sequencer made sure is free by
    // holding the group's last sample until it was (pending covers the sums on their way)
    logic dump, pending;
    assign dump = ctrl7.valid && ctrl7.last;

    // hold_busy == pending || out_count != 0, worked out a clock ahead into a register, since
    // it holds the sequencer
    always_ff @(posedge clk) begin
        if(~rstn) begin
            pending   <= 1'b0;
            hold_busy <= 1'b0;
        end else begin
            if(dump) begin
                pending <= 1'b0;
            end else if(issue && t_last) begin
                pending <= 1'b1;
            end

            if(dump) begin
                hold_busy <= 1'b1; // A group goes into hold
            end else begin
                hold_busy <= pending || (issue && t_last) ||
                             (dout.tvalid && dout.tready ? out_count != CNT_W'(1) : out_count != '0);
            end
        end
    end

    assign dout.tvalid = out_count != '0;
    assign dout.tdata  = {hold_re[out_index], hold_im[out_index]};
    assign dout.tid    = out_pair;
    assign dout.tlast  = out_last_group && out_count == 1;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            out_count      <= '0;
            out_index      <= '0;
            out_pair       <= '0;
            out_last_group <= 1'b0;
        end else begin
            if(dump) begin
                for(int j = 0; j < MACS; j++) begin
                    hold_re[j] <= scale(acc_re[j]);
                    hold_im[j] <= scale(acc_im[j]);
                end
                out_last_group <= ctrl7.pass == PASS_W'(PASSES-1);
                out_count      <= ctrl7.pass == PASS_W'(PASSES-1) ? CNT_W'(LAST_MACS) : CNT_W'(MACS);
                out_index      <= '0;
                out_pair       <= ctrl7.pair;
            end else if(dout.tvalid && dout.tready) begin
                out_count <= out_count - 1'b1;
                out_index <= out_index + 1'b1;
            end
        end
    end

endmodule
