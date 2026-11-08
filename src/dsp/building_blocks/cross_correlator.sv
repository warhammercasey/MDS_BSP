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
    logic [FILL_W-1:0] fill[CHANNELS];
    logic [AW-1:0]     write_ptr[CHANNELS];
    logic              release_window; // Done with the current window
    logic              din_in_range;
    logic              din_write;

    assign din_in_range = din.tid < CHANNELS;
    assign din.tready   = !din_in_range || fill[din.tid] < FILL_W'(2*N);
    assign din_write    = din.tvalid && din.tready && din_in_range;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            fill      <= '{default: '0};
            write_ptr <= '{default: '0};
        end else begin
            for(int c = 0; c < CHANNELS; c++) begin
                fill[c] <= fill[c] + FILL_W'(din_write && din.tid == c) - (release_window ? FILL_W'(N) : '0);
                if(din_write && din.tid == c) begin
                    write_ptr[c] <= inc(write_ptr[c]);
                end
            end
        end
    end

    logic window_ready;
    always_comb begin
        window_ready = 1'b1;
        for(int c = 0; c < CHANNELS; c++) begin
            if(fill[c] < FILL_W'(N)) begin
                window_ready = 1'b0;
            end
        end
    end


    // ------------------------------------------------------------------
    // Sequencing: for each pair, for each group of MACS lags, stream the window through
    // ------------------------------------------------------------------
    typedef enum logic {
        IDLE,
        RUN
    } state_t;
    state_t state;

    logic advance; // Every pipeline stage moves on together, and holds while the output is full

    logic [AW-1:0]            start_ptr;  // Window start in the ring buffers
    logic [SKIP_W-1:0]        skip;
    logic [T_W-1:0]           t;          // Position in the group's stream
    logic [PASS_W-1:0]        pass;
    logic signed [M0_W-1:0]   m0;         // First lag of the group
    logic [PAIR_W-1:0]        pair;
    logic [CH_W-1:0]          chan_a, chan_b;
    logic [AW-1:0]            a_ptr, b_ptr;

    logic run_last;
    assign run_last = state == RUN && t == T_W'(T_LAST) && pass == PASS_W'(PASSES-1) && pair == PAIR_W'(PAIRS-1);
    assign release_window = (state == IDLE && window_ready && skip != '0) || (run_last && advance);

    always_ff @(posedge clk) begin
        if(~rstn) begin
            state     <= IDLE;
            start_ptr <= AW'(START);
            skip      <= SKIP_W'(SKIP);
            t         <= '0;
            pass      <= '0;
            m0        <= M0_W'(-MAX_LAG);
            pair      <= '0;
            chan_a    <= '0;
            chan_b    <= CH_W'(1);
            a_ptr     <= '0;
            b_ptr     <= '0;
        end else begin
            if(release_window) begin
                start_ptr <= wrap(ptr(start_ptr) + ptr_sum_t'(N));
            end

            case(state)
                IDLE: begin
                    if(window_ready) begin
                        if(skip != '0) begin
                            skip <= skip - 1'b1;
                        end else begin
                            state  <= RUN;
                            t      <= '0;
                            pass   <= '0;
                            m0     <= M0_W'(-MAX_LAG);
                            pair   <= '0;
                            chan_a <= '0;
                            chan_b <= CH_W'(1);
                            // x_b streams from the group's first lag, x_a from MACS-1 samples
                            // before the window so it lines up with the end of the delay line
                            b_ptr  <= wrap(ptr(start_ptr) - ptr_sum_t'(MAX_LAG));
                            a_ptr  <= wrap(ptr(start_ptr) - ptr_sum_t'(MACS-1));
                        end
                    end
                end

                RUN: if(advance) begin
                    if(t == T_W'(T_LAST)) begin
                        t     <= '0;
                        a_ptr <= wrap(ptr(start_ptr) - ptr_sum_t'(MACS-1));
                        if(pass == PASS_W'(PASSES-1)) begin
                            pass  <= '0;
                            m0    <= M0_W'(-MAX_LAG);
                            b_ptr <= wrap(ptr(start_ptr) - ptr_sum_t'(MAX_LAG));
                            pair  <= pair + 1'b1;
                            if(chan_b == CH_W'(CHANNELS-1)) begin
                                chan_a <= chan_a + 1'b1;
                                chan_b <= chan_a + CH_W'(2);
                            end else begin
                                chan_b <= chan_b + 1'b1;
                            end
                            if(run_last) begin
                                state <= IDLE;
                            end
                        end else begin
                            pass  <= pass + 1'b1;
                            m0    <= m0 + M0_W'(MACS);
                            b_ptr <= wrap(ptr(start_ptr) + ptr_sum_t'(m0) + ptr_sum_t'(MACS));
                        end
                    end else begin
                        t     <= t + 1'b1;
                        a_ptr <= inc(a_ptr);
                        b_ptr <= inc(b_ptr);
                    end
                end
            endcase
        end
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
        ctrl0.valid  = state == RUN;
        ctrl0.a_ok   = t >= T_W'(MACS-1);
        ctrl0.b_ok   = kb >= 0 && kb < KB_W'(L);
        ctrl0.first  = t == T_W'(MACS-1);
        ctrl0.last   = t == T_W'(T_LAST);
        ctrl0.chan_a = chan_a;
        ctrl0.chan_b = chan_b;
        ctrl0.pair   = pair;
        ctrl0.pass   = pass;
    end


    // ------------------------------------------------------------------
    // Ring buffers
    // ------------------------------------------------------------------
    logic [WIDTH*2-1:0] buffer_out[CHANNELS];
    generate
        for(genvar c = 0; c < CHANNELS; c++) begin : g_buffer
            simple_dual_port_bram #(
                .WIDTH(WIDTH*2), // *2 for complex samples
                .AWIDTH(AW),
                .DEPTH(DEPTH)
            ) ring (
                .clka(clk),
                .wea(din_write && din.tid == c),
                .ena(1'b1),
                .addra(write_ptr[c]),
                .dina(din.tdata),

                .clkb(clk),
                .enb(advance),
                .addrb(chan_b == c ? b_ptr : a_ptr),
                .doutb(buffer_out[c])
            );
        end
    endgenerate


    // ------------------------------------------------------------------
    // Pipeline
    // ------------------------------------------------------------------
    // Register every buffer's output for timing, before the channel select
    logic [WIDTH*2-1:0] buffer_q[CHANNELS];
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl1 <= '0;
            ctrl2 <= '0;
        end else if(advance) begin
            ctrl1 <= ctrl0;
            ctrl2 <= ctrl1;
        end
    end

    always_ff @(posedge clk) begin
        if(advance) begin
            buffer_q <= buffer_out;
        end
    end

    // Pick out the pair's samples
    logic signed [WIDTH-1:0] xa3_re, xa3_im, xb3_re, xb3_im;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl3 <= '0;
        end else if(advance) begin
            ctrl3 <= ctrl2;
            {xa3_re, xa3_im} <= buffer_q[ctrl2.chan_a];
            {xb3_re, xb3_im} <= ctrl2.b_ok ? buffer_q[ctrl2.chan_b] : '0;
        end
    end

    // Delay line: lane j works on lag m0+j, which needs x_b MACS-1-j samples back
    logic signed [WIDTH-1:0] line_re[MACS], line_im[MACS];
    logic signed [WIDTH-1:0] xa4_re, xa4_im;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl4 <= '0;
        end else if(advance) begin
            ctrl4      <= ctrl3;
            xa4_re     <= xa3_re;
            xa4_im     <= xa3_im;
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
        end else if(advance) begin
            ctrl5 <= ctrl4;
            for(int j = 0; j < MACS; j++) begin
                p_rr[j] <= line_re[MACS-1-j] * xa4_re;
                p_ii[j] <= line_im[MACS-1-j] * xa4_im;
                p_ir[j] <= line_im[MACS-1-j] * xa4_re;
                p_ri[j] <= line_re[MACS-1-j] * xa4_im;
            end
        end
    end

    // Add the products, separately from the accumulator to keep the adders short
    logic signed [WIDTH*2:0] term_re[MACS], term_im[MACS];
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl6 <= '0;
        end else if(advance) begin
            ctrl6 <= ctrl5;
            for(int j = 0; j < MACS; j++) begin
                term_re[j] <= p_rr[j] + p_ii[j];
                term_im[j] <= p_ir[j] - p_ri[j];
            end
        end
    end

    // Accumulate. ACC_ZERO is signed so the terms are sign extended ('0 would make the sum unsigned)
    localparam logic signed [ACC_WIDTH-1:0] ACC_ZERO = '0;
    logic signed [ACC_WIDTH-1:0] acc_re[MACS], acc_im[MACS];
    always_ff @(posedge clk) begin
        if(~rstn) begin
            ctrl7 <= '0;
        end else if(advance) begin
            ctrl7 <= ctrl6;
            if(ctrl6.valid && ctrl6.a_ok) begin
                for(int j = 0; j < MACS; j++) begin
                    acc_re[j] <= (ctrl6.first ? ACC_ZERO : acc_re[j]) + term_re[j];
                    acc_im[j] <= (ctrl6.first ? ACC_ZERO : acc_im[j]) + term_im[j];
                end
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

    logic dump;
    assign dump    = ctrl7.valid && ctrl7.last;
    assign advance = !(dump && out_count != '0); // The last group is still going out

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
            if(dump && advance) begin
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
