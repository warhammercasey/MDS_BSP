// Index of the largest magnitude in each packet of complex samples
// din:  packets of {re, im}, tlast on the last beat (e.g. cross_correlator's output)
// dout: one beat per packet: tdata = index of the beat with the largest re^2 + im^2 (the first
//       one on a tie), counting from 0, and the packet's tid
//
// The magnitude is exact. It's built from 17-bit pieces of |re| and |im| on a single 18x18
// multiplier, and accumulated in ACC_P-bit pieces with the carries between them registered.
// It's compared with the packet's best so far in CMP_W-bit pieces, all at once and registered,
// then the pieces are combined in registered groups of 4, so no add or compare is wide.
// Each beat takes STEPS + ACC_N + 9 clocks, 19 at WIDTH 32 (din.tready is low meanwhile).
module peak_finder #(
    parameter WIDTH = 32,    // re and im width, up to 34
    parameter MAX_LEN = 21,  // Longest packet: sets the index width
    parameter TID_WIDTH = 5
) (
    input  logic clk,
    input  logic rstn,

    axis_tid_tlast_if.slv din,
    axis_tid_if.mst       dout
);

    localparam IDX_W = MAX_LEN > 1 ? $clog2(MAX_LEN) : 1;
    localparam MAG_W = 2*WIDTH;
    localparam CHUNK = 17;
    localparam STEPS = 6;   // Products per beat
    localparam ACC_P = 16;  // Accumulator piece width (adders use the carry chain)
    localparam ACC_N = (MAG_W + ACC_P - 1) / ACC_P;
    localparam ACC_W = ACC_N * ACC_P;
    localparam CMP_W = 8;   // Compare piece width (compares are built from LUTs)
    localparam CMP_N = (MAG_W + CMP_W - 1) / CMP_W;
    localparam GRP_N = (CMP_N + 3) / 4; // Groups of 4 pieces, at most 4 groups for MAG_W <= 128

    typedef enum logic [2:0] {
        IDLE,
        ABS_LO,   // |re|, |im|: the bottom half, with its carry
        ABS_HI,   // and the top half
        MULTIPLY,
        SPLIT,    // Compare each piece
        GROUP,    // Combine the pieces in groups of 4
        COMPARE   // Combine the groups, keep the best
    } state_t;
    state_t state;

    // The beat being measured
    logic [WIDTH-1:0]     raw_re, raw_im;
    logic [WIDTH-1:0]     abs_re, abs_im;
    logic [IDX_W-1:0]     beat_index, next_index;
    logic                 beat_last;
    logic [TID_WIDTH-1:0] beat_tid;

    // |x| = (x ^ s) + s with s the sign, in two halves so neither add is long.
    // -(-2^(WIDTH-1)) is 2^(WIDTH-1), which fits unsigned.
    localparam ABS_LO_W = WIDTH / 2;
    localparam ABS_HI_W = WIDTH - ABS_LO_W;
    logic carry_re, carry_im;

    assign din.tready = state == IDLE;

    // |x|^2 = lo^2 + 2*lo*hi*2^17 + hi^2*2^34, with x = hi*2^17 + lo
    // The last product is added STEPS+3 clocks into MULTIPLY, then the accumulator's carries
    // take ACC_N-1 more to settle
    localparam LAST_STEP = STEPS + ACC_N + 2;
    logic [$clog2(LAST_STEP+1)-1:0] step;
    logic [2*CHUNK-1:0]         re_full, im_full;
    logic [CHUNK-1:0]           re_lo, re_hi, im_lo, im_hi;
    assign re_full = (2*CHUNK)'(abs_re);
    assign im_full = (2*CHUNK)'(abs_im);
    assign {re_hi, re_lo} = re_full;
    assign {im_hi, im_lo} = im_full;

    // Where each product goes: lo*lo at 0, 2*lo*hi at 2^17 (shifted 18), hi*hi at 2^34
    typedef enum logic [1:0] {
        AT_0,
        AT_18,
        AT_34
    } place_t;

    logic signed [17:0]   mul_a, mul_b;    // Pieces are unsigned 17 bit, so they're positive here
    place_t               mul_place, product_place;
    logic                 mul_valid, product_valid;
    logic signed [35:0]   product;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            mul_valid <= 1'b0;
        end else begin
            mul_valid <= state == MULTIPLY && step < STEPS;
            case(step)
                0:       begin mul_a <= {1'b0, re_lo}; mul_b <= {1'b0, re_lo}; mul_place <= AT_0;  end
                1:       begin mul_a <= {1'b0, re_lo}; mul_b <= {1'b0, re_hi}; mul_place <= AT_18; end
                2:       begin mul_a <= {1'b0, re_hi}; mul_b <= {1'b0, re_hi}; mul_place <= AT_34; end
                3:       begin mul_a <= {1'b0, im_lo}; mul_b <= {1'b0, im_lo}; mul_place <= AT_0;  end
                4:       begin mul_a <= {1'b0, im_lo}; mul_b <= {1'b0, im_hi}; mul_place <= AT_18; end
                default: begin mul_a <= {1'b0, im_hi}; mul_b <= {1'b0, im_hi}; mul_place <= AT_34; end
            endcase
        end
    end

    // Register again, so the multiplier's input registers (packed into it) are fed straight
    // from flip flops, not from the piece select
    logic signed [17:0] mul_a_q, mul_b_q;
    place_t             mul_place_q;
    logic               mul_valid_q;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            mul_valid_q <= 1'b0;
        end else begin
            mul_valid_q <= mul_valid;
            mul_a_q     <= mul_a;
            mul_b_q     <= mul_b;
            mul_place_q <= mul_place;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            product_valid <= 1'b0;
        end else begin
            product_valid <= mul_valid_q;
            product       <= mul_a_q * mul_b_q;
            product_place <= mul_place_q;
        end
    end

    // Shift the product into place, registered: a 3-way select, not a barrel shifter
    logic [ACC_W-1:0] addend;
    always_ff @(posedge clk) begin
        if(~rstn || !product_valid) begin
            addend <= '0;
        end else begin
            case(product_place)
                AT_0:    addend <= ACC_W'(product);
                AT_18:   addend <= ACC_W'(product) << (CHUNK+1);
                default: addend <= ACC_W'(product) << (2*CHUNK);
            endcase
        end
    end

    // Accumulate in ACC_P-bit pieces, each piece's carry added into the next piece a clock
    // later, so no add is wider than ACC_P bits. The top carry is dropped: |x|^2 fits in MAG_W.
    logic [ACC_P-1:0] acc[ACC_N];
    logic [ACC_N-1:0] acc_carry;
    logic [ACC_W-1:0] acc_flat;
    logic [MAG_W-1:0] mag;
    always_comb begin
        for(int k = 0; k < ACC_N; k++) begin
            acc_flat[k*ACC_P +: ACC_P] = acc[k];
        end
    end
    assign mag = acc_flat[MAG_W-1:0];

    always_ff @(posedge clk) begin
        if(state == IDLE) begin
            acc       <= '{default: '0};
            acc_carry <= '0;
        end else if(state == MULTIPLY) begin
            for(int k = 0; k < ACC_N; k++) begin
                {acc_carry[k], acc[k]} <= {1'b0, acc[k]} + {1'b0, addend[k*ACC_P +: ACC_P]}
                                        + (k == 0 ? (ACC_P+1)'(0) : (ACC_P+1)'(acc_carry[k == 0 ? 0 : k-1]));
            end
        end
    end

    // Best so far in the packet
    logic [MAG_W-1:0] best_mag;
    logic [IDX_W-1:0] best_index;
    logic             new_best;

    // mag > best_mag, a piece at a time. Neither changes from SPLIT to COMPARE: the last carry
    // is in by SPLIT, and best_mag only changes leaving COMPARE.
    logic [CMP_N*CMP_W-1:0] mag_pad, best_pad;
    logic [CMP_N-1:0]       piece_gt, piece_eq;
    logic [GRP_N-1:0]       group_gt, group_eq;
    logic                   mag_gt;
    assign mag_pad  = (CMP_N*CMP_W)'(mag);
    assign best_pad = (CMP_N*CMP_W)'(best_mag);

    always_ff @(posedge clk) begin
        if(state == SPLIT) begin
            for(int k = 0; k < CMP_N; k++) begin
                piece_gt[k] <= mag_pad[k*CMP_W +: CMP_W] >  best_pad[k*CMP_W +: CMP_W];
                piece_eq[k] <= mag_pad[k*CMP_W +: CMP_W] == best_pad[k*CMP_W +: CMP_W];
            end
        end
    end

    // Greater if the top differing piece is: work up from the bottom, 4 pieces per group
    logic [GRP_N-1:0] group_gt_next, group_eq_next;
    always_comb begin
        for(int g = 0; g < GRP_N; g++) begin
            group_gt_next[g] = 1'b0;
            group_eq_next[g] = 1'b1;
            for(int k = 4*g; k < 4*g + 4; k++) begin
                if(k < CMP_N) begin
                    group_gt_next[g] = piece_gt[k] || (piece_eq[k] && group_gt_next[g]);
                    group_eq_next[g] = group_eq_next[g] && piece_eq[k];
                end
            end
        end
    end

    always_ff @(posedge clk) begin
        if(state == GROUP) begin
            group_gt <= group_gt_next;
            group_eq <= group_eq_next;
        end
    end

    always_comb begin
        mag_gt = 1'b0;
        for(int g = 0; g < GRP_N; g++) begin
            mag_gt = group_gt[g] || (group_eq[g] && mag_gt);
        end
    end

    assign new_best = beat_index == '0 || mag_gt;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            state       <= IDLE;
            step        <= '0;
            next_index  <= '0;
            best_mag    <= '0;
            best_index  <= '0;
            dout.tvalid <= 1'b0;
            dout.tdata  <= '0;
            dout.tid    <= '0;
        end else begin
            if(dout.tvalid && dout.tready) begin
                dout.tvalid <= 1'b0;
            end

            case(state)
                IDLE: begin
                    if(din.tvalid) begin
                        raw_re     <= din.tdata[WIDTH*2-1:WIDTH];
                        raw_im     <= din.tdata[WIDTH-1:0];
                        beat_index <= next_index;
                        beat_last  <= din.tlast;
                        beat_tid   <= din.tid;
                        next_index <= din.tlast ? '0 : next_index + 1'b1;
                        state      <= ABS_LO;
                    end
                end

                ABS_LO: begin
                    {carry_re, abs_re[ABS_LO_W-1:0]} <= {1'b0, raw_re[ABS_LO_W-1:0] ^ {ABS_LO_W{raw_re[WIDTH-1]}}}
                                                      + (ABS_LO_W+1)'(raw_re[WIDTH-1]);
                    {carry_im, abs_im[ABS_LO_W-1:0]} <= {1'b0, raw_im[ABS_LO_W-1:0] ^ {ABS_LO_W{raw_im[WIDTH-1]}}}
                                                      + (ABS_LO_W+1)'(raw_im[WIDTH-1]);
                    state <= ABS_HI;
                end

                ABS_HI: begin
                    abs_re[WIDTH-1:ABS_LO_W] <= (raw_re[WIDTH-1:ABS_LO_W] ^ {ABS_HI_W{raw_re[WIDTH-1]}}) + ABS_HI_W'(carry_re);
                    abs_im[WIDTH-1:ABS_LO_W] <= (raw_im[WIDTH-1:ABS_LO_W] ^ {ABS_HI_W{raw_im[WIDTH-1]}}) + ABS_HI_W'(carry_im);
                    step  <= '0;
                    state <= MULTIPLY;
                end

                MULTIPLY: begin
                    // The last carry settles on the clock that leaves this state
                    step <= step + 1'b1;
                    if(step == LAST_STEP) begin
                        state <= SPLIT;
                    end
                end

                SPLIT: begin
                    state <= GROUP;
                end

                GROUP: begin
                    state <= COMPARE;
                end

                COMPARE: begin
                    // A packet's result waits for the last one to be taken
                    if(!beat_last || !dout.tvalid || dout.tready) begin
                        if(new_best) begin
                            best_mag   <= mag;
                            best_index <= beat_index;
                        end
                        if(beat_last) begin
                            dout.tvalid <= 1'b1;
                            dout.tdata  <= new_best ? beat_index : best_index;
                            dout.tid    <= beat_tid;
                        end
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
