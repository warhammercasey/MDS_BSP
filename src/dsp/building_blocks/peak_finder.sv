// Index of the largest magnitude in each packet of complex samples
// din:  packets of {re, im}, tlast on the last beat (e.g. cross_correlator's output)
// dout: one beat per packet: tdata = index of the beat with the largest re^2 + im^2 (the first
//       one on a tie), counting from 0, and the packet's tid
//
// The magnitude is exact. It's built from 17-bit pieces of |re| and |im| on a single 18x18
// multiplier, so each beat takes 11 clocks (din.tready is low meanwhile). It's compared with
// the packet's best so far in CMP_W-bit pieces, all at once and registered, then the pieces
// are combined the next clock, so no compare is wider than CMP_W bits.
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
    localparam STEPS = 6; // Products per beat
    localparam CMP_W = 16; // Compare piece width
    localparam CMP_N = (MAG_W + CMP_W - 1) / CMP_W;

    typedef enum logic [1:0] {
        IDLE,
        MULTIPLY,
        SPLIT,   // Compare each piece
        COMPARE  // Combine the pieces, keep the best
    } state_t;
    state_t state;

    // The beat being measured
    logic [WIDTH-1:0]     abs_re, abs_im;
    logic [IDX_W-1:0]     beat_index, next_index;
    logic                 beat_last;
    logic [TID_WIDTH-1:0] beat_tid;

    function automatic logic [WIDTH-1:0] magnitude(logic signed [WIDTH-1:0] x);
        return x[WIDTH-1] ? -x : x; // -(-2^(WIDTH-1)) is 2^(WIDTH-1), which fits unsigned
    endfunction

    assign din.tready = state == IDLE;

    // |x|^2 = lo^2 + 2*lo*hi*2^17 + hi^2*2^34, with x = hi*2^17 + lo
    logic [$clog2(STEPS+2)-1:0] step;
    logic [2*CHUNK-1:0]         re_full, im_full;
    logic [CHUNK-1:0]           re_lo, re_hi, im_lo, im_hi;
    assign re_full = (2*CHUNK)'(abs_re);
    assign im_full = (2*CHUNK)'(abs_im);
    assign {re_hi, re_lo} = re_full;
    assign {im_hi, im_lo} = im_full;

    logic signed [17:0]   mul_a, mul_b;    // Pieces are unsigned 17 bit, so they're positive here
    logic [5:0]           mul_shift, product_shift;
    logic                 mul_valid, product_valid;
    logic signed [35:0]   product;
    logic [MAG_W-1:0]     mag;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            mul_valid <= 1'b0;
        end else begin
            mul_valid <= state == MULTIPLY && step < STEPS;
            case(step)
                0:       begin mul_a <= {1'b0, re_lo}; mul_b <= {1'b0, re_lo}; mul_shift <= 6'd0;         end
                1:       begin mul_a <= {1'b0, re_lo}; mul_b <= {1'b0, re_hi}; mul_shift <= 6'(CHUNK+1);  end
                2:       begin mul_a <= {1'b0, re_hi}; mul_b <= {1'b0, re_hi}; mul_shift <= 6'(2*CHUNK);  end
                3:       begin mul_a <= {1'b0, im_lo}; mul_b <= {1'b0, im_lo}; mul_shift <= 6'd0;         end
                4:       begin mul_a <= {1'b0, im_lo}; mul_b <= {1'b0, im_hi}; mul_shift <= 6'(CHUNK+1);  end
                default: begin mul_a <= {1'b0, im_hi}; mul_b <= {1'b0, im_hi}; mul_shift <= 6'(2*CHUNK);  end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            product_valid <= 1'b0;
        end else begin
            product_valid <= mul_valid;
            product       <= mul_a * mul_b;
            product_shift <= mul_shift;
        end
    end

    // Best so far in the packet
    logic [MAG_W-1:0] best_mag;
    logic [IDX_W-1:0] best_index;
    logic             new_best;

    // mag > best_mag, a piece at a time. Neither changes between SPLIT and COMPARE: the last
    // product is in by SPLIT, and best_mag only changes leaving COMPARE.
    logic [CMP_N*CMP_W-1:0] mag_pad, best_pad;
    logic [CMP_N-1:0]       piece_gt, piece_eq;
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

    // Greater if the top differing piece is: work up from the bottom
    always_comb begin
        mag_gt = 1'b0;
        for(int k = 0; k < CMP_N; k++) begin
            mag_gt = piece_gt[k] || (piece_eq[k] && mag_gt);
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

            if(product_valid) begin
                mag <= mag + (MAG_W'(product) << product_shift);
            end

            case(state)
                IDLE: begin
                    if(din.tvalid) begin
                        abs_re     <= magnitude(din.tdata[WIDTH*2-1:WIDTH]);
                        abs_im     <= magnitude(din.tdata[WIDTH-1:0]);
                        beat_index <= next_index;
                        beat_last  <= din.tlast;
                        beat_tid   <= din.tid;
                        next_index <= din.tlast ? '0 : next_index + 1'b1;
                        mag        <= '0;
                        step       <= '0;
                        state      <= MULTIPLY;
                    end
                end

                MULTIPLY: begin
                    // The last product is added on the clock that leaves this state
                    step <= step + 1'b1;
                    if(step == STEPS + 1) begin
                        state <= SPLIT;
                    end
                end

                SPLIT: begin
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
