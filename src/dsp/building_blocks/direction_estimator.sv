// Least-squares direction of arrival from the delay between each pair of microphones
// din:  one beat per pair per window, in pair order, tid = pair (e.g. peak_finder's output from
//       cross_correlator): tdata = peak index 0..2*MAX_LAG, so the delay is index - MAX_LAG
// dout: one beat per window, after the last pair: tdata = {x, y, z}, each OUT_WIDTH bits with
//       OUT_FRAC fraction bits (rounded, saturated)
//
// direction = M * lags, the least-squares solution of A * direction = c * delays from
// model/time_correlation_model.m, with M = (c / fs) * pinv(A) worked out from the microphone
// locations by gen_direction_matrix.py into COEF_FILE (18-bit, COEF_FRAC fraction bits). Each
// pair's lag is multiplied into the 3 sums as it arrives, on one 18-bit multiplier, so a pair
// takes 9 clocks. A window's sums are output and cleared on the last pair (tid PAIRS-1).
module direction_estimator #(
    parameter CHANNELS  = 8,
    parameter MAX_LAG   = 10,
    parameter COEF_FILE = "",
    parameter COEF_FRAC = 23, // Must match gen_direction_matrix.py --coef-frac
    parameter OUT_WIDTH = 16,
    parameter OUT_FRAC  = 14  // Less than COEF_FRAC
) (
    input  logic clk,
    input  logic rstn,

    axis_tid_if.slv din,
    axis_if.mst     dout
);

    localparam PAIRS  = CHANNELS*(CHANNELS-1)/2;
    localparam PAIR_W = PAIRS > 1 ? $clog2(PAIRS) : 1;
    localparam IDX_W  = $clog2(2*MAX_LAG + 1);
    localparam LAG_W  = IDX_W + 1;
    localparam COEF_W = 18;
    localparam ACC_W  = COEF_W + LAG_W + $clog2(PAIRS + 1);
    localparam SHIFT  = COEF_FRAC - OUT_FRAC;
    localparam ROM_AW = $clog2(3*PAIRS);

    typedef enum logic [1:0] {
        IDLE,
        RUN,
        ROUND_SUMS,
        FINISH
    } state_t;
    state_t state;

    logic signed [LAG_W-1:0] lag;
    logic [PAIR_W-1:0]       pair;
    logic                    last_pair;
    logic [2:0]              step;

    // Hold the input off while a direction waits to go out
    assign din.tready = state == IDLE && !dout.tvalid;

    // Coefficients, row-major: x for every pair, then y, then z
    logic signed [COEF_W-1:0] coef;
    rom #(
        .WIDTH(COEF_W),
        .AWIDTH(ROM_AW),
        .INIT_FILE(COEF_FILE),
        .INIT_HEX(1)
    ) coef_rom (
        .clk(clk),
        .en(1'b1),
        .addr(ROM_AW'(step[1:0] * PAIRS + pair)),
        .dout(coef)
    );

    // Step k reads row k's coefficient, registers it a clock later (straight off the ROM, for
    // timing), multiplies it after 2 and adds it in after 3
    logic [1:0]                       coef_k, coef_q_k, product_k;
    logic                             coef_valid, coef_q_valid, product_valid;
    logic signed [COEF_W-1:0]         coef_q;
    logic signed [COEF_W+LAG_W-1:0]   product;
    logic signed [ACC_W-1:0]          acc[3];

    always_ff @(posedge clk) begin
        if(~rstn) begin
            coef_valid    <= 1'b0;
            coef_q_valid  <= 1'b0;
            product_valid <= 1'b0;
        end else begin
            coef_valid    <= state == RUN && step < 3;
            coef_k        <= step[1:0];
            coef_q_valid  <= coef_valid;
            coef_q_k      <= coef_k;
            product_valid <= coef_q_valid;
            product_k     <= coef_q_k;
            product       <= coef_q * lag;
        end
    end

    // No enable and no reset, so synthesis packs this into the ROM's output register
    always_ff @(posedge clk) begin
        coef_q <= coef;
    end

    // Round, then saturate a clock later. The rounded sum fits OUT_WIDTH bits if its bits from
    // the sign down to bit OUT_WIDTH-1 are all the same, which is cheaper than comparing it.
    localparam logic signed [ACC_W-1:0] ROUND = ACC_W'(1) <<< (SHIFT-1);
    localparam logic signed [OUT_WIDTH-1:0] OUT_MAX = {1'b0, {(OUT_WIDTH-1){1'b1}}};
    localparam logic signed [OUT_WIDTH-1:0] OUT_MIN = {1'b1, {(OUT_WIDTH-1){1'b0}}};
    function automatic logic signed [OUT_WIDTH-1:0] saturate(logic signed [ACC_W-1:0] rounded);
        logic [ACC_W-OUT_WIDTH:0] top = rounded[ACC_W-1:OUT_WIDTH-1];
        if(&top || ~|top) begin
            return rounded[OUT_WIDTH-1:0];
        end
        return rounded[ACC_W-1] ? OUT_MIN : OUT_MAX;
    endfunction

    logic signed [ACC_W-1:0] rounded[3];

    // Each row's sum has its own adder, with no enable or reset: the row select (decoded a
    // stage early) gates what's added, and a registered clear zeroes them after the last pair
    // and in reset. The next pair's products come well after the clear.
    localparam logic signed [COEF_W+LAG_W-1:0] PRODUCT_ZERO = '0; // Signed, so the sums stay signed
    localparam logic signed [ACC_W-1:0]        ACC_ZERO     = '0;
    logic [2:0] product_sel;
    logic       acc_clear;
    always_ff @(posedge clk) begin
        for(int k = 0; k < 3; k++) begin
            product_sel[k] <= coef_q_valid && coef_q_k == k;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            acc_clear <= 1'b1;
        end else begin
            acc_clear <= state == ROUND_SUMS && last_pair;
        end
    end

    always_ff @(posedge clk) begin
        for(int k = 0; k < 3; k++) begin
            acc[k] <= (acc_clear ? ACC_ZERO : acc[k]) + (product_sel[k] ? product : PRODUCT_ZERO);
        end
    end

    always_ff @(posedge clk) begin
        if(state == ROUND_SUMS) begin
            for(int k = 0; k < 3; k++) begin
                rounded[k] <= (acc[k] + ROUND) >>> SHIFT;
            end
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            state       <= IDLE;
            step        <= '0;
            dout.tvalid <= 1'b0;
            dout.tdata  <= '0;
        end else begin
            if(dout.tvalid && dout.tready) begin
                dout.tvalid <= 1'b0;
            end

            case(state)
                IDLE: begin
                    if(din.tvalid && din.tready) begin
                        lag       <= $signed({1'b0, din.tdata[IDX_W-1:0]}) - LAG_W'(MAX_LAG);
                        pair      <= din.tid;
                        last_pair <= din.tid == PAIR_W'(PAIRS-1);
                        step      <= '0;
                        state     <= RUN;
                    end
                end

                RUN: begin
                    // The last product is added on the clock that leaves this state
                    step <= step + 1'b1;
                    if(step == 3'd5) begin
                        state <= ROUND_SUMS;
                    end
                end

                ROUND_SUMS: begin
                    state <= FINISH;
                end

                FINISH: begin
                    if(last_pair) begin
                        dout.tvalid <= 1'b1;
                        dout.tdata  <= {saturate(rounded[0]), saturate(rounded[1]), saturate(rounded[2])};
                    end
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
