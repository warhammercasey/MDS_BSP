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
// takes 8 clocks. A window's sums are output and cleared on the last pair (tid PAIRS-1).
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
            coef_q        <= coef;
            product_valid <= coef_q_valid;
            product_k     <= coef_q_k;
            product       <= coef_q * lag;
        end
    end

    localparam logic signed [ACC_W-1:0] ROUND   = ACC_W'(1) <<< (SHIFT-1);
    localparam logic signed [ACC_W-1:0] OUT_MAX = (ACC_W'(1) <<< (OUT_WIDTH-1)) - 1;
    localparam logic signed [ACC_W-1:0] OUT_MIN = -(ACC_W'(1) <<< (OUT_WIDTH-1));
    function automatic logic signed [OUT_WIDTH-1:0] scale(logic signed [ACC_W-1:0] sum);
        logic signed [ACC_W-1:0] rounded = (sum + ROUND) >>> SHIFT;
        if(rounded > OUT_MAX) begin
            return OUT_MAX[OUT_WIDTH-1:0];
        end else if(rounded < OUT_MIN) begin
            return OUT_MIN[OUT_WIDTH-1:0];
        end
        return rounded[OUT_WIDTH-1:0];
    endfunction

    always_ff @(posedge clk) begin
        if(~rstn) begin
            state       <= IDLE;
            step        <= '0;
            acc         <= '{default: '0};
            dout.tvalid <= 1'b0;
            dout.tdata  <= '0;
        end else begin
            if(dout.tvalid && dout.tready) begin
                dout.tvalid <= 1'b0;
            end

            if(product_valid) begin
                acc[product_k] <= acc[product_k] + product;
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
                        state <= FINISH;
                    end
                end

                FINISH: begin
                    if(last_pair) begin
                        dout.tvalid <= 1'b1;
                        dout.tdata  <= {scale(acc[0]), scale(acc[1]), scale(acc[2])};
                        acc         <= '{default: '0};
                    end
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
