// Multichannel real to complex mixer
// Mixes each sample with exp(-j*2*pi*TONE_FREQ/SAMPLE_RATE*n), moving +TONE_FREQ to DC:
//   I = round(x*cos / 2^17), Q = round(-x*sin / 2^17), dout.tdata = {I, Q}, tid passed through
// Each TID has its own phase, counting that channel's samples from 0 after reset, so channels
// sampled together are mixed with the same LO phase whatever order they arrive in.
// Two multipliers shared by all channels, one sample per clock, full backpressure. Latency 5.
//
// The LO comes from a quarter-wave {cos, sin} table in INIT_FILE (see gen_mixer_lut.py), so
// it has 2^(LUT_AWIDTH+2) points per cycle. The phase is truncated to that, which delays the
// LO by up to one point: a fixed phase offset, the same for every channel.
module mixer #(
    parameter WIDTH = 18,           // Input sample width. The output is complex, WIDTH*2
    parameter TID_WIDTH = $clog2(8),
    parameter SAMPLE_RATE = 48_000, // Only TONE_FREQ / SAMPLE_RATE matters: both can be scaled
    parameter TONE_FREQ = 1_000,    // (e.g. in mHz) for precision. Can be negative, |TONE_FREQ| < 2^31
    parameter PHASE_WIDTH = 32,     // LUT_AWIDTH+2 to 32. Sets the LO frequency step, SAMPLE_RATE / 2^PHASE_WIDTH
    parameter LUT_AWIDTH = 8,
    parameter INIT_FILE = "",
    parameter INIT_HEX = 1
) (
    input  logic clk,
    input  logic rstn,

    axis_tid_if.slv din,
    axis_tid_if.mst dout
);

    localparam LO_WIDTH = 18;
    localparam CHANNELS = (1 << TID_WIDTH);

    // Phase step per sample: round(TONE_FREQ / SAMPLE_RATE * 2^PHASE_WIDTH), wrapped
    localparam longint PHASE_INC_FULL = ((longint'(TONE_FREQ) <<< PHASE_WIDTH)
                                        + (TONE_FREQ < 0 ? -longint'(SAMPLE_RATE / 2) : longint'(SAMPLE_RATE / 2)))
                                        / longint'(SAMPLE_RATE);
    localparam logic [PHASE_WIDTH-1:0] PHASE_INC = PHASE_WIDTH'(PHASE_INC_FULL);

    // Every stage moves on together, and holds while the output is stalled
    logic advance;
    assign advance = !dout.tvalid || dout.tready;
    assign din.tready = advance;

    // Per channel phase
    logic [PHASE_WIDTH-1:0] phase[CHANNELS];
    always_ff @(posedge clk) begin
        if(~rstn) begin
            phase <= '{default: '0};
        end else begin
            if(din.tvalid && din.tready) begin
                phase[din.tid] <= phase[din.tid] + PHASE_INC;
            end
        end
    end

    // Look up the LO: the top 2 phase bits are the quadrant, the next LUT_AWIDTH the table entry
    logic [LUT_AWIDTH+1:0] lut_phase;
    logic [LO_WIDTH*2-1:0] lut_out;
    assign lut_phase = phase[din.tid][PHASE_WIDTH-1 -: LUT_AWIDTH+2];
    rom #(
        .WIDTH(LO_WIDTH*2),
        .AWIDTH(LUT_AWIDTH),
        .INIT_FILE(INIT_FILE),
        .INIT_HEX(INIT_HEX)
    ) lo_rom (
        .clk(clk),
        .en(advance),
        .addr(lut_phase[LUT_AWIDTH-1:0]),
        .dout(lut_out)
    );

    logic                    s1_valid;
    logic [TID_WIDTH-1:0]    s1_tid;
    logic signed [WIDTH-1:0] s1_x;
    logic [1:0]              s1_quadrant;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            s1_valid <= 1'b0;
        end else if(advance) begin
            s1_valid    <= din.tvalid;
            s1_tid      <= din.tid;
            s1_x        <= din.tdata;
            s1_quadrant <= lut_phase[LUT_AWIDTH+1 -: 2];
        end
    end

    // Register the ROM output for timing, before any logic
    logic                       s1q_valid;
    logic [TID_WIDTH-1:0]       s1q_tid;
    logic signed [WIDTH-1:0]    s1q_x;
    logic [1:0]                 s1q_quadrant;
    logic [LO_WIDTH*2-1:0]      lut_q;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            s1q_valid <= 1'b0;
        end else if(advance) begin
            s1q_valid    <= s1_valid;
            s1q_tid      <= s1_tid;
            s1q_x        <= s1_x;
            s1q_quadrant <= s1_quadrant;
            lut_q        <= lut_out;
        end
    end

    // Rotate the table's first quadrant {cos, sin} into the right quadrant, and conjugate:
    // lo = cos - j*sin
    logic signed [LO_WIDTH-1:0] lut_cos, lut_sin;
    assign lut_cos = lut_q[LO_WIDTH*2-1:LO_WIDTH];
    assign lut_sin = lut_q[LO_WIDTH-1:0];

    logic                       s2_valid;
    logic [TID_WIDTH-1:0]       s2_tid;
    logic signed [WIDTH-1:0]    s2_x;
    logic signed [LO_WIDTH-1:0] lo_i, lo_q;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            s2_valid <= 1'b0;
        end else if(advance) begin
            s2_valid <= s1q_valid;
            s2_tid   <= s1q_tid;
            s2_x     <= s1q_x;
            case(s1q_quadrant)
                2'd0: begin lo_i <=  lut_cos; lo_q <= -lut_sin; end
                2'd1: begin lo_i <= -lut_sin; lo_q <= -lut_cos; end
                2'd2: begin lo_i <= -lut_cos; lo_q <=  lut_sin; end
                2'd3: begin lo_i <=  lut_sin; lo_q <=  lut_cos; end
            endcase
        end
    end

    // Multiply
    logic                             s3_valid;
    logic [TID_WIDTH-1:0]             s3_tid;
    logic signed [WIDTH+LO_WIDTH-1:0] product_i, product_q;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            s3_valid <= 1'b0;
        end else if(advance) begin
            s3_valid  <= s2_valid;
            s3_tid    <= s2_tid;
            product_i <= s2_x * lo_i;
            product_q <= s2_x * lo_q;
        end
    end

    // Round back to WIDTH bits. |lo| < 2^17, so this can't overflow
    localparam logic signed [WIDTH+LO_WIDTH-1:0] ROUND = 1 <<< (LO_WIDTH-2);
    logic signed [WIDTH+LO_WIDTH-1:0] rounded_i, rounded_q;
    assign rounded_i = (product_i + ROUND) >>> (LO_WIDTH-1);
    assign rounded_q = (product_q + ROUND) >>> (LO_WIDTH-1);

    always_ff @(posedge clk) begin
        if(~rstn) begin
            dout.tvalid <= 1'b0;
            dout.tdata  <= '0;
            dout.tid    <= '0;
        end else if(advance) begin
            dout.tvalid <= s3_valid;
            dout.tid    <= s3_tid;
            dout.tdata  <= {rounded_i[WIDTH-1:0], rounded_q[WIDTH-1:0]};
        end
    end

endmodule
