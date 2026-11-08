`timescale 1ns/1ps
`include "tb_macros.svh"

// direction_estimator test: one peak index per pair per window in, a least-squares direction
// out per window.
//
// Checked:
//   - the coefficient file from gen_direction_matrix.py against (c / fs) * pinv(A) worked out
//     here from the same microphone locations (within half an LSB)
//   - every output bit-exact against a model using the file's coefficients, and against the
//     real-valued least-squares solution from pinv (within 1.5 output LSBs, unless saturated)
//   - plane waves from random directions u, with each pair's lag the rounded delay
//     (loc_a - loc_b) . u * fs / c: the direction must come out within the angle that lags up
//     to half a sample off can account for (about 9 degrees for these arrays)
//   - random lags across the full range, and in one configuration sums big enough to saturate
//   - the output holds while stalled, nothing extra or missing
module direction_estimator_inst #(
    parameter int  CHANNELS  = 8,
    parameter int  D         = 10,
    parameter int  GEOMETRY  = 0,   // 0: the cube in gen_direction_matrix.py, 1: the test tetrahedron
    parameter      COEF_FILE = "",  // untyped: see tb_fir_decimator
    parameter int  COEF_FRAC = 23,
    parameter int  OUT_WIDTH = 16,
    parameter int  OUT_FRAC  = 14,
    parameter int  WINDOWS   = 300
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int  PAIRS  = CHANNELS * (CHANNELS - 1) / 2;
    localparam int  PAIR_W = PAIRS > 1 ? $clog2(PAIRS) : 1;
    localparam int  IDX_W  = $clog2(2 * D + 1);
    localparam int  SHIFT  = COEF_FRAC - OUT_FRAC;
    localparam real C      = 343.0;
    localparam real FS     = 73_571_400.0 / (24 * 64) / 4;  // as gen_direction_matrix.py
    localparam real PI     = 3.14159265358979;

    logic rstn = 1'b0;

    axis_tid_if #(.WIDTH(IDX_W), .ID_WIDTH(PAIR_W)) din ();
    axis_if     #(.WIDTH(3 * OUT_WIDTH))            dout ();

    direction_estimator #(
        .CHANNELS(CHANNELS),
        .MAX_LAG(D),
        .COEF_FILE(COEF_FILE),
        .COEF_FRAC(COEF_FRAC),
        .OUT_WIDTH(OUT_WIDTH),
        .OUT_FRAC(OUT_FRAC)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .din(din),
        .dout(dout)
    );

    string NAME = $sformatf("%0d mics, +/-%0d, out Q%0d.%0d", CHANNELS, D, OUT_WIDTH - OUT_FRAC, OUT_FRAC);

    // ------------------------------------------------------------------
    // Geometry and least squares, worked out here
    // ------------------------------------------------------------------
    real loc [CHANNELS][3];
    real pinv_m [3][PAIRS];  // (c / fs) * pinv(A)
    real max_angle;          // worst case direction error from lags rounded to whole samples
    int  pair_a [PAIRS], pair_b [PAIRS];

    initial begin
        real s = 0.08;
        real a [PAIRS][3];
        real ata [3][3], inv [3][3], det;
        for (int c = 0; c < CHANNELS; c++)
            if (GEOMETRY == 0) begin
                // Same order as MIC_LOCATIONS / the MATLAB model
                localparam int CX [8] = '{-1, -1, -1, -1,  1,  1,  1,  1};
                localparam int CY [8] = '{-1,  1, -1,  1, -1,  1, -1,  1};
                localparam int CZ [8] = '{-1, -1,  1,  1,  1,  1, -1, -1};
                loc[c] = '{s * CX[c], s * CY[c], s * CZ[c]};
            end else begin
                localparam int TX [4] = '{1,  1, -1, -1};
                localparam int TY [4] = '{1, -1,  1, -1};
                localparam int TZ [4] = '{1, -1, -1,  1};
                loc[c] = '{s * TX[c], s * TY[c], s * TZ[c]};
            end
        begin
            automatic int p = 0;
            for (int i = 0; i < CHANNELS - 1; i++)
                for (int j = i + 1; j < CHANNELS; j++) begin
                    pair_a[p] = i;
                    pair_b[p] = j;
                    for (int k = 0; k < 3; k++) a[p][k] = loc[i][k] - loc[j][k];
                    p++;
                end
        end
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) begin
                ata[i][j] = 0.0;
                for (int p = 0; p < PAIRS; p++) ata[i][j] += a[p][i] * a[p][j];
            end
        det = ata[0][0] * (ata[1][1] * ata[2][2] - ata[1][2] * ata[2][1])
            - ata[0][1] * (ata[1][0] * ata[2][2] - ata[1][2] * ata[2][0])
            + ata[0][2] * (ata[1][0] * ata[2][1] - ata[1][1] * ata[2][0]);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) begin
                // inverse = cofactor transpose / det
                automatic int r0 = (j + 1) % 3, r1 = (j + 2) % 3, c0 = (i + 1) % 3, c1 = (i + 2) % 3;
                inv[i][j] = (ata[r0][c0] * ata[r1][c1] - ata[r0][c1] * ata[r1][c0]) / det;
            end
        for (int k = 0; k < 3; k++)
            for (int p = 0; p < PAIRS; p++) begin
                pinv_m[k][p] = 0.0;
                for (int j = 0; j < 3; j++) pinv_m[k][p] += C / FS * inv[k][j] * a[p][j];
            end
        // Rounding each lag to a sample is up to 0.5 off, which moves each component by up
        // to 0.5 * sum |M[k][p]|: the largest angle that can turn a unit vector by
        begin
            automatic real err = 0.0;
            for (int k = 0; k < 3; k++) begin
                automatic real row = 0.0;
                for (int p = 0; p < PAIRS; p++) row += 0.5 * (pinv_m[k][p] < 0 ? -pinv_m[k][p] : pinv_m[k][p]);
                err += row ** 2;
            end
            err = $sqrt(err) + 3.0 / 2.0 ** OUT_FRAC;
            max_angle = $asin(err < 1.0 ? err : 1.0) * 180.0 / PI;
        end
    end

    // The coefficients the DUT uses
    logic signed [17:0] coef [3 * PAIRS];
    initial $readmemh(COEF_FILE, coef);

    task automatic check_coefficients();
        int unsigned bad = 0;
        for (int k = 0; k < 3; k++)
            for (int p = 0; p < PAIRS; p++) begin
                automatic real ideal = pinv_m[k][p] * 2.0 ** COEF_FRAC;
                if ((coef[k * PAIRS + p] - ideal) ** 2 > 0.25 + 1e-6) begin
                    if (bad < 3)
                        tb_error($sformatf("%s: coefficient %0d of pair %0d is %0d, ideal %.2f", NAME, k, p, coef[k * PAIRS + p], ideal));
                    bad++;
                end
            end
        `CHECK(bad == 0, $sformatf("%s: %0d coefficients off by more than 0.5 LSB", NAME, bad))
    endtask

    // ------------------------------------------------------------------
    // Source: {tid, index} beats
    // ------------------------------------------------------------------
    logic [PAIR_W+IDX_W-1:0] src_q[$];

    always @(posedge clk) begin
        bit hs;
        if (!rstn) begin
            din.tvalid <= 1'b0;
            din.tdata  <= '0;
            din.tid    <= '0;
        end else begin
            hs = din.tvalid && din.tready;
            if (hs) void'(src_q.pop_front());
            if (!din.tvalid || hs) begin
                if (src_q.size() > 0 && $urandom_range(99) >= 40) begin
                    din.tvalid <= 1'b1;
                    {din.tid, din.tdata} <= src_q[0];
                end else begin
                    din.tvalid <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Model: one window's lags in, the expected outputs queued
    // ------------------------------------------------------------------
    typedef struct {
        logic [3*OUT_WIDTH-1:0] bits;
        real                    ideal [3];
        bit                     plane;   // a plane wave, from the direction in plane_u
    } expected_t;
    expected_t expected[$];
    // x, y, z of each plane wave's direction. Kept out of expected_t: Verilator 5.020 reads a
    // real array member of a struct back as 0 in arithmetic
    real       plane_u[$];
    int unsigned n_saturated = 0;

    localparam longint OUT_MAX = (longint'(1) <<< (OUT_WIDTH - 1)) - 1;
    localparam longint OUT_MIN = -(longint'(1) <<< (OUT_WIDTH - 1));

    task automatic send_window(int lags [PAIRS], bit plane, real ux, real uy, real uz);
        expected_t e;
        for (int k = 0; k < 3; k++) begin
            longint acc = 0, r;
            e.ideal[k] = 0.0;
            for (int p = 0; p < PAIRS; p++) begin
                acc += longint'(coef[k * PAIRS + p]) * lags[p];
                e.ideal[k] += pinv_m[k][p] * lags[p];
            end
            r = (acc + (longint'(1) <<< (SHIFT - 1))) >>> SHIFT;
            if (r > OUT_MAX || r < OUT_MIN) n_saturated++;
            r = r > OUT_MAX ? OUT_MAX : r < OUT_MIN ? OUT_MIN : r;
            e.bits[(2 - k) * OUT_WIDTH +: OUT_WIDTH] = OUT_WIDTH'(r);
        end
        e.plane = plane;
        if (plane) begin
            plane_u.push_back(ux);
            plane_u.push_back(uy);
            plane_u.push_back(uz);
        end
        expected.push_back(e);
        for (int p = 0; p < PAIRS; p++)
            src_q.push_back({PAIR_W'(p), IDX_W'(lags[p] + D)});
    endtask

    task automatic random_window(int range);
        int lags [PAIRS];
        foreach (lags[p]) lags[p] = $urandom_range(2 * range) - range;
        send_window(lags, 1'b0, 0.0, 0.0, 0.0);
    endtask

    task automatic plane_window();
        int  lags [PAIRS];
        real u [3];
        // A random direction, uniform on the sphere
        real z = 2.0 * ($urandom_range(100000) / 100000.0) - 1.0;
        real t = 2.0 * PI * ($urandom_range(100000) / 100000.0);
        u = '{$sqrt(1.0 - z * z) * $cos(t), $sqrt(1.0 - z * z) * $sin(t), z};
        foreach (lags[p]) begin
            real delay = 0.0;
            for (int k = 0; k < 3; k++) delay += (loc[pair_a[p]][k] - loc[pair_b[p]][k]) * u[k];
            lags[p] = int'(delay * FS / C);  // rounds to nearest
            if (lags[p] > D) lags[p] = D;
            if (lags[p] < -D) lags[p] = -D;
        end
        send_window(lags, 1'b1, u[0], u[1], u[2]);
    endtask

    // ------------------------------------------------------------------
    // Sink
    // ------------------------------------------------------------------
    int unsigned            n_recv = 0, n_plane = 0;
    real                    worst_angle = 0.0;
    bit                     stalled = 1'b0;
    logic [3*OUT_WIDTH-1:0] stalled_beat;

    always @(posedge clk) begin
        if (rstn) begin
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
                    automatic expected_t e = expected.pop_front();
                    automatic real       got [3];
                    if (dout.tdata !== e.bits)
                        tb_error_once({NAME, " data"}, $sformatf("%s: window %0d is 0x%0h, expected 0x%0h", NAME, n_recv, dout.tdata, e.bits));
                    for (int k = 0; k < 3; k++) begin
                        automatic logic signed [OUT_WIDTH-1:0] v = dout.tdata[(2 - k) * OUT_WIDTH +: OUT_WIDTH];
                        got[k] = real'(v) / 2.0 ** OUT_FRAC;
                        tb_checks++;
                        if (v != OUT_MAX && v != OUT_MIN && (got[k] - e.ideal[k]) ** 2 > (1.5 / 2.0 ** OUT_FRAC) ** 2)
                            tb_error_once({NAME, " ideal"}, $sformatf("%s: window %0d component %0d is %.5f, least squares gives %.5f",
                                                                      NAME, n_recv, k, got[k], e.ideal[k]));
                    end
                    if (e.plane) begin
                        automatic real ux    = plane_u.pop_front();
                        automatic real uy    = plane_u.pop_front();
                        automatic real uz    = plane_u.pop_front();
                        automatic real dot   = got[0] * ux + got[1] * uy + got[2] * uz;
                        automatic real len   = $sqrt(got[0] ** 2 + got[1] ** 2 + got[2] ** 2);
                        automatic real angle = $acos(dot / len) * 180.0 / PI;
                        tb_checks++;
                        if (angle > max_angle)
                            tb_error_once({NAME, " angle"}, $sformatf("%s: window %0d is %.1f degrees from the source direction (limit %.1f)",
                                                                      NAME, n_recv, angle, max_angle));
                        if (angle > worst_angle) worst_angle = angle;
                        n_plane++;
                    end
                end
                n_recv++;
            end
            stalled      = dout.tvalid && !dout.tready;
            stalled_beat = dout.tdata;
        end
        dout.tready <= $urandom_range(99) < 40;
    end

    initial begin
        int unsigned timeout = 0;
        done = 1'b0;
        repeat (5) @(posedge clk);
        check_coefficients();
        rstn = 1'b1;
        @(posedge clk);
        `CHECK_EQ(dout.tvalid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        for (int w = 0; w < WINDOWS; w++)
            case (w % 3)
                0: plane_window();
                1: random_window(D);
                default: random_window($urandom_range(D));
            endcase
        while (n_recv < WINDOWS && timeout < 100 * WINDOWS * PAIRS) begin
            @(posedge clk);
            timeout++;
        end
        repeat (20) @(posedge clk);
        `CHECK(n_recv == WINDOWS, $sformatf("%s: %0d of %0d windows came out", NAME, n_recv, WINDOWS))
        `CHECK(expected.size() == 0, $sformatf("%s: %0d directions never came out", NAME, expected.size()))
        tb_info($sformatf("%s: %0d windows, %0d plane waves within %.2f degrees (limit %.2f), %0d components saturated",
                          NAME, n_recv, n_plane, worst_angle, max_angle, n_saturated));
        done = 1'b1;
    end

endmodule


module tb_direction_estimator;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(20_000_000) // 20 ms

    // The sim runs in sim/out/<test>, and $readmemh paths are relative to it
    localparam CUBE_FILE  = "../../../src/dsp/building_blocks/direction_matrix.hex";
    localparam TETRA_FILE = "../../tests/direction_estimator/tetra_matrix.hex";

    logic clk = 1'b0;
    always #2.5 clk = ~clk; // 200 MHz, clk_core

    logic [2:0] done;

    // The real array and matrix
    direction_estimator_inst #(.CHANNELS(8), .D(10), .GEOMETRY(0), .COEF_FILE(CUBE_FILE), .COEF_FRAC(23))
        g_cube (.clk(clk), .done(done[0]));
    // A different array (and coefficient scale), 6 pairs
    direction_estimator_inst #(.CHANNELS(4), .D(8), .GEOMETRY(1), .COEF_FILE(TETRA_FILE), .COEF_FRAC(21))
        g_tetra (.clk(clk), .done(done[1]));
    // The cube with a +/-1 output range, so big lag sets saturate
    direction_estimator_inst #(.CHANNELS(8), .D(10), .GEOMETRY(0), .COEF_FILE(CUBE_FILE), .COEF_FRAC(23), .OUT_FRAC(15))
        g_sat (.clk(clk), .done(done[2]));

    initial begin
        tb_info("Testing direction_estimator: 8 mic cube, 4 mic tetrahedron, saturating cube");
        wait (&done);
        tb_finish();
    end

endmodule
