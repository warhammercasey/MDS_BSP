`timescale 1ns/1ps
`include "tb_macros.svh"

// pulse_cdc test: one instance per ordered pair of the design's clock domains.
//
// Input pulses are one clka cycle wide.
//   - spaced phase: pulses 1-2x the documented minimum spacing (3 clkb periods)
//     apart. Each must produce exactly one output pulse (one clkb cycle of dout
//     high), in order, within MAX_LAT_NS.
//   - dense phase: pulses closer than that, which may be lost. Every output pulse
//     must still follow an input pulse, and there can never be more output pulses
//     than input pulses.
module pulse_cdc_pair #(
    parameter real   FREQ_A   = 100.0e6,
    parameter real   FREQ_B   = 100.0e6,
    parameter string NAME     = "",
    parameter int    N_SPACED = 60,
    parameter int    N_DENSE  = 200
) (
    input  logic clka,
    input  logic clkb,
    output logic done
);
    import tb_pkg::*;

    localparam real TA = 1.0e9 / FREQ_A;
    localparam real TB = 1.0e9 / FREQ_B;

    // Toggle flop + ff_sync source flop in clka, up to 4 synchronizer flops + edge detector in clkb
    localparam real MAX_LAT_NS = 3 * TA + 6 * TB;
    // Minimum spacing of input pulses, in clka cycles: 3 clkb periods
    localparam int  GAP_SPACED = int'($ceil(3.0 * TB / TA)) > 1 ? int'($ceil(3.0 * TB / TA)) : 1;

    logic din = 1'b0;
    logic dout;

    pulse_cdc dut (
        .clka(clka),
        .clkb(clkb),
        .din(din),
        .dout(dout)
    );

    // ------------------------------------------------------------------
    // Source (clka): single-cycle pulses
    // ------------------------------------------------------------------
    realtime     t_in[$];          // spaced phase: input pulses without an output yet
    realtime     t_last_in = -1.0;
    int unsigned n_in = 0;
    bit          dense = 1'b0;

    // din changes on negedges, so the posedge in between registers it race-free
    task automatic send_pulses(int unsigned n, int unsigned gap_min, int unsigned gap_max);
        repeat (n) begin
            @(negedge clka) din = 1'b1;
            @(posedge clka);               // the DUT registers the pulse on this edge
            t_last_in = $realtime;
            if (!dense)
                t_in.push_back($realtime);
            n_in++;
            @(negedge clka) din = 1'b0;
            repeat ($urandom_range(gap_max, gap_min) - 1) @(posedge clka);
        end
    endtask

    // ------------------------------------------------------------------
    // Destination (clkb): count and latency of output pulses
    // Each clkb cycle with dout high is one output pulse (back-to-back pulses are
    // separate events), so dout is sampled at clkb edges, before they update it.
    // ------------------------------------------------------------------
    int unsigned n_out = 0;
    real         lat_max = 0.0;

    always @(posedge clkb) if (dout === 1'b1) begin
        automatic realtime t = $realtime;
        n_out++;
        if (!dense) begin
            tb_checks++;
            if (t_in.size() == 0) begin
                tb_error($sformatf("%s: output pulse %0d with no input pulse waiting", NAME, n_out));
            end else begin
                automatic realtime lat = t - t_in.pop_front();
                lat_max = lat > lat_max ? lat : lat_max;
                `CHECK(lat <= MAX_LAT_NS,
                       $sformatf("%s: output pulse %0.1f ns after its input (max %0.1f ns)", NAME, lat, MAX_LAT_NS))
            end
        end else begin
            `CHECK(t_last_in >= 0 && t - t_last_in <= MAX_LAT_NS,
                   $sformatf("%s: output pulse %0d not preceded by an input pulse within %0.1f ns", NAME, n_out, MAX_LAT_NS))
        end
    end

    task automatic drain();
        repeat (int'($ceil(2 * MAX_LAT_NS / TA))) @(posedge clka);
    endtask

    initial begin
        done = 1'b0;
        repeat (8) @(posedge clka);

        send_pulses(N_SPACED, GAP_SPACED, 2 * GAP_SPACED);
        drain();
        `CHECK_EQ(n_out, N_SPACED, $sformatf("%s: output pulses for %0d spaced input pulses", NAME, N_SPACED))
        `CHECK(t_in.size() == 0, $sformatf("%s: %0d spaced input pulses produced no output", NAME, t_in.size()))

        dense = 1'b1;
        n_in  = 0;
        n_out = 0;
        send_pulses(N_DENSE, 1, GAP_SPACED);
        drain();
        `CHECK(n_out <= n_in, $sformatf("%s: %0d output pulses from %0d input pulses", NAME, n_out, n_in))
        `CHECK(n_out > 0, $sformatf("%s: no output pulses in the dense phase", NAME))

        tb_info($sformatf("%-20s spaced (>= %0d clka cycles): latency max %0.1f ns; dense: %0d in -> %0d out",
                          NAME, GAP_SPACED, lat_max, n_in, n_out));
        done = 1'b1;
    end

endmodule


module tb_pulse_cdc;
    import tb_pkg::*;
    import cain_clocks_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(2_000_000) // 2 ms

    logic clk [NUM_CLKS];
    for (genvar i = 0; i < NUM_CLKS; i++) begin : g_clk
        clk_gen #(.FREQ_HZ(FREQ_HZ[i])) gen (.clk(clk[i]));
    end

    logic [NUM_PAIRS-1:0] done;
    for (genvar p = 0; p < NUM_PAIRS; p++) begin : g_pair
        pulse_cdc_pair #(
            .FREQ_A(FREQ_HZ[pair_a(p)]),
            .FREQ_B(FREQ_HZ[pair_b(p)]),
            .NAME(pair_name(p))
        ) h (
            .clka(clk[pair_a(p)]),
            .clkb(clk[pair_b(p)]),
            .done(done[p])
        );
    end

    initial begin
        tb_info($sformatf("Testing pulse_cdc on %0d clock pairs", NUM_PAIRS));
        wait (&done);
        tb_finish();
    end

endmodule
