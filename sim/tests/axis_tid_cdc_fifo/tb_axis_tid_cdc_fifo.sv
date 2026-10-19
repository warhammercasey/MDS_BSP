`timescale 1ns/1ps
`include "tb_macros.svh"

// axis_tid_cdc_fifo test: one FIFO per ordered pair of the design's clock domains,
// sized like the I2S FIFO in cain_top (18-bit data, 3-bit tid). tid and tdata are
// randomized independently and checked together, so swapped or truncated fields
// show up as data errors. The checks are in ../axis_cdc_fifo/axis_cdc_fifo_pair.sv.
module tb_axis_tid_cdc_fifo;
    import tb_pkg::*;
    import cain_clocks_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(5_000_000) // 5 ms

    logic clk [NUM_CLKS];
    for (genvar i = 0; i < NUM_CLKS; i++) begin : g_clk
        clk_gen #(.FREQ_HZ(FREQ_HZ[i])) gen (.clk(clk[i]));
    end

    logic [NUM_PAIRS-1:0] done;
    for (genvar p = 0; p < NUM_PAIRS; p++) begin : g_pair
        axis_cdc_fifo_pair #(
            .FREQ_A(FREQ_HZ[pair_a(p)]),
            .FREQ_B(FREQ_HZ[pair_b(p)]),
            .NAME(pair_name(p)),
            .USE_TID(1'b1),
            .WIDTH(18),
            .ID_WIDTH(3),
            .AWIDTH(4)
        ) h (
            .clka(clk[pair_a(p)]),
            .clkb(clk[pair_b(p)]),
            .done(done[p])
        );
    end

    initial begin
        tb_info($sformatf("Testing axis_tid_cdc_fifo on %0d clock pairs", NUM_PAIRS));
        wait (&done);
        tb_finish();
    end

endmodule
