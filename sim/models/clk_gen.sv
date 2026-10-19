`timescale 1ns/1ps

// Free-running clock with a random start phase and random per-edge jitter.
//
// The CAIN clocks come from PLLs off one 50 MHz reference, so their ratios are
// exact and an ideal sim would repeat the same few edge alignments forever. The
// jitter makes the relative phase of any two clocks drift, so a CDC test sees
// edges landing at every offset (including nearly coincident ones) instead.
module clk_gen #(
    parameter real FREQ_HZ    = 100.0e6,
    parameter real JITTER_PCT = 2.0      // max deviation of each half period, in %
) (
    output logic clk
);

    localparam real HALF_NS = 0.5e9 / FREQ_HZ;

    function automatic real rand_unit(); // uniform in [-1, 1]
        return $urandom_range(2000) / 1000.0 - 1.0;
    endfunction

    initial begin
        clk = 1'b0;
        #(HALF_NS * $urandom_range(2000) / 1000.0);
        forever #(HALF_NS * (1.0 + JITTER_PCT / 100.0 * rand_unit())) clk = ~clk;
    end

endmodule
