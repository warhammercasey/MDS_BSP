`timescale 1ns/1ps

// Usage rule for multi-bit synchronizers. The bind at the bottom of this file
// attaches the checker to every ff_sync instance in a sim that compiles it.
//
// Each bit of a synchronized bus resolves independently in the destination
// domain, so if two bits change together the receiver can capture a mix of the
// old and new values (e.g. a binary pointer going 0111 -> 1000 can be seen as
// 1111 or 0000). A bus is only safe to synchronize if at most one bit changes
// per source clock, as with a Gray-coded counter.
//
// A change to all zeros is allowed: that is a counter being reset, which is safe
// as long as the receiving side is held in reset while it propagates.
module ff_sync_checker #(
    parameter WIDTH = 1
) (
    input logic             clka,
    input logic [WIDTH-1:0] din
);
    import tb_pkg::*;

    logic [WIDTH-1:0] prev;
    bit               started = 1'b0;

    always @(posedge clka) begin
        if (WIDTH > 1 && started && din != '0 && $countones(din ^ prev) > 1)
            tb_error_once("ff_sync multi-bit change",
                $sformatf("%m: %0d-bit bus changed %0d bits in one clock (0x%0h -> 0x%0h); synchronize Gray code, not binary",
                          WIDTH, $countones(din ^ prev), prev, din));
        prev    <= din;
        started <= 1'b1;
    end

endmodule

bind ff_sync ff_sync_checker #(.WIDTH(WIDTH)) u_ff_sync_checker (.clka(clka), .din(din));
