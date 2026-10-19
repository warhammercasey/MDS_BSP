module pulse_cdc (
    input  logic clka,
    input  logic clkb,

    input  logic din,  // One pulse per clka cycle that din is high
    output logic dout  // One clkb cycle pulse per input pulse
);

    // Toggle synchronizer: each input pulse flips a level in clka, the level crosses
    // through ff_sync, and clkb turns each change of the level back into a pulse.
    // Works for any clock ratio in either direction.
    //
    // Input pulses must be at least 3 clkb periods apart. Closer pulses can cancel
    // in pairs (the level flips back before clkb sees it), so they are lost, not merged.

    logic toggle_a;
    logic toggle_b;
    logic toggle_b_last;

    always_ff @(posedge clka) begin
        if (din) begin
            toggle_a <= ~toggle_a;
        end
    end

    ff_sync #(
        .WIDTH(1)
    ) ff_sync_i (
        .clka(clka),
        .clkb(clkb),

        .din(toggle_a),
        .dout(toggle_b)
    );

    // Pulse on every change of the synchronized level
    always_ff @(posedge clkb) begin
        toggle_b_last <= toggle_b;
        dout          <= toggle_b ^ toggle_b_last;
    end

endmodule
