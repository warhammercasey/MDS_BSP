`timescale 1ns/1ps

// Behavioral model of the TI ADC088S022 (8-channel, 8-bit SAR ADC with SPI interface).
// With BITS = 12 it also models the ADC128S022, which uses the same frame format.
//
// Behavior per datasheet SNAS341F:
//  - CS falling starts a frame; a frame holds a multiple of 16 SCLK rising edges.
//    CS falling while SCLK is low counts as falling edge 1.
//  - DIN is sampled on rising edges 1-8 of each conversion, MSB first. Bits 5:3
//    (ADD2..ADD0) select the channel for the NEXT conversion; IN0 after power-up.
//  - The input is tracked for 3 SCLK cycles and held on falling edge 4.
//  - DOUT changes on falling edges: edges 1-4 leading zeros, then the result MSB
//    first, then zeros. DOUT is high-Z while CS is high.
//  - Timing checks: SCLK 0.8-3.2 MHz, 40-60% duty, tCSS/tCSH/tDS/tDH >= 10 ns.
//
// Each conversion is pushed to `conversions` once its last bit is on DOUT, for the
// testbench scoreboard to consume.
module adc088s022_model #(
    parameter int BITS = 8
)(
    input  logic            cs_n,
    input  logic            sclk,
    input  logic            din,
    output wire             dout,
    input  logic [BITS-1:0] analog_in [8]  // input level on IN0-IN7, as an ideal output code
);
    import tb_pkg::*;

    localparam real T_CSS_NS   = 10.0;
    localparam real T_CSH_NS   = 10.0;
    localparam real T_DS_NS    = 10.0;
    localparam real T_DH_NS    = 10.0;
    localparam real F_SCLK_MIN = 0.8e6;
    localparam real F_SCLK_MAX = 3.2e6;
    localparam real DUTY_MIN   = 0.40;
    localparam real DUTY_MAX   = 0.60;

    typedef struct {
        int unsigned     channel;
        logic [BITS-1:0] value;
    } conv_t;

    conv_t       conversions[$];  // completed conversions, consumed by the testbench
    int unsigned n_conversions = 0;

    // Device state (all visible in waves)
    logic [2:0]      ctrl_channel = 3'd0; // control register: channel for the next conversion
    logic [2:0]      conv_channel = 3'd0; // channel of the conversion in progress
    logic [BITS-1:0] held_value   = '0;   // value captured at the start of hold
    logic [7:0]      din_shift    = '0;
    int              rise_edge    = 0;    // rising edge number within the conversion (1-16)
    int              fall_edge    = 0;    // falling edge number within the conversion (1-16)
    logic            dout_en      = 1'b0;
    logic            dout_bit     = 1'b0;
    real             sclk_freq    = 0.0;  // last measured SCLK frequency (Hz)

    assign dout = dout_en ? dout_bit : 1'bz;

    // Timing violations are reported once, then counted
    function automatic void violation(string kind, string msg);
        tb_error_once({"ADC088S022 ", kind}, {"ADC model: ", msg});
    endfunction

    function automatic void report();
        tb_info($sformatf("ADC model: %0d conversions, last SCLK %.3f MHz", n_conversions, sclk_freq / 1e6));
    endfunction

    // ------------------------------------------------------------------
    // Serial interface
    // ------------------------------------------------------------------
    realtime t_cs_fall = -1.0e9;
    realtime t_rise    = -1.0e9;
    realtime t_fall    = -1.0e9;
    realtime t_din     = -1.0e9;

    function automatic void on_fall();
        fall_edge = fall_edge % 16 + 1;
        if (fall_edge == 1)
            conv_channel = ctrl_channel;           // track phase starts
        if (fall_edge == 4)
            held_value = analog_in[conv_channel];  // hold phase starts

        if (fall_edge >= 5 && fall_edge < 5 + BITS)
            dout_bit = held_value[BITS - 1 - (fall_edge - 5)];
        else
            dout_bit = 1'b0;
    endfunction

    function automatic void on_rise();
        rise_edge = rise_edge % 16 + 1;
        if (rise_edge <= 8) begin
            din_shift = {din_shift[6:0], din};
            if (rise_edge == 8)
                ctrl_channel = din_shift[5:3];
        end
        if (rise_edge == 4 + BITS) begin           // last result bit is on DOUT
            conv_t c;
            c.channel = conv_channel;
            c.value   = held_value;
            conversions.push_back(c);
            n_conversions++;
        end
    endfunction

    always @(negedge cs_n) begin
        if ($realtime - t_rise < T_CSH_NS)
            violation("tCSH", $sformatf("CS fell %.1f ns after SCLK rose (tCSH min %.0f ns)",
                                        $realtime - t_rise, T_CSH_NS));
        t_cs_fall = $realtime;
        rise_edge = 0;
        fall_edge = 0;
        dout_en   = 1'b1;
        dout_bit  = 1'b0;
        if (sclk === 1'b0)
            on_fall();
    end

    always @(posedge cs_n) begin
        dout_en = 1'b0;
        if (rise_edge != 0 && rise_edge != 16)
            violation("frame", $sformatf("CS rose after %0d SCLK rising edges in a conversion (must be 16)",
                                         rise_edge));
    end

    always @(posedge sclk) begin
        if (cs_n === 1'b0) begin
            automatic realtime now = $realtime;
            automatic int next_edge = rise_edge % 16 + 1;

            if (now - t_cs_fall < T_CSS_NS)
                violation("tCSS", $sformatf("SCLK rose %.1f ns after CS fell (tCSS min %.0f ns)",
                                            now - t_cs_fall, T_CSS_NS));

            // Period and duty cycle over the last full SCLK cycle within this frame
            if (t_rise > t_cs_fall && t_fall > t_rise) begin
                automatic real period = now - t_rise;
                automatic real duty   = (t_fall - t_rise) / period;
                sclk_freq = 1.0e9 / period;
                if (sclk_freq < F_SCLK_MIN || sclk_freq > F_SCLK_MAX)
                    violation("fSCLK", $sformatf("SCLK is %.3f MHz (spec %.1f-%.1f MHz)",
                                                 sclk_freq / 1e6, F_SCLK_MIN / 1e6, F_SCLK_MAX / 1e6));
                if (duty < DUTY_MIN || duty > DUTY_MAX)
                    violation("duty", $sformatf("SCLK duty cycle is %.1f%% (spec %.0f-%.0f%%)",
                                                duty * 100, DUTY_MIN * 100, DUTY_MAX * 100));
            end

            // Only the address bits (rising edges 3-5) need DIN timing
            if (next_edge >= 3 && next_edge <= 5 && now - t_din < T_DS_NS)
                violation("tDS", $sformatf("DIN changed %.1f ns before SCLK rising edge %0d (tDS min %.0f ns)",
                                           now - t_din, next_edge, T_DS_NS));

            t_rise = now;
            on_rise();
        end
    end

    always @(negedge sclk) begin
        if (cs_n === 1'b0) begin
            t_fall = $realtime;
            on_fall();
        end
    end

    always @(din) begin
        if (cs_n === 1'b0 && rise_edge >= 3 && rise_edge <= 5 && $realtime - t_rise < T_DH_NS)
            violation("tDH", $sformatf("DIN changed %.1f ns after SCLK rising edge %0d (tDH min %.0f ns)",
                                       $realtime - t_rise, rise_edge, T_DH_NS));
        t_din = $realtime;
    end

endmodule
