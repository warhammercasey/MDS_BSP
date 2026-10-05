`timescale 1ns/1ps

// Behavioral model of the Knowles SPH0645LM4H-B I2S MEMS microphone.
//
// Behavior per datasheet (Rev B):
//  - I2S slave: the master drives BCLK and WS. WS must be BCLK/64 and change on
//    BCLK falling edges.
//  - DATA changes tDC after BCLK rising edges. The MSB is driven one BCLK after
//    WS changes (slot 1), followed by 18 bits of 2's complement data and zeros up
//    to 24 bits. DATA is tri-state for slots 25-32.
//  - DATA is driven only while WS == SELECT (two mics share one data line).
//  - Mode by BCLK frequency: active >= 1 MHz, sleep (DATA tri-state) < 900 kHz.
//  - Timing checks: BCLK period 244.14-488.28 ns, high/low >= 85.45 ns, duty
//    40-60%, WS setup/hold to BCLK rising >= 85.45 ns, WS half period = 32 BCLK.
//
// The real DATA pin is tri-state; here it is `data` plus `data_oe` so the
// testbench can resolve a shared line (and detect contention) explicitly.
// Each word is reported on `word`/`word_count` once its last data bit is driven.
module sph0645_model #(
    parameter bit SEL = 1'b0  // SELECT pin: 0 = drive while WS low, 1 = drive while WS high
)(
    input  logic               bclk,
    input  logic               ws,
    output logic               data,
    output logic               data_oe,
    input  logic signed [17:0] sample_in,  // acoustic input, as the ideal 18-bit output code
    output logic signed [17:0] word,       // last word transmitted
    output int unsigned        word_count  // increments when `word` updates
);
    import tb_pkg::*;

    localparam real T_MIN_NS     = 244.14;
    localparam real T_MAX_NS     = 488.28;
    localparam real T_HC_MIN_NS  = 85.45;
    localparam real T_LC_MIN_NS  = 85.45;
    localparam real T_SWS_NS     = 85.45;
    localparam real T_HWS_NS     = 85.45;
    localparam real T_DC_NS      = 65.92;   // modelled at the max data delay
    localparam real DUTY_MIN     = 0.40;
    localparam real DUTY_MAX     = 0.60;
    localparam real ACTIVE_T_NS  = 1000.0;  // >= 1 MHz -> active
    localparam real SLEEP_T_NS   = 1.0e9 / 900.0e3;  // < 900 kHz -> sleep

    // State (all visible in waves)
    logic               active   = 1'b0;   // normal mode vs sleep
    logic               ws_q     = 1'b0;   // WS as sampled on the last BCLK rising edge
    logic               synced   = 1'b0;   // a WS edge has been seen, so slots are known
    logic               word_ws  = 1'b0;   // WS level of the current word
    int                 slot     = 0;      // 1..32 within the current WS half
    logic        [23:0] tx_word  = '0;     // 24-bit word being shifted out
    logic signed [17:0] tx_sample = '0;

    initial begin
        data       = 1'b0;
        data_oe    = 1'b0;
        word       = '0;
        word_count = 0;
    end

    // Timing violations are reported once across all mic instances, then counted
    function automatic void violation(string kind, string msg);
        tb_error_once({"SPH0645 ", kind}, {"Mic model: ", msg});
    endfunction

    // ------------------------------------------------------------------
    // Serial interface
    // ------------------------------------------------------------------
    realtime t_rise = -1.0e9;
    realtime t_fall = -1.0e9;
    realtime t_ws   = -1.0e9;

    always @(posedge bclk) begin
        automatic realtime now   = $realtime;
        automatic logic    drive;

        // Clock period, pulse widths and operating mode
        if (t_rise > 0 && t_fall > t_rise) begin
            automatic real period = now - t_rise;
            automatic real t_high = t_fall - t_rise;
            automatic real t_low  = now - t_fall;
            if (period < T_MIN_NS || period > T_MAX_NS)
                violation("T", $sformatf("BCLK period %.2f ns (%.3f MHz) is outside %.2f-%.2f ns (2.048-4.096 MHz)",
                                         period, 1.0e3 / period, T_MIN_NS, T_MAX_NS));
            if (t_high < T_HC_MIN_NS || t_low < T_LC_MIN_NS)
                violation("tHC/tLC", $sformatf("BCLK high %.2f ns / low %.2f ns (min %.2f ns)",
                                               t_high, t_low, T_HC_MIN_NS));
            if (t_high / period < DUTY_MIN || t_high / period > DUTY_MAX)
                violation("duty", $sformatf("BCLK duty cycle %.1f%% (spec 40-60%%)", 100.0 * t_high / period));
            if (period <= ACTIVE_T_NS + 0.001)
                active = 1'b1;
            else if (period > SLEEP_T_NS)
                active = 1'b0;
        end

        if (now - t_ws < T_SWS_NS)
            violation("tSWS", $sformatf("WS changed %.2f ns before BCLK rising (setup min %.2f ns)",
                                        now - t_ws, T_SWS_NS));

        // Slot tracking: a WS change seen on this edge starts a new word in slot 1
        if (ws !== ws_q) begin
            if (synced && slot != 32)
                violation("WS", $sformatf("WS half period was %0d BCLK cycles (must be 32, WS = BCLK/64)", slot));
            synced    = 1'b1;
            slot      = 1;
            word_ws   = ws;
            tx_sample = sample_in;
            tx_word   = {tx_sample, 6'b0};
        end else if (synced) begin
            slot++;
        end
        ws_q = ws;

        drive = active && synced && word_ws == SEL && slot >= 1 && slot <= 24;
        data    <= #(T_DC_NS) drive ? tx_word[24 - slot] : 1'b0;
        data_oe <= #(T_DC_NS) drive;

        if (drive && slot == 18) begin  // last data bit is on the line
            word = tx_sample;
            word_count++;
        end

        t_rise = now;
    end

    always @(negedge bclk)
        t_fall = $realtime;

    always @(ws) begin
        if ($realtime - t_rise < T_HWS_NS)
            violation("tHWS", $sformatf("WS changed %.2f ns after BCLK rising (hold min %.2f ns)",
                                        $realtime - t_rise, T_HWS_NS));
        t_ws = $realtime;
    end

endmodule
