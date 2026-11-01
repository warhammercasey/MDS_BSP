`timescale 1ns/1ps

// Behavioral model of the InvenSense INMP441 I2S MEMS microphone.
//
// Behavior per datasheet (DS-INMP441-00 rev 1.1):
//  - I2S slave: the master drives SCK and WS. There must be 64 SCK per stereo frame
//    (WS half period = 32 SCK). WS is sampled on SCK rising edges.
//  - SD changes on SCK falling edges, for the master to sample on rising edges. The
//    MSB of a 24-bit 2's complement word is on the line one SCK after WS changes
//    (sampled on the 2nd rising edge of the half frame), the LSB on the 25th; SD is
//    tri-state for the rest of the half frame.
//  - SD is driven only in the channel L/R selects: 0 = left (WS low), 1 = right
//    (WS high). Two mics share one data line.
//  - Startup: zero output for the first 2^18 SCK cycles after power-up. If SCK stops
//    the mic goes to standby, and outputs zeros for 2^14 cycles once SCK restarts.
//  - Timing checks: SCK 0.5-3.2 MHz (tSCP >= 312 ns), high/low >= 50 ns, WS setup
//    >= 0 ns / hold >= 20 ns to SCK rising, WS half period = 32 SCK. The WS range
//    (7.8-50 kHz) follows from the SCK range.
//
// The datasheet gives no SD output delay; T_DV_NS is an assumed value.
//
// The real SD pin is tri-state; here it is `data` plus `data_oe` so the testbench
// can resolve a shared line (and detect contention) explicitly. Each word is
// reported on `word`/`word_count` once its LSB is driven.
module inmp441_model #(
    parameter bit LR          = 1'b0,     // L/R pin: 0 = left (drive while WS low), 1 = right
    parameter int STARTUP_SCK = 2**18,    // zero-output SCK cycles after power-up
    parameter int WAKE_SCK    = 2**14     // zero-output SCK cycles after leaving standby
)(
    input  logic               sck,
    input  logic               ws,
    output logic               data,
    output logic               data_oe,
    input  logic signed [23:0] sample_in,  // acoustic input, as the ideal 24-bit output code
    output logic signed [23:0] word,       // last word transmitted
    output int unsigned        word_count  // increments when `word` updates
);
    import tb_pkg::*;

    localparam real T_SCP_MIN_NS = 312.0;   // 3.2 MHz
    localparam real T_SCP_MAX_NS = 2000.0;  // 0.5 MHz; slower than this counts as SCK stopped
    localparam real T_SCH_MIN_NS = 50.0;
    localparam real T_SCL_MIN_NS = 50.0;
    localparam real T_WSS_NS     = 0.0;
    localparam real T_WSH_NS     = 20.0;
    localparam real T_DV_NS      = 40.0;    // SD valid after SCK falling (not in the datasheet)

    // State (all visible in waves)
    int unsigned        sck_count  = 0;      // SCK rising edges seen
    int unsigned        zero_until = STARTUP_SCK; // output zeros until sck_count reaches this
    logic               ws_q       = 1'b0;   // WS as sampled on the last SCK rising edge
    logic               synced     = 1'b0;   // a WS edge has been seen, so slots are known
    logic               word_ws    = 1'b0;   // WS level of the current word
    int                 slot       = 0;      // 1..32 within the current WS half
    logic signed [23:0] tx_sample  = '0;     // word being shifted out

    initial begin
        data       = 1'b0;
        data_oe    = 1'b0;
        word       = '0;
        word_count = 0;
    end

    // Timing violations are reported once across all mic instances, then counted
    function automatic void violation(string kind, string msg);
        tb_error_once({"INMP441 ", kind}, {"Mic model: ", msg});
    endfunction

    // ------------------------------------------------------------------
    // Serial interface
    // ------------------------------------------------------------------
    realtime t_rise = -1.0e9;
    realtime t_fall = -1.0e9;
    realtime t_ws   = -1.0e9;

    always @(posedge sck) begin
        automatic realtime now = $realtime;

        if (t_rise > 0 && t_fall > t_rise) begin
            automatic real period = now - t_rise;
            automatic real t_high = t_fall - t_rise;
            automatic real t_low  = now - t_fall;
            if (period > T_SCP_MAX_NS) begin
                // SCK stopped: standby, then a restart delay
                zero_until = sck_count + WAKE_SCK;
            end else begin
                if (period < T_SCP_MIN_NS)
                    violation("tSCP", $sformatf("SCK period %.2f ns (%.3f MHz) is below %.0f ns (max 3.2 MHz)",
                                                period, 1.0e3 / period, T_SCP_MIN_NS));
                if (t_high < T_SCH_MIN_NS || t_low < T_SCL_MIN_NS)
                    violation("tSCH/tSCL", $sformatf("SCK high %.2f ns / low %.2f ns (min %.0f ns)",
                                                     t_high, t_low, T_SCH_MIN_NS));
            end
        end

        if (now - t_ws <= T_WSS_NS)
            violation("tWSS", $sformatf("WS changed %.2f ns before SCK rising (setup min %.0f ns)",
                                        now - t_ws, T_WSS_NS));

        // Slot tracking: a WS change seen on this edge starts a new word in slot 1
        if (ws !== ws_q) begin
            if (synced && slot != 32)
                violation("WS", $sformatf("WS half period was %0d SCK cycles (must be 32, 64 SCK per frame)", slot));
            synced    = 1'b1;
            slot      = 1;
            word_ws   = ws;
            tx_sample = sck_count < zero_until ? '0 : sample_in;
        end else if (synced) begin
            slot++;
        end
        ws_q = ws;

        sck_count++;
        t_rise = now;
    end

    // Bit k of the word (1 = MSB) goes out on the falling edge in slot k
    always @(negedge sck) begin
        automatic logic drive = synced && word_ws == LR && slot >= 1 && slot <= 24;

        data    <= #(T_DV_NS) drive ? tx_sample[24 - slot] : 1'b0;
        data_oe <= #(T_DV_NS) drive;

        if (drive && slot == 24) begin  // LSB is on the line
            word = tx_sample;
            word_count++;
        end

        t_fall = $realtime;
    end

    always @(ws) begin
        if ($realtime - t_rise < T_WSH_NS)
            violation("tWSH", $sformatf("WS changed %.2f ns after SCK rising (hold min %.0f ns)",
                                        $realtime - t_rise, T_WSH_NS));
        t_ws = $realtime;
    end

endmodule
