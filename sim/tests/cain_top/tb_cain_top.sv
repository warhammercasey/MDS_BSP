`timescale 1ns/1ps
`include "tb_macros.svh"

// Top-level end to end test: sound reaching the 8 microphones from a direction, through
// cain_top, to the RTP packets on the RMII TX pins.
//
// Acoustics: a plane wave from unit direction u reaches mic m (position p_m, the 16 cm cube
// of gen_direction_matrix.py, in channel order) (p_m . u) / c early, so at each I2S frame
// (WS falling) mic m's input is s(t + (p_m . u) / c): an exact fractional delay, worked out
// from the stimulus formula. Every mic's word in a frame is from the same instant, as on
// the real array. The INMP441 models put the words on the 4 shared data lines.
//
// Stimuli, one after another (no reset between them):
//   tone   a 1 kHz tone. Checks the audio only: a pure tone's cross correlation has no peak
//          to find the delay from (its magnitude is flat over the lags, so the window's
//          taper puts the peak at lag 0)
//   fm     a 4 kHz tone, frequency modulated +/-3 kHz at 170 Hz
//   chirp  1.5 to 8.5 kHz linear sweeps, 10 ms each
//   fm2    a 6 kHz tone, frequency modulated +/-2.5 kHz at 230 Hz (with +scenarios=3)
// each from its own direction. The DSP passes 0-10 kHz (5 kHz +/- 5 kHz around the mixer).
//
// The wire monitor rebuilds every frame from its RTP packet (Ethernet, IPv4 and UDP headers
// from the reset settings: broadcast, ports 5000, IP identification counting up, own CRC-32
// and IP checksum) and compares byte for byte, then checks the RTP header (sequence,
// timestamp, marker, payload type) and the payload of each stream:
//   mic        every frame of 8 samples must be one the mics sent, frames in order with none
//              missing: the first packet is found in the audio sent, and each packet after
//              must follow it by its timestamp
//   ADC        every sample must be its channel's code (the ADC inputs are held)
//   direction  once the DSP has had a full correlation window and moving average of a
//              stimulus, each direction must point within ANGLE_TOL of the source, with
//              length MAG_LO..MAG_HI (1 for exact lags), and within IDEAL_TOL of the least
//              squares answer from the true lags rounded to whole samples (what the peak
//              finder should pick for these broadband stimuli)
// And no decimator result may be lost: it has no backpressure, so the correlator has to
// keep up (that's what sets cain_top's DSP_XCORR_MACS).
//
// Plusargs: +scenarios=N directional stimuli (1-3, default 2), +check_ms=M how long each is
// checked for (default 15), +stop_ms=M to stop early (for a look at the waves).
module tb_cain_top;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(1_500_000_000) // 1.5 s

    // ------------------------------------------------------------------
    // Settings (mirror cain_top, udp_stack_regs' reset values and gen_direction_matrix.py)
    // ------------------------------------------------------------------
    localparam real PI       = 3.14159265358979;
    localparam real C_SOUND  = 343.0;                       // m/s
    localparam real MIC_HALF = 0.08;                        // the cube is 16 cm across
    localparam real FS_IN    = 73_571_400.0 / 24.0 / 64.0;  // I2S frame rate
    localparam int  DECIM    = 4;
    localparam real FS_DEC   = FS_IN / DECIM;               // at the correlator
    localparam int  MAX_LAG  = 10;
    localparam int  PAIRS    = 28;
    localparam int  DF_FRAC  = 14;                          // directions are Q2.14
    localparam int  COEF_FRAC = 23;

    // How long the DSP takes to forget the last stimulus: a correlation window of 1024
    // decimated samples, the 512 tap decimation filter, the 16 direction moving average,
    // and some for the pipeline and Ethernet
    localparam real WINDOW_MS = 1024.0 * DECIM / FS_IN * 1.0e3;
    localparam real SETTLE_MS = WINDOW_MS + 512.0 / FS_IN * 1.0e3 + 16.0 * 16.0 * DECIM / FS_IN * 1.0e3 + 5.0;

    localparam real ANGLE_TOL = 10.0;   // degrees from the source
    localparam real IDEAL_TOL = 0.05;   // from the whole-sample-lag answer (a lag off by one moves it ~0.1)
    localparam real MAG_LO    = 0.8;
    localparam real MAG_HI    = 1.2;

    localparam logic [47:0] LOCAL_MAC = 48'h02_00_00_00_00_01;
    localparam logic [47:0] DST_MAC   = 48'hFF_FF_FF_FF_FF_FF;
    localparam logic [31:0] LOCAL_IP  = 32'hC0A8_0064;      // 192.168.0.100
    localparam logic [31:0] DST_IP    = 32'hFFFF_FFFF;
    localparam logic [15:0] PORT      = 16'd5000;
    localparam logic [7:0]  TTL       = 8'd64;

    // Per stream: 0 = ADC, 1 = mics, 2 = directions
    localparam int          NS            = 3;
    localparam logic [31:0] SSRCS    [NS] = '{32'h4144_4330, 32'h4D49_4330, 32'h4449_5230};
    localparam logic [6:0]  PTS      [NS] = '{7'd96, 7'd97, 7'd98};
    localparam int          CHANNELS [NS] = '{8, 8, 3};
    localparam int          BYTES    [NS] = '{1, 3, 2};
    localparam int          FRAMES   [NS] = '{1152 / 8, 1152 / 24, 1};
    localparam string       NAMES    [NS] = '{"ADC", "mic", "direction"};

    localparam logic [7:0]  ADC_CODES [8] = '{8'h0F, 8'h2E, 8'h4D, 8'h6C, 8'h8B, 8'hAA, 8'hC9, 8'hE8};

    `define PUSH_BE(Q, V, N) for (int k_ = (N) - 1; k_ >= 0; k_--) Q.push_back(8'((V) >> (8 * k_)));

    // ------------------------------------------------------------------
    // Clocks and DUT
    // ------------------------------------------------------------------
    logic clk_50, clk_core, clk_sram, clk_io;
    clk_gen #(.FREQ_HZ(50.0e6),     .JITTER_PCT(0.5)) gen_50   (.clk(clk_50));
    clk_gen #(.FREQ_HZ(125.0e6),    .JITTER_PCT(0.5)) gen_core (.clk(clk_core));
    clk_gen #(.FREQ_HZ(100.0e6),    .JITTER_PCT(0.5)) gen_sram (.clk(clk_sram));
    // clk_io: little jitter, or the ADC's SCLK (clk_io / 23 = 3.199 MHz) measures over its 3.2 MHz limit
    clk_gen #(.FREQ_HZ(73.5714e6),  .JITTER_PCT(0.01)) gen_io  (.clk(clk_io));

    logic       rst_in_n = 1'b0;
    logic       i2s_sck, i2s_ws;
    logic [3:0] i2s_data;
    logic       adc_sck, adc_mosi, adc_cs_n;
    wire        adc_miso;
    logic [1:0] rmii_txd;
    logic       rmii_txen;

    cain_top #(
        .DSP_INIT_DIR("../../../src/dsp/building_blocks/") // The sim runs in sim/out/cain_top
    ) dut (
        .clk_50(clk_50), .clk_core(clk_core), .clk_sram(clk_sram), .clk_locked(1'b1),
        .clk_io(clk_io), .clk_io_locked(1'b1), .rst_in_n(rst_in_n),
        .rgb_led(), .gpio_OUT(), .gpio_IN(8'h00), .gpio_OE(),
        .i2s_sck(i2s_sck), .i2s_ws(i2s_ws), .i2s_data(i2s_data),
        .adc_spi_sck(adc_sck), .adc_spi_mosi(adc_mosi), .adc_spi_miso(adc_miso), .adc_spi_cs_n(adc_cs_n),
        .uart_rx(1'b1), .uart_tx(),
        .bb_OUT(), .bb_IN(8'h00), .bb_OE(),
        .rmii_txd(rmii_txd), .rmii_txen(rmii_txen), .rmii_rstn(), .rmii_int(1'b1),
        .rmii_crs_dv(1'b0), .rmii_rxd(2'b00), .rmii_rxerr(1'b0),
        .rmii_mdc(), .rmii_mdio_IN(1'b1), .rmii_mdio_OUT(), .rmii_mdio_OE(),
        .qspi_sio_IN(4'h0), .qspi_sio_OUT(), .qspi_sio_OE(), .qspi_cs_n()
    );

    // ------------------------------------------------------------------
    // ADC: every input held at its own code
    // ------------------------------------------------------------------
    logic [7:0] adc_in [8];
    initial foreach (adc_in[c]) adc_in[c] = ADC_CODES[c];

    adc088s022_model #(.BITS(8)) adc (
        .cs_n(adc_cs_n), .sclk(adc_sck), .din(adc_mosi), .dout(adc_miso), .analog_in(adc_in)
    );

    // The model queues every conversion for a scoreboard; this test checks them in the packets
    always @(posedge adc_cs_n) adc.conversions.delete();

    // ------------------------------------------------------------------
    // Microphones, 2 per data line: mic m on line m/2, L/R = m%2, so its tid is m
    // ------------------------------------------------------------------
    logic signed [23:0] mic_in    [8];
    logic               mic_data  [8];
    logic               mic_oe    [8];
    logic signed [23:0] mic_word  [8];
    int unsigned        mic_words [8];

    initial foreach (mic_in[m]) mic_in[m] = '0;

    for (genvar m = 0; m < 8; m++) begin : g_mic
        inmp441_model #(.LR(1'(m % 2)), .STARTUP_SCK(0)) mic (
            .sck(i2s_sck), .ws(i2s_ws), .data(mic_data[m]), .data_oe(mic_oe[m]),
            .sample_in(mic_in[m]), .word(mic_word[m]), .word_count(mic_words[m])
        );
    end

    for (genvar l = 0; l < 4; l++) begin : g_line
        assign i2s_data[l] = mic_oe[2*l]   ? mic_data[2*l]
                           : mic_oe[2*l+1] ? mic_data[2*l+1]
                           : 1'b0; // 100k pull-down
    end

    // ------------------------------------------------------------------
    // Acoustics
    // ------------------------------------------------------------------
    // Mic positions, the cube in channel order (gen_direction_matrix.py MIC_LOCATIONS)
    function automatic real mic_pos(int m, int axis);
        case (axis)
            0:       return m < 4 ? -MIC_HALF : MIC_HALF;
            1:       return m % 2 ? MIC_HALF : -MIC_HALF;
            default: return (m >= 2 && m <= 5) ? MIC_HALF : -MIC_HALF;
        endcase
    endfunction

    typedef enum int { TONE, FM, CHIRP } kind_t;

    // The stimulus playing (Verilator: reals kept out of structs)
    kind_t       s_kind = TONE;
    real         s_amp = 0.0, s_f0 = 1000.0, s_f1 = 0.0, s_fm = 0.0, s_dev = 0.0;
    real         s_u [3] = '{1.0, 0.0, 0.0};
    string       s_name = "silence";

    function automatic real stimulus(real t);
        case (s_kind)
            TONE:  return $sin(2.0 * PI * s_f0 * t);
            FM:    return $sin(2.0 * PI * s_f0 * t + s_dev / s_fm * $sin(2.0 * PI * s_fm * t));
            default: begin // CHIRP: s_f0 to s_f1 over 1 / s_fm, repeating
                automatic real period = 1.0 / s_fm;
                automatic real tau    = t - period * $floor(t / period);
                return $sin(2.0 * PI * (s_f0 * tau + (s_f1 - s_f0) * tau * tau / (2.0 * period)));
            end
        endcase
    endfunction

    // Every frame's 8 words as the DUT keeps them (top 18 bits), by frame number
    logic [8*18-1:0] audio[$];

    always @(negedge i2s_ws) begin
        automatic real             t = $realtime * 1.0e-9;
        automatic logic [8*18-1:0] frame;
        for (int m = 0; m < 8; m++) begin
            automatic real ahead = (mic_pos(m, 0) * s_u[0] + mic_pos(m, 1) * s_u[1] + mic_pos(m, 2) * s_u[2]) / C_SOUND;
            automatic real v     = s_amp * stimulus(t + ahead);
            mic_in[m] = 24'($rtoi(v * 8388607.0));
            frame[(7 - m) * 18 +: 18] = mic_in[m][23 -: 18];
        end
        audio.push_back(frame);
    end

    // ------------------------------------------------------------------
    // Direction estimator reference: the least-squares answer from whole-sample lags
    // ------------------------------------------------------------------
    logic [17:0] df_coef [3 * PAIRS];
    initial $readmemh("../../../src/dsp/building_blocks/direction_matrix.hex", df_coef);

    function automatic void ideal_direction(output real d[3]);
        int p = 0;
        for (int k = 0; k < 3; k++) d[k] = 0.0;
        for (int a = 0; a < 7; a++)
            for (int b = a + 1; b < 8; b++) begin
                // How far b lags a, in decimated samples
                automatic real delay = ((mic_pos(a, 0) - mic_pos(b, 0)) * s_u[0] + (mic_pos(a, 1) - mic_pos(b, 1)) * s_u[1]
                                       + (mic_pos(a, 2) - mic_pos(b, 2)) * s_u[2]) / C_SOUND;
                automatic int  lag   = $rtoi($floor(delay * FS_DEC + 0.5));
                if (lag > MAX_LAG) lag = MAX_LAG;
                if (lag < -MAX_LAG) lag = -MAX_LAG;
                for (int k = 0; k < 3; k++)
                    d[k] += real'($signed(df_coef[k * PAIRS + p])) / real'(1 << COEF_FRAC) * lag;
                p++;
            end
    endfunction

    // ------------------------------------------------------------------
    // Frame reference: CRC-32 / FCS, IPv4 checksum, frame builder (as tb_ethernet)
    // ------------------------------------------------------------------
    function automatic logic [31:0] ref_fcs(byte unsigned d[$]);
        logic [31:0] c = 32'hFFFF_FFFF;
        foreach (d[i]) begin
            c = c ^ {24'h0, d[i]};
            repeat (8) c = (c >> 1) ^ (32'hEDB8_8320 & {32{c[0]}});
        end
        return ~c;
    endfunction

    function automatic logic [15:0] ones_sum(byte unsigned h[$]);
        int unsigned s = 0;
        for (int i = 0; i < h.size(); i += 2)
            s += {h[i], i + 1 < h.size() ? h[i+1] : 8'h00};
        while (s >> 16) s = (s & 32'hFFFF) + (s >> 16);
        return 16'(s);
    endfunction

    function automatic void make_frame(output byte unsigned f[$], input byte unsigned payload[$], input logic [15:0] ident);
        byte unsigned ip[$];
        logic [15:0]  csum;
        logic [31:0]  fcs;
        f  = {};
        ip = {};
        `PUSH_BE(f, DST_MAC, 6)
        `PUSH_BE(f, LOCAL_MAC, 6)
        `PUSH_BE(f, 16'h0800, 2)
        ip.push_back(8'h45);
        ip.push_back(8'h00);
        `PUSH_BE(ip, 20 + 8 + payload.size(), 2)
        `PUSH_BE(ip, ident, 2)
        `PUSH_BE(ip, 16'h4000, 2) // Don't fragment
        ip.push_back(TTL);
        ip.push_back(8'd17);
        `PUSH_BE(ip, 16'h0000, 2)
        `PUSH_BE(ip, LOCAL_IP, 4)
        `PUSH_BE(ip, DST_IP, 4)
        csum = ~ones_sum(ip);
        ip[10] = csum[15:8];
        ip[11] = csum[7:0];
        foreach (ip[i]) f.push_back(ip[i]);
        `PUSH_BE(f, PORT, 2)
        `PUSH_BE(f, PORT, 2)
        `PUSH_BE(f, 8 + payload.size(), 2)
        `PUSH_BE(f, 16'h0000, 2)
        foreach (payload[i]) f.push_back(payload[i]);
        while (f.size() < 60) f.push_back(8'h00);
        fcs = ref_fcs(f);
        for (int k = 0; k < 4; k++) f.push_back(fcs[8*k +: 8]);
    endfunction

    function automatic string show(byte unsigned q[$], int from = 0);
        string s = "";
        for (int i = from; i < q.size(); i++) begin
            if (i == from + 24) return {s, " ..."};
            s = {s, $sformatf(" %02h", q[i])};
        end
        return s;
    endfunction

    // ------------------------------------------------------------------
    // Stream checks
    // ------------------------------------------------------------------
    int unsigned n_frames = 0;
    int unsigned n_pkts [NS] = '{default: 0};

    // Mics: audio[] index of the frame with RTP timestamp 0, once the first packet is found
    bit          mic_found = 1'b0, mic_lost = 1'b0;
    longint      mic_base = 0;
    int unsigned mic_bad_frames = 0;

    function automatic logic [8*18-1:0] mic_frame(byte unsigned rtp[$], int j);
        logic [8*18-1:0] fr;
        for (int c = 0; c < 8; c++) begin
            automatic int          i = 12 + (j * 8 + c) * 3;
            automatic logic [23:0] v = {rtp[i], rtp[i+1], rtp[i+2]};
            fr[(7 - c) * 18 +: 18] = v[23 -: 18];
        end
        return fr;
    endfunction

    function automatic string show_frame(logic [8*18-1:0] fr);
        string s = "";
        for (int c = 0; c < 8; c++) s = {s, $sformatf(" %0d", $signed(fr[(7 - c) * 18 +: 18]))};
        return s;
    endfunction

    // Where each channel's samples in a packet are in the audio sent, if they're anywhere
    function automatic string channel_offsets(byte unsigned rtp[$]);
        string s = "";
        for (int c = 0; c < 8; c++) begin
            automatic int at = -1;
            for (int k = 0; k + FRAMES[1] <= audio.size() && at < 0; k++) begin
                automatic bit ok = 1'b1;
                for (int j = 0; j < FRAMES[1] && ok; j++) begin
                    automatic logic [8*18-1:0] fr = mic_frame(rtp, j);
                    ok = fr[(7 - c) * 18 +: 18] == audio[k + j][(7 - c) * 18 +: 18];
                end
                if (ok) at = k;
            end
            s = {s, $sformatf(" %0d", at)};
        end
        return s;
    endfunction

    function automatic void check_mic(byte unsigned rtp[$], logic [31:0] ts);
        if (!mic_found && !mic_lost) begin
            // The first packet: find its frames in the audio sent. It can start with frames
            // of zeros, from before the mics saw WS and started driving their data lines.
            automatic int j0 = 0;
            while (j0 < FRAMES[1] && mic_frame(rtp, j0) == '0) j0++;
            for (int k = 0; j0 < FRAMES[1] && k + FRAMES[1] - j0 <= audio.size(); k++) begin
                automatic bit ok = 1'b1;
                for (int j = j0; j < FRAMES[1] && ok; j++)
                    ok = mic_frame(rtp, j) == audio[k + j - j0];
                if (ok) begin
                    mic_base  = k - j0 - longint'(ts);
                    mic_found = 1'b1;
                    break;
                end
            end
            if (j0 > 0)
                tb_info($sformatf("Mic packet 0 starts with %0d frames of zeros (the mics starting up)", j0));
            tb_checks++;
            if (!mic_found) begin
                tb_error($sformatf("Mic packet 0 isn't in the audio sent. Its first frame:%s; each channel's samples are at audio frame (-1: nowhere)%s",
                                   show_frame(mic_frame(rtp, 0)), channel_offsets(rtp)));
                mic_lost = 1'b1; // Don't look again
            end else begin
                tb_info($sformatf("Mic packet 0 is audio frame %0d on", mic_base + ts + j0));
            end
            return;
        end
        if (!mic_found) return;
        for (int j = 0; j < FRAMES[1]; j++) begin
            automatic longint k = mic_base + longint'(ts) + j;
            tb_checks++;
            if (k < 0 || k >= audio.size() || mic_frame(rtp, j) != audio[k]) begin
                mic_bad_frames++;
                tb_error_once("mic frame", $sformatf("Mic packet %0d frame %0d (audio frame %0d):%s, sent%s", n_pkts[1], j, k,
                              show_frame(mic_frame(rtp, j)), k < audio.size() ? show_frame(audio[k]) : " (not yet)"));
            end
        end
    endfunction

    int unsigned adc_bad = 0;
    function automatic void check_adc(byte unsigned rtp[$]);
        for (int i = 0; i < FRAMES[0] * 8; i++) begin
            tb_checks++;
            if (rtp[12 + i] != ADC_CODES[i % 8]) begin
                adc_bad++;
                tb_error_once("adc sample", $sformatf("ADC packet %0d sample %0d (channel %0d) is 0x%02h, expected 0x%02h",
                              n_pkts[0], i, i % 8, rtp[12 + i], ADC_CODES[i % 8]));
            end
        end
    endfunction

    // Directions: per stimulus, the window in which they're checked and what was seen
    localparam int MAX_SC = 4;
    realtime     sc_from [MAX_SC], sc_to [MAX_SC];
    real         sc_u    [MAX_SC][3];
    string       sc_name [MAX_SC];
    int unsigned sc_n    [MAX_SC] = '{default: 0};
    real         sc_sum  [MAX_SC][3];
    real         sc_max_angle [MAX_SC], sc_mag_lo [MAX_SC], sc_mag_hi [MAX_SC];
    real         sc_ideal [MAX_SC][3];  // least squares from whole-sample lags
    real         sc_max_dev [MAX_SC];   // furthest from that
    int          n_sc = 0;
    int unsigned n_dir_unchecked = 0;

    initial for (int s = 0; s < MAX_SC; s++) begin
        sc_from[s] = 1.0e18;
        sc_to[s]   = 0.0;
        for (int k = 0; k < 3; k++) sc_sum[s][k] = 0.0;
        sc_max_angle[s] = 0.0;
        sc_max_dev[s] = 0.0;
        sc_mag_lo[s] = 1.0e9;
        sc_mag_hi[s] = 0.0;
    end

    function automatic void check_dir(byte unsigned rtp[$]);
        real v[3];
        real mag, cosine, angle, dev;
        int  s = -1;
        for (int k = 0; k < 3; k++)
            v[k] = real'($signed({rtp[12 + 2*k], rtp[13 + 2*k]})) / real'(1 << DF_FRAC);
        for (int i = 0; i < n_sc; i++)
            if ($realtime >= sc_from[i] && $realtime <= sc_to[i]) s = i;
        if (s < 0) begin
            n_dir_unchecked++;
            return;
        end
        mag    = $sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        cosine = mag > 0.0 ? (v[0] * sc_u[s][0] + v[1] * sc_u[s][1] + v[2] * sc_u[s][2]) / mag : -1.0;
        if (cosine > 1.0) cosine = 1.0;
        angle  = $acos(cosine) * 180.0 / PI;
        sc_n[s]++;
        for (int k = 0; k < 3; k++) sc_sum[s][k] += v[k];
        if (angle > sc_max_angle[s]) sc_max_angle[s] = angle;
        if (mag < sc_mag_lo[s]) sc_mag_lo[s] = mag;
        if (mag > sc_mag_hi[s]) sc_mag_hi[s] = mag;
        dev = $sqrt((v[0] - sc_ideal[s][0]) ** 2 + (v[1] - sc_ideal[s][1]) ** 2 + (v[2] - sc_ideal[s][2]) ** 2);
        if (dev > sc_max_dev[s]) sc_max_dev[s] = dev;
        tb_checks++;
        if (dev > IDEAL_TOL)
            tb_error_once({"direction ideal ", sc_name[s]},
                $sformatf("%s: direction (%.3f, %.3f, %.3f) is %.3f from the whole-sample-lag answer (%.3f, %.3f, %.3f)",
                          sc_name[s], v[0], v[1], v[2], dev, sc_ideal[s][0], sc_ideal[s][1], sc_ideal[s][2]));
        tb_checks++;
        if (angle > ANGLE_TOL || mag < MAG_LO || mag > MAG_HI)
            tb_error_once({"direction ", sc_name[s]},
                $sformatf("%s: direction (%.3f, %.3f, %.3f) is %.1f degrees from the source (%.3f, %.3f, %.3f), length %.3f",
                          sc_name[s], v[0], v[1], v[2], angle, sc_u[s][0], sc_u[s][1], sc_u[s][2], mag));
    endfunction

    // A frame off the wire, preamble and SFD gone
    function automatic void check_frame(byte unsigned f[$]);
        byte unsigned rtp[$], exp[$];
        logic [31:0]  ssrc, ts;
        logic [15:0]  seq;
        int           s, udp_len;
        bit           ok;
        tb_checks++;
        if (f.size() < 14 + 20 + 8 + 12 + 4) begin
            tb_error($sformatf("Frame %0d: only %0d bytes {%s }", n_frames, f.size(), show(f)));
            return;
        end
        udp_len = {f[38], f[39]};
        rtp = {};
        for (int i = 42; i < 34 + udp_len && i < f.size() - 4; i++) rtp.push_back(f[i]);

        // Headers and CRC: rebuild the frame around its RTP packet
        make_frame(exp, rtp, 16'(n_frames));
        ok = exp.size() == f.size();
        foreach (exp[i]) if (ok && exp[i] != f[i]) ok = 1'b0;
        if (!ok) begin
            tb_error($sformatf("Frame %0d: %0d bytes, headers or CRC not as expected\n    got      {%s }\n    expected {%s }",
                               n_frames, f.size(), show(f), show(exp)));
            return;
        end

        // RTP header
        ssrc = {rtp[8], rtp[9], rtp[10], rtp[11]};
        s = ssrc == SSRCS[0] ? 0 : ssrc == SSRCS[1] ? 1 : ssrc == SSRCS[2] ? 2 : -1;
        if (s < 0) begin
            tb_error($sformatf("Frame %0d: unknown SSRC 0x%08h", n_frames, ssrc));
            return;
        end
        seq = {rtp[2], rtp[3]};
        ts  = {rtp[4], rtp[5], rtp[6], rtp[7]};
        tb_checks++;
        if (rtp[0] != 8'h80 || rtp[1] != {n_pkts[s] == 0, PTS[s]} || seq != 16'(n_pkts[s]) || ts != n_pkts[s] * FRAMES[s]
                || rtp.size() != 12 + FRAMES[s] * CHANNELS[s] * BYTES[s])
            tb_error_once({"rtp ", NAMES[s]},
                $sformatf("%s packet %0d: RTP header {%s }, %0d bytes: expected sequence %0d, timestamp %0d, %0d bytes",
                          NAMES[s], n_pkts[s], show(rtp), rtp.size(), 16'(n_pkts[s]), n_pkts[s] * FRAMES[s],
                          12 + FRAMES[s] * CHANNELS[s] * BYTES[s]));
        else
            case (s)
                0: check_adc(rtp);
                1: check_mic(rtp, ts);
                default: check_dir(rtp);
            endcase
        n_pkts[s]++;
    endfunction

    // ------------------------------------------------------------------
    // Wire monitor
    // ------------------------------------------------------------------
    logic [1:0] tx_dibits[$];

    function automatic void wire_frame_done();
        byte unsigned b[$];
        bit           ok;
        b = {};
        for (int i = 0; i + 3 < tx_dibits.size(); i += 4)
            b.push_back({tx_dibits[i+3], tx_dibits[i+2], tx_dibits[i+1], tx_dibits[i]});
        tb_checks++;
        ok = tx_dibits.size() % 4 == 0 && b.size() >= 8 && b[7] == 8'hD5;
        for (int i = 0; i < 7 && ok; i++) ok = b[i] == 8'h55;
        if (!ok) begin
            tb_error($sformatf("Bad frame on the wire (%0d dibits): {%s }", tx_dibits.size(), show(b)));
        end else begin
            repeat (8) void'(b.pop_front());
            check_frame(b);
        end
        n_frames++;
    endfunction

    always @(posedge clk_50) begin
        if (rmii_txen === 1'b1) begin
            tx_dibits.push_back(rmii_txd);
        end else if (tx_dibits.size() > 0) begin
            wire_frame_done();
            tx_dibits.delete();
        end
    end

    // ------------------------------------------------------------------
    // DSP monitor: what goes through each stage, and decimator results lost. The decimator
    // has no backpressure, so a result the correlator hasn't taken by the time the next one
    // is ready is overwritten: the correlator must keep up.
    // ------------------------------------------------------------------
    int unsigned       n_decimated = 0, n_decim_lost = 0, n_xcorr = 0, n_peaks = 0, n_estimates = 0;
    bit                decim_stalled = 1'b0;
    logic [3+36-1:0]   decim_stalled_beat;

    always @(posedge clk_core) begin
        automatic logic [3+36-1:0] beat = {dut.tdoa_direction_finder_i.decim_out_if.tid, dut.tdoa_direction_finder_i.decim_out_if.tdata};
        if (decim_stalled && dut.tdoa_direction_finder_i.decim_out_if.tvalid && beat !== decim_stalled_beat) begin
            n_decim_lost++;
            tb_error_once("decimator lost", "A decimator result was overwritten before the correlator took it: the correlator isn't keeping up");
        end
        decim_stalled      = dut.tdoa_direction_finder_i.decim_out_if.tvalid && !dut.tdoa_direction_finder_i.decim_out_if.tready;
        decim_stalled_beat = beat;
        if (dut.tdoa_direction_finder_i.decim_out_if.tvalid && dut.tdoa_direction_finder_i.decim_out_if.tready) n_decimated++;
        if (dut.tdoa_direction_finder_i.xcorr_out_if.tvalid && dut.tdoa_direction_finder_i.xcorr_out_if.tready
                && dut.tdoa_direction_finder_i.xcorr_out_if.tlast) n_xcorr++;
        if (dut.tdoa_direction_finder_i.peak_finder_out_if.tvalid && dut.tdoa_direction_finder_i.peak_finder_out_if.tready) n_peaks++;
        if (dut.tdoa_direction_finder_i.direction_estimator_out_if.tvalid
                && dut.tdoa_direction_finder_i.direction_estimator_out_if.tready) n_estimates++;
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic play(string name, kind_t kind, real amp, real f0, real f1, real fm, real dev,
                        real az_deg, real el_deg, real ms, bit check);
        real az = az_deg * PI / 180.0, el = el_deg * PI / 180.0;
        real ideal[3];
        s_name = name;
        s_kind = kind;
        s_amp  = amp;
        s_f0   = f0;
        s_f1   = f1;
        s_fm   = fm;
        s_dev  = dev;
        s_u[0] = $cos(el) * $cos(az);
        s_u[1] = $cos(el) * $sin(az);
        s_u[2] = $sin(el);
        if (check) begin
            sc_name[n_sc] = name;
            for (int k = 0; k < 3; k++) sc_u[n_sc][k] = s_u[k];
            sc_from[n_sc] = $realtime + SETTLE_MS * 1.0e6;
            sc_to[n_sc]   = $realtime + ms * 1.0e6;
            n_sc++;
            ideal_direction(ideal);
            for (int k = 0; k < 3; k++) sc_ideal[n_sc - 1][k] = ideal[k];
            tb_info($sformatf("%s from azimuth %.0f, elevation %.0f: (%.3f, %.3f, %.3f), %.1f ms (checked after %.1f ms); whole-sample lags give (%.3f, %.3f, %.3f)",
                              name, az_deg, el_deg, s_u[0], s_u[1], s_u[2], ms, SETTLE_MS, ideal[0], ideal[1], ideal[2]));
        end else begin
            tb_info($sformatf("%s, %.1f ms (audio only)", name, ms));
        end
        #(ms * 1.0e6);
    endtask

    initial begin
        int unsigned n_dir_sc = 2;
        real         check_ms = 15.0;
        real         stop_ms  = 0.0;
        void'($value$plusargs("scenarios=%d", n_dir_sc));
        void'($value$plusargs("check_ms=%f", check_ms));
        if (n_dir_sc < 1) n_dir_sc = 1;
        if (n_dir_sc > 3) n_dir_sc = 3;
        if ($value$plusargs("stop_ms=%f", stop_ms))
            fork begin
                #(stop_ms * 1.0e6);
                tb_info($sformatf("Stopped at %.1f ms by +stop_ms: %0d frames, %0d ADC, %0d mic, %0d direction packets",
                                  stop_ms, n_frames, n_pkts[0], n_pkts[1], n_pkts[2]));
                tb_finish();
            end join_none

        tb_info($sformatf("Testing cain_top: I2S mics to RTP over RMII, %0d directional stimuli, each checked for %.1f ms. Correlator MAC lanes: %0d",
                          n_dir_sc, check_ms, dut.DSP_XCORR_MACS));
        #2000;
        rst_in_n = 1'b1;

        play("tone", TONE, 0.5, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 1'b0);
        play("fm", FM, 0.5, 4000.0, 0.0, 170.0, 3000.0, 30.0, 20.0, SETTLE_MS + check_ms, 1'b1);
        if (n_dir_sc >= 2)
            play("chirp", CHIRP, 0.4, 1500.0, 8500.0, 100.0, 0.0, -120.0, -35.0, SETTLE_MS + check_ms, 1'b1);
        if (n_dir_sc >= 3)
            play("fm2", FM, 0.6, 6000.0, 0.0, 230.0, 2500.0, 160.0, 55.0, SETTLE_MS + check_ms, 1'b1);
        #(3.0e6); // Last packets out

        // Results
        for (int s = 0; s < n_sc; s++) begin
            automatic real mean[3];
            for (int k = 0; k < 3; k++) mean[k] = sc_n[s] > 0 ? sc_sum[s][k] / sc_n[s] : 0.0;
            tb_info($sformatf("%s: %0d directions, mean (%.3f, %.3f, %.3f) for (%.3f, %.3f, %.3f): worst %.1f degrees off, length %.3f-%.3f, at most %.3f from the whole-sample-lag answer",
                              sc_name[s], sc_n[s], mean[0], mean[1], mean[2], sc_u[s][0], sc_u[s][1], sc_u[s][2],
                              sc_max_angle[s], sc_mag_lo[s], sc_mag_hi[s], sc_max_dev[s]));
            `CHECK(sc_n[s] >= int'(check_ms * FS_DEC / 16.0 / 1.0e3 * 0.8),
                   $sformatf("%s: only %0d directions checked in %.1f ms", sc_name[s], sc_n[s], check_ms))
        end
        tb_info($sformatf("%0d frames: %0d ADC, %0d mic, %0d direction packets (%0d directions while settling, not checked)",
                          n_frames, n_pkts[0], n_pkts[1], n_pkts[2], n_dir_unchecked));
        tb_info($sformatf("DSP: %0d decimated samples, %0d correlation packets, %0d peaks, %0d direction estimates",
                          n_decimated, n_xcorr, n_peaks, n_estimates));
        `CHECK(n_decim_lost == 0, $sformatf("%0d decimator results lost: the correlator didn't keep up", n_decim_lost))
        // About one mic packet per FRAMES[1] I2S frames from the first
        `CHECK(mic_found, "Mic packets never matched the audio")
        `CHECK(n_pkts[1] * FRAMES[1] + 2 * FRAMES[1] >= audio.size() - (mic_base > 0 ? mic_base : 0),
               $sformatf("%0d mic packets for %0d audio frames", n_pkts[1], audio.size()))
        `CHECK(n_pkts[0] > 0, "No ADC packets")
        `CHECK(mic_bad_frames == 0 && adc_bad == 0, $sformatf("%0d mic frames and %0d ADC samples wrong", mic_bad_frames, adc_bad))
        tb_finish();
    end

endmodule
