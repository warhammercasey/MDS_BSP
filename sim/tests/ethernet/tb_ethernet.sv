`timescale 1ns/1ps
`include "tb_macros.svh"

// ethernet test: ADC and microphone sample streams in (on clk_io), direction vectors in
// (on clk_dir, like clk_core), RTP/UDP frames out on TXEN/TXD (clk_50), as on the board.
//
// RTP_PAYLOAD_BYTES is 64 here, which isn't a multiple of 24, so the streams have
// different packet sizes: ADC 8 frames x 8 x 1 byte = 64, mics 2 frames x 8 x 3 bytes
// = 48, and directions 2 x {x, y, z} x 2 bytes = 12. That checks the datagram length
// follows the stream.
//
// Every frame on the wire is rebuilt here and compared byte for byte: Ethernet, IPv4
// (identification counting up per frame) and UDP headers from the registers, own
// CRC-32 and checksum, and the RTP packet from a model of each stream (sequence,
// timestamp, marker on the first, SSRC, samples big endian and left justified).
//
//   rx         a datagram sent to us shows in the RX registers (RX path is wired)
//   real rate  samples at about the board's rates
//   saturated  both streams send as fast as they're taken: everything backs up to the
//              sources, and frames must still be whole, correct and in order, with
//              both streams getting through
// At the end TX_COUNT must match the frames seen.
//
// TX_BUFFER is the DUT's: the ethernet_no_tx_buffer test runs this with it off, where
// rmii_tx must never be left waiting for a byte mid-frame (checked directly too).
module tb_ethernet #(
    parameter int TX_BUFFER = 1
);
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(30_000_000) // 30 ms

    localparam real CLK_PERIOD_NS     = 20.0;                 // clk_50
    localparam real CLK_IO_PERIOD_NS  = 1.0e9 / 73.5714e6;    // clk_io
    localparam real CLK_DIR_PERIOD_NS = 5.0;                  // clk_core

    localparam int          RTP_PAYLOAD_BYTES = 64;
    localparam int          DIR_FRAMES = 2;
    localparam logic [31:0] ADC_SSRC = 32'h4144_4330;
    localparam logic [31:0] MIC_SSRC = 32'h4D49_4330;
    localparam logic [31:0] DIR_SSRC = 32'h4449_5230;
    localparam logic [6:0]  ADC_PT   = 7'd96;
    localparam logic [6:0]  MIC_PT   = 7'd97;
    localparam logic [6:0]  DIR_PT   = 7'd98;

    // Per stream: 0 = ADC, 1 = mics, 2 = directions
    localparam int          NS            = 3;
    localparam int          BITS     [NS] = '{8, 18, 16};
    localparam int          BYTES    [NS] = '{1, 3, 2};
    localparam int          CHANNELS [NS] = '{8, 8, 3};
    localparam int          FRAMES   [NS] = '{RTP_PAYLOAD_BYTES / (8 * 1), RTP_PAYLOAD_BYTES / (8 * 3), DIR_FRAMES};
    localparam logic [31:0] SSRCS    [NS] = '{ADC_SSRC, MIC_SSRC, DIR_SSRC};
    localparam logic [6:0]  PTS      [NS] = '{ADC_PT, MIC_PT, DIR_PT};
    localparam string       NAMES    [NS] = '{"ADC", "mic", "direction"};

    `define PUSH_BE(Q, V, N) for (int k_ = (N) - 1; k_ >= 0; k_--) Q.push_back(8'((V) >> (8 * k_)));

    logic clk = 1'b0, clk_io = 1'b0, clk_dir = 1'b0;
    always #(CLK_PERIOD_NS / 2)     clk     = ~clk;
    always #(CLK_IO_PERIOD_NS / 2)  clk_io  = ~clk_io;
    always #(CLK_DIR_PERIOD_NS / 2) clk_dir = ~clk_dir;
    logic rstn = 1'b0, rstn_io = 1'b0, rstn_dir = 1'b0;

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    apb_if #(.ADDR_WIDTH(8), .DATA_WIDTH(32), .SEL_WIDTH(1)) apb ();
    axis_tid_if #(.WIDTH(8),  .ID_WIDTH(3)) adc_axis ();
    axis_tid_if #(.WIDTH(18), .ID_WIDTH(3)) mic_axis ();
    axis_if     #(.WIDTH(48))               dir_axis ();

    logic [1:0] txd;
    logic       txen;
    logic [1:0] phy_rxd = 2'b00;
    logic       phy_crs = 1'b0;

    ethernet #(
        .RTP_PAYLOAD_BYTES(RTP_PAYLOAD_BYTES),
        .ADC_BITS(8),  .ADC_CHANNELS(8), .ADC_ID_WIDTH(3), .ADC_SSRC(ADC_SSRC), .ADC_PAYLOAD_TYPE(ADC_PT),
        .MIC_BITS(18), .MIC_CHANNELS(8), .MIC_ID_WIDTH(3), .MIC_SSRC(MIC_SSRC), .MIC_PAYLOAD_TYPE(MIC_PT),
        .DIR_BITS(16), .DIR_FRAMES(DIR_FRAMES), .DIR_SSRC(DIR_SSRC), .DIR_PAYLOAD_TYPE(DIR_PT),
        .TX_BUFFER(TX_BUFFER)
    ) dut (
        .clk(clk), .rstn(rstn), .clk_io(clk_io), .rstn_io(rstn_io), .clk_dir(clk_dir), .rstn_dir(rstn_dir),
        .rmii_txd(txd), .rmii_txen(txen), .rmii_rstn(), .rmii_int(1'b1),
        .rmii_crs_dv(phy_crs), .rmii_rxd(phy_rxd), .rmii_rxerr(1'b0),
        .apb(apb),
        .adc_axis(adc_axis),
        .mic_axis(mic_axis),
        .dir_axis(dir_axis)
    );

    // ------------------------------------------------------------------
    // Reference: CRC-32 / FCS, IPv4 checksum, frame builder
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

    // An Ethernet/IPv4/UDP frame, FCS included
    function automatic void make_frame(output byte unsigned f[$], input byte unsigned payload[$],
            input logic [47:0] dst_mac, input logic [47:0] src_mac,
            input logic [31:0] src_ip,  input logic [31:0] dst_ip,
            input logic [15:0] src_port, input logic [15:0] dst_port,
            input logic [7:0]  ttl, input logic [15:0] ident, input bit df, input logic [15:0] udp_csum);
        byte unsigned ip[$];
        logic [15:0]  csum;
        logic [31:0]  fcs;
        f  = {};
        ip = {};
        `PUSH_BE(f, dst_mac, 6)
        `PUSH_BE(f, src_mac, 6)
        `PUSH_BE(f, 16'h0800, 2)
        ip.push_back(8'h45);
        ip.push_back(8'h00);
        `PUSH_BE(ip, 20 + 8 + payload.size(), 2)
        `PUSH_BE(ip, ident, 2)
        `PUSH_BE(ip, {1'b0, df, 14'h0}, 2)
        ip.push_back(ttl);
        ip.push_back(8'd17);
        `PUSH_BE(ip, 16'h0000, 2)
        `PUSH_BE(ip, src_ip, 4)
        `PUSH_BE(ip, dst_ip, 4)
        csum = ~ones_sum(ip);
        ip[10] = csum[15:8];
        ip[11] = csum[7:0];
        foreach (ip[i]) f.push_back(ip[i]);
        `PUSH_BE(f, src_port, 2)
        `PUSH_BE(f, dst_port, 2)
        `PUSH_BE(f, 8 + payload.size(), 2)
        `PUSH_BE(f, udp_csum, 2)
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
    // RTP model: the packets each stream should send, built as samples are queued
    // (in this test every sample goes in, in channel order)
    // ------------------------------------------------------------------
    // Queue arrays sized to a power of 2: Verilator 5.020 generates C++ that doesn't
    // compile when indexing an array of queues of another size
    byte unsigned expected [4][$][$];
    byte unsigned m_pkt    [4][$];
    int unsigned  m_count  [NS] = '{default: 0};
    int unsigned  m_seq    [NS] = '{default: 0};
    int unsigned  m_frames [NS] = '{default: 0};

    function automatic void model(int s, int unsigned tid, logic [23:0] data);
        logic [23:0] v;
        if (m_count[s] == 0) begin
            m_pkt[s] = {};
            m_pkt[s].push_back(8'h80);
            m_pkt[s].push_back({m_seq[s] == 0, PTS[s]});
            `PUSH_BE(m_pkt[s], m_seq[s], 2)
            `PUSH_BE(m_pkt[s], m_frames[s], 4)
            `PUSH_BE(m_pkt[s], SSRCS[s], 4)
        end
        v = data << (BYTES[s] * 8 - BITS[s]);
        for (int k = BYTES[s] - 1; k >= 0; k--) m_pkt[s].push_back(v[8*k +: 8]);
        if (tid == 0) m_frames[s]++;
        m_count[s]++;
        if (m_count[s] == FRAMES[s] * CHANNELS[s]) begin
            expected[s].push_back(m_pkt[s]);
            m_seq[s]++;
            m_count[s] = 0;
        end
    endfunction

    // ------------------------------------------------------------------
    // Sample sources (clk_io): {tid, data} from a queue, `period` cycles apart
    // ------------------------------------------------------------------
    logic [26:0] src_q  [2][$];
    int unsigned period [2] = '{0, 0};

    function automatic void queue_frames(int s, int unsigned n);
        repeat (n) begin
            for (int c = 0; c < CHANNELS[s]; c++) begin
                logic [23:0] d = 24'($urandom) & ((24'h1 << BITS[s]) - 1);
                src_q[s].push_back({3'(c), d});
                model(s, c, d);
            end
        end
    endfunction

    // Directions (clk_dir): {x, y, z} beats from a queue, dir_period cycles apart
    logic [47:0] dir_q[$];
    int unsigned dir_period = 0, dir_wait = 0;

    function automatic void queue_directions(int unsigned n);
        repeat (n) begin
            logic [47:0] v = {16'($urandom), 16'($urandom), 16'($urandom)};
            dir_q.push_back(v);
            for (int c = 0; c < 3; c++) model(2, c, 24'(v[(2 - c) * 16 +: 16]));
        end
    endfunction

    always @(posedge clk_dir) begin
        bit hs;
        if (!rstn_dir) begin
            dir_axis.tvalid <= 1'b0;
            dir_axis.tdata  <= '0;
        end else begin
            hs = dir_axis.tvalid && dir_axis.tready;
            if (hs) begin
                void'(dir_q.pop_front());
                dir_wait = dir_period;
            end
            if (!dir_axis.tvalid || hs) begin
                if (dir_wait > 0) begin
                    dir_wait--;
                    dir_axis.tvalid <= 1'b0;
                end else if (dir_q.size() > 0) begin
                    dir_axis.tvalid <= 1'b1;
                    dir_axis.tdata  <= dir_q[0];
                end else begin
                    dir_axis.tvalid <= 1'b0;
                end
            end
        end
    end

    int unsigned adc_wait = 0, mic_wait = 0;

    always @(posedge clk_io) begin
        bit hs;
        if (!rstn_io) begin
            adc_axis.tvalid <= 1'b0;
            adc_axis.tdata  <= '0;
            adc_axis.tid    <= '0;
        end else begin
            hs = adc_axis.tvalid && adc_axis.tready;
            if (hs) begin
                void'(src_q[0].pop_front());
                adc_wait = period[0];
            end
            if (!adc_axis.tvalid || hs) begin
                if (adc_wait > 0) begin
                    adc_wait--;
                    adc_axis.tvalid <= 1'b0;
                end else if (src_q[0].size() > 0) begin
                    adc_axis.tvalid <= 1'b1;
                    adc_axis.tid    <= src_q[0][0][26:24];
                    adc_axis.tdata  <= src_q[0][0][7:0];
                end else begin
                    adc_axis.tvalid <= 1'b0;
                end
            end
        end
    end

    always @(posedge clk_io) begin
        bit hs;
        if (!rstn_io) begin
            mic_axis.tvalid <= 1'b0;
            mic_axis.tdata  <= '0;
            mic_axis.tid    <= '0;
        end else begin
            hs = mic_axis.tvalid && mic_axis.tready;
            if (hs) begin
                void'(src_q[1].pop_front());
                mic_wait = period[1];
            end
            if (!mic_axis.tvalid || hs) begin
                if (mic_wait > 0) begin
                    mic_wait--;
                    mic_axis.tvalid <= 1'b0;
                end else if (src_q[1].size() > 0) begin
                    mic_axis.tvalid <= 1'b1;
                    mic_axis.tid    <= src_q[1][0][26:24];
                    mic_axis.tdata  <= src_q[1][0][17:0];
                end else begin
                    mic_axis.tvalid <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Wire monitor: every frame checked as it ends
    // ------------------------------------------------------------------
    logic [47:0]  cfg_local_mac, cfg_dst_mac;
    logic [31:0]  cfg_local_ip, cfg_dst_ip;
    logic [7:0]   cfg_ttl;
    logic [15:0]  cfg_src_port, cfg_dst_port;

    logic [1:0]   tx_dibits[$];
    int unsigned  n_frames = 0;
    int unsigned  n_from [NS] = '{default: 0};

    function automatic void check_frame(byte unsigned f[$]);
        byte unsigned rtp[$], exp_rtp[$], exp[$];
        logic [31:0]  ssrc;
        int           s;
        bit           ok;
        tb_checks++;
        if (f.size() < 14 + 20 + 8 + 12 + 4) begin
            tb_error($sformatf("Frame %0d: only %0d bytes {%s }", n_frames, f.size(), show(f)));
            return;
        end
        // The RTP packet is the UDP payload (by the UDP length, so any padding is left out)
        rtp = {};
        for (int i = 42; i < 34 + {f[38], f[39]} && i < f.size() - 4; i++) rtp.push_back(f[i]);
        ssrc = {rtp[8], rtp[9], rtp[10], rtp[11]};
        s = ssrc == ADC_SSRC ? 0 : ssrc == MIC_SSRC ? 1 : ssrc == DIR_SSRC ? 2 : -1;
        if (s < 0) begin
            tb_error($sformatf("Frame %0d: unknown SSRC 0x%08h", n_frames, ssrc));
            return;
        end
        if (expected[s].size() == 0) begin
            tb_error($sformatf("Frame %0d: %s packet that wasn't expected {%s }", n_frames, NAMES[s], show(rtp)));
            return;
        end
        exp_rtp = expected[s].pop_front();
        make_frame(exp, exp_rtp, cfg_dst_mac, cfg_local_mac, cfg_local_ip, cfg_dst_ip,
                   cfg_src_port, cfg_dst_port, cfg_ttl, 16'(n_frames), 1'b1, 16'h0000);
        ok = exp.size() == f.size();
        foreach (exp[i]) if (ok && exp[i] != f[i]) ok = 1'b0;
        if (!ok)
            tb_error($sformatf("Frame %0d (%s packet %0d): %0d bytes, expected %0d\n    got      {%s }\n    expected {%s }\n    RTP got      {%s }\n    RTP expected {%s }",
                               n_frames, NAMES[s], n_from[s], f.size(), exp.size(), show(f), show(exp), show(f, 42), show(exp, 42)));
        n_from[s]++;
    endfunction

    function automatic void wire_frame_done();
        byte unsigned b[$];
        b = {};
        for (int i = 0; i + 3 < tx_dibits.size(); i += 4)
            b.push_back({tx_dibits[i+3], tx_dibits[i+2], tx_dibits[i+1], tx_dibits[i]});
        tb_checks++;
        if (tx_dibits.size() % 4 != 0 || b.size() < 8 || b[7] != 8'hD5) begin
            tb_error($sformatf("Bad frame on the wire (%0d dibits): {%s }", tx_dibits.size(), show(b)));
            return;
        end
        repeat (8) void'(b.pop_front()); // Preamble and SFD
        check_frame(b);
        n_frames++;
    endfunction

    // rmii_tx asks for each byte just as it needs it: no byte then is an underrun
    int unsigned n_underruns = 0;

    always @(posedge clk) begin
        if (rstn) begin
            if (dut.mac_tx.tready && !dut.mac_tx.tvalid) begin
                n_underruns++;
                tb_error_once("underrun", $sformatf("TX underrun: rmii_tx needed a byte mid-frame (frame %0d)", n_frames));
            end
            if (txen === 1'b1) begin
                tx_dibits.push_back(txd);
            end else if (tx_dibits.size() > 0) begin
                wire_frame_done();
                tx_dibits = {};
            end
        end
    end

    // ------------------------------------------------------------------
    // PHY RX
    // ------------------------------------------------------------------
    task automatic phy_byte(byte unsigned b);
        for (int k = 0; k < 4; k++) begin
            phy_crs <= 1'b1;
            phy_rxd <= b[2*k +: 2];
            @(posedge clk);
        end
    endtask

    task automatic phy_send(byte unsigned f[$]);
        repeat (7) phy_byte(8'h55);
        phy_byte(8'hD5);
        foreach (f[i]) phy_byte(f[i]);
        phy_crs <= 1'b0;
        phy_rxd <= 2'b00;
        repeat (48) @(posedge clk);
    endtask

    // ------------------------------------------------------------------
    // APB master (drives and samples on the falling edge)
    // ------------------------------------------------------------------
    initial begin
        apb.psel[0] = 1'b0;
        apb.penable = 1'b0;
        apb.pwrite  = 1'b0;
        apb.paddr   = '0;
        apb.pwdata  = '0;
    end

    task automatic apb_xfer(bit wr, int addr, logic [31:0] wdata, output logic [31:0] rdata, output bit err);
        @(negedge clk);
        apb.psel[0] = 1'b1;
        apb.penable = 1'b0;
        apb.paddr   = 8'(addr);
        apb.pwrite  = wr;
        apb.pwdata  = wdata;
        @(negedge clk);
        apb.penable = 1'b1;
        do @(negedge clk); while (!apb.pready);
        rdata = apb.prdata;
        err   = apb.pslverr;
        @(negedge clk);
        apb.psel[0] = 1'b0;
        apb.penable = 1'b0;
    endtask

    task automatic apb_write(int addr, logic [31:0] data);
        logic [31:0] r;
        bit          err;
        apb_xfer(1'b1, addr, data, r, err);
        `CHECK(!err, $sformatf("APB write 0x%02h = 0x%08h: PSLVERR", addr, data))
    endtask

    task automatic apb_check(int addr, logic [31:0] exp);
        logic [31:0] r;
        bit          err;
        apb_xfer(1'b0, addr, '0, r, err);
        `CHECK(!err, $sformatf("APB read 0x%02h: PSLVERR", addr))
        `CHECK_EQ(r, exp, $sformatf("APB read 0x%02h", addr))
    endtask

    localparam int TX_DST_MAC_HI = 'h01, TX_DST_MAC_LO = 'h02, LOCAL_MAC_HI = 'h03, LOCAL_MAC_LO = 'h04,
                   LOCAL_IP = 'h05, TX_DST_IP = 'h06, TX_TTL = 'h07, TX_PORTS = 'h08, TX_COUNT = 'h09,
                   RX_COUNT = 'h10, RX_SRC_IP = 'h15, RX_PORTS = 'h18, RX_LENGTH = 'h19;

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic wait_sent(string name);
        int unsigned t = 0;
        while ((expected[0].size() > 0 || expected[1].size() > 0 || expected[2].size() > 0) && t < 5_000_000) begin
            @(posedge clk);
            t++;
        end
        repeat (200) @(posedge clk);
        for (int s = 0; s < NS; s++)
            `CHECK(expected[s].size() == 0, $sformatf("%s: %0d %s packets never came out", name, expected[s].size(), NAMES[s]))
    endtask

    initial begin
        byte unsigned p[$], f[$];
        int unsigned  adc0, mic0, dir0;

        tb_info($sformatf("Testing ethernet: RTP payload %0d bytes (ADC %0d frames, mics %0d, directions %0d per packet), TX buffer %s",
                          RTP_PAYLOAD_BYTES, FRAMES[0], FRAMES[1], FRAMES[2], TX_BUFFER ? "on" : "off"));
        repeat (5) @(posedge clk);
        rstn     = 1'b1;
        rstn_io  = 1'b1;
        rstn_dir = 1'b1;
        repeat (5) @(posedge clk);

        // Settings: our addresses stay at reset values
        cfg_local_mac = 48'h02_00_00_00_00_01;
        cfg_local_ip  = 32'hC0A8_0064; // 192.168.0.100
        cfg_dst_mac   = 48'h00_11_22_33_44_55;
        cfg_dst_ip    = 32'hC0A8_000A; // 192.168.0.10
        cfg_ttl       = 8'd32;
        cfg_src_port  = 16'd6000;
        cfg_dst_port  = 16'd6001;
        apb_write(TX_DST_MAC_HI, {16'h0, cfg_dst_mac[47:32]});
        apb_write(TX_DST_MAC_LO, cfg_dst_mac[31:0]);
        apb_write(TX_DST_IP, cfg_dst_ip);
        apb_write(TX_TTL, {24'h0, cfg_ttl});
        apb_write(TX_PORTS, {cfg_src_port, cfg_dst_port});

        // --- RX: a datagram to us shows in the registers ---
        tb_info("RX");
        p = {};
        repeat (20) p.push_back(8'($urandom));
        make_frame(f, p, cfg_local_mac, cfg_dst_mac, cfg_dst_ip, cfg_local_ip, 4000, 4001, 8'd64, 16'h1, 1'b0, 16'h0);
        phy_send(f);
        repeat (500) @(posedge clk);
        apb_check(RX_COUNT, 1);
        apb_check(RX_SRC_IP, cfg_dst_ip);
        apb_check(RX_PORTS, {16'd4000, 16'd4001});
        apb_check(RX_LENGTH, 20);

        // --- About the board's rates: ADC 200k samples/s, mics 4 every 576 clk_io ---
        // (directions faster than the board's ~375/s, so the test doesn't take all day)
        tb_info("Real rate");
        period     = '{368, 144};
        dir_period = 4000;
        queue_frames(0, 3 * FRAMES[0]);
        queue_frames(1, 30 * FRAMES[1]);
        queue_directions(10 * FRAMES[2]);
        wait_sent("real rate");

        // --- Saturated: the wire is the bottleneck ---
        tb_info("Saturated");
        adc0       = n_from[0];
        mic0       = n_from[1];
        dir0       = n_from[2];
        period     = '{0, 0};
        dir_period = 0;
        queue_frames(0, 30 * FRAMES[0]);
        queue_frames(1, 30 * FRAMES[1]);
        queue_directions(30 * FRAMES[2]);
        wait_sent("saturated");
        `CHECK(n_from[0] - adc0 == 30 && n_from[1] - mic0 == 30 && n_from[2] - dir0 == 30,
               $sformatf("Saturated: %0d ADC, %0d mic and %0d direction packets (expected 30 each)",
                         n_from[0] - adc0, n_from[1] - mic0, n_from[2] - dir0))

        apb_check(TX_COUNT, n_frames);
        `CHECK(n_underruns == 0, $sformatf("%0d TX underruns", n_underruns))
        tb_info($sformatf("%0d frames: %0d ADC packets, %0d mic packets, %0d direction packets",
                          n_frames, n_from[0], n_from[1], n_from[2]));
        tb_finish();
    end

endmodule
