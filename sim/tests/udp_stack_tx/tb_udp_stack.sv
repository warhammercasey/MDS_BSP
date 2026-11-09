`timescale 1ns/1ps
`include "tb_macros.svh"

// udp_stack + udp_stack_regs test, through rmii as on the board. Frames are built
// independently here (own CRC-32 and IPv4 checksum) and compared byte for byte.
//
// TX test (udp_stack_tx):
//   registers  reset values, read/write, PSLVERR for no register and read-only writes
//   TX         datagrams of several sizes (tx_length, set here) from the payload
//              stream (with stalls) must
//              appear on TXEN/TXD exactly as built here: headers, padding, FCS, and
//              the IPv4 identification counting up. TX_COUNT counts them.
// RX tests (udp_stack_rx):
//   RX         frames sent on CRS_DV/RXD: good ones (unicast, broadcast, padded, IP
//              options, UDP shorter than IP, maximum size) come out with the right
//              headers and payload, and show in the RX registers. Bad or unwanted ones
//              (FCS, EtherType, MAC, IP, checksum, protocol, fragments, version, UDP
//              length) give nothing. PROMISC lets other addresses in.
//   overflow   the app stalls while frames pile up: the buffer drops what doesn't fit,
//              and what comes out is whole and in order
//   random     a mix of the RX cases with random sizes and app stalls
//   loopback   TXEN/TXD wired to CRS_DV/RXD, addressed to ourselves: every datagram
//              sent comes back
//
// The RX tests are chosen with +define+TB_UDP_STACK_RX rather than a parameter: they look
// inside the RX path, and Verilator resolves hierarchical names even in generate branches
// that aren't built, so the TX test couldn't build without the RX path otherwise.
module tb_udp_stack;
`ifdef TB_UDP_STACK_RX
    localparam bit RX = 1;
`else
    localparam bit RX = 0;
`endif
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(200_000_000) // 200 ms

    localparam real CLK_PERIOD_NS = 20.0; // clk_50
    localparam int  MAX_PAYLOAD   = 1472;

    `define PUSH_BE(Q, V, N) for (int k_ = (N) - 1; k_ >= 0; k_--) Q.push_back(8'((V) >> (8 * k_)));

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;
    logic rstn = 1'b0;

    // ------------------------------------------------------------------
    // DUT: regs + stack + rmii
    // ------------------------------------------------------------------
    apb_if #(.ADDR_WIDTH(8), .DATA_WIDTH(32), .SEL_WIDTH(1)) apb ();
    eth_hdr_if  tx_eth_hdr ();
    ipv4_hdr_if tx_ip_hdr ();
    udp_hdr_if  tx_udp_hdr ();
    eth_hdr_if  rx_eth_hdr ();
    ipv4_hdr_if rx_ip_hdr ();
    udp_hdr_if  rx_udp_hdr ();
    axis_if       #(.WIDTH(8)) tx_payload ();
    axis_tlast_if #(.WIDTH(8)) rx_payload ();
    axis_tlast_if #(.WIDTH(8)) mac_tx ();
    axis_tlast_if #(.WIDTH(8)) mac_rx ();

    logic [47:0] local_mac;
    logic [31:0] local_ip;
    logic        promiscuous;
    logic [15:0] tx_len = 16'd64;

    udp_stack_regs regs (
        .clk(clk), .rstn(rstn), .apb(apb),
        .local_mac(local_mac), .local_ip(local_ip), .promiscuous(promiscuous), .tx_length(tx_len),
        .tx_eth_hdr(tx_eth_hdr), .tx_ip_hdr(tx_ip_hdr), .tx_udp_hdr(tx_udp_hdr),
        .rx_eth_hdr(rx_eth_hdr), .rx_ip_hdr(rx_ip_hdr), .rx_udp_hdr(rx_udp_hdr)
    );

    udp_stack stack (
        .clk(clk), .rstn(rstn),
        .local_mac(local_mac), .local_ip(local_ip), .promiscuous(promiscuous),
        .mac_tx(mac_tx), .mac_rx(mac_rx),
        .tx_eth_hdr(tx_eth_hdr), .tx_ip_hdr(tx_ip_hdr), .tx_udp_hdr(tx_udp_hdr), .tx_payload(tx_payload),
        .rx_eth_hdr(rx_eth_hdr), .rx_ip_hdr(rx_ip_hdr), .rx_udp_hdr(rx_udp_hdr), .rx_payload(rx_payload)
    );

    assign rx_eth_hdr.ready = 1'b1;
    assign rx_ip_hdr.ready  = 1'b1;

    logic [1:0] txd, rxd;
    logic       txen, crs_dv;
    bit         loopback = 1'b0;
    logic [1:0] phy_rxd = 2'b00;
    logic       phy_crs = 1'b0;
    assign crs_dv = loopback ? txen : phy_crs;
    assign rxd    = loopback ? txd  : phy_rxd;

    rmii rmii_i (
        .clk(clk), .rstn(rstn),
        .rmii_txd(txd), .rmii_txen(txen), .rmii_rstn(), .rmii_int(1'b1),
        .rmii_crs_dv(crs_dv), .rmii_rxd(rxd), .rmii_rxerr(1'b0),
        .axis_in(mac_tx), .axis_out(mac_rx)
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

    // An Ethernet/IPv4/UDP frame, FCS included. Defaults are a well-formed datagram.
    function automatic void make_frame(output byte unsigned f[$], input byte unsigned payload[$],
            input logic [47:0] dst_mac, input logic [47:0] src_mac,
            input logic [31:0] src_ip,  input logic [31:0] dst_ip,
            input logic [15:0] src_port, input logic [15:0] dst_port,
            input logic [7:0]  ttl = 8'd64,
            input logic [15:0] ethertype = 16'h0800, input logic [7:0] protocol = 8'd17,
            input logic [3:0]  version = 4'd4, input int ihl = 5,
            input bit df = 1'b0, input bit mf = 1'b0, input logic [12:0] frag = '0,
            input logic [15:0] ident = 16'h1234, input bit bad_ip_csum = 1'b0,
            input int udp_len = -1, input logic [15:0] udp_csum = 16'hBEEF,
            input bit bad_fcs = 1'b0);
        byte unsigned ip[$];
        logic [15:0]  csum;
        logic [31:0]  fcs;
        f  = {};
        ip = {};
        `PUSH_BE(f, dst_mac, 6)
        `PUSH_BE(f, src_mac, 6)
        `PUSH_BE(f, ethertype, 2)
        ip.push_back({version, 4'(ihl)});
        ip.push_back(8'h00);
        `PUSH_BE(ip, ihl * 4 + 8 + payload.size(), 2)
        `PUSH_BE(ip, ident, 2)
        `PUSH_BE(ip, {1'b0, df, mf, frag}, 2)
        ip.push_back(ttl);
        ip.push_back(protocol);
        `PUSH_BE(ip, 16'h0000, 2)
        `PUSH_BE(ip, src_ip, 4)
        `PUSH_BE(ip, dst_ip, 4)
        for (int i = 20; i < ihl * 4; i++) ip.push_back(8'h01); // NOP options
        csum = ~ones_sum(ip);
        if (bad_ip_csum) csum = csum ^ 16'h0100;
        ip[10] = csum[15:8];
        ip[11] = csum[7:0];
        foreach (ip[i]) f.push_back(ip[i]);
        `PUSH_BE(f, src_port, 2)
        `PUSH_BE(f, dst_port, 2)
        `PUSH_BE(f, udp_len < 0 ? 8 + payload.size() : udp_len, 2)
        `PUSH_BE(f, udp_csum, 2)
        foreach (payload[i]) f.push_back(payload[i]);
        while (f.size() < 60) f.push_back(8'h00);
        fcs = ref_fcs(f);
        if (bad_fcs) fcs = fcs ^ 32'h1;
        for (int k = 0; k < 4; k++) f.push_back(fcs[8*k +: 8]);
    endfunction

    function automatic void rand_bytes(output byte unsigned q[$], input int unsigned n);
        q = {};
        repeat (n) q.push_back(8'($urandom));
    endfunction

    function automatic bit q_eq(byte unsigned a[$], byte unsigned b[$]);
        if (a.size() != b.size()) return 1'b0;
        foreach (a[i]) if (a[i] != b[i]) return 1'b0;
        return 1'b1;
    endfunction

    function automatic string show(byte unsigned q[$]);
        string s = "";
        foreach (q[i]) begin
            if (i == 16) return {s, " ..."};
            s = {s, $sformatf(" %02h", q[i])};
        end
        return s;
    endfunction

    // ------------------------------------------------------------------
    // TX payload source
    // ------------------------------------------------------------------
    byte unsigned tx_q[$];
    int unsigned  src_stall_pct = 0;

    always @(posedge clk) begin
        bit hs;
        if (!rstn) begin
            tx_payload.tvalid <= 1'b0;
            tx_payload.tdata  <= '0;
        end else begin
            hs = tx_payload.tvalid && tx_payload.tready;
            if (hs) void'(tx_q.pop_front());
            if (!tx_payload.tvalid || hs) begin
                if (tx_q.size() > 0 && $urandom_range(99) >= src_stall_pct) begin
                    tx_payload.tvalid <= 1'b1;
                    tx_payload.tdata  <= tx_q[0];
                end else begin
                    tx_payload.tvalid <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Wire monitor: frames on TXEN/TXD, preamble and SFD checked and stripped
    // ------------------------------------------------------------------
    byte unsigned tx_frames[$][$];
    logic [1:0]   tx_dibits[$];
    byte unsigned tx_bytes[$];

    function automatic void wire_frame_done();
        tx_bytes = {};
        for (int i = 0; i + 3 < tx_dibits.size(); i += 4)
            tx_bytes.push_back({tx_dibits[i+3], tx_dibits[i+2], tx_dibits[i+1], tx_dibits[i]});
        tb_checks++;
        if (tx_dibits.size() % 4 != 0 || tx_bytes.size() < 8 || tx_bytes[7] != 8'hD5) begin
            tb_error($sformatf("TX: bad frame on the wire (%0d dibits): {%s }", tx_dibits.size(), show(tx_bytes)));
            return;
        end
        repeat (8) void'(tx_bytes.pop_front()); // Preamble and SFD
        tx_frames.push_back(tx_bytes);
    endfunction

    always @(posedge clk) begin
        if (rstn) begin
            if (txen === 1'b1) begin
                tx_dibits.push_back(txd);
            end else if (tx_dibits.size() > 0) begin
                wire_frame_done();
                tx_dibits = {};
            end
        end
    end

    // ------------------------------------------------------------------
    // RX app: takes headers and payloads (with stalls), records them
    // ------------------------------------------------------------------
    typedef struct packed {
        logic [47:0] dst_mac, src_mac;
        logic [31:0] src_ip, dst_ip;
        logic [7:0]  ttl, protocol;
        logic [15:0] ip_length, src_port, dst_port, length;
    } hdr_t;

    hdr_t         rx_hdrs[$];
    byte unsigned rx_pkts[$][$];
    byte unsigned rx_cur[$];
    int unsigned  sink_pct = 100;
    int unsigned  rx_quiet = 0;

    // Whether the Ethernet layer's frame buffer has anything waiting (RX tests only)
    logic rx_buf_busy;
`ifdef TB_UDP_STACK_RX
    assign rx_buf_busy = stack.eth_depacketizer_i.buf_out.tvalid;
`else
    assign rx_buf_busy = 1'b0;
`endif

    always @(posedge clk) begin
        if (rstn) begin
            if (rx_udp_hdr.valid && rx_udp_hdr.ready)
                rx_hdrs.push_back({rx_eth_hdr.dst_mac, rx_eth_hdr.src_mac, rx_ip_hdr.src_ip, rx_ip_hdr.dst_ip,
                                    rx_ip_hdr.ttl, rx_ip_hdr.protocol, rx_ip_hdr.length,
                                    rx_udp_hdr.src_port, rx_udp_hdr.dst_port, rx_udp_hdr.length});
            if (rx_payload.tvalid && rx_payload.tready) begin
                rx_cur.push_back(rx_payload.tdata);
                if (rx_payload.tlast) begin
                    rx_pkts.push_back(rx_cur);
                    rx_cur = {};
                end
            end
            // Nothing left anywhere in the RX path
            if (rx_buf_busy || rx_payload.tvalid || rx_udp_hdr.valid)
                rx_quiet = 0;
            else
                rx_quiet++;
        end
        rx_payload.tready <= $urandom_range(99) < sink_pct;
        rx_udp_hdr.ready  <= $urandom_range(99) < sink_pct;
    end

    function automatic string hdr_str(hdr_t h);
        return $sformatf("mac %012h <- %012h, ip %08h -> %08h ttl %0d proto %0d len %0d, port %0d -> %0d len %0d",
                         h.dst_mac, h.src_mac, h.src_ip, h.dst_ip, h.ttl, h.protocol, h.ip_length,
                         h.src_port, h.dst_port, h.length);
    endfunction

    function automatic void rx_clear();
        rx_hdrs = {};
        rx_pkts = {};
        rx_cur  = {};
    endfunction

    int unsigned n_rx_ok = 0, n_rx_dropped = 0;

    function automatic void expect_none(string name);
        n_rx_dropped++;
        `CHECK(rx_hdrs.size() == 0 && rx_pkts.size() == 0 && rx_cur.size() == 0,
               $sformatf("RX %s: expected it dropped, got %0d headers, %0d payloads", name, rx_hdrs.size(), rx_pkts.size()))
        rx_clear();
    endfunction

    function automatic void expect_pkt(string name, hdr_t exp, byte unsigned payload[$]);
        n_rx_ok++;
        tb_checks++;
        if (rx_hdrs.size() != 1 || rx_pkts.size() != 1 || rx_cur.size() != 0) begin
            tb_error($sformatf("RX %s: expected 1 datagram, got %0d headers, %0d payloads (%0d bytes pending)",
                               name, rx_hdrs.size(), rx_pkts.size(), rx_cur.size()));
        end else begin
            `CHECK(rx_hdrs[0] == exp, $sformatf("RX %s: header\n    got      %s\n    expected %s",
                                                name, hdr_str(rx_hdrs[0]), hdr_str(exp)))
            `CHECK(q_eq(rx_pkts[0], payload), $sformatf("RX %s: payload %0d bytes {%s }, expected %0d {%s }",
                                                    name, rx_pkts[0].size(), show(rx_pkts[0]), payload.size(), show(payload)))
        end
        rx_clear();
    endfunction

    // ------------------------------------------------------------------
    // PHY model (RX) and waits
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
        repeat (48) @(posedge clk); // IPG
    endtask

    task automatic rx_settle();
        repeat (2) @(posedge clk);
        while (rx_quiet < 64) @(posedge clk);
    endtask

    task automatic wait_tx_frames(int unsigned n);
        int unsigned t = 0;
        while (tx_frames.size() < n && t < 2_000_000) begin
            @(posedge clk);
            t++;
        end
        `CHECK(tx_frames.size() >= n, $sformatf("TX: %0d of %0d frames came out", tx_frames.size(), n))
        repeat (10) @(posedge clk);
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

    task automatic apb_write(int addr, logic [31:0] data, bit exp_err = 1'b0);
        logic [31:0] r;
        bit          err;
        apb_xfer(1'b1, addr, data, r, err);
        `CHECK(err == exp_err, $sformatf("APB write 0x%02h = 0x%08h: PSLVERR %b, expected %b", addr, data, err, exp_err))
    endtask

    task automatic apb_check(int addr, logic [31:0] exp, bit exp_err = 1'b0);
        logic [31:0] r;
        bit          err;
        apb_xfer(1'b0, addr, '0, r, err);
        `CHECK(err == exp_err, $sformatf("APB read 0x%02h: PSLVERR %b, expected %b", addr, err, exp_err))
        if (!exp_err)
            `CHECK_EQ(r, exp, $sformatf("APB read 0x%02h", addr))
    endtask

    // Register addresses (udp_stack_regs.sv)
    localparam int CTRL = 'h00, TX_DST_MAC_HI = 'h01, TX_DST_MAC_LO = 'h02, LOCAL_MAC_HI = 'h03,
                   LOCAL_MAC_LO = 'h04, LOCAL_IP = 'h05, TX_DST_IP = 'h06, TX_TTL = 'h07,
                   TX_PORTS = 'h08, TX_COUNT = 'h09, RX_COUNT = 'h10,
                   RX_DST_MAC_HI = 'h11, RX_DST_MAC_LO = 'h12, RX_SRC_MAC_HI = 'h13,
                   RX_SRC_MAC_LO = 'h14, RX_SRC_IP = 'h15, RX_DST_IP = 'h16, RX_IP_INFO = 'h17,
                   RX_PORTS = 'h18, RX_LENGTH = 'h19;

    // What the TB has configured
    logic [47:0] cfg_local_mac, cfg_dst_mac;
    logic [31:0] cfg_local_ip, cfg_dst_ip;
    logic [7:0]  cfg_ttl;
    logic [15:0] cfg_src_port, cfg_dst_port;

    task automatic configure(logic [47:0] lmac, logic [47:0] dmac, logic [31:0] lip, logic [31:0] dip,
                             logic [7:0] ttl, logic [15:0] sport, logic [15:0] dport);
        apb_write(LOCAL_MAC_HI, {16'h0, lmac[47:32]});
        apb_write(LOCAL_MAC_LO, lmac[31:0]);
        apb_write(TX_DST_MAC_HI, {16'h0, dmac[47:32]});
        apb_write(TX_DST_MAC_LO, dmac[31:0]);
        apb_write(LOCAL_IP, lip);
        apb_write(TX_DST_IP, dip);
        apb_write(TX_TTL, {24'h0, ttl});
        apb_write(TX_PORTS, {sport, dport});
        cfg_local_mac = lmac;
        cfg_dst_mac   = dmac;
        cfg_local_ip  = lip;
        cfg_dst_ip    = dip;
        cfg_ttl       = ttl;
        cfg_src_port  = sport;
        cfg_dst_port  = dport;
    endtask

    task automatic check_rx_regs(hdr_t h, int unsigned count);
        apb_check(RX_COUNT, count);
        apb_check(RX_DST_MAC_HI, {16'h0, h.dst_mac[47:32]});
        apb_check(RX_DST_MAC_LO, h.dst_mac[31:0]);
        apb_check(RX_SRC_MAC_HI, {16'h0, h.src_mac[47:32]});
        apb_check(RX_SRC_MAC_LO, h.src_mac[31:0]);
        apb_check(RX_SRC_IP, h.src_ip);
        apb_check(RX_DST_IP, h.dst_ip);
        apb_check(RX_IP_INFO, {16'h0, h.protocol, h.ttl});
        apb_check(RX_PORTS, {h.src_port, h.dst_port});
        apb_check(RX_LENGTH, {16'h0, h.length});
    endtask

    // ------------------------------------------------------------------
    // TX: datagrams of `len` bytes, checked against frames built here
    // ------------------------------------------------------------------
    logic [15:0] tx_ident = '0; // The stack counts from 0 after reset
    int unsigned n_tx = 0;

    task automatic tx_datagrams(int unsigned len, int unsigned count, bit expect_on_wire = 1'b1);
        byte unsigned payloads[$][$];
        byte unsigned p[$], exp[$];
        payloads = {};
        tx_len    = 16'(len);
        tx_frames = {};
        repeat (count) begin
            rand_bytes(p, len);
            payloads.push_back(p);
            foreach (p[i]) tx_q.push_back(p[i]);
        end
        wait_tx_frames(count);
        foreach (payloads[d]) begin
            if (d >= tx_frames.size()) break;
            make_frame(exp, payloads[d], cfg_dst_mac, cfg_local_mac, cfg_local_ip, cfg_dst_ip,
                       cfg_src_port, cfg_dst_port, .ttl(cfg_ttl), .df(1'b1), .ident(tx_ident), .udp_csum(16'h0000));
            `CHECK(q_eq(tx_frames[d], exp), $sformatf("TX %0d byte datagram %0d: frame %0d bytes {%s }\n    expected %0d bytes {%s }",
                                                  len, d, tx_frames[d].size(), show(tx_frames[d]), exp.size(), show(exp)))
            tx_ident++;
            n_tx++;
        end
    endtask

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    localparam logic [47:0] PEER_MAC  = 48'h00_11_22_33_44_55;
    localparam logic [31:0] PEER_IP   = 32'h0A00_0001; // 10.0.0.1
    localparam logic [47:0] OTHER_MAC = 48'h00_11_22_33_44_66;
    localparam logic [31:0] OTHER_IP  = 32'h0A00_0003;
    localparam int          LOOP_LENS [5] = '{1, 18, 64, 500, MAX_PAYLOAD};

    // An RX case: send `f`, then expect a datagram with `payload` or nothing. The
    // expected headers are what the frames below are built with: from PEER_MAC/PEER_IP,
    // TTL 77, ports 4000 -> 4001, IP payload ip_len (default: the UDP datagram).
    task automatic rx_case(string name, byte unsigned f[$], bit ok, byte unsigned payload[$],
                           logic [47:0] dst_mac = '0, logic [31:0] dst_ip = '0, int ip_len = -1);
        hdr_t h;
        phy_send(f);
        rx_settle();
        if (ok) begin
            h = {dst_mac, PEER_MAC, PEER_IP, dst_ip, 8'd77, 8'd17,
                  16'(ip_len < 0 ? 8 + payload.size() : ip_len),
                  16'd4000, 16'd4001, 16'(payload.size())};
            expect_pkt(name, h, payload);
        end else begin
            expect_none(name);
        end
    endtask

    // The app stalls while 6 frames arrive: the RX buffer keeps what fits and drops the
    // rest. What comes out must be whole datagrams that were sent, in order.
    byte unsigned ovf_sent[$][$];

    task automatic rx_overflow();
        byte unsigned p[$], f[$];
        int           j;
        ovf_sent = {};
        sink_pct = 0;
        repeat (6) begin
            rand_bytes(p, 1000);
            ovf_sent.push_back(p);
            make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .ttl(77));
            phy_send(f);
        end
        sink_pct = 100;
        rx_settle();
        tb_checks++;
        j = 0;
        for (int i = 0; i < rx_pkts.size(); i++) begin
            bit found = 1'b0;
            f = rx_pkts[i];
            while (!found && j < ovf_sent.size()) begin
                p = ovf_sent[j];
                found = q_eq(p, f);
                j = j + 1;
            end
            if (!found) begin
                tb_error($sformatf("RX overflow: datagram %0d came out, but isn't one sent (in order)", i));
                break;
            end
        end
        `CHECK(rx_pkts.size() >= 2 && rx_pkts.size() < 6,
               $sformatf("RX overflow: %0d of 6 datagrams came out (expected some dropped, at least 2 kept)", rx_pkts.size()))
        `CHECK(rx_hdrs.size() == rx_pkts.size(), "RX overflow: headers and payloads don't pair up")
        tb_info($sformatf("RX overflow: %0d of 6 datagrams kept", rx_pkts.size()));
        n_rx_ok += rx_pkts.size();
        rx_clear();
    endtask

    initial begin
        byte unsigned p[$], f[$];
        logic [31:0]  fcs;
        hdr_t         h;

        tb_info($sformatf("Testing udp_stack + udp_stack_regs through rmii: %s", RX ? "RX" : "TX"));
        p = {};
        for (int i = 0; i < 9; i++) p.push_back(8'("1" + i)); // "123456789"
        `CHECK_EQ(ref_fcs(p), 32'hCBF4_3926, "TB CRC-32 of \"123456789\"")

        repeat (5) @(posedge clk);
        rstn = 1'b1;
        repeat (5) @(posedge clk);

        if (!RX) begin
            // --- Registers ---
            tb_info("Registers");
            apb_check(CTRL, 32'h0);
            apb_check(TX_DST_MAC_HI, 32'h0000_FFFF);
            apb_check(TX_DST_MAC_LO, 32'hFFFF_FFFF);
            apb_check(LOCAL_MAC_HI, 32'h0000_0200);
            apb_check(LOCAL_MAC_LO, 32'h0000_0001);
            apb_check(LOCAL_IP, 32'hC0A8_0064);
            apb_check(TX_DST_IP, 32'hFFFF_FFFF);
            apb_check(TX_TTL, 32'd64);
            apb_check(TX_PORTS, {16'd5000, 16'd5000});
            apb_check(TX_COUNT, 32'd0);
            apb_check(RX_COUNT, 32'd0);
            apb_check('h0A, 0, 1'b1);
            apb_check('h0F, 0, 1'b1);
            apb_check('h1A, 0, 1'b1);
            apb_check('hFF, 0, 1'b1);
            apb_write(TX_COUNT, 32'h1, 1'b1);
            apb_write(RX_COUNT, 32'h1, 1'b1);
            apb_write(RX_SRC_IP, 32'h1, 1'b1);
            apb_write('h0A, 32'h1, 1'b1);
            apb_write('h40, 32'h1, 1'b1);
            apb_write(TX_TTL, 32'hFFFF_FF80);
            apb_check(TX_TTL, 32'h80);
            apb_write(CTRL, 32'h1);
            apb_check(CTRL, 32'h1);
            apb_write(CTRL, 32'h0);
            configure(48'h02_AA_BB_CC_DD_EE, PEER_MAC, 32'h0A00_0002, PEER_IP, 8'd32, 16'd1234, 16'd5678);
            apb_check(LOCAL_MAC_HI, 32'h0000_02AA);
            apb_check(LOCAL_MAC_LO, 32'hBBCC_DDEE);
            apb_check(TX_DST_MAC_HI, 32'h0000_0011);
            apb_check(TX_DST_MAC_LO, 32'h2233_4455);
            apb_check(LOCAL_IP, 32'h0A00_0002);
            apb_check(TX_DST_IP, PEER_IP);
            apb_check(TX_PORTS, {16'd1234, 16'd5678});

            // --- TX ---
            tb_info("TX datagrams");
            tx_datagrams(1, 1);      // Padded to the minimum frame
            tx_datagrams(17, 1);
            tx_datagrams(18, 1);     // Exactly the minimum
            tx_datagrams(19, 1);
            tx_datagrams(100, 3);    // Back to back
            tx_datagrams(MAX_PAYLOAD, 2);
            tb_info("TX datagrams, payload stream stalling");
            src_stall_pct = 60;
            repeat (10) tx_datagrams($urandom_range(300, 1), $urandom_range(3, 1));
            src_stall_pct = 0;
            apb_check(TX_COUNT, n_tx);
            tb_info($sformatf("%0d datagrams sent", n_tx));
            tb_finish();
        end

        // RX tests
        configure(48'h02_AA_BB_CC_DD_EE, PEER_MAC, 32'h0A00_0002, PEER_IP, 8'd32, 16'd1234, 16'd5678);

        // --- RX directed ---
        tb_info("RX directed");
        rand_bytes(p, 100);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .ttl(77));
        rx_case("unicast", f, 1'b1, p, cfg_local_mac, cfg_local_ip);
        check_rx_regs({cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 8'd77, 8'd17, 16'd108, 16'd4000, 16'd4001, 16'd100}, 1);

        rand_bytes(p, 5);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .ttl(77));
        rx_case("5 bytes (frame padded)", f, 1'b1, p, cfg_local_mac, cfg_local_ip);

        rand_bytes(p, 40);
        make_frame(f, p, 48'hFFFF_FFFF_FFFF, PEER_MAC, PEER_IP, 32'hFFFF_FFFF, 4000, 4001, .ttl(77));
        rx_case("broadcast MAC and IP", f, 1'b1, p, 48'hFFFF_FFFF_FFFF, 32'hFFFF_FFFF);
        check_rx_regs({48'hFFFF_FFFF_FFFF, PEER_MAC, PEER_IP, 32'hFFFF_FFFF, 8'd77, 8'd17, 16'd48, 16'd4000, 16'd4001, 16'd40}, 3);

        rand_bytes(p, 40);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .ttl(77), .ihl(7));
        rx_case("IP options", f, 1'b1, p, cfg_local_mac, cfg_local_ip);

        rand_bytes(p, 20);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .ttl(77), .udp_len(8 + 10));
        while (p.size() > 10) void'(p.pop_back());
        rx_case("UDP shorter than IP", f, 1'b1, p, cfg_local_mac, cfg_local_ip, .ip_len(28));

        rand_bytes(p, MAX_PAYLOAD);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .ttl(77));
        rx_case("maximum size", f, 1'b1, p, cfg_local_mac, cfg_local_ip);

        rand_bytes(p, 50);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .bad_fcs(1));
        rx_case("bad FCS", f, 1'b0, p);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .ethertype(16'h0806));
        rx_case("ARP EtherType", f, 1'b0, p);
        make_frame(f, p, OTHER_MAC, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001);
        rx_case("other MAC", f, 1'b0, p);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, OTHER_IP, 4000, 4001);
        rx_case("other IP", f, 1'b0, p);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .bad_ip_csum(1));
        rx_case("bad IP checksum", f, 1'b0, p);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .protocol(8'd6));
        rx_case("TCP", f, 1'b0, p);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .mf(1));
        rx_case("first fragment", f, 1'b0, p);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .frag(13'd100));
        rx_case("later fragment", f, 1'b0, p);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .version(4'd6));
        rx_case("version 6", f, 1'b0, p);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .udp_len(4));
        rx_case("UDP length 4", f, 1'b0, p);
        p = {};
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001);
        rx_case("empty datagram", f, 1'b0, p);
        apb_check(RX_COUNT, n_rx_ok);

        tb_info("RX promiscuous");
        apb_write(CTRL, 32'h1);
        rand_bytes(p, 30);
        make_frame(f, p, OTHER_MAC, PEER_MAC, PEER_IP, OTHER_IP, 4000, 4001, .ttl(77));
        rx_case("promiscuous: other MAC and IP", f, 1'b1, p, OTHER_MAC, OTHER_IP);
        make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .protocol(8'd6));
        rx_case("promiscuous: TCP still dropped", f, 1'b0, p);
        apb_write(CTRL, 32'h0);

        // --- RX overflow: the app stalls while frames arrive ---
        tb_info("RX overflow");
        rx_overflow();

        // --- RX random ---
        tb_info("RX random");
        repeat (150) begin
            automatic int unsigned kind = $urandom_range(9);
            sink_pct = $urandom_range(100, 20);
            rand_bytes(p, $urandom_range(9) == 0 ? $urandom_range(MAX_PAYLOAD, 300) : $urandom_range(80, 1));
            case (kind)
                0, 1, 2, 3: begin
                    make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .ttl(77));
                    rx_case("random unicast", f, 1'b1, p, cfg_local_mac, cfg_local_ip);
                end
                4: begin
                    make_frame(f, p, 48'hFFFF_FFFF_FFFF, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .ttl(77));
                    rx_case("random broadcast", f, 1'b1, p, 48'hFFFF_FFFF_FFFF, cfg_local_ip);
                end
                5: begin
                    make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .bad_fcs(1));
                    rx_case("random bad FCS", f, 1'b0, p);
                end
                6: begin
                    make_frame(f, p, OTHER_MAC, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001);
                    rx_case("random other MAC", f, 1'b0, p);
                end
                7: begin
                    make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .bad_ip_csum(1));
                    rx_case("random bad IP checksum", f, 1'b0, p);
                end
                8: begin
                    make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .protocol(8'd1));
                    rx_case("random ICMP", f, 1'b0, p);
                end
                default: begin
                    make_frame(f, p, cfg_local_mac, PEER_MAC, PEER_IP, cfg_local_ip, 4000, 4001, .ttl(77), .ihl(6));
                    rx_case("random IP options", f, 1'b1, p, cfg_local_mac, cfg_local_ip);
                end
            endcase
        end
        sink_pct = 100;
        apb_check(RX_COUNT, n_rx_ok);

        // --- Loopback ---
        tb_info("Loopback");
        loopback = 1'b1;
        configure(cfg_local_mac, cfg_local_mac, cfg_local_ip, cfg_local_ip, 8'd9, 16'd7000, 16'd7001);
        foreach (LOOP_LENS[i]) begin
            automatic int unsigned len = LOOP_LENS[i];
            rx_clear();
            tx_datagrams(len, 1);
            rx_settle();
            h = {cfg_local_mac, cfg_local_mac, cfg_local_ip, cfg_local_ip, 8'd9, 8'd17, 16'(len + 8), 16'd7000, 16'd7001, 16'(len)};
            // The payload sent is the one in the frame just checked
            p = {};
            for (int b = 42; b < 42 + len; b++) p.push_back(tx_frames[0][b]);
            expect_pkt($sformatf("loopback %0d bytes", len), h, p);
        end
        apb_check(TX_COUNT, n_tx);
        apb_check(RX_COUNT, n_rx_ok);

        tb_info($sformatf("%0d datagrams sent, %0d received, %0d frames dropped as expected", n_tx, n_rx_ok, n_rx_dropped));
        tb_finish();
    end

endmodule
