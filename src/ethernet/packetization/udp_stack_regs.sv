// APB registers for udp_stack: the TX headers, and the headers of the last datagram
// received.
//
// One 32-bit register per APB address (paddr is a register index, not a byte
// address). Bits not listed read as 0 and are ignored on writes. PSLVERR is returned
// for addresses with no register and for writes to read-only registers.
//
// MACs are split HI/LO with the first byte on the wire in the top bits:
// aa:bb:cc:dd:ee:ff is HI = 0x0000aabb, LO = 0xccddeeff. IPs are one register,
// 192.168.1.10 = 0xC0A8010A. Reset values are for the default parameters.
//
//  Addr  Name            Access  Bits     Reset        Description
//  ----  --------------  ------  -------  -----------  ---------------------------------
//  0x00  CTRL            RW      [0]      0            PROMISC: receive packets for any
//                                                      MAC/IP
//  0x01  TX_DST_MAC_HI   RW      [15:0]   0x0000FFFF   TX destination MAC, bytes 0-1
//  0x02  TX_DST_MAC_LO   RW      [31:0]   0xFFFFFFFF   ... bytes 2-5 (broadcast)
//  0x03  LOCAL_MAC_HI    RW      [15:0]   0x00000200   Our MAC, bytes 0-1: TX source,
//                                                      RX filter
//  0x04  LOCAL_MAC_LO    RW      [31:0]   0x00000001   ... bytes 2-5 (02:00:00:00:00:01)
//  0x05  LOCAL_IP        RW      [31:0]   0xC0A80064   Our IP: TX source, RX filter
//                                                      (192.168.0.100)
//  0x06  TX_DST_IP       RW      [31:0]   0xFFFFFFFF   TX destination IP (broadcast)
//  0x07  TX_TTL          RW      [7:0]    0x40         TX time to live (64)
//  0x08  TX_PORTS        RW      [31:16]  0x1388       TX source port (5000)
//                                [15:0]   0x1388       TX destination port (5000)
//  0x09  TX_COUNT        RO      [31:0]   0            Datagrams sent (wraps)
//  0x10  RX_COUNT        RO      [31:0]   0            Datagrams received (wraps)
//  0x11  RX_DST_MAC_HI   RO      [15:0]   0            Last datagram received:
//                                                      destination MAC
//  0x12  RX_DST_MAC_LO   RO      [31:0]   0
//  0x13  RX_SRC_MAC_HI   RO      [15:0]   0            ... source MAC
//  0x14  RX_SRC_MAC_LO   RO      [31:0]   0
//  0x15  RX_SRC_IP       RO      [31:0]   0            ... source IP
//  0x16  RX_DST_IP       RO      [31:0]   0            ... destination IP
//  0x17  RX_IP_INFO      RO      [15:8]   0            ... IP protocol (always 17, UDP)
//                                [7:0]    0            ... time to live
//  0x18  RX_PORTS        RO      [31:16]  0            ... source port
//                                [15:0]   0            ... destination port
//  0x19  RX_LENGTH       RO      [15:0]   0            ... payload bytes
//
// The TX payload size isn't a register: it comes in on tx_length, for the datagram
// about to start (it's set at compile time by the RTP packet sizes, see ethernet.sv).
//
// The RX registers update together when a datagram's header is taken, so read
// RX_COUNT before and after them to tell whether one arrived in between. Each TX
// datagram takes the TX settings as they are when it starts; a MAC written in two
// halves can be half-changed for a datagram starting between the writes.
`ifdef LINT_INCLUDES
`include "net_pkg.sv"
`endif
module udp_stack_regs
    import net_pkg::*;
#(
    // Reset values
    parameter logic [47:0] DEFAULT_LOCAL_MAC = 48'h02_00_00_00_00_01, // Locally administered
    parameter logic [31:0] DEFAULT_LOCAL_IP  = 32'hC0A8_0064,         // 192.168.0.100
    parameter logic [47:0] DEFAULT_DST_MAC   = MAC_BROADCAST,
    parameter logic [31:0] DEFAULT_DST_IP    = IP_BROADCAST,
    parameter logic [7:0]  DEFAULT_TTL       = 8'd64,
    parameter logic [15:0] DEFAULT_SRC_PORT  = 16'd5000,
    parameter logic [15:0] DEFAULT_DST_PORT  = 16'd5000
)(
    input  logic        clk,
    input  logic        rstn,

    apb_if.slv          apb,          // DATA_WIDTH = 32, ADDR_WIDTH >= 5

    // To udp_stack
    output logic [47:0] local_mac,
    output logic [31:0] local_ip,
    output logic        promiscuous,
    input  logic [15:0] tx_length,    // Payload bytes of the next TX datagram
    eth_hdr_if.mst      tx_eth_hdr,   // Always valid: every datagram uses the settings
    ipv4_hdr_if.mst     tx_ip_hdr,
    udp_hdr_if.mst      tx_udp_hdr,

    // Watched, to count datagrams and keep the last RX headers
    eth_hdr_if.mon      rx_eth_hdr,
    ipv4_hdr_if.mon     rx_ip_hdr,
    udp_hdr_if.mon      rx_udp_hdr
);

    localparam int A_CTRL          = 'h00;
    localparam int A_TX_DST_MAC_HI = 'h01;
    localparam int A_TX_DST_MAC_LO = 'h02;
    localparam int A_LOCAL_MAC_HI  = 'h03;
    localparam int A_LOCAL_MAC_LO  = 'h04;
    localparam int A_LOCAL_IP      = 'h05;
    localparam int A_TX_DST_IP     = 'h06;
    localparam int A_TX_TTL        = 'h07;
    localparam int A_TX_PORTS      = 'h08;
    localparam int A_TX_COUNT      = 'h09;
    localparam int A_RX_COUNT      = 'h10;
    localparam int A_RX_DST_MAC_HI = 'h11;
    localparam int A_RX_DST_MAC_LO = 'h12;
    localparam int A_RX_SRC_MAC_HI = 'h13;
    localparam int A_RX_SRC_MAC_LO = 'h14;
    localparam int A_RX_SRC_IP     = 'h15;
    localparam int A_RX_DST_IP     = 'h16;
    localparam int A_RX_IP_INFO    = 'h17;
    localparam int A_RX_PORTS      = 'h18;
    localparam int A_RX_LENGTH     = 'h19;

    // TX settings
    logic [47:0] tx_dst_mac;
    logic [31:0] tx_dst_ip;
    logic [7:0]  tx_ttl;
    logic [15:0] tx_src_port, tx_dst_port;
    logic [31:0] tx_count;

    // Last RX datagram
    logic [31:0] rx_count;
    logic [47:0] rx_dst_mac, rx_src_mac;
    logic [31:0] rx_src_ip, rx_dst_ip;
    logic [7:0]  rx_protocol, rx_ttl;
    logic [15:0] rx_src_port, rx_dst_port, rx_length;


    // ------------------------------------------------------------------
    // APB
    //   One wait state: pready the cycle after the access phase starts
    // ------------------------------------------------------------------
    logic        access;
    logic        mapped;
    logic [31:0] rdata;

    assign access = apb.psel[0] && apb.penable && !apb.pready;

    always_comb begin
        mapped = 1'b1;
        rdata  = '0;
        case(int'(apb.paddr))
        A_CTRL:          rdata = {31'h0, promiscuous};
        A_TX_DST_MAC_HI: rdata = {16'h0, tx_dst_mac[47:32]};
        A_TX_DST_MAC_LO: rdata = tx_dst_mac[31:0];
        A_LOCAL_MAC_HI:  rdata = {16'h0, local_mac[47:32]};
        A_LOCAL_MAC_LO:  rdata = local_mac[31:0];
        A_LOCAL_IP:      rdata = local_ip;
        A_TX_DST_IP:     rdata = tx_dst_ip;
        A_TX_TTL:        rdata = {24'h0, tx_ttl};
        A_TX_PORTS:      rdata = {tx_src_port, tx_dst_port};
        A_TX_COUNT:      rdata = tx_count;
        A_RX_COUNT:      rdata = rx_count;
        A_RX_DST_MAC_HI: rdata = {16'h0, rx_dst_mac[47:32]};
        A_RX_DST_MAC_LO: rdata = rx_dst_mac[31:0];
        A_RX_SRC_MAC_HI: rdata = {16'h0, rx_src_mac[47:32]};
        A_RX_SRC_MAC_LO: rdata = rx_src_mac[31:0];
        A_RX_SRC_IP:     rdata = rx_src_ip;
        A_RX_DST_IP:     rdata = rx_dst_ip;
        A_RX_IP_INFO:    rdata = {16'h0, rx_protocol, rx_ttl};
        A_RX_PORTS:      rdata = {rx_src_port, rx_dst_port};
        A_RX_LENGTH:     rdata = {16'h0, rx_length};
        default:         mapped = 1'b0;
        endcase
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            apb.pready  <= 1'b0;
            apb.pslverr <= 1'b0;
            apb.prdata  <= '0;
            promiscuous <= 1'b0;
            tx_dst_mac  <= DEFAULT_DST_MAC;
            local_mac   <= DEFAULT_LOCAL_MAC;
            local_ip    <= DEFAULT_LOCAL_IP;
            tx_dst_ip   <= DEFAULT_DST_IP;
            tx_ttl      <= DEFAULT_TTL;
            tx_src_port <= DEFAULT_SRC_PORT;
            tx_dst_port <= DEFAULT_DST_PORT;

        end else begin
            apb.pready  <= access;
            apb.pslverr <= 1'b0;

            if(access && !apb.pwrite) begin
                apb.prdata  <= rdata;
                apb.pslverr <= !mapped;
            end

            if(access && apb.pwrite) begin
                case(int'(apb.paddr))
                A_CTRL:          promiscuous        <= apb.pwdata[0];
                A_TX_DST_MAC_HI: tx_dst_mac[47:32]  <= apb.pwdata[15:0];
                A_TX_DST_MAC_LO: tx_dst_mac[31:0]   <= apb.pwdata;
                A_LOCAL_MAC_HI:  local_mac[47:32]   <= apb.pwdata[15:0];
                A_LOCAL_MAC_LO:  local_mac[31:0]    <= apb.pwdata;
                A_LOCAL_IP:      local_ip           <= apb.pwdata;
                A_TX_DST_IP:     tx_dst_ip          <= apb.pwdata;
                A_TX_TTL:        tx_ttl             <= apb.pwdata[7:0];
                A_TX_PORTS: begin
                    tx_src_port <= apb.pwdata[31:16];
                    tx_dst_port <= apb.pwdata[15:0];
                end
                default:         apb.pslverr        <= 1'b1; // Read-only or no register
                endcase
            end
        end
    end


    // ------------------------------------------------------------------
    // TX headers: the settings, always on offer
    // ------------------------------------------------------------------
    assign tx_eth_hdr.valid    = 1'b1;
    assign tx_eth_hdr.dst_mac  = tx_dst_mac;
    assign tx_eth_hdr.src_mac  = local_mac;

    assign tx_ip_hdr.valid     = 1'b1;
    assign tx_ip_hdr.src_ip    = local_ip;
    assign tx_ip_hdr.dst_ip    = tx_dst_ip;
    assign tx_ip_hdr.ttl       = tx_ttl;
    assign tx_ip_hdr.protocol  = IP_PROTO_UDP;
    assign tx_ip_hdr.length    = 16'(UDP_HDR_BYTES) + tx_length;

    assign tx_udp_hdr.valid    = 1'b1;
    assign tx_udp_hdr.src_port = tx_src_port;
    assign tx_udp_hdr.dst_port = tx_dst_port;
    assign tx_udp_hdr.length   = tx_length;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            tx_count <= '0;
        end else if(tx_udp_hdr.valid && tx_udp_hdr.ready) begin
            tx_count <= tx_count + 1;
        end
    end


    // ------------------------------------------------------------------
    // RX: keep the headers of each datagram taken
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if(~rstn) begin
            rx_count    <= '0;
            rx_dst_mac  <= '0;
            rx_src_mac  <= '0;
            rx_src_ip   <= '0;
            rx_dst_ip   <= '0;
            rx_protocol <= '0;
            rx_ttl      <= '0;
            rx_src_port <= '0;
            rx_dst_port <= '0;
            rx_length   <= '0;
        end else if(rx_udp_hdr.valid && rx_udp_hdr.ready) begin
            rx_count    <= rx_count + 1;
            rx_dst_mac  <= rx_eth_hdr.dst_mac;
            rx_src_mac  <= rx_eth_hdr.src_mac;
            rx_src_ip   <= rx_ip_hdr.src_ip;
            rx_dst_ip   <= rx_ip_hdr.dst_ip;
            rx_protocol <= rx_ip_hdr.protocol;
            rx_ttl      <= rx_ip_hdr.ttl;
            rx_src_port <= rx_udp_hdr.src_port;
            rx_dst_port <= rx_udp_hdr.dst_port;
            rx_length   <= rx_udp_hdr.length;
        end
    end

endmodule
