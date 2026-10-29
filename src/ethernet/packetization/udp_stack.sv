// UDP/IPv4/Ethernet stack between the RMII MAC (rmii) and the application.
//
// TX:  tx_payload -> udp_packetizer -> ipv4_packetizer -> eth_packetizer
//                 -> axis_packet_fifo (if TX_BUFFER) -> mac_tx
//      Each datagram carries tx_udp_hdr.length bytes of tx_payload, with the
//      addresses, TTL and ports offered on tx_eth_hdr / tx_ip_hdr / tx_udp_hdr at its
//      start. rmii_tx can't wait for data mid-frame:
//        TX_BUFFER = 1: frames are buffered whole before going to rmii_tx, so
//                       tx_payload may stall freely.
//        TX_BUFFER = 0: no buffer (saves 2^TX_AWIDTH bytes of RAM). Once a datagram's
//                       first byte is offered, tx_payload must keep up with rmii_tx (a
//                       byte every 4 cycles) until its last, or the frame goes out
//                       corrupted. A store-and-forward source like axis_packet_switch
//                       does this.
//
// RX:  mac_rx -> eth_depacketizer -> ipv4_depacketizer -> udp_depacketizer
//                 -> rx_payload
//      Only good UDP datagrams for us come out (see the depacketizers for what is
//      dropped). "For us" means to local_mac / local_ip or broadcast, or anything when
//      promiscuous is set.
//      rx_udp_hdr is offered before each payload. rx_eth_hdr and rx_ip_hdr hold the
//      same packet's lower headers: their valid follows rx_udp_hdr.valid and their
//      ready is ignored.
`ifdef LINT_INCLUDES
`include "net_pkg.sv"
`endif
module udp_stack
    import net_pkg::*;
#(
    parameter TX_BUFFER = 1,   // Buffer TX frames whole (see above)
    parameter TX_AWIDTH = 11,  // TX frame buffer, 2^TX_AWIDTH bytes: fits a full frame
    parameter RX_AWIDTH = 12   // RX frame buffer, 2^RX_AWIDTH bytes: fits two
)(
    input  logic        clk,
    input  logic        rstn,

    // Our addresses, for the RX filters
    input  logic [47:0] local_mac,
    input  logic [31:0] local_ip,
    input  logic        promiscuous,

    // To/from rmii
    axis_tlast_if.mst   mac_tx,
    axis_tlast_if.slv   mac_rx,

    // Application TX
    eth_hdr_if.slv      tx_eth_hdr,
    ipv4_hdr_if.slv     tx_ip_hdr,    // protocol and length are ignored
    udp_hdr_if.slv      tx_udp_hdr,
    axis_if.slv         tx_payload,

    // Application RX
    eth_hdr_if.mst      rx_eth_hdr,
    ipv4_hdr_if.mst     rx_ip_hdr,
    udp_hdr_if.mst      rx_udp_hdr,
    axis_tlast_if.mst   rx_payload
);

    // ------------------------------------------------------------------
    // TX
    // ------------------------------------------------------------------
    ipv4_hdr_if                 tx_udp_ip_hdr ();
    axis_tlast_if #(.WIDTH(8))  tx_udp ();
    axis_tlast_if #(.WIDTH(8))  tx_ip ();
    axis_tlast_if #(.WIDTH(8))  tx_frame ();

    udp_packetizer udp_packetizer_i (
        .clk(clk),
        .rstn(rstn),
        .hdr(tx_udp_hdr),
        .ip_hdr_in(tx_ip_hdr),
        .axis_in(tx_payload),
        .ip_hdr_out(tx_udp_ip_hdr),
        .axis_out(tx_udp)
    );

    ipv4_packetizer ipv4_packetizer_i (
        .clk(clk),
        .rstn(rstn),
        .hdr(tx_udp_ip_hdr),
        .axis_in(tx_udp),
        .axis_out(tx_ip)
    );

    eth_packetizer eth_packetizer_i (
        .clk(clk),
        .rstn(rstn),
        .hdr(tx_eth_hdr),
        .axis_in(tx_ip),
        .axis_out(tx_frame)
    );

    generate
        if (TX_BUFFER) begin : g_tx_buffer
            axis_packet_fifo #(
                .WIDTH(8),
                .AWIDTH(TX_AWIDTH),
                .DROP_WHEN_FULL(0)
            ) tx_frame_buffer (
                .clk(clk),
                .rstn(rstn),
                .axis_in(tx_frame),
                .in_drop(1'b0),
                .axis_out(mac_tx)
            );
        end else begin : g_no_tx_buffer
            assign mac_tx.tvalid   = tx_frame.tvalid;
            assign mac_tx.tdata    = tx_frame.tdata;
            assign mac_tx.tlast    = tx_frame.tlast;
            assign tx_frame.tready = mac_tx.tready;
        end
    endgenerate


    // ------------------------------------------------------------------
    // RX
    // ------------------------------------------------------------------
    eth_hdr_if                  rx_eth_hdr_i ();
    ipv4_hdr_if                 rx_ip_hdr_i ();
    axis_tlast_if #(.WIDTH(8))  rx_eth ();
    axis_tlast_if #(.WIDTH(8))  rx_ip ();

    eth_depacketizer #(
        .AWIDTH(RX_AWIDTH)
    ) eth_depacketizer_i (
        .clk(clk),
        .rstn(rstn),
        .local_mac(local_mac),
        .promiscuous(promiscuous),
        .axis_in(mac_rx),
        .hdr(rx_eth_hdr_i),
        .axis_out(rx_eth)
    );

    ipv4_depacketizer ipv4_depacketizer_i (
        .clk(clk),
        .rstn(rstn),
        .local_ip(local_ip),
        .promiscuous(promiscuous),
        .axis_in(rx_eth),
        .hdr(rx_ip_hdr_i),
        .axis_out(rx_ip)
    );

    udp_depacketizer udp_depacketizer_i (
        .clk(clk),
        .rstn(rstn),
        .axis_in(rx_ip),
        .hdr(rx_udp_hdr),
        .axis_out(rx_payload)
    );

    // Hold each packet's Ethernet and IPv4 headers until its UDP header goes out.
    // Packets go through the layers in order, so the last ones taken belong to the
    // UDP header on offer, as long as the next frame's Ethernet header waits for it:
    // a datagram that ends with its header lets the Ethernet layer run ahead.
    logic [47:0] rx_dst_mac, rx_src_mac;
    logic [31:0] rx_src_ip, rx_dst_ip;
    logic [7:0]  rx_ttl, rx_protocol;
    logic [15:0] rx_ip_length;

    assign rx_eth_hdr_i.ready = !rx_udp_hdr.valid;
    assign rx_ip_hdr_i.ready  = 1'b1;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            rx_dst_mac <= '0;
            rx_src_mac <= '0;
        end else if(rx_eth_hdr_i.valid && rx_eth_hdr_i.ready) begin
            rx_dst_mac <= rx_eth_hdr_i.dst_mac;
            rx_src_mac <= rx_eth_hdr_i.src_mac;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            rx_src_ip    <= '0;
            rx_dst_ip    <= '0;
            rx_ttl       <= '0;
            rx_protocol  <= '0;
            rx_ip_length <= '0;
        end else if(rx_ip_hdr_i.valid && rx_ip_hdr_i.ready) begin
            rx_src_ip    <= rx_ip_hdr_i.src_ip;
            rx_dst_ip    <= rx_ip_hdr_i.dst_ip;
            rx_ttl       <= rx_ip_hdr_i.ttl;
            rx_protocol  <= rx_ip_hdr_i.protocol;
            rx_ip_length <= rx_ip_hdr_i.length;
        end
    end

    assign rx_eth_hdr.valid   = rx_udp_hdr.valid;
    assign rx_eth_hdr.dst_mac = rx_dst_mac;
    assign rx_eth_hdr.src_mac = rx_src_mac;

    assign rx_ip_hdr.valid    = rx_udp_hdr.valid;
    assign rx_ip_hdr.src_ip   = rx_src_ip;
    assign rx_ip_hdr.dst_ip   = rx_dst_ip;
    assign rx_ip_hdr.ttl      = rx_ttl;
    assign rx_ip_hdr.protocol = rx_protocol;
    assign rx_ip_hdr.length   = rx_ip_length;

endmodule
