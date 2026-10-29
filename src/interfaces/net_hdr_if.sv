// Packet header channels for the network stack (src/ethernet/packetization).
//
// Each carries one header per packet: the fields are valid while `valid` is high, and
// the header is taken when valid && ready. Only the fields that matter outside a
// layer are here; the rest of each header is fixed or worked out by the
// (de)packetizer.
//
// Byte order: fields are numbers, so the first byte on the wire is in the top bits.
// MAC aa:bb:cc:dd:ee:ff is 48'haabbccddeeff, IP 192.168.1.10 is 32'hC0A8010A.
//
// mon is for watching a channel without taking part in the handshake.

interface eth_hdr_if;
    logic        valid;
    logic        ready;
    logic [47:0] dst_mac;
    logic [47:0] src_mac;

    modport mst (
        output valid,
        output dst_mac,
        output src_mac,
        input  ready
    );

    modport slv (
        input  valid,
        input  dst_mac,
        input  src_mac,
        output ready
    );

    modport mon (
        input  valid,
        input  dst_mac,
        input  src_mac,
        input  ready
    );
endinterface

interface ipv4_hdr_if;
    logic        valid;
    logic        ready;
    logic [31:0] src_ip;
    logic [31:0] dst_ip;
    logic [7:0]  ttl;
    logic [7:0]  protocol;
    logic [15:0] length;   // Payload bytes (total length minus the IPv4 header)

    modport mst (
        output valid,
        output src_ip,
        output dst_ip,
        output ttl,
        output protocol,
        output length,
        input  ready
    );

    modport slv (
        input  valid,
        input  src_ip,
        input  dst_ip,
        input  ttl,
        input  protocol,
        input  length,
        output ready
    );

    modport mon (
        input  valid,
        input  src_ip,
        input  dst_ip,
        input  ttl,
        input  protocol,
        input  length,
        input  ready
    );
endinterface

interface udp_hdr_if;
    logic        valid;
    logic        ready;
    logic [15:0] src_port;
    logic [15:0] dst_port;
    logic [15:0] length;   // Payload bytes (UDP length minus the UDP header)

    modport mst (
        output valid,
        output src_port,
        output dst_port,
        output length,
        input  ready
    );

    modport slv (
        input  valid,
        input  src_port,
        input  dst_port,
        input  length,
        output ready
    );

    modport mon (
        input  valid,
        input  src_port,
        input  dst_port,
        input  length,
        input  ready
    );
endinterface
