// Constants and helpers shared by the Ethernet / IPv4 / UDP (de)packetizers.
// Modules that use it `include it; the guard makes it compile once however many do.
`ifndef NET_PKG_SV
`define NET_PKG_SV
package net_pkg;

    localparam logic [15:0] ETHERTYPE_IPV4 = 16'h0800;
    localparam logic [7:0]  IP_PROTO_UDP   = 8'd17;

    localparam int ETH_HDR_BYTES   = 14;
    localparam int ETH_MIN_PAYLOAD = 46;   // Shorter payloads are padded with zeros
    localparam int ETH_FCS_BYTES   = 4;
    localparam int ETH_MTU         = 1500; // Largest Ethernet payload
    localparam int IPV4_HDR_BYTES  = 20;   // Without options
    localparam int UDP_HDR_BYTES   = 8;
    localparam int UDP_MAX_PAYLOAD = ETH_MTU - IPV4_HDR_BYTES - UDP_HDR_BYTES; // 1472
    localparam int RTP_HDR_BYTES   = 12;   // Without CSRCs or extension

    localparam logic [47:0] MAC_BROADCAST = 48'hFFFF_FFFF_FFFF;
    localparam logic [31:0] IP_BROADCAST  = 32'hFFFF_FFFF;

    // CRC-32 as Ethernet uses it: reflected polynomial 0xEDB88320, data LSB first.
    // Start from 0xFFFFFFFF; the FCS is ~crc, sent low byte first. Run over a frame
    // and its FCS, a good frame leaves CRC32_RESIDUE.
    localparam logic [31:0] CRC32_INIT    = 32'hFFFF_FFFF;
    localparam logic [31:0] CRC32_RESIDUE = 32'hDEBB_20E3;

    function automatic logic [31:0] crc32_byte(input logic [31:0] crc, input logic [7:0] data);
        crc = crc ^ {24'h0, data};
        for (int i = 0; i < 8; i++)
            crc = crc[0] ? (crc >> 1) ^ 32'hEDB8_8320 : crc >> 1;
        return crc;
    endfunction

    // Fold a 32-bit sum of 16-bit words into a 16-bit ones' complement sum
    function automatic logic [15:0] csum_fold(input logic [31:0] sum);
        sum = {16'h0, sum[15:0]} + {16'h0, sum[31:16]};
        sum = {16'h0, sum[15:0]} + {16'h0, sum[31:16]};
        return sum[15:0];
    endfunction

endpackage
`endif
