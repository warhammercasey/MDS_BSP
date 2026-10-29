// UDP packetizer: cuts a byte stream into UDP datagrams.
//
//   hdr:         ports, and how many payload bytes go in each datagram
//   ip_hdr_in:   addresses and TTL for the IPv4 header (protocol and length ignored)
//   axis_in:     payload bytes. A plain stream: each datagram takes hdr.length bytes
//                from it.
//   ip_hdr_out:  the IPv4 header for the datagram, to ipv4_packetizer: addresses and
//                TTL from ip_hdr_in, protocol UDP, length the whole datagram
//   axis_out:    the datagram (UDP header and payload), tlast on the last byte
//
// A datagram starts once hdr and ip_hdr_in are offered and the first payload byte is
// waiting; both headers are taken then. The UDP checksum is sent as 0 (none), which
// IPv4 allows: computing it would need the whole payload before the header.
`ifdef LINT_INCLUDES
`include "net_pkg.sv"
`endif
module udp_packetizer
    import net_pkg::*;
(
    input  logic      clk,
    input  logic      rstn,

    udp_hdr_if.slv    hdr,
    ipv4_hdr_if.slv   ip_hdr_in,
    axis_if.slv       axis_in,     // Payload stream

    ipv4_hdr_if.mst   ip_hdr_out,
    axis_tlast_if.mst axis_out     // UDP datagram
);

    typedef enum {
        IDLE,
        IP_HDR,   // Offering the IPv4 header
        HEADER,
        PAYLOAD
    } state_t;
    state_t state, next_state;

    logic [15:0] src_port, dst_port, payload_length, udp_length;
    logic [31:0] src_ip, dst_ip;
    logic [7:0]  ttl;
    logic [UDP_HDR_BYTES*8-1:0] header;   // First byte in the top bits
    logic [$clog2(UDP_HDR_BYTES)-1:0] byte_counter;
    logic [15:0] payload_counter;

    logic start, out_beat, header_last, payload_last;
    assign out_beat     = axis_out.tvalid && axis_out.tready;
    assign header_last  = byte_counter == UDP_HDR_BYTES - 1;
    assign payload_last = payload_counter == payload_length - 1;

    // Both headers are taken together, once there is payload to send
    assign start           = state == IDLE && hdr.valid && ip_hdr_in.valid && axis_in.tvalid;
    assign hdr.ready       = start;
    assign ip_hdr_in.ready = start;

    always_comb begin
        next_state = state;
        if(~rstn) begin
            next_state = IDLE;
        end else begin
            case(state)
            IDLE: if(start) begin
                next_state = IP_HDR;
            end

            IP_HDR: if(ip_hdr_out.ready) begin
                next_state = HEADER;
            end

            HEADER: if(out_beat && header_last) begin
                next_state = payload_length == 0 ? IDLE : PAYLOAD;
            end

            PAYLOAD: if(out_beat && payload_last) begin
                next_state = IDLE;
            end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        state <= next_state;
    end


    // Latch the headers
    always_ff @(posedge clk) begin
        if(~rstn) begin
            src_port       <= '0;
            dst_port       <= '0;
            payload_length <= '0;
            src_ip         <= '0;
            dst_ip         <= '0;
            ttl            <= '0;
        end else if(start) begin
            src_port       <= hdr.src_port;
            dst_port       <= hdr.dst_port;
            payload_length <= hdr.length;
            src_ip         <= ip_hdr_in.src_ip;
            dst_ip         <= ip_hdr_in.dst_ip;
            ttl            <= ip_hdr_in.ttl;
        end
    end

    assign udp_length = payload_length + 16'(UDP_HDR_BYTES);
    assign header     = {src_port, dst_port, udp_length, 16'h0000};

    // Counters
    always_ff @(posedge clk) begin
        if(~rstn || state != HEADER) begin
            byte_counter <= '0;
        end else if(out_beat) begin
            byte_counter <= byte_counter + 1;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn || state != PAYLOAD) begin
            payload_counter <= '0;
        end else if(out_beat) begin
            payload_counter <= payload_counter + 1;
        end
    end


    // Outputs
    assign ip_hdr_out.valid    = state == IP_HDR;
    assign ip_hdr_out.src_ip   = src_ip;
    assign ip_hdr_out.dst_ip   = dst_ip;
    assign ip_hdr_out.ttl      = ttl;
    assign ip_hdr_out.protocol = IP_PROTO_UDP;
    assign ip_hdr_out.length   = udp_length;

    assign axis_in.tready = state == PAYLOAD && axis_out.tready;

    always_comb begin
        axis_out.tvalid = 1'b0;
        axis_out.tdata  = '0;
        axis_out.tlast  = 1'b0;
        case(state)
        HEADER: begin
            axis_out.tvalid = 1'b1;
            axis_out.tdata  = header[(UDP_HDR_BYTES - 1 - byte_counter)*8 +: 8];
            axis_out.tlast  = header_last && payload_length == 0;
        end

        PAYLOAD: begin
            axis_out.tvalid = axis_in.tvalid;
            axis_out.tdata  = axis_in.tdata;
            axis_out.tlast  = payload_last;
        end

        default: ;
        endcase
    end

endmodule
