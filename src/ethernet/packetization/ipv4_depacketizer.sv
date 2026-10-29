// IPv4 depacketizer: checks each packet's header, drops the ones it can't or
// shouldn't handle, and splits the rest into a header (hdr) and payload (axis_out).
//
//   axis_in:  IPv4 packets, e.g. Ethernet payloads (may end in padding)
//   hdr:      addresses, TTL, protocol and payload length, offered before each payload
//   axis_out: the payload: exactly hdr.length bytes, tlast on the last. Padding after
//             it is dropped. If the packet ends early, tlast comes early.
//
// Packets are dropped when:
//   - it isn't version 4, the IHL is under 5, or the total length is shorter than the
//     header (options are skipped)
//   - the header checksum is wrong
//   - it is a fragment (more fragments set, or a fragment offset)
//   - the protocol isn't PROTOCOL
//   - the destination is neither local_ip nor 255.255.255.255 (unless promiscuous)
//   - there is no payload
// A dropped packet is read and thrown away; nothing of it comes out.
`ifdef LINT_INCLUDES
`include "net_pkg.sv"
`endif
module ipv4_depacketizer
    import net_pkg::*;
#(
    parameter logic [7:0] PROTOCOL = IP_PROTO_UDP  // The only protocol passed on
)(
    input  logic        clk,
    input  logic        rstn,

    input  logic [31:0] local_ip,
    input  logic        promiscuous,  // Accept any destination IP

    axis_tlast_if.slv   axis_in,      // IPv4 packet

    ipv4_hdr_if.mst     hdr,
    axis_tlast_if.mst   axis_out      // Payload
);

    localparam MAX_HDR_BYTES = 60;    // IHL 15

    typedef enum {
        HEADER,   // Reading the header, options included
        CHECK,    // Header read: keep the packet or not
        HDR_OUT,  // Offering the header
        PAYLOAD,  // Passing the payload through
        DISCARD   // Throwing away the rest of the packet
    } state_t;
    state_t state, next_state;

    logic [IPV4_HDR_BYTES*8-1:0] header;   // The first 20 bytes, first byte in the top bits
    logic [7:0]  version_ihl;              // Byte 0, kept on its own: header only lines up after 20 bytes
    logic [$clog2(MAX_HDR_BYTES)-1:0] byte_counter;
    logic [31:0] csum;                     // Sum of the header's 16-bit words
    logic [15:0] payload_counter;

    logic [3:0]  version, ihl;
    logic [15:0] total_length;
    logic [15:0] header_length;
    logic [15:0] payload_length;
    logic        more_fragments;
    logic [12:0] fragment_offset;
    logic [7:0]  ttl, protocol;
    logic [31:0] src_ip, dst_ip;
    logic [3:0]  ihl_now;                  // IHL, from this beat while it is byte 0
    logic        header_end;
    logic        accept;

    logic in_beat, out_beat, last_out;
    assign in_beat  = axis_in.tvalid && axis_in.tready;
    assign out_beat = axis_out.tvalid && axis_out.tready;

    assign version         = version_ihl[7:4];
    assign ihl             = version_ihl[3:0];
    assign total_length    = header[143:128];
    assign more_fragments  = header[109];
    assign fragment_offset = header[108:96];
    assign ttl             = header[95:88];
    assign protocol        = header[87:80];
    assign src_ip          = header[63:32];
    assign dst_ip          = header[31:0];
    assign header_length   = {10'h0, ihl, 2'b00};
    assign payload_length  = total_length - header_length;

    // The header ends after IHL * 4 bytes. Under 20 is a bad header: stop after byte 0.
    assign ihl_now    = byte_counter == 0 ? axis_in.tdata[3:0] : ihl;
    assign header_end = ihl_now < 5 ? 1'b1 : byte_counter == {ihl_now, 2'b00} - 1;

    assign accept = version == 4 && ihl >= 5 &&
                    csum_fold(csum) == 16'hFFFF &&
                    total_length > header_length &&
                    !more_fragments && fragment_offset == 0 &&
                    protocol == PROTOCOL &&
                    (promiscuous || dst_ip == local_ip || dst_ip == IP_BROADCAST);

    always_comb begin
        next_state = state;
        if(~rstn) begin
            next_state = HEADER;
        end else begin
            case(state)
            HEADER: if(in_beat && header_end) begin
                next_state = axis_in.tlast ? HEADER : CHECK; // No payload: drop
            end

            CHECK: begin
                next_state = accept ? HDR_OUT : DISCARD;
            end

            HDR_OUT: if(hdr.ready) begin
                next_state = PAYLOAD;
            end

            PAYLOAD: if(out_beat && last_out) begin
                next_state = axis_in.tlast ? HEADER : DISCARD; // Discard the padding
            end

            DISCARD: if(in_beat && axis_in.tlast) begin
                next_state = HEADER;
            end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        state <= next_state;
    end

    // Byte counter - header bytes. A packet that ends inside the header is dropped.
    always_ff @(posedge clk) begin
        if(~rstn || state != HEADER || (in_beat && (axis_in.tlast || header_end))) begin
            byte_counter <= '0;
        end else if(in_beat) begin
            byte_counter <= byte_counter + 1;
        end
    end

    // Keep the first 20 bytes; sum all of them as 16-bit words, options included
    always_ff @(posedge clk) begin
        if(~rstn) begin
            header      <= '0;
            version_ihl <= '0;
            csum        <= '0;
        end else if(state == HEADER && in_beat) begin
            if(byte_counter == 0) begin
                version_ihl <= axis_in.tdata;
            end
            if(byte_counter < IPV4_HDR_BYTES) begin
                header <= {header[IPV4_HDR_BYTES*8-9:0], axis_in.tdata};
            end
            if(byte_counter == 0) begin
                csum <= {16'h0, axis_in.tdata, 8'h00};
            end else begin
                csum <= csum + (byte_counter[0] ? {24'h0, axis_in.tdata} : {16'h0, axis_in.tdata, 8'h00});
            end
        end
    end

    // Payload counter
    always_ff @(posedge clk) begin
        if(~rstn || state != PAYLOAD) begin
            payload_counter <= '0;
        end else if(out_beat) begin
            payload_counter <= payload_counter + 1;
        end
    end

    // Outputs
    assign hdr.valid    = state == HDR_OUT;
    assign hdr.src_ip   = src_ip;
    assign hdr.dst_ip   = dst_ip;
    assign hdr.ttl      = ttl;
    assign hdr.protocol = protocol;
    assign hdr.length   = payload_length;

    assign last_out        = payload_counter == payload_length - 1 || axis_in.tlast;
    assign axis_out.tvalid = state == PAYLOAD && axis_in.tvalid;
    assign axis_out.tdata  = axis_in.tdata;
    assign axis_out.tlast  = last_out;

    assign axis_in.tready = state == HEADER || state == DISCARD ||
                            (state == PAYLOAD && axis_out.tready);

endmodule
