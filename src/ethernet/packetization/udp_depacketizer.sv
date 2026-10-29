// UDP depacketizer: splits each datagram into a header (hdr) and payload (axis_out).
//
//   axis_in:  UDP datagrams, e.g. IPv4 payloads
//   hdr:      ports and payload length, offered before each payload
//   axis_out: the payload: exactly hdr.length bytes, tlast on the last. Anything after
//             the UDP length is dropped. If the datagram ends early, tlast comes early.
//
// Datagrams are dropped when the UDP length is under 8 (bad), or there is no payload.
// The checksum isn't checked: the Ethernet FCS already covers the link, and checking
// it would mean holding the payload back until its end.
`ifdef LINT_INCLUDES
`include "net_pkg.sv"
`endif
module udp_depacketizer
    import net_pkg::*;
(
    input  logic      clk,
    input  logic      rstn,

    axis_tlast_if.slv axis_in,    // UDP datagram

    udp_hdr_if.mst    hdr,
    axis_tlast_if.mst axis_out    // Payload
);

    typedef enum {
        HEADER,   // Reading the header
        CHECK,    // Header read: keep the datagram or not
        HDR_OUT,  // Offering the header
        PAYLOAD,  // Passing the payload through
        DISCARD   // Throwing away the rest of the datagram
    } state_t;
    state_t state, next_state;

    logic [UDP_HDR_BYTES*8-1:0] header;   // First byte in the top bits
    logic [$clog2(UDP_HDR_BYTES)-1:0] byte_counter;
    logic [15:0] payload_counter;

    logic [15:0] src_port, dst_port, udp_length, payload_length;

    logic in_beat, out_beat, last_out;
    assign in_beat  = axis_in.tvalid && axis_in.tready;
    assign out_beat = axis_out.tvalid && axis_out.tready;

    assign src_port       = header[63:48];
    assign dst_port       = header[47:32];
    assign udp_length     = header[31:16];
    assign payload_length = udp_length - 16'(UDP_HDR_BYTES);

    always_comb begin
        next_state = state;
        if(~rstn) begin
            next_state = HEADER;
        end else begin
            case(state)
            HEADER: if(in_beat && byte_counter == UDP_HDR_BYTES - 1) begin
                next_state = axis_in.tlast ? HEADER : CHECK; // No payload: drop
            end

            CHECK: begin
                next_state = udp_length > 16'(UDP_HDR_BYTES) ? HDR_OUT : DISCARD;
            end

            HDR_OUT: if(hdr.ready) begin
                next_state = PAYLOAD;
            end

            PAYLOAD: if(out_beat && last_out) begin
                next_state = axis_in.tlast ? HEADER : DISCARD;
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

    // Byte counter - header bytes. A datagram that ends inside the header is dropped.
    always_ff @(posedge clk) begin
        if(~rstn || state != HEADER || (in_beat && axis_in.tlast)) begin
            byte_counter <= '0;
        end else if(in_beat) begin
            byte_counter <= byte_counter + 1;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            header <= '0;
        end else if(state == HEADER && in_beat) begin
            header <= {header[UDP_HDR_BYTES*8-9:0], axis_in.tdata};
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
    assign hdr.valid    = state == HDR_OUT;
    assign hdr.src_port = src_port;
    assign hdr.dst_port = dst_port;
    assign hdr.length   = payload_length;

    assign last_out        = payload_counter == payload_length - 1 || axis_in.tlast;
    assign axis_out.tvalid = state == PAYLOAD && axis_in.tvalid;
    assign axis_out.tdata  = axis_in.tdata;
    assign axis_out.tlast  = last_out;

    assign axis_in.tready = state == HEADER || state == DISCARD ||
                            (state == PAYLOAD && axis_out.tready);

endmodule
