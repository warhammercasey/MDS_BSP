// IPv4 packetizer: puts a 20 byte IPv4 header (no options) in front of each payload.
//
//   hdr:      addresses, TTL, protocol and payload length for the next packet. Taken
//             as soon as the packetizer is idle.
//   axis_in:  the payload, hdr.length bytes, tlast on the last (passed through as is)
//   axis_out: the IPv4 packet
//
// Fixed fields: version 4, IHL 5, DSCP/ECN 0, flags DF (no fragmenting), fragment
// offset 0. Identification counts up by one per packet.
`ifdef LINT_INCLUDES
`include "net_pkg.sv"
`endif
module ipv4_packetizer
    import net_pkg::*;
(
    input  logic      clk,
    input  logic      rstn,

    ipv4_hdr_if.slv   hdr,
    axis_tlast_if.slv axis_in,   // Payload

    axis_tlast_if.mst axis_out   // IPv4 packet
);

    localparam logic [15:0] FLAGS_DF = 16'h4000; // Flags and fragment offset: don't fragment

    typedef enum {
        IDLE,
        CSUM,    // Header latched, working out the checksum
        HEADER,
        PAYLOAD
    } state_t;
    state_t state, next_state;

    logic [31:0] src_ip, dst_ip;
    logic [7:0]  ttl, protocol;
    logic [15:0] total_length;
    logic [15:0] identification;
    logic [15:0] checksum;
    logic [IPV4_HDR_BYTES*8-1:0] header;  // First byte in the top bits
    logic [$clog2(IPV4_HDR_BYTES)-1:0] byte_counter;

    logic out_beat;
    assign out_beat = axis_out.tvalid && axis_out.tready;

    assign hdr.ready = state == IDLE;

    always_comb begin
        next_state = state;
        if(~rstn) begin
            next_state = IDLE;
        end else begin
            case(state)
            IDLE: if(hdr.valid) begin
                next_state = CSUM;
            end

            CSUM: begin
                next_state = HEADER;
            end

            HEADER: if(out_beat && byte_counter == IPV4_HDR_BYTES - 1) begin
                next_state = PAYLOAD;
            end

            PAYLOAD: if(out_beat && axis_in.tlast) begin
                next_state = IDLE;
            end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        state <= next_state;
    end


    // Latch the header
    always_ff @(posedge clk) begin
        if(~rstn) begin
            src_ip       <= '0;
            dst_ip       <= '0;
            ttl          <= '0;
            protocol     <= '0;
            total_length <= '0;
        end else if(hdr.valid && hdr.ready) begin
            src_ip       <= hdr.src_ip;
            dst_ip       <= hdr.dst_ip;
            ttl          <= hdr.ttl;
            protocol     <= hdr.protocol;
            total_length <= hdr.length + 16'(IPV4_HDR_BYTES);
        end
    end

    // Identification - one per packet
    always_ff @(posedge clk) begin
        if(~rstn) begin
            identification <= '0;
        end else if(state == PAYLOAD && next_state == IDLE) begin
            identification <= identification + 1;
        end
    end

    // Header checksum: ones' complement of the ones' complement sum of the header's
    // 16-bit words (with the checksum taken as 0)
    always_ff @(posedge clk) begin
        if(~rstn) begin
            checksum <= '0;
        end else if(state == CSUM) begin
            checksum <= ~csum_fold(32'h4500 + 32'(total_length) + 32'(identification) + 32'(FLAGS_DF) +
                                   32'({ttl, protocol}) + 32'(src_ip[31:16]) + 32'(src_ip[15:0]) +
                                   32'(dst_ip[31:16]) + 32'(dst_ip[15:0]));
        end
    end

    assign header = {8'h45, 8'h00, total_length, identification, FLAGS_DF,
                     ttl, protocol, checksum, src_ip, dst_ip};

    // Byte counter - header bytes
    always_ff @(posedge clk) begin
        if(~rstn || state != HEADER) begin
            byte_counter <= '0;
        end else if(out_beat) begin
            byte_counter <= byte_counter + 1;
        end
    end


    // Output
    assign axis_in.tready = state == PAYLOAD && axis_out.tready;

    always_comb begin
        axis_out.tvalid = 1'b0;
        axis_out.tdata  = '0;
        axis_out.tlast  = 1'b0;
        case(state)
        HEADER: begin
            axis_out.tvalid = 1'b1;
            axis_out.tdata  = header[(IPV4_HDR_BYTES - 1 - byte_counter)*8 +: 8];
        end

        PAYLOAD: begin
            axis_out.tvalid = axis_in.tvalid;
            axis_out.tdata  = axis_in.tdata;
            axis_out.tlast  = axis_in.tlast;
        end

        default: ;
        endcase
    end

endmodule
