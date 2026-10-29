// Ethernet packetizer: wraps each packet on axis_in in an Ethernet frame.
//
//   axis_out: destination MAC, source MAC, EtherType (ETHERTYPE), the payload,
//             zero padding up to the 46 byte minimum payload, and the FCS. tlast is on
//             the last FCS byte. This is what rmii_tx sends (it adds the preamble/SFD).
//
// A frame starts once a header is offered on hdr and the payload's first byte is
// waiting; the header is taken then. The payload passes straight through, so
// axis_out only stalls when axis_in does. rmii_tx can't wait for data mid-frame,
// so put a packet FIFO in between if the source might stall.
`ifdef LINT_INCLUDES
`include "net_pkg.sv"
`endif
module eth_packetizer
    import net_pkg::*;
#(
    parameter logic [15:0] ETHERTYPE = ETHERTYPE_IPV4
)(
    input  logic      clk,
    input  logic      rstn,

    eth_hdr_if.slv    hdr,
    axis_tlast_if.slv axis_in,   // Payload, tlast on the last byte

    axis_tlast_if.mst axis_out   // Frame
);

    typedef enum {
        IDLE,
        HEADER,
        PAYLOAD,
        PAD,
        FCS
    } state_t;
    state_t state, next_state;

    logic [ETH_HDR_BYTES*8-1:0] header;      // The header, first byte in the top bits
    logic [$clog2(ETH_HDR_BYTES)-1:0] byte_counter; // Header byte, then FCS byte
    logic [$clog2(ETH_MIN_PAYLOAD+1)-1:0] payload_counter; // Payload + pad bytes, up to the minimum
    logic [31:0] crc;

    logic out_beat;
    logic header_done, fcs_done, min_reached;

    assign out_beat    = axis_out.tvalid && axis_out.tready;
    assign header_done = byte_counter == ETH_HDR_BYTES - 1;
    assign fcs_done    = byte_counter == ETH_FCS_BYTES - 1;
    // Counting this beat, the payload is at least the minimum
    assign min_reached = payload_counter >= ETH_MIN_PAYLOAD - 1;

    assign hdr.ready = state == IDLE && axis_in.tvalid;


    // State machine
    always_comb begin
        next_state = state;
        if(~rstn) begin
            next_state = IDLE;
        end else begin
            case(state)
            IDLE: if(hdr.valid && hdr.ready) begin
                next_state = HEADER;
            end

            HEADER: if(out_beat && header_done) begin
                next_state = PAYLOAD;
            end

            PAYLOAD: if(out_beat && axis_in.tlast) begin
                next_state = min_reached ? FCS : PAD;
            end

            PAD: if(out_beat && min_reached) begin
                next_state = FCS;
            end

            FCS: if(out_beat && fcs_done) begin
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
            header <= '0;
        end else if(hdr.valid && hdr.ready) begin
            header <= {hdr.dst_mac, hdr.src_mac, ETHERTYPE};
        end
    end

    // Byte counter - counts header bytes, then FCS bytes
    always_ff @(posedge clk) begin
        if(~rstn || state != next_state) begin
            byte_counter <= '0;
        end else if(out_beat) begin
            byte_counter <= byte_counter + 1;
        end
    end

    // Payload counter - payload and pad bytes, saturating at the minimum
    always_ff @(posedge clk) begin
        if(~rstn || state == IDLE) begin
            payload_counter <= '0;
        end else if(out_beat && (state == PAYLOAD || state == PAD) && !min_reached) begin
            payload_counter <= payload_counter + 1;
        end
    end

    // CRC over everything before the FCS
    always_ff @(posedge clk) begin
        if(~rstn || state == IDLE) begin
            crc <= CRC32_INIT;
        end else if(out_beat && state != FCS) begin
            crc <= crc32_byte(crc, axis_out.tdata);
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
            axis_out.tdata  = header[(ETH_HDR_BYTES - 1 - byte_counter)*8 +: 8];
        end

        PAYLOAD: begin
            axis_out.tvalid = axis_in.tvalid;
            axis_out.tdata  = axis_in.tdata;
        end

        PAD: begin
            axis_out.tvalid = 1'b1;
            axis_out.tdata  = 8'h00;
        end

        FCS: begin
            axis_out.tvalid = 1'b1;
            axis_out.tdata  = ~crc[byte_counter*8 +: 8];
            axis_out.tlast  = fcs_done;
        end

        default: ;
        endcase
    end

endmodule
