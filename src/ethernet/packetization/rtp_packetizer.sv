// RTP packetizer: packs a stream of multichannel samples into RTP packets (RFC 3550).
//
//   axis_in:  samples, tid = channel, in channel order: 0, 1, ..., CHANNELS-1, 0, ...
//   axis_out: RTP packets: the 12 byte header, then FRAMES_PER_PACKET frames of CHANNELS
//             interleaved samples. tlast on the last byte. Every packet is
//             RTP_HDR_BYTES + FRAMES_PER_PACKET * CHANNELS * SAMPLE_BYTES long.
//
// Each sample goes out as SAMPLE_BYTES = ceil(SAMPLE_BITS / 8) bytes, big endian and
// left justified: the sample's MSB is the MSB of the first byte, unused bottom bits
// are 0. So unsigned 8-bit samples are RTP's L8, and two's complement samples of 17 to
// 24 bits are L24.
//
// Header: version 2, no padding, extension or CSRCs, payload type PAYLOAD_TYPE, SSRC.
//   Sequence number: counts packets, from 0 after reset.
//   Timestamp:       the first frame's number. Frames are counted (one per sample
//                    period) from 0 after reset, including frames thrown away below.
//   Marker:          set on the first packet after reset, and on the first packet
//                    after samples were thrown away.
//
// Packets always start on channel 0: other channels arriving when a packet is due to
// start are thrown away. Within a packet samples are taken in turn without waiting on
// tid, but one with the wrong tid (a sample was lost upstream) makes the next packet
// start over at channel 0.
`ifdef LINT_INCLUDES
`include "net_pkg.sv"
`endif
module rtp_packetizer
    import net_pkg::*;
#(
    parameter logic [31:0] SSRC              = 32'h0000_0000,
    parameter logic [6:0]  PAYLOAD_TYPE      = 7'd96,  // Dynamic payload types are 96 to 127
    parameter              SAMPLE_BITS       = 16,
    parameter              CHANNELS          = 1,
    parameter              ID_WIDTH          = 1,
    parameter              FRAMES_PER_PACKET = 1
)(
    input  logic      clk,
    input  logic      rstn,

    axis_tid_if.slv   axis_in,   // Samples
    axis_tlast_if.mst axis_out   // RTP packets
);

    localparam SAMPLE_BYTES = (SAMPLE_BITS + 7) / 8;
    localparam SAMPLES      = FRAMES_PER_PACKET * CHANNELS; // Per packet

    typedef enum {
        IDLE,     // Waiting for channel 0
        HEADER,
        PAYLOAD
    } state_t;
    state_t state, next_state;

    logic [RTP_HDR_BYTES*8-1:0] header;           // First byte in the top bits
    logic [SAMPLE_BYTES*8-1:0]  sample;           // Left justified
    logic [$clog2(RTP_HDR_BYTES)-1:0] byte_counter; // Header byte, then byte of the sample
    logic [$clog2(SAMPLES+1)-1:0] sample_counter;
    logic [ID_WIDTH-1:0] channel;                 // Channel the next sample should be
    logic [15:0] sequence_number;
    logic [31:0] frame_count;
    logic [31:0] timestamp;
    logic        marker;
    logic        resync;                          // Samples were thrown away: mark the next packet

    logic in_beat, out_beat;
    logic start, drop, out_of_step;
    logic sample_done, packet_done;

    assign in_beat     = axis_in.tvalid && axis_in.tready;
    assign out_beat    = axis_out.tvalid && axis_out.tready;
    assign start       = state == IDLE && axis_in.tvalid && axis_in.tid == 0;
    assign drop        = state == IDLE && in_beat;
    assign out_of_step = state == PAYLOAD && in_beat && axis_in.tid != channel;
    assign sample_done = byte_counter == SAMPLE_BYTES - 1;
    assign packet_done = sample_done && sample_counter == SAMPLES - 1;


    // State machine
    always_comb begin
        next_state = state;
        if(~rstn) begin
            next_state = IDLE;
        end else begin
            case(state)
            IDLE: if(start) begin
                next_state = HEADER;
            end

            HEADER: if(out_beat && byte_counter == RTP_HDR_BYTES - 1) begin
                next_state = PAYLOAD;
            end

            PAYLOAD: if(out_beat && packet_done) begin
                next_state = IDLE;
            end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        state <= next_state;
    end


    // Frame count: one per channel 0 sample, taken or thrown away
    always_ff @(posedge clk) begin
        if(~rstn) begin
            frame_count <= '0;
        end else if(in_beat && axis_in.tid == 0) begin
            frame_count <= frame_count + 1;
        end
    end

    // Per packet header fields
    always_ff @(posedge clk) begin
        if(~rstn) begin
            timestamp <= '0;
            marker    <= 1'b0;
            resync    <= 1'b1;   // The first packet is marked
        end else begin
            if(start) begin
                timestamp <= frame_count;
                marker    <= resync;
                resync    <= 1'b0;
            end else if(drop || out_of_step) begin
                resync    <= 1'b1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            sequence_number <= '0;
        end else if(state == PAYLOAD && next_state == IDLE) begin
            sequence_number <= sequence_number + 1;
        end
    end

    assign header = {2'd2, 1'b0, 1'b0, 4'd0,  // Version, padding, extension, CSRC count
                     marker, PAYLOAD_TYPE,
                     sequence_number, timestamp, SSRC};


    // Counters
    always_ff @(posedge clk) begin
        if(~rstn || state != next_state || (state == PAYLOAD && out_beat && sample_done)) begin
            byte_counter <= '0;
        end else if(out_beat) begin
            byte_counter <= byte_counter + 1;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn || state != PAYLOAD) begin
            sample_counter <= '0;
            channel        <= '0;
        end else if(in_beat) begin
            sample_counter <= sample_counter + 1;
            channel        <= channel == ID_WIDTH'(CHANNELS - 1) ? '0 : channel + 1;
        end
    end


    // Output. A sample is taken once its last byte goes out.
    assign sample = (SAMPLE_BYTES*8)'(axis_in.tdata) << (SAMPLE_BYTES*8 - SAMPLE_BITS);

    assign axis_in.tready = (state == IDLE && axis_in.tid != 0) ||
                            (state == PAYLOAD && axis_out.tready && sample_done);

    always_comb begin
        axis_out.tvalid = 1'b0;
        axis_out.tdata  = '0;
        axis_out.tlast  = 1'b0;
        case(state)
        HEADER: begin
            axis_out.tvalid = 1'b1;
            axis_out.tdata  = header[(RTP_HDR_BYTES - 1 - byte_counter)*8 +: 8];
        end

        PAYLOAD: begin
            axis_out.tvalid = axis_in.tvalid;
            axis_out.tdata  = sample[(SAMPLE_BYTES - 1 - byte_counter)*8 +: 8];
            axis_out.tlast  = packet_done;
        end

        default: ;
        endcase
    end

endmodule
