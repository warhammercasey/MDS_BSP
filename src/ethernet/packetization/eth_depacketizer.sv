// Ethernet depacketizer: takes frames from rmii_rx, drops the bad and unwanted ones,
// and splits the rest into a header (hdr) and payload (axis_out).
//
//   axis_in:  frames as rmii_rx gives them (destination MAC through FCS). Never
//             stalled: tready is always high.
//   hdr:      MAC addresses, offered before each payload
//   axis_out: the payload, tlast on its last byte. Includes any padding (the layer
//             above knows its own length).
//
// Frames are dropped when:
//   - the FCS is wrong, or the frame is too short for a header and some payload
//   - the EtherType isn't ETHERTYPE
//   - the destination MAC is neither local_mac nor broadcast (unless promiscuous)
//   - the buffer is full
//
// The FCS comes last, so frames are buffered whole (axis_packet_fifo) and only the
// good ones are passed on. The buffer also soaks up stalls on axis_out. 2^AWIDTH bytes
// should hold at least two full frames (1514 bytes each, the FCS isn't stored).
`ifdef LINT_INCLUDES
`include "net_pkg.sv"
`endif
module eth_depacketizer
    import net_pkg::*;
#(
    parameter logic [15:0] ETHERTYPE = ETHERTYPE_IPV4,
    parameter AWIDTH = 12
)(
    input  logic        clk,
    input  logic        rstn,

    input  logic [47:0] local_mac,
    input  logic        promiscuous,  // Accept any destination MAC

    axis_tlast_if.slv   axis_in,      // Frame, with FCS

    eth_hdr_if.mst      hdr,
    axis_tlast_if.mst   axis_out      // Payload
);

    // ------------------------------------------------------------------
    // FCS check and strip
    //   The last 4 bytes are the FCS, but that is only known at tlast. So each byte
    //   waits in a 4 byte delay line and goes into the buffer when the next 4 have
    //   arrived: when tlast comes in, the byte leaving the line is the last one before
    //   the FCS.
    // ------------------------------------------------------------------
    axis_tlast_if #(.WIDTH(8)) buf_in ();
    axis_tlast_if #(.WIDTH(8)) buf_out ();

    logic [7:0]  delay [ETH_FCS_BYTES];
    logic [2:0]  delay_fill;   // Bytes in the delay line, 0 to 4
    logic [31:0] crc;
    logic [31:0] crc_next;

    assign crc_next = crc32_byte(crc, axis_in.tdata);
    assign axis_in.tready = 1'b1;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            crc        <= CRC32_INIT;
            delay_fill <= '0;
        end else if(axis_in.tvalid) begin
            crc        <= axis_in.tlast ? CRC32_INIT : crc_next;
            delay_fill <= axis_in.tlast ? 3'd0 : (delay_fill == ETH_FCS_BYTES ? delay_fill : delay_fill + 1);
        end
    end

    always_ff @(posedge clk) begin
        if(axis_in.tvalid) begin
            delay[0] <= axis_in.tdata;
            for (int i = 1; i < ETH_FCS_BYTES; i++)
                delay[i] <= delay[i-1];
        end
    end

    // Frames of 4 bytes or less never reach the buffer
    assign buf_in.tvalid = axis_in.tvalid && delay_fill == ETH_FCS_BYTES;
    assign buf_in.tdata  = delay[ETH_FCS_BYTES-1];
    assign buf_in.tlast  = axis_in.tlast;

    axis_packet_fifo #(
        .WIDTH(8),
        .AWIDTH(AWIDTH),
        .DROP_WHEN_FULL(1)
    ) frame_buffer (
        .clk(clk),
        .rstn(rstn),
        .axis_in(buf_in),
        .in_drop(crc_next != CRC32_RESIDUE),
        .axis_out(buf_out)
    );


    // ------------------------------------------------------------------
    // Header parser, on frames coming out of the buffer
    // ------------------------------------------------------------------
    typedef enum {
        HEADER,   // Reading the header
        CHECK,    // Header read: keep the frame or not
        HDR_OUT,  // Offering the header
        PAYLOAD,  // Passing the payload through
        DISCARD   // Throwing away the rest of the frame
    } state_t;
    state_t state, next_state;

    logic [ETH_HDR_BYTES*8-1:0] header;
    logic [$clog2(ETH_HDR_BYTES)-1:0] byte_counter;
    logic [47:0] dst_mac;
    logic [47:0] src_mac;
    logic [15:0] ethertype;
    logic        accept;

    logic in_beat;
    assign in_beat = buf_out.tvalid && buf_out.tready;

    assign dst_mac   = header[111:64];
    assign src_mac   = header[63:16];
    assign ethertype = header[15:0];
    assign accept    = ethertype == ETHERTYPE &&
                       (promiscuous || dst_mac == local_mac || dst_mac == MAC_BROADCAST);

    always_comb begin
        next_state = state;
        if(~rstn) begin
            next_state = HEADER;
        end else begin
            case(state)
            HEADER: if(in_beat && byte_counter == ETH_HDR_BYTES - 1) begin
                next_state = buf_out.tlast ? HEADER : CHECK; // No payload: drop
            end

            CHECK: begin
                next_state = accept ? HDR_OUT : DISCARD;
            end

            HDR_OUT: if(hdr.ready) begin
                next_state = PAYLOAD;
            end

            PAYLOAD, DISCARD: if(in_beat && buf_out.tlast) begin
                next_state = HEADER;
            end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        state <= next_state;
    end

    // Byte counter - header bytes. A frame that ends inside the header is dropped.
    always_ff @(posedge clk) begin
        if(~rstn || state != HEADER || (in_beat && buf_out.tlast)) begin
            byte_counter <= '0;
        end else if(in_beat) begin
            byte_counter <= byte_counter + 1;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            header <= '0;
        end else if(state == HEADER && in_beat) begin
            header <= {header[ETH_HDR_BYTES*8-9:0], buf_out.tdata};
        end
    end

    // Outputs
    assign hdr.valid   = state == HDR_OUT;
    assign hdr.dst_mac = dst_mac;
    assign hdr.src_mac = src_mac;

    assign axis_out.tvalid = state == PAYLOAD && buf_out.tvalid;
    assign axis_out.tdata  = buf_out.tdata;
    assign axis_out.tlast  = buf_out.tlast;

    assign buf_out.tready = state == HEADER || state == DISCARD ||
                            (state == PAYLOAD && axis_out.tready);

endmodule
