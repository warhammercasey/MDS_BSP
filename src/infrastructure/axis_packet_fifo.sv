// Store-and-forward packet FIFO: a packet only appears on axis_out once all of it is
// in, and then comes out without gaps (as fast as axis_out takes it). in_drop, given
// with the last beat of a packet, throws the packet away instead.
//
// DROP_WHEN_FULL = 0: axis_in.tready goes low while the FIFO is full. A packet longer
//   than the FIFO can never complete, and blocks it for good.
// DROP_WHEN_FULL = 1: axis_in.tready is always high, for sources with no flow control
//   (like rmii_rx). A packet that doesn't fit is dropped.
module axis_packet_fifo #(
    parameter WIDTH          = 8,
    parameter AWIDTH         = 11, // Holds 2^AWIDTH beats
    parameter DROP_WHEN_FULL = 0
)(
    input  logic      clk,
    input  logic      rstn,

    axis_tlast_if.slv axis_in,
    input  logic      in_drop,   // With the last beat: drop this packet

    axis_tlast_if.mst axis_out
);

    localparam logic [AWIDTH:0] DEPTH = 1 << AWIDTH;

    // One bit wider than the address, so full and empty differ
    logic [AWIDTH:0] wr_ptr;      // Next free entry
    logic [AWIDTH:0] commit_ptr;  // End of the last complete packet
    logic [AWIDTH:0] commit_q;    // commit_ptr a cycle later, once the RAM read has caught up
    logic [AWIDTH:0] rd_ptr;
    logic [AWIDTH:0] rd_ptr_next;

    logic full;
    logic dropping;   // This packet didn't fit: throw away the rest of it
    logic in_beat;

    assign full           = wr_ptr - rd_ptr == DEPTH;
    assign axis_in.tready = DROP_WHEN_FULL ? 1'b1 : !full;
    assign in_beat        = axis_in.tvalid && axis_in.tready;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            wr_ptr     <= '0;
            commit_ptr <= '0;
            dropping   <= 1'b0;
        end else if(in_beat) begin
            if(axis_in.tlast) begin
                if(dropping || full || in_drop) begin
                    wr_ptr <= commit_ptr;
                end else begin
                    wr_ptr     <= wr_ptr + 1;
                    commit_ptr <= wr_ptr + 1;
                end
                dropping <= 1'b0;
            end else if(dropping || full) begin
                dropping <= 1'b1; // Only with DROP_WHEN_FULL: otherwise there's no beat while full
            end else begin
                wr_ptr <= wr_ptr + 1;
            end
        end
    end

    // Output: the RAM read is registered, so it is addressed with the pointer for the
    // next cycle. commit_q holds an entry back until that read has seen it.
    always_ff @(posedge clk) begin
        if(~rstn) begin
            commit_q <= '0;
            rd_ptr   <= '0;
        end else begin
            commit_q <= commit_ptr;
            rd_ptr   <= rd_ptr_next;
        end
    end

    assign axis_out.tvalid = commit_q != rd_ptr;
    assign rd_ptr_next     = (axis_out.tvalid && axis_out.tready) ? rd_ptr + 1 : rd_ptr;

    simple_dual_port_bram #(
        .WIDTH(WIDTH + 1),
        .AWIDTH(AWIDTH)
    ) simple_dual_port_bram_i (
        .clka   (clk),
        .wea    (in_beat && !full && !dropping),
        .ena    (1'b1),
        .addra  (wr_ptr[AWIDTH-1:0]),
        .dina   ({axis_in.tlast, axis_in.tdata}),

        .clkb   (clk),
        .enb    (1'b1),
        .addrb  (rd_ptr_next[AWIDTH-1:0]),
        .doutb  ({axis_out.tlast, axis_out.tdata})
    );

endmodule
