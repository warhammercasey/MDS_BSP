// Packet switch: merges N packet streams onto one output, a whole packet at a time.
//
// Each input has its own store-and-forward buffer (axis_packet_fifo), and an input is
// only picked once a whole packet is waiting in it. So a packet comes out without
// gaps, as fast as axis_out takes it, and is never cut into by another input. Inputs
// with a packet waiting take turns (round robin).
//
//   out_sel: the input the packet on axis_out came from. Valid while axis_out.tvalid.
//
// Each buffer holds 2^AWIDTH beats. It must hold a whole packet, or the input blocks for
// good (see axis_packet_fifo), plus whatever the input sends while it waits for its
// turn. AWIDTHS can size inputs separately: byte i is input i's AWIDTH, 0 for the
// default. E.g. N = 3, AWIDTHS = 24'h05_00_00 gives input 2 a 32 beat buffer.
module axis_packet_switch #(
    parameter WIDTH  = 8,
    parameter N      = 2,
    parameter AWIDTH = 11,
    parameter logic [8*N-1:0] AWIDTHS = '0
)(
    input  logic clk,
    input  logic rstn,

    axis_tlast_if.slv axis_in [N],
    axis_tlast_if.mst axis_out,
    output logic [(N > 1 ? $clog2(N) : 1)-1:0] out_sel
);

    localparam IDX_WIDTH = (N > 1) ? $clog2(N) : 1;

    // ------------------------------------------------------------------
    // Input buffers
    //   Interface arrays can only be indexed by constants, so their outputs are
    //   flattened into plain arrays that can be indexed by the selected input
    // ------------------------------------------------------------------
    axis_tlast_if #(.WIDTH(WIDTH)) buf_out [N] ();

    logic [N-1:0]     buf_valid;
    logic [WIDTH-1:0] buf_data [N];
    logic [N-1:0]     buf_last;
    logic [N-1:0]     buf_ready;

    generate
        for (genvar i = 0; i < N; i++) begin : g_in
            localparam IN_AWIDTH = AWIDTHS[8*i +: 8] != 0 ? AWIDTHS[8*i +: 8] : AWIDTH;
            axis_packet_fifo #(
                .WIDTH(WIDTH),
                .AWIDTH(IN_AWIDTH),
                .DROP_WHEN_FULL(0)
            ) packet_buffer (
                .clk(clk),
                .rstn(rstn),
                .axis_in(axis_in[i]),
                .in_drop(1'b0),
                .axis_out(buf_out[i])
            );

            assign buf_valid[i]      = buf_out[i].tvalid;
            assign buf_data[i]       = buf_out[i].tdata;
            assign buf_last[i]       = buf_out[i].tlast;
            assign buf_out[i].tready = buf_ready[i];
        end
    endgenerate


    // ------------------------------------------------------------------
    // Arbiter
    //   Between packets, picks the next input with a packet waiting, starting from the
    //   one after the last picked. The pick holds until that packet's tlast.
    // ------------------------------------------------------------------
    logic                 active;  // A packet is going out from input sel
    logic [IDX_WIDTH-1:0] sel;
    logic [IDX_WIDTH-1:0] grant_idx;
    logic                 grant_valid;

    always_comb begin
        int idx;
        grant_valid = 1'b0;
        grant_idx   = '0;
        for (int k = 1; k <= N; k++) begin
            idx = int'(sel) + k;
            if (idx >= N)
                idx -= N;
            if (!grant_valid && buf_valid[idx]) begin
                grant_valid = 1'b1;
                grant_idx   = IDX_WIDTH'(idx);
            end
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            active <= 1'b0;
            sel    <= '0;
        end else if(!active) begin
            if(grant_valid) begin
                active <= 1'b1;
                sel    <= grant_idx;
            end
        end else if(axis_out.tvalid && axis_out.tready && axis_out.tlast) begin
            active <= 1'b0;
        end
    end


    // Output
    assign axis_out.tvalid = active && buf_valid[sel];
    assign axis_out.tdata  = buf_data[sel];
    assign axis_out.tlast  = buf_last[sel];
    assign out_sel         = sel;

    always_comb begin
        buf_ready = '0;
        if (active)
            buf_ready[sel] = axis_out.tready;
    end

endmodule
