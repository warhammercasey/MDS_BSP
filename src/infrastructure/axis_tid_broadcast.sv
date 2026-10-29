// Copies one stream to N outputs. A beat goes to all outputs at once, when all of them
// are ready, so the slowest output sets the pace. Combinational: no added latency.
//
// Each output's tvalid waits on the other outputs' tready, so an output's tready must
// not depend on its tvalid (true of anything that follows the AXI-stream rules).
module axis_tid_broadcast #(
    parameter N = 2
)(
    axis_tid_if.slv axis_in,
    axis_tid_if.mst axis_out [N]
);

    logic [N-1:0] out_ready;

    assign axis_in.tready = &out_ready;

    generate
        for (genvar i = 0; i < N; i++) begin : g_out
            logic [N-1:0] others_ready;
            always_comb begin
                others_ready    = out_ready;
                others_ready[i] = 1'b1;
            end

            assign out_ready[i]       = axis_out[i].tready;
            assign axis_out[i].tvalid = axis_in.tvalid && &others_ready;
            assign axis_out[i].tdata  = axis_in.tdata;
            assign axis_out[i].tid    = axis_in.tid;
        end
    endgenerate

endmodule
