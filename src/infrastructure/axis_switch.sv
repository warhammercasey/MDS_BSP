module axis_switch #(
    parameter WIDTH = 8, // Data width
    parameter N     = 2  // Number of input streams
)(
    input  logic clk,
    input  logic rstn,

    axis_if.slv axis_in [N],
    axis_if.mst axis_out
);

    localparam IDX_WIDTH = (N > 1) ? $clog2(N) : 1;

    // Interface arrays can only be indexed by constants, so flatten them into
    // plain arrays that the arbiter can index with the grant
    logic [N-1:0]     in_valid;
    logic [WIDTH-1:0] in_data [N];
    logic [N-1:0]     in_ready;

    generate
        for (genvar i = 0; i < N; i++) begin : g_in
            assign in_valid[i]       = axis_in[i].tvalid;
            assign in_data[i]        = axis_in[i].tdata;
            assign axis_in[i].tready = in_ready[i];
        end
    endgenerate

    // Output register. A new beat can be loaded when the register is empty or
    // its current beat is being consumed this cycle.
    logic             out_valid;
    logic [WIDTH-1:0] out_data;
    logic             load;

    assign load            = ~out_valid || axis_out.tready;
    assign axis_out.tvalid = out_valid;
    assign axis_out.tdata  = out_data;

    // Round-robin arbiter: search starts at the input after the last one granted,
    // so every input with valid data is serviced within N transfers
    logic [IDX_WIDTH-1:0] last_grant;
    logic [IDX_WIDTH-1:0] grant_idx;
    logic                 grant_valid;

    always_comb begin
        int idx;
        grant_valid = 1'b0;
        grant_idx   = '0;
        for (int k = 1; k <= N; k++) begin
            idx = int'(last_grant) + k;
            if (idx >= N)
                idx -= N;
            if (!grant_valid && in_valid[idx]) begin
                grant_valid = 1'b1;
                grant_idx   = IDX_WIDTH'(idx);
            end
        end
    end

    always_comb begin
        in_ready = '0;
        if (load && grant_valid)
            in_ready[grant_idx] = 1'b1;
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            out_valid  <= 1'b0;
            out_data   <= '0;
            last_grant <= '0;
        end else if(load) begin
            out_valid <= grant_valid;
            if(grant_valid) begin
                out_data   <= in_data[grant_idx];
                last_grant <= grant_idx;
            end
        end
    end

endmodule
