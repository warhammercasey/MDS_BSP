// Connects one APB master to N peripherals. The master drives one psel line per
// peripheral; each peripheral gets its own single-select bus, and the selected
// peripheral's response (pready, prdata, pslverr) is muxed back to the master.
module apb_splitter #(
    parameter DATA_WIDTH = 8,
    parameter N          = 2  // Number of peripherals
)(
    apb_if.slv apb_in,        // SEL_WIDTH = N
    apb_if.mst apb_out [N]    // SEL_WIDTH = 1
);

    // Interface arrays can only be indexed by constants, so flatten the responses
    logic [N-1:0]          sel;
    logic [N-1:0]          ready;
    logic [N-1:0]          slverr;
    logic [DATA_WIDTH-1:0] rdata [N];

    generate
        for (genvar i = 0; i < N; i++) begin : g_out
            assign apb_out[i].paddr   = apb_in.paddr;
            assign apb_out[i].psel[0] = apb_in.psel[i];
            assign apb_out[i].penable = apb_in.penable;
            assign apb_out[i].pwrite  = apb_in.pwrite;
            assign apb_out[i].pwdata  = apb_in.pwdata;

            assign sel[i]    = apb_in.psel[i];
            assign ready[i]  = apb_out[i].pready;
            assign slverr[i] = apb_out[i].pslverr;
            assign rdata[i]  = apb_out[i].prdata;
        end
    endgenerate

    // psel is one-hot, so an AND-OR mux is enough
    logic [DATA_WIDTH-1:0] rdata_mux;
    always_comb begin
        rdata_mux = '0;
        for (int i = 0; i < N; i++)
            rdata_mux = rdata_mux | (rdata[i] & {DATA_WIDTH{sel[i]}});
    end

    assign apb_in.pready  = |(sel & ready);
    assign apb_in.pslverr = |(sel & slverr);
    assign apb_in.prdata  = rdata_mux;

endmodule
