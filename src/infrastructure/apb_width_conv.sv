// Connects APB buses of different address and data widths. Purely combinational.
//
// Addresses and write data are zero padded when the downstream bus is wider and
// truncated (MSBs dropped) when it is narrower; read data the same way upstream.
//
// A transfer whose address, or write data, has non-zero bits that truncation would
// drop is not passed downstream: psel stays low on apb_out and the converter ends
// the transfer itself with pready and pslverr in the first access cycle. A read whose
// data has non-zero bits that don't fit upstream also ends with pslverr (the read has
// already happened downstream by then).
module apb_width_conv #(
    parameter IN_ADDR_WIDTH  = 8,
    parameter IN_DATA_WIDTH  = 8,
    parameter OUT_ADDR_WIDTH = 8,
    parameter OUT_DATA_WIDTH = 8,
    parameter SEL_WIDTH      = 1  // psel lines, the same on both sides
)(
    apb_if.slv apb_in,   // ADDR_WIDTH = IN_ADDR_WIDTH,  DATA_WIDTH = IN_DATA_WIDTH
    apb_if.mst apb_out   // ADDR_WIDTH = OUT_ADDR_WIDTH, DATA_WIDTH = OUT_DATA_WIDTH
);

    logic addr_ok;   // Address fits downstream
    logic wdata_ok;  // Write data fits downstream
    logic rdata_ok;  // Read data fits upstream

    generate
        if (OUT_ADDR_WIDTH >= IN_ADDR_WIDTH) begin : g_addr_pad
            assign apb_out.paddr = OUT_ADDR_WIDTH'(apb_in.paddr);
            assign addr_ok       = 1'b1;
        end else begin : g_addr_trunc
            assign apb_out.paddr = apb_in.paddr[OUT_ADDR_WIDTH-1:0];
            assign addr_ok       = apb_in.paddr[IN_ADDR_WIDTH-1:OUT_ADDR_WIDTH] == '0;
        end

        if (OUT_DATA_WIDTH >= IN_DATA_WIDTH) begin : g_wdata_pad
            assign apb_out.pwdata = OUT_DATA_WIDTH'(apb_in.pwdata);
            assign wdata_ok       = 1'b1;
        end else begin : g_wdata_trunc
            assign apb_out.pwdata = apb_in.pwdata[OUT_DATA_WIDTH-1:0];
            assign wdata_ok       = apb_in.pwdata[IN_DATA_WIDTH-1:OUT_DATA_WIDTH] == '0;
        end

        if (IN_DATA_WIDTH >= OUT_DATA_WIDTH) begin : g_rdata_pad
            assign apb_in.prdata = IN_DATA_WIDTH'(apb_out.prdata);
            assign rdata_ok      = 1'b1;
        end else begin : g_rdata_trunc
            assign apb_in.prdata = apb_out.prdata[IN_DATA_WIDTH-1:0];
            assign rdata_ok      = apb_out.prdata[OUT_DATA_WIDTH-1:IN_DATA_WIDTH] == '0;
        end
    endgenerate

    // paddr/pwrite/pwdata are held for the whole transfer, so this is too
    logic req_ok;
    assign req_ok = addr_ok && (!apb_in.pwrite || wdata_ok);

    logic sel_any;
    always_comb begin
        sel_any = 1'b0;
        for (int i = 0; i < SEL_WIDTH; i++)
            sel_any = sel_any | apb_in.psel[i];
    end

    generate
        for (genvar i = 0; i < SEL_WIDTH; i++) begin : g_sel
            assign apb_out.psel[i] = apb_in.psel[i] && req_ok;
        end
    endgenerate
    assign apb_out.penable = apb_in.penable && req_ok;
    assign apb_out.pwrite  = apb_in.pwrite;

    assign apb_in.pready  = req_ok ? apb_out.pready : sel_any && apb_in.penable;
    assign apb_in.pslverr = req_ok ? apb_out.pslverr || (!apb_in.pwrite && !rdata_ok) : 1'b1;

endmodule
