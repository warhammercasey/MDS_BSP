module reset_sync #(
    parameter NUM_CLKS = 4    
)( // Synchronizes an asynchronous reset signal to each clock domain
    input  logic clk[NUM_CLKS],
    input  logic rst_in_n,
    input  logic clk_locked,
    output logic rstn[NUM_CLKS]
);

    genvar i;
    generate
        for(i = 0; i < NUM_CLKS; i++) begin : gen_rst_sync
            ff_sync #(
                .WIDTH(1)
            ) ff_sync_rstn (
                .clka(clk[i]),
                .clkb(clk[i]),
                .din(rst_in_n && clk_locked),
                .dout(rstn[i])
            );
        end
    endgenerate

endmodule