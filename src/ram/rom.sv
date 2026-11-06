module rom #(
    parameter WIDTH     = 8,
    parameter AWIDTH    = 4,  // Depth is 2^AWIDTH
    parameter INIT_FILE = "", // One word per line, no comments or prefixes
    parameter INIT_HEX  = 1   // 1: INIT_FILE is hex ($readmemh), 0: binary ($readmemb)
) (
    input  logic              clk,
    input  logic              en,
    input  logic [AWIDTH-1:0] addr,
    output logic [WIDTH-1:0]  dout
);

    (* syn_romstyle = "block_rom" *) logic [WIDTH-1:0] mem [1 << AWIDTH];

    initial begin
        if(INIT_HEX) begin
            $readmemh(INIT_FILE, mem);
        end else begin
            $readmemb(INIT_FILE, mem);
        end
    end

    always @(posedge clk) begin
        if(en) begin
            dout <= mem[addr];
        end
    end

endmodule
