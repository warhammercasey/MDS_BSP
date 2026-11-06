module dual_port_rom #(
    parameter WIDTH     = 8,
    parameter AWIDTH    = 4,  // Depth is 2^AWIDTH
    parameter INIT_FILE = "", // One word per line, no comments or prefixes
    parameter INIT_HEX  = 1   // 1: INIT_FILE is hex ($readmemh), 0: binary ($readmemb)
) (
    input  logic              clka,
    input  logic              ena,
    input  logic [AWIDTH-1:0] addra,
    output logic [WIDTH-1:0]  douta,

    input  logic              clkb,
    input  logic              enb,
    input  logic [AWIDTH-1:0] addrb,
    output logic [WIDTH-1:0]  doutb
);

    (* syn_romstyle = "block_rom" *) logic [WIDTH-1:0] mem [1 << AWIDTH];

    initial begin
        if(INIT_HEX) begin
            $readmemh(INIT_FILE, mem);
        end else begin
            $readmemb(INIT_FILE, mem);
        end
    end

    always @(posedge clka) begin
        if(ena) begin
            douta <= mem[addra];
        end
    end

    always @(posedge clkb) begin
        if(enb) begin
            doutb <= mem[addrb];
        end
    end

endmodule
