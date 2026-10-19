module simple_dual_port_bram #(
    parameter WIDTH  = 8,
    parameter AWIDTH = 4 // Depth is 2^AWIDTH
) (
    input  logic              clka,
    input  logic              wea,
    input  logic              ena,
    input  logic [AWIDTH-1:0] addra,
    input  logic [WIDTH-1:0]  dina,
    
    input  logic              clkb,
    input  logic              enb,
    input  logic [AWIDTH-1:0] addrb,
    output logic [WIDTH-1:0]  doutb
);

    logic [WIDTH-1:0] ram[1 << AWIDTH];

    always_ff @(posedge clka) begin
        if(ena) begin
            if(wea) begin
                ram[addra] <= dina;
            end
        end
    end


    always_ff @(posedge clkb) begin
        if(enb) begin
            doutb <= ram[addrb];
        end
    end

endmodule