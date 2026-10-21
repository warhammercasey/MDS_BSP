module two_port_bram #(
    parameter WIDTH  = 8,
    parameter AWIDTH = 4 // Depth is 2^AWIDTH
)(
    input  logic              clka,
    input  logic              wea,
    input  logic              ena,
    input  logic [AWIDTH-1:0] addra,
    input  logic [WIDTH-1:0]  dina,
    output logic [WIDTH-1:0]  douta,
    
    input  logic              clkb,
    input  logic              web,
    input  logic              enb,
    input  logic [AWIDTH-1:0] addrb,
    input  logic [WIDTH-1:0]  dinb,
    output logic [WIDTH-1:0]  doutb
);

    logic [WIDTH-1:0] ram [1 << AWIDTH];

    always @(posedge clka) begin
        if(ena) begin
            if(wea) begin
                ram[addra] <= dina;
            end

            douta <= ram[addra];
        end
    end
    
    always @(posedge clkb) begin
        if(enb) begin
            if(web) begin
                ram[addrb] <= dinb;
            end

            doutb <= ram[addrb];
        end
    end

endmodule