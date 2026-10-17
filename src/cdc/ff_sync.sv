module ff_sync #(
    parameter WIDTH = 8
) (
    input  logic clka,
    input  logic clkb,

    input  logic [WIDTH-1:0] din,
    output logic [WIDTH-1:0] dout
);

    logic [WIDTH-1:0] ff;

    always_ff @(posedge clka) begin
        ff <= din;
    end
    
    always_ff @(posedge clkb) begin
        dout <= ff;
    end

endmodule