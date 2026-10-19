module ff_sync #(
    parameter WIDTH = 8
) (
    input  logic clka,
    input  logic clkb,

    input  logic [WIDTH-1:0] din,
    output logic [WIDTH-1:0] dout
);

    // A multi-bit din must change at most one bit per clka cycle (e.g. Gray code):
    // each bit resolves independently in clkb, so mixed old/new values can appear otherwise.

    logic [WIDTH-1:0] ffa;
    // async_reg (Efinity synthesis attribute) marks the synchronizer flops: they are not
    // merged or duplicated, and place and route keeps them close together.
    (* async_reg = "true" *) logic [WIDTH-1:0] ffb;
    (* async_reg = "true" *) logic [WIDTH-1:0] ffc;

    always_ff @(posedge clka) begin
        ffa <= din;
    end

    always_ff @(posedge clkb) begin
        ffb <= ffa;
        ffc <= ffb;
    end

    assign dout = ffc;

endmodule