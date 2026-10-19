interface apb_if #(
    parameter ADDR_WIDTH = 1,
    parameter DATA_WIDTH = 8,
    parameter SEL_WIDTH  = 1
);

    logic [ADDR_WIDTH-1:0] paddr;
    logic                  psel[SEL_WIDTH];
    logic                  penable;
    logic                  pwrite;
    logic [DATA_WIDTH-1:0] pwdata;
    logic                  pready;
    logic [DATA_WIDTH-1:0] prdata;

    modport mst (
        output paddr,
        output psel,
        output penable,
        output pwrite,
        output pwdata,
        input  pready,
        input  prdata
    );
    

    modport slv (
        input  paddr,
        input  psel,
        input  penable,
        input  pwrite,
        input  pwdata,
        output pready,
        output prdata
    );

endinterface