interface axis_if #(
    parameter WIDTH = 8
);
    logic [WIDTH-1:0] tdata;
    logic             tvalid;
    logic             tready;

    modport mst (
        output tdata,
        output tvalid,
        input  tready
    );

    modport slv (
        input  tdata,
        input  tvalid,
        output tready
    );
endinterface

interface axis_tid_if #(
    parameter WIDTH = 8,
    parameter ID_WIDTH = 8
);
    logic [WIDTH-1:0] tdata;
    logic             tvalid;
    logic             tready;
    logic [ID_WIDTH-1:0] tid;

    modport mst (
        output tdata,
        output tvalid,
        input  tready,
        output tid
    );

    modport slv (
        input  tdata,
        input  tvalid,
        output tready,
        input  tid
    );
endinterface