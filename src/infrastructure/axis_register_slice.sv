// AXI-Stream register slices, for timing: STAGES pipeline registers between axis_in and
// axis_out. Every stage runs at a beat per clock, so it only adds latency (STAGES clocks).
//
// REGISTER_READY = 1: each stage is a skid buffer (a main register plus a spare). tdata,
//                     tvalid and tready are all registered, so no path goes through a stage,
//                     at the cost of twice the flip flops.
// REGISTER_READY = 0: each stage registers tdata and tvalid only. tready passes back through
//                     an OR per stage, which is fine for a few stages.
// STAGES = 0 connects straight through.
//
// axis_register_slice_core works on a flat vector; the wrappers below pack each interface
// type into it:
//   axis_register_slice            axis_if
//   axis_tid_register_slice        axis_tid_if
//   axis_tlast_register_slice      axis_tlast_if
//   axis_tid_tlast_register_slice  axis_tid_tlast_if

module axis_register_slice_core #(
    parameter WIDTH          = 8,
    parameter STAGES         = 1,
    parameter REGISTER_READY = 1
)(
    input  logic             clk,
    input  logic             rstn,

    input  logic [WIDTH-1:0] in_data,
    input  logic             in_valid,
    output logic             in_ready,

    output logic [WIDTH-1:0] out_data,
    output logic             out_valid,
    input  logic             out_ready
);

    // Stage k takes from [k] and gives to [k+1]
    logic [WIDTH-1:0] data  [STAGES+1];
    logic             valid [STAGES+1];
    logic             ready [STAGES+1];

    assign data[0]  = in_data;
    assign valid[0] = in_valid;
    assign in_ready = ready[0];

    assign out_data       = data[STAGES];
    assign out_valid      = valid[STAGES];
    assign ready[STAGES]  = out_ready;

    for(genvar k = 0; k < STAGES; k++) begin : g_stage
        if(REGISTER_READY) begin : g_skid
            // Main register drives the output; the spare catches the beat accepted in the
            // clock the output stalled, since tready is a clock late
            logic [WIDTH-1:0] main_data, spare_data;
            logic             main_valid, spare_valid;

            assign data[k+1]  = main_data;
            assign valid[k+1] = main_valid;
            assign ready[k]   = ~spare_valid;

            always_ff @(posedge clk) begin
                if(~rstn) begin
                    main_valid  <= 1'b0;
                    spare_valid <= 1'b0;
                end else begin
                    if(ready[k+1] || ~main_valid) begin
                        // Output moving (or empty): refill from the spare first
                        if(spare_valid) begin
                            main_valid  <= 1'b1;
                            spare_valid <= 1'b0;
                        end else begin
                            main_valid  <= valid[k];
                        end
                    end else if(valid[k] && ready[k]) begin
                        spare_valid <= 1'b1;
                    end
                end
            end

            always_ff @(posedge clk) begin
                if(ready[k+1] || ~main_valid) begin
                    if(spare_valid) begin
                        main_data <= spare_data;
                    end else if(valid[k]) begin
                        main_data <= data[k];
                    end
                end else if(valid[k] && ready[k]) begin
                    spare_data <= data[k];
                end
            end

        end else begin : g_fwd
            logic [WIDTH-1:0] stage_data;
            logic             stage_valid;

            assign data[k+1]  = stage_data;
            assign valid[k+1] = stage_valid;
            assign ready[k]   = ready[k+1] || ~stage_valid;

            always_ff @(posedge clk) begin
                if(~rstn) begin
                    stage_valid <= 1'b0;
                end else if(ready[k]) begin
                    stage_valid <= valid[k];
                end
            end

            always_ff @(posedge clk) begin
                if(ready[k] && valid[k]) begin
                    stage_data <= data[k];
                end
            end
        end
    end

endmodule


module axis_register_slice #(
    parameter WIDTH          = 8,
    parameter STAGES         = 1,
    parameter REGISTER_READY = 1
)(
    input  logic clk,
    input  logic rstn,

    axis_if.slv axis_in,
    axis_if.mst axis_out
);

    axis_register_slice_core #(
        .WIDTH(WIDTH),
        .STAGES(STAGES),
        .REGISTER_READY(REGISTER_READY)
    ) core_i (
        .clk(clk),
        .rstn(rstn),

        .in_data(axis_in.tdata),
        .in_valid(axis_in.tvalid),
        .in_ready(axis_in.tready),

        .out_data(axis_out.tdata),
        .out_valid(axis_out.tvalid),
        .out_ready(axis_out.tready)
    );

endmodule


module axis_tid_register_slice #(
    parameter WIDTH          = 8,
    parameter ID_WIDTH       = 8,
    parameter STAGES         = 1,
    parameter REGISTER_READY = 1
)(
    input  logic clk,
    input  logic rstn,

    axis_tid_if.slv axis_in,
    axis_tid_if.mst axis_out
);

    logic [ID_WIDTH+WIDTH-1:0] in_flat, out_flat;

    assign in_flat         = {axis_in.tid, axis_in.tdata};
    assign axis_out.tdata  = out_flat[0 +: WIDTH];
    assign axis_out.tid    = out_flat[WIDTH +: ID_WIDTH];

    axis_register_slice_core #(
        .WIDTH(ID_WIDTH + WIDTH),
        .STAGES(STAGES),
        .REGISTER_READY(REGISTER_READY)
    ) core_i (
        .clk(clk),
        .rstn(rstn),

        .in_data(in_flat),
        .in_valid(axis_in.tvalid),
        .in_ready(axis_in.tready),

        .out_data(out_flat),
        .out_valid(axis_out.tvalid),
        .out_ready(axis_out.tready)
    );

endmodule


module axis_tlast_register_slice #(
    parameter WIDTH          = 8,
    parameter STAGES         = 1,
    parameter REGISTER_READY = 1
)(
    input  logic clk,
    input  logic rstn,

    axis_tlast_if.slv axis_in,
    axis_tlast_if.mst axis_out
);

    logic [WIDTH:0] in_flat, out_flat;

    assign in_flat         = {axis_in.tlast, axis_in.tdata};
    assign axis_out.tdata  = out_flat[0 +: WIDTH];
    assign axis_out.tlast  = out_flat[WIDTH];

    axis_register_slice_core #(
        .WIDTH(1 + WIDTH),
        .STAGES(STAGES),
        .REGISTER_READY(REGISTER_READY)
    ) core_i (
        .clk(clk),
        .rstn(rstn),

        .in_data(in_flat),
        .in_valid(axis_in.tvalid),
        .in_ready(axis_in.tready),

        .out_data(out_flat),
        .out_valid(axis_out.tvalid),
        .out_ready(axis_out.tready)
    );

endmodule


module axis_tid_tlast_register_slice #(
    parameter WIDTH          = 8,
    parameter ID_WIDTH       = 8,
    parameter STAGES         = 1,
    parameter REGISTER_READY = 1
)(
    input  logic clk,
    input  logic rstn,

    axis_tid_tlast_if.slv axis_in,
    axis_tid_tlast_if.mst axis_out
);

    logic [ID_WIDTH+WIDTH:0] in_flat, out_flat;

    assign in_flat         = {axis_in.tlast, axis_in.tid, axis_in.tdata};
    assign axis_out.tdata  = out_flat[0 +: WIDTH];
    assign axis_out.tid    = out_flat[WIDTH +: ID_WIDTH];
    assign axis_out.tlast  = out_flat[ID_WIDTH + WIDTH];

    axis_register_slice_core #(
        .WIDTH(1 + ID_WIDTH + WIDTH),
        .STAGES(STAGES),
        .REGISTER_READY(REGISTER_READY)
    ) core_i (
        .clk(clk),
        .rstn(rstn),

        .in_data(in_flat),
        .in_valid(axis_in.tvalid),
        .in_ready(axis_in.tready),

        .out_data(out_flat),
        .out_valid(axis_out.tvalid),
        .out_ready(axis_out.tready)
    );

endmodule
