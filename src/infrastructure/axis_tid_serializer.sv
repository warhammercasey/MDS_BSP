// Splits each wide beat into N narrow ones, tagged by tid: the top WIDTH bits go out
// first with tid 0, the bottom WIDTH bits last with tid N-1. E.g. a {x, y, z} vector
// becomes 3 channels of a multichannel stream.
//
// Takes a new beat while the last piece of the previous one goes out, so it can run a
// piece every clock.
module axis_tid_serializer #(
    parameter WIDTH    = 16, // Output width
    parameter N        = 3,  // Pieces per input beat
    parameter ID_WIDTH = 2
)(
    input  logic clk,
    input  logic rstn,

    axis_if.slv     axis_in,  // WIDTH*N bits
    axis_tid_if.mst axis_out  // WIDTH bits
);

    logic [WIDTH*N-1:0]  word;
    logic [ID_WIDTH-1:0] index;
    logic                full;
    logic                last_piece;

    assign last_piece      = index == ID_WIDTH'(N-1);
    assign axis_in.tready  = !full || (axis_out.tready && last_piece);
    assign axis_out.tvalid = full;
    assign axis_out.tdata  = word[(N-1-index)*WIDTH +: WIDTH];
    assign axis_out.tid    = index;

    always_ff @(posedge clk) begin
        if(~rstn) begin
            full  <= 1'b0;
            index <= '0;
        end else begin
            if(axis_in.tvalid && axis_in.tready) begin
                word  <= axis_in.tdata;
                full  <= 1'b1;
                index <= '0;
            end else if(axis_out.tvalid && axis_out.tready) begin
                if(last_piece) begin
                    full <= 1'b0;
                end else begin
                    index <= index + 1'b1;
                end
            end
        end
    end

endmodule
