// Puts a stream of samples tagged with a channel number (tid) into channel order:
// 0, 1, ..., CHANNELS-1, 0, 1, ...
//
// For sources that send every channel once per cycle, but not in order (i2s sends
// 0 2 4 6 1 3 5 7). One sample per channel is held until its turn to go out. A
// channel's next sample waits (tready low) while its last one is still held, so a
// source must not send a channel twice before the others have come round. tids of
// CHANNELS and up are thrown away.
//
// The output order follows the cycle the source starts on: a source that starts
// halfway through its cycle pairs samples from neighbouring cycles for good.
module axis_tid_reorder #(
    parameter WIDTH    = 8,
    parameter ID_WIDTH = 3,
    parameter CHANNELS = 1 << ID_WIDTH
)(
    input  logic    clk,
    input  logic    rstn,

    axis_tid_if.slv axis_in,
    axis_tid_if.mst axis_out
);

    logic [WIDTH-1:0]    sample [CHANNELS];
    logic [CHANNELS-1:0] held;      // sample[c] is waiting to go out
    logic [ID_WIDTH-1:0] next_id;   // Channel to go out next

    logic in_range, in_beat, out_beat;
    assign in_range = int'(axis_in.tid) < CHANNELS;
    assign in_beat  = axis_in.tvalid && axis_in.tready;
    assign out_beat = axis_out.tvalid && axis_out.tready;

    assign axis_in.tready = !in_range || !held[axis_in.tid];

    // A channel can't be written and read in the same cycle: it is only written while
    // empty, and only read while held
    always_ff @(posedge clk) begin
        if(in_beat && in_range) begin
            sample[axis_in.tid] <= axis_in.tdata;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            held    <= '0;
            next_id <= '0;
        end else begin
            if(in_beat && in_range) begin
                held[axis_in.tid] <= 1'b1;
            end
            if(out_beat) begin
                held[next_id] <= 1'b0;
                next_id       <= next_id == ID_WIDTH'(CHANNELS - 1) ? '0 : next_id + 1;
            end
        end
    end

    assign axis_out.tvalid = held[next_id];
    assign axis_out.tdata  = sample[next_id];
    assign axis_out.tid    = next_id;

endmodule
