// Single-multiplier multichannel complex FIR decimator
// Each TID is filtered independantly
module fir_decimator #(
    parameter WIDTH = 18,
    parameter TAPS = 32,
    parameter DECIMATION = 4,
    parameter TID_WIDTH = $clog2(8),
    parameter INIT_FILE = "",
    parameter INIT_HEX = 1
) (
    input  logic clk,
    input  logic rstn,

    axis_tid_if.slv din,
    axis_tid_if.mst dout
);

    localparam CHANNELS = (1 << TID_WIDTH);
    localparam BUFFER_AWIDTH = $clog2(TAPS*CHANNELS);

    function automatic int[CHANNELS] init_buffer_address();
        int res[CHANNELS];
        for(int i = 0; i < CHANNELS; i++) begin
            res[i] = i*TAPS;
        end
    endfunction

    localparam BUFFER_ADDRS = init_buffer_address();

    logic [BUFFER_AWIDTH-1:0] input_offsets[CHANNELS];

    // Buffer up samples in input BRAM
    simple_dual_port_bram #(
        .WIDTH(WIDTH*2), // *2 for complex samples
        .AWIDTH(BUFFER_AWIDTH)
    ) input_buffer (
        .clka(clk),
        .wea(din.tvalid && din.tready),
        .ena(1'b1),
        .addra(BUFFER_ADDRS[din.tid] + input_offsets[din.tid]),
        .dina(din.tdata),
    
        .clkb(clk),
        .enb(1'b1),
        .addrb(),
        .doutb()
    );

    // Increment input offsets as data comes in
    always_ff @(posedge clk) begin
        if(~rstn) begin
            input_offsets <= '{default: '0};

        end else begin
            if(din.tvalid && din.tready) begin
                if(input_offsets[din.tid] == TAPS - 1) begin
                    input_offsets[din.tid] <= '0;
                end else begin
                    input_offsets[din.tid] <= input_offsets[din.tid] + 1;
                end
            end
        end
    end



endmodule