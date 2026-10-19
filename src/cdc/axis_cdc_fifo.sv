module axis_cdc_fifo #(
    parameter WIDTH  = 8, // Data width
    parameter AWIDTH = 4 // Address width. FIFO size is 2^AWIDTH
)(
    input  logic clka,
    input  logic rstna,
    
    input  logic clkb,
    input  logic rstnb,

    axis_if.slv axis_in,
    axis_if.mst axis_out
);

    // Pointers cross clock domains as Gray code, so only one bit changes per increment
    // and the receiving side always sees either the old or the new pointer.
    function automatic logic [AWIDTH-1:0] bin2gray(input logic [AWIDTH-1:0] bin);
        return bin ^ (bin >> 1);
    endfunction

    function automatic logic [AWIDTH-1:0] gray2bin(input logic [AWIDTH-1:0] gray);
        logic [AWIDTH-1:0] bin;
        bin[AWIDTH-1] = gray[AWIDTH-1];
        for (int i = AWIDTH - 2; i >= 0; i--)
            bin[i] = bin[i + 1] ^ gray[i];
        return bin;
    endfunction

    logic [AWIDTH-1:0] rd_pointer_a;
    logic [AWIDTH-1:0] rd_pointer_b;
    logic [AWIDTH-1:0] rd_pointer_next_b;
    logic [AWIDTH-1:0] wr_pointer_a;
    logic [AWIDTH-1:0] wr_pointer_b;
    logic [AWIDTH-1:0] rd_gray_a;
    logic [AWIDTH-1:0] wr_gray_b;
    logic [AWIDTH-1:0] count_a;
    logic [AWIDTH-1:0] count_b;

    assign rd_pointer_a = gray2bin(rd_gray_a);
    assign wr_pointer_b = gray2bin(wr_gray_b);

    assign count_a = wr_pointer_a - rd_pointer_a;
    assign count_b = wr_pointer_b - rd_pointer_b;

    assign axis_in.tready = count_a < '1;
    always_ff @(posedge clka) begin
        if(~rstna) begin
            wr_pointer_a <= '0;
        end else begin
            if(axis_in.tvalid && axis_in.tready) begin
                wr_pointer_a <= wr_pointer_a + 1;
            end
        end
    end

    assign axis_out.tvalid = count_b > 0;
    // The RAM read is registered, so it is addressed with the pointer for the next
    // cycle: after a pop, tdata already holds the new head entry.
    assign rd_pointer_next_b = (axis_out.tvalid && axis_out.tready) ? rd_pointer_b + 1 : rd_pointer_b;
    always_ff @(posedge clkb) begin
        if(~rstnb) begin
            rd_pointer_b <= '0;
        end else begin
            rd_pointer_b <= rd_pointer_next_b;
        end
    end

    // ff_sync registers din in its source clock before crossing, so the
    // combinational Gray conversion cannot glitch across the boundary
    ff_sync #(
        .WIDTH(AWIDTH)
    ) ff_sync_rd_pointer (
        .clka(clkb),
        .clkb(clka),

        .din(bin2gray(rd_pointer_b)),
        .dout(rd_gray_a)
    );

    ff_sync #(
        .WIDTH(AWIDTH)
    ) ff_sync_wr_pointer (
        .clka(clka),
        .clkb(clkb),

        .din(bin2gray(wr_pointer_a)),
        .dout(wr_gray_b)
    );


    simple_dual_port_bram #(
        .WIDTH(WIDTH),
        .AWIDTH(AWIDTH)
    ) simple_dual_port_bram_i (
        .clka   (clka),
        .wea    (axis_in.tvalid && axis_in.tready),
        .ena    (1'b1),
        .addra  (wr_pointer_a),
        .dina   (axis_in.tdata),
    
        .clkb   (clkb),
        .enb    (1'b1),
        .addrb  (rd_pointer_next_b),
        .doutb  (axis_out.tdata)
    );


endmodule