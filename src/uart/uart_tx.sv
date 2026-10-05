module uart_tx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
)(
    input  logic clk,
    input  logic rstn,
    output logic tx,
    axis_if.slv axis_tx
);

    localparam CLK_PER_BIT = CLK_FREQ/BAUD;

    logic [7+2:0] tx_data_latched;
    logic latched_valid;
    logic done;

    // Latch incoming data on valid
    assign axis_tx.tready = ~latched_valid;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            tx_data_latched <= '0;
            latched_valid <= 1'b0;

        end else begin
            if(axis_tx.tvalid && axis_tx.tready) begin
                tx_data_latched <= {1'b1, axis_tx.tdata, 1'b0}; // Include start/stop bits
                latched_valid <= 1'b1;

            end else if(done) begin
                latched_valid <= 1'b0;
            end
        end
    end


    // Shift out data
    logic [$clog2(8+2)-1:0] bit_count;
    logic [$clog2(CLK_PER_BIT)-1:0] baud_counter;
    
    assign done = (bit_count == 9) && (baud_counter == CLK_PER_BIT-1);
    assign tx = ~latched_valid || tx_data_latched[bit_count];

    always_ff @(posedge clk) begin
        if(~latched_valid) begin
            bit_count <= '0;
            baud_counter <= '0;

        end else begin
            if(baud_counter == CLK_PER_BIT-1) begin
                baud_counter <= '0;
                bit_count <= bit_count + 1;
                
            end else begin
                baud_counter <= baud_counter + 1;
            end
        end
    end

endmodule