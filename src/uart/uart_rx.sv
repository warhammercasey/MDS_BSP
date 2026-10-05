module uart_rx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
)(
    input  logic clk,
    input  logic rstn,
    input  logic rx,
    axis_if.mst axis_rx
);

    localparam CLK_PER_BIT = CLK_FREQ/BAUD;
    localparam CLK_PER_BIT_2 = CLK_PER_BIT/2;

    // Start module on falling edge of the rx line
    logic enable, done;
    always_ff @(posedge clk) begin
        if (!rstn) begin
            enable <= 1'b0;
        end else begin
            if(done) begin
                enable <= 1'b0;
            end else if(~rx && $past(rx)) begin
                enable <= 1'b1;
            end
        end
    end

    // Generate a pulse for whenever a bit needs to be sampled
    logic bit_latch;
    logic [$clog2(CLK_PER_BIT)-1:0] baud_counter;
    logic [$clog2(9+1)-1:0] bit_counter;

    assign bit_latch = (bit_counter == '0) ? (baud_counter == CLK_PER_BIT_2 - 1) : (baud_counter == CLK_PER_BIT - 1);
    assign done = bit_counter == 9 && baud_counter == CLK_PER_BIT - 1;
    always_ff @(posedge clk) begin
        if(!enable) begin
            baud_counter <= '0;
            bit_counter <= '0;

        end else begin
            if(bit_latch) begin
                baud_counter <= '0;
                bit_counter  <= bit_counter + 1;
            end else begin
                baud_counter <= baud_counter + 1;
            end
        end
    end

    
    // Shift data into rx register
    logic [7:0] rx_data;
    always_ff @(posedge clk) begin
        if(!enable) begin
            rx_data <= '0;
        end else begin
            if(bit_latch) begin
                rx_data <= {rx, rx_data[7:1]};
            end
        end
    end

    // Output the received data when done
    always_ff @(posedge clk) begin
        if(!rstn) begin
            axis_rx.tdata <= '0;
            axis_rx.tvalid <= 1'b0;
        end else begin
            if(done) begin
                axis_rx.tdata <= rx_data;
                axis_rx.tvalid <= 1'b1;
            end else if(axis_rx.tready || enable) begin
                axis_rx.tvalid <= 1'b0;
            end
        end
    end

endmodule