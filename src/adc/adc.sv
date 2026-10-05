module adc #(
    parameter CLK_FREQ = 50_000_000,
    parameter SAMPLE_RATE = 100_000,
    parameter ADC_BITS = 8 // Design was made around 12-bit adc, but only 8-bit was in stock
)(
    input  logic clk,
    input  logic rstn,

    // ADC SPI interface
    output logic adc_spi_sck,
    output logic adc_spi_mosi,
    input  logic adc_spi_miso,
    output logic adc_spi_cs_n,

    axis_tid_if.mst adc_axis
);

    // ADC uses 16 clocks per sample
    localparam SCK_PERIOD = (CLK_FREQ + SAMPLE_RATE * 16 - 1) / (SAMPLE_RATE * 16); // Period of SCK in clock cycles
    localparam DOUT_START_BIT = 4;
    localparam DOUT_END_BIT = DOUT_START_BIT + ADC_BITS - 1;

    // Activate ADC as soon as out of reset
    assign adc_spi_cs_n = ~rstn;

    // Generate SPI clock
    // All module timing should be done based on adc_spi_sck
    logic [$clog2(SCK_PERIOD)-1:0] sck_counter;
    logic sck_count_hit;
    assign sck_count_hit = (sck_counter == SCK_PERIOD - 1);
    always_ff @(posedge clk) begin
        if(~rstn) begin
            sck_counter <= '0;
            adc_spi_sck <= 1'b1;

        end else begin
            if(sck_count_hit) begin
                sck_counter <= '0;
            end else begin
                sck_counter <= sck_counter + 1;
            end

            adc_spi_sck <= sck_counter < (SCK_PERIOD / 2);
        end
    end

    
    // Track current bit in SCK sequence
    logic [$clog2(16)-1:0] sck_bit_counter;
    logic [$clog2(16)-1:0] sck_bit_counter_fe; // Bit counter delayed to change on the falling edge of sck
    always_ff @(posedge clk) begin
        if(~rstn) begin
            sck_bit_counter <= '0;
            sck_bit_counter_fe <= '0;

        end else begin
            // Counter automatically overflows at 16
            if(sck_count_hit) begin // Increment bit counter on sck count hit
                sck_bit_counter <= sck_bit_counter + 1;
            end

            if(~adc_spi_sck && $past(adc_spi_sck)) begin // Update the falling edge bit counter on the falling edge of sck
                sck_bit_counter_fe <= sck_bit_counter;
            end
        end
    end


    // Track which channel is being converted
    logic conversion_done;
    logic [$clog2(8)-1:0] current_channel; // Channel currently being received this frame
    logic [$clog2(8)-1:0] next_channel; // Channel to be received in the next 
    assign conversion_done = (sck_bit_counter == 15) && sck_count_hit;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            current_channel <= '0;
            next_channel <= 1;

        end else begin
            if(conversion_done) begin
                current_channel <= next_channel;
                next_channel <= next_channel + 1; // Counter automatically overflows at 8 channels
            end
        end
    end


    // Shift out address
    // Address is latched on first 8 rising edges of sck
    // Bits 7, 6, 2, 1, 0 are ignored. Bits 5, 4, 3, are ADD2, ADD1, ADD0 respectively
    logic [7:0] full_address;
    assign full_address = {<<{2'b00, next_channel, 3'b000}};
    always_ff @(posedge clk) begin
        if(~rstn) begin
            adc_spi_mosi <= 1'b0;

        end else begin
            adc_spi_mosi <= full_address[sck_bit_counter_fe[$clog2(8)-1:0]];

        end
    end


    // Shift in data
    // Data is latched on rising edge of bits 4-11
    logic [ADC_BITS-1:0] adc_data;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            adc_data <= '0;
        end else begin
            if(sck_count_hit && sck_bit_counter_fe >= DOUT_START_BIT && sck_bit_counter_fe <= DOUT_END_BIT) begin
                adc_data <= {adc_data[ADC_BITS-2:0], adc_spi_miso};
            end
        end
    end


    // Write output data to AXI stream
    always_ff @(posedge clk) begin
        if(~rstn) begin
            adc_axis.tvalid <= 1'b0;
            adc_axis.tdata <= '0;
            adc_axis.tid <= '0;

        end else begin
            if(conversion_done) begin
                adc_axis.tvalid <= 1'b1;
                adc_axis.tdata <= adc_data;
                adc_axis.tid <= current_channel;

            end else if(adc_axis.tready) begin
                adc_axis.tvalid <= 1'b0;

            end
        end
    end


endmodule