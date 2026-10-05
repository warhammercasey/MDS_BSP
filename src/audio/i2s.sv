module i2s #(
    parameter CLK_FREQ = 50_000_000,
    parameter I2S_CLK = 4_096_000, // 4.096 MHz
    parameter I2S_BITS = 18 // data format is 24-bit but only 18 bits are used
) (
    input  logic       clk,
    input  logic       rstn,

    // 8 microphones, 2 per data line. Share sck/ws
    output logic       i2s_sck,
    output logic       i2s_ws,
    input  logic [3:0] i2s_data,

    axis_tid_if.mst    axis_i2s
);
    localparam I2S_CLK_PERIOD = (CLK_FREQ + I2S_CLK - 1) / I2S_CLK;
    localparam I2S_CLK_PERIOD_2 = (I2S_CLK_PERIOD + 1) / 2;

    // Generate i2s clk
    logic [$clog2(I2S_CLK_PERIOD_2)-1:0] i2s_clk_counter;
    logic clk_count_reached;
    assign clk_count_reached = (i2s_clk_counter == I2S_CLK_PERIOD_2-1);
    always_ff @(posedge clk) begin
        if(~rstn) begin
            i2s_clk_counter <= '0;
            i2s_sck <= 1'b0;

        end else begin
            if(clk_count_reached) begin
                i2s_clk_counter <= '0;
                i2s_sck <= ~i2s_sck;
            end else begin
                i2s_clk_counter <= i2s_clk_counter + 1;
            end

        end
    end


    logic re_trigger, fe_trigger; // Triggers to activate logic on rising/falling edge of i2s_sck
    assign re_trigger = clk_count_reached && ~i2s_sck; // Rising edge trigger
    assign fe_trigger = clk_count_reached && i2s_sck; // Falling edge trigger


    // Generate bit counter
    logic [$clog2(32)-1:0] bit_counter;
    logic bit_counter_hit;
    assign bit_counter_hit = (bit_counter == 32-1);
    always_ff @(posedge clk) begin
        if(~rstn) begin
            bit_counter <= '0;

        end else begin
            // Bit counter naturally overflows at 32
            if(re_trigger) begin
                bit_counter <= bit_counter + 1;
            end
        end
    end

    
    // Generate WS signal - toggled on the falling edge of i2s_sck every 32 bits
    always_ff @(posedge clk) begin
        if(~rstn) begin
            i2s_ws <= 1'b0;

        end else begin
            if(bit_counter_hit && fe_trigger) begin
                i2s_ws <= ~i2s_ws;
            end
        end
    end

    
    // Shift in data
    logic [I2S_BITS-1:0] i2s_data_reg[4];
    always_ff @(posedge clk) begin
        if(~rstn) begin
            i2s_data_reg <= '{default: '0};

        end else begin
            if(fe_trigger && bit_counter < I2S_BITS) begin
                for(int i = 0; i < 4; i++) begin
                    i2s_data_reg[i] <= {i2s_data_reg[i][I2S_BITS-2:0], i2s_data[i]};
                end
            end
        end
    end


    // Write data to axis interface
    logic [I2S_BITS-1:0] i2s_data_reg_buffer[4];
    logic [$clog2(4+1)-1:0] available_samples; // Number of samples buffered waiting to be written to the axis interface
    logic [$clog2(4)-1:0] current_sample;
    logic available_samples_channel; // State of WS when samples were gathered

    assign current_sample = 4 - available_samples;
    assign axis_i2s.tdata = i2s_data_reg_buffer[current_sample];
    assign axis_i2s.tvalid = (available_samples > 0);
    assign axis_i2s.tid = {current_sample, available_samples_channel}; // Id LSB is channel, MSBs are interface index
    always_ff @(posedge clk) begin
        if(~rstn) begin
            i2s_data_reg_buffer <= '{default: '0};
            available_samples <= '0;
            available_samples_channel <= 1'b0;

        end else begin
            if(bit_counter_hit && fe_trigger) begin
                available_samples <= 4;
                available_samples_channel <= i2s_ws;
                i2s_data_reg_buffer <= i2s_data_reg;

            end else if(axis_i2s.tvalid && axis_i2s.tready) begin
                available_samples <= available_samples - 1;
            end
        end
    end
    

endmodule