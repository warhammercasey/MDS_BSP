module led_cube #(
    parameter PWM_TIM_WIDTH = 8,
    parameter INC_TIM_WIDTH = 16
) (
    input  logic       clk,
    input  logic       rstn,

    output logic [2:0] led_out
);

    logic [PWM_TIM_WIDTH-1:0] pwm_counter;
    logic [PWM_TIM_WIDTH-1:0] pwm_vals[3];
    always_ff @(posedge clk) begin
        if(~rstn) begin
            led_out <= '0;
            pwm_counter <= '0;

        end else begin
            pwm_counter <= pwm_counter + 1;
            for(int i = 0; i < 3; i++) begin
                led_out[i] <= pwm_counter < pwm_vals[i];
            end
        end
    end


    logic [INC_TIM_WIDTH-1:0] prescaler;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            prescaler <= '0;

        end else begin
            prescaler <= prescaler + 1;
        end
    end


    logic upcounting;
    logic [1:0] state;
    logic [1:0] next_state;
    assign next_state = (state >= 2) ? (0) : (state + 1);
    always_ff @(posedge clk) begin
        if(~rstn) begin
            pwm_vals[0] <= '1;
            pwm_vals[1] <= '1;
            pwm_vals[2] <= '0;

            upcounting <= 1'b0;
            state <= '0;

        end else if(prescaler == '0) begin
            if(upcounting) begin
                pwm_vals[next_state] <= pwm_vals[next_state] + 1;

                if(pwm_vals[next_state] == (1 << PWM_TIM_WIDTH) - 2) begin // Last increment
                    upcounting <= 1'b0;
                end

            end else begin
                pwm_vals[state] <= pwm_vals[state] - 1;

                if(pwm_vals[state] == 1) begin
                    upcounting <= 1'b1;
                    state <= next_state;
                end
            end
        end
    end

endmodule