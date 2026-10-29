module led_cube #(
    parameter PWM_TIM_WIDTH = 8,
    parameter INC_TIM_WIDTH = 16
) (
    input  logic       clk,
    input  logic       rstn,

    output logic       led_out
);

    logic [PWM_TIM_WIDTH-1:0] pwm_counter;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            pwm_counter <= '0;
        end else begin
            pwm_counter <= pwm_counter + 1;
        end
    end

    logic [INC_TIM_WIDTH-1:0] inc_counter;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            inc_counter <= '0;
        end else begin
            inc_counter <= inc_counter + 1;
        end
    end

    logic [PWM_TIM_WIDTH-1:0] pwm_val;
    logic upcounting;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            pwm_val <= '0;
            upcounting <= 1'b1;

        end else begin
            if(inc_counter == '0) begin
                if(upcounting) begin
                    pwm_val <= pwm_val + 1;
                    if(pwm_val == {PWM_TIM_WIDTH{1'b1}} - 1) begin
                        upcounting <= 1'b0;
                    end

                end else begin
                    pwm_val <= pwm_val - 1;
                    if(pwm_val == 1) begin
                        upcounting <= 1'b1;
                    end
                end
            end
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            led_out <= 1'b0;

        end else begin
            led_out <= pwm_counter < pwm_val;
        end
    end

endmodule