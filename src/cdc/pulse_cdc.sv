module pulse_cdc #(
    parameter EN_FAST_TO_SLOW = 0,
    parameter CLK_RATIO       = 2
)(
    input  logic clka,
    input  logic clkb,

    input  logic din,
    output logic dout
);

    generate
    if(EN_FAST_TO_SLOW) begin

    // Extend the pulse to 2x slower clock periods
    logic [CLK_RATIO*2-1:0] delay_line;
    logic extended_pulse, cdc_pulse;
    always_ff @(posedge clka) begin
        for(int i = 0; i < CLK_RATIO*2-1; i = i + 1) begin
            delay_line[i + 1] <= delay_line[i];
        end
        delay_line[0] <= din;

        extended_pulse <= |delay_line;
    end

    // 2 ff sync across clocks
    ff_sync #(
        .WIDTH(1)
    ) ff_sync_i (
        .clka(clka),
        .clkb(clkb),

        .din(extended_pulse),
        .dout(cdc_pulse)
    );

    // Pulse output on rising edge of cdc pulse
    always_ff @(posedge clkb) begin
        dout <= cdc_pulse && ~$past(cdc_pulse);
    end



    end else begin // Slow to fast clock


    logic cdc_pulse;

    // 2 ff sync across clocks
    ff_sync #(
        .WIDTH(1)
    ) ff_sync_i (
        .clka(clka),
        .clkb(clkb),

        .din(din),
        .dout(cdc_pulse)
    );

    // Pulse output on rising edge of cdc pulse
    always_ff @(posedge clkb) begin
        dout <= cdc_pulse && ~$past(cdc_pulse);
    end

    end
    endgenerate

endmodule