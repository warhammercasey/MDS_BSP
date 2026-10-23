module rmii #(
    parameter WIDTH = 8
) (
    input  logic       clk,
    input  logic       rstn,

    output logic [1:0] rmii_txd,
    output logic       rmii_txen,
    output logic       rmii_rstn,
    input  logic       rmii_int,
    input  logic       rmii_crs_dv,
    input  logic [1:0] rmii_rxd,
    input  logic       rmii_rxerr,

    // axis_in is layer 2 ethernet frame
    // This module appends preamble, SFD, and IPG
    axis_tlast_if.slv  axis_in
);

    localparam PREAMBLE_BYTE = 8'b10101010;
    localparam SFD_BYTE      = 8'b10101011;

    localparam PREAMBLE_LENGTH = 7;
    localparam IPG_LENGTH = 12;


    typedef enum {
        IDLE,
        PREAMBLE,
        SFD,
        PAYLOAD,
        IPG
    } state_t;
    state_t state, next_state;

    logic [$clog2(8/2)-1:0] bit_counter; // Counts bits transferred for each byte
    logic [$clog2(PREAMBLE_LENGTH)-1:0] preamble_counter;
    logic [$clog2(IPG_LENGTH)-1:0] ipg_counter;

    logic bit_counter_done;
    logic preamble_counter_done;
    logic ipg_counter_done;

    assign bit_counter_done = bit_counter == 8/2-1;
    assign preamble_counter_done = preamble_counter == PREAMBLE_LENGTH - 1;
    assign ipg_counter_done = ipg_counter == IPG_LENGTH - 1;

    logic last_payload_byte;

    // State machine
    always_comb begin
        if(~rstn) begin
            next_state = IDLE;
        end else begin
            case(state)
            IDLE: if(axis_in.tvalid) begin
                next_state = PREAMBLE;
            end

            PREAMBLE: if(bit_counter_done && preamble_counter_done) begin
                next_state = SFD_BYTE;
            end

            SFD_BYTE: if(bit_counter_done) begin
                next_state = PAYLOAD;
            end

            PAYLOAD: if(bit_counter_done && last_payload_byte) begin
                next_state = IPG;
            end

            IPG: if(bit_counter_done && ipg_counter_done) begin
                next_state = IDLE;
            end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        state <= next_state;
    end



endmodule