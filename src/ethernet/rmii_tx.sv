module rmii_tx (
    input  logic       clk,
    input  logic       rstn,

    output logic [1:0] rmii_txd,
    output logic       rmii_txen,

    // axis_in is layer 2 ethernet frame
    // This module appends preamble, SFD, and IPG
    axis_tlast_if.slv  axis_in
);

    // Byte values, sent LSB first like the payload. (On the wire, in order, they
    // read 10101010 and 10101011.)
    localparam PREAMBLE_BYTE = 8'h55;
    localparam SFD_BYTE      = 8'hD5;

    localparam PREAMBLE_LENGTH = 7;
    localparam IPG_LENGTH = 12;
    localparam AXIS_WIDTH = 8;


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

    assign bit_counter_done = bit_counter == 8/2 - 1;
    assign preamble_counter_done = preamble_counter == PREAMBLE_LENGTH - 1;
    assign ipg_counter_done = ipg_counter == IPG_LENGTH - 1;

    logic last_payload_byte;

    logic run_bit_counter, run_preamble_counter, run_ipg_counter;

    // State machine
    always_comb begin
        next_state = state;
        if(~rstn) begin
            next_state = IDLE;
        end else begin
            case(state)
            IDLE: if(axis_in.tvalid) begin
                next_state = PREAMBLE;
            end

            PREAMBLE: if(bit_counter_done && preamble_counter_done) begin
                next_state = SFD;
            end

            SFD: if(bit_counter_done) begin
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



    // Bit counter
    assign run_bit_counter = state != IDLE;
    always_ff @(posedge clk) begin
        if(~rstn || ~run_bit_counter) begin
            bit_counter <= '0;
        end else begin
            bit_counter <= bit_counter + 1;
        end
    end

    // Preamble counter
    assign run_preamble_counter = state == PREAMBLE;
    always_ff @(posedge clk) begin
        if(~rstn || ~run_preamble_counter) begin
            preamble_counter <= '0;
        end else begin
            if(bit_counter_done) begin
                preamble_counter <= preamble_counter + 1;
            end
        end
    end

    // IPG counter
    assign run_ipg_counter = state == IPG;
    always_ff @(posedge clk) begin
        if(~rstn || ~run_ipg_counter) begin
            ipg_counter <= '0;
        end else begin
            if(bit_counter_done) begin
                ipg_counter <= ipg_counter + 1;
            end
        end
    end


    // Latch payload data from input
    logic [AXIS_WIDTH-1:0] payload_byte;
    assign axis_in.tready = (state == SFD && bit_counter_done) ||
                            (state == PAYLOAD && bit_counter_done && !last_payload_byte);
    always_ff @(posedge clk) begin
        if(~rstn) begin
            payload_byte <= '0;
        end else if(axis_in.tvalid && axis_in.tready) begin
            payload_byte <= axis_in.tdata;
        end
    end

    // Latch last payload byte signal
    always_ff @(posedge clk) begin
        if(~rstn || state == IDLE) begin
            last_payload_byte <= 1'b0;
        end else if(axis_in.tvalid && axis_in.tready) begin
            last_payload_byte <= axis_in.tlast;
        end
    end

    // Drive TXD and TXEN
    always_ff @(posedge clk) begin
        if(~rstn) begin
            rmii_txd <= 2'b00;
            rmii_txen <= 1'b0;
        end else begin
            case(state)
            PREAMBLE: begin
                rmii_txd <= PREAMBLE_BYTE[{bit_counter, 1'b0} +: 2];
                rmii_txen <= 1'b1;
            end

            SFD: begin
                rmii_txd <= SFD_BYTE[{bit_counter, 1'b0} +: 2];
                rmii_txen <= 1'b1;
            end

            PAYLOAD: begin
                rmii_txd <= payload_byte[{bit_counter, 1'b0} +: 2];
                rmii_txen <= 1'b1;
            end

            IPG: begin
                rmii_txd <= 2'b00;
                rmii_txen <= 1'b0;
            end

            default: begin
                rmii_txd <= 2'b00;
                rmii_txen <= 1'b0;
            end
            endcase

        end
    end


endmodule