// UART console: echoes what is typed and runs register commands on the APB bus.
//
//   r <sel> <addr>          read  -> prints the data, e.g. "DE"
//   w <sel> <addr> <data>   write -> prints "OK"
//
// <sel> is the peripheral (psel line). Fields are hex, in either case, with any
// number of leading zeros, separated by one or more spaces. A line ends with CR, LF
// or CR LF. These print "ERR":
//   - a malformed line
//   - a <sel> with no peripheral, or an <addr>/<data> too wide for the bus
//   - a transfer the peripheral ends with PSLVERR (e.g. no register at <addr>)
//   - a peripheral that doesn't raise pready within APB_TIMEOUT cycles
// An empty line just prints a new line.
//
// The parser works on characters as they arrive, so there is no line buffer and no
// line editing: a typo, including a backspace, gives ERR when the line is entered.
// It stops taking characters while it runs a transfer and prints the response, so
// only one character typed in that time survives (the one waiting in uart_rx).
module uart_controller #(
    parameter CLK_FREQ       = 50_000_000,
    parameter BAUD           = 115200,
    parameter APB_ADDR_WIDTH = 8,
    parameter APB_SEL_WIDTH  = 1,    // Number of APB peripherals (psel lines)
    parameter APB_TIMEOUT    = 1024  // Cycles to wait for pready before giving up
) (
    input  logic clk,
    input  logic rstn,

    input  logic rx,
    output logic tx,

    apb_if.mst apb
);

    localparam APB_DATA_WIDTH = 8;
    localparam NUM_UART_SOURCES = 2;

    // Ceiling division
    localparam NUM_DATA_CHARS = (APB_DATA_WIDTH + 3) / 4;

    localparam SEL_IDX_WIDTH = APB_SEL_WIDTH > 1 ? $clog2(APB_SEL_WIDTH) : 1;

    // Field accumulator: wide enough for any field, in whole hex digits
    localparam MAX_FIELD_WIDTH = APB_ADDR_WIDTH > APB_DATA_WIDTH ?
                                 (APB_ADDR_WIDTH > SEL_IDX_WIDTH ? APB_ADDR_WIDTH : SEL_IDX_WIDTH) :
                                 (APB_DATA_WIDTH > SEL_IDX_WIDTH ? APB_DATA_WIDTH : SEL_IDX_WIDTH);
    localparam ACC_WIDTH = 4 * ((MAX_FIELD_WIDTH + 3) / 4);

    localparam logic [7:0] CR = 8'h0D;
    localparam logic [7:0] LF = 8'h0A;

    function automatic logic is_hex(logic [7:0] c);
        return (c >= "0" && c <= "9") || (c >= "a" && c <= "f") || (c >= "A" && c <= "F");
    endfunction

    function automatic logic [3:0] hex_val(logic [7:0] c);
        if (c <= "9")
            return 4'(c - "0");
        return 4'((c | 8'h20) - "a" + 8'd10); // | 8'h20 makes the letter lower case
    endfunction

    function automatic logic [7:0] hex_char(logic [3:0] n);
        return n < 4'd10 ? "0" + 8'(n) : "A" - 8'd10 + 8'(n);
    endfunction

    // Instantiate uart
    axis_if #(
        .WIDTH(8)
    ) uart_tx_if ();
    axis_if #(
        .WIDTH(8)
    ) uart_rx_if ();
    uart #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD(BAUD)
    ) uart_i (
        .clk(clk),
        .rstn(rstn),
        .tx(tx),
        .rx(rx),
        .axis_tx(uart_tx_if),
        .axis_rx(uart_rx_if)
    );

    // UART tx received inputs from multiple sources, combine them here
    //   0: echo of the received characters
    //   1: command responses
    axis_if #(
        .WIDTH(8)
    ) uart_tx_inputs[NUM_UART_SOURCES] ();
    axis_switch #(
        .WIDTH(8),
        .N(NUM_UART_SOURCES)
    ) uart_tx_switch (
        .clk(clk),
        .rstn(rstn),
        .axis_in(uart_tx_inputs),
        .axis_out(uart_tx_if)
    );

    // ------------------------------------------------------------------
    // Received characters
    // ------------------------------------------------------------------
    typedef enum logic [3:0] {
        S_CMD,        // Start of line: spaces, then 'r' or 'w'
        S_SEP,        // A space must follow the command letter
        S_GAP,        // Spaces before the next field
        S_FIELD,      // Hex digits of field `field`
        S_TRAIL,      // All fields read: only spaces until the end of the line
        S_DISCARD,    // The line is bad: ignore the rest of it
        S_APB_SETUP,
        S_APB_ACCESS,
        S_RESP        // Printing the response
    } state_t;

    typedef enum logic [1:0] {
        R_NEWLINE,    // CR LF
        R_ERR,        // CR LF "ERR" CR LF
        R_OK,         // CR LF "OK" CR LF
        R_DATA        // CR LF <data in hex> CR LF
    } resp_t;

    state_t state;
    resp_t  resp;

    localparam RESP_MAX_LEN = 4 + (NUM_DATA_CHARS > 3 ? NUM_DATA_CHARS : 3);

    logic [$clog2(RESP_MAX_LEN)-1:0] resp_idx;  // Character of the response being printed
    logic [7:0] resp_char;
    logic       resp_fire;
    logic       resp_last;

    logic       parse_ready; // Parser is waiting for characters
    logic [7:0] rx_char;
    logic       rx_eol;
    logic       rx_fire;

    assign parse_ready = state != S_APB_SETUP && state != S_APB_ACCESS && state != S_RESP;
    assign rx_char     = uart_rx_if.tdata;
    assign rx_eol      = rx_char == CR || rx_char == LF;

    // The received character goes to the parser and back out as an echo. It is only
    // taken when both can take it, so the echo is always queued ahead of the
    // response. CR and LF aren't echoed: the response starts with its own CR LF.
    assign uart_tx_inputs[0].tdata  = rx_char;
    assign uart_tx_inputs[0].tvalid = uart_rx_if.tvalid && parse_ready && !rx_eol;
    assign uart_rx_if.tready        = parse_ready && (uart_tx_inputs[0].tready || rx_eol);
    assign rx_fire                  = uart_rx_if.tvalid && uart_rx_if.tready;

    // ------------------------------------------------------------------
    // Command parser and APB master
    // ------------------------------------------------------------------
    logic                      last_cr;   // Previous character was CR
    logic                      is_write;
    logic [1:0]                field;     // 0: sel, 1: addr, 2: data
    logic [ACC_WIDTH-1:0]      acc;       // Value of the field being read
    logic                      acc_ovf;   // A non-zero digit was shifted out of acc
    logic [SEL_IDX_WIDTH-1:0]  sel_r;
    logic [APB_ADDR_WIDTH-1:0] addr_r;
    logic [APB_DATA_WIDTH-1:0] data_r;    // Write data, or read data for the response
    localparam TIMER_WIDTH = $clog2(APB_TIMEOUT + 1);
    logic [TIMER_WIDTH-1:0]    apb_timer;

    logic last_field, field_ok;
    assign last_field = field == (is_write ? 2'd2 : 2'd1);

    // Field in acc has a valid value
    always_comb begin
        case (field)
            2'd0:    field_ok = acc < APB_SEL_WIDTH;
            2'd1:    field_ok = (acc >> APB_ADDR_WIDTH) == '0;
            default: field_ok = (acc >> APB_DATA_WIDTH) == '0;
        endcase
        field_ok = field_ok && !acc_ovf;
    end

    always_ff @(posedge clk) begin
        if (~rstn) begin
            state     <= S_CMD;
            resp      <= R_NEWLINE;
            last_cr   <= 1'b0;
            is_write  <= 1'b0;
            field     <= '0;
            acc       <= '0;
            acc_ovf   <= 1'b0;
            sel_r     <= '0;
            addr_r    <= '0;
            data_r    <= '0;
            apb_timer <= '0;
        end else begin
            apb_timer <= state == S_APB_ACCESS ? apb_timer + 1'b1 : '0;

            // Keep the field being read stored. It is complete by the time the
            // character after it arrives.
            if (state == S_FIELD) begin
                case (field)
                    2'd0:    sel_r  <= SEL_IDX_WIDTH'(acc);
                    2'd1:    addr_r <= APB_ADDR_WIDTH'(acc);
                    default: data_r <= APB_DATA_WIDTH'(acc);
                endcase
            end

            case (state)
                S_APB_SETUP: state <= S_APB_ACCESS;

                S_APB_ACCESS: begin
                    if (apb.pready) begin
                        if (!is_write)
                            data_r <= apb.prdata;
                        resp  <= apb.pslverr ? R_ERR : (is_write ? R_OK : R_DATA);
                        state <= S_RESP;
                    end else if (apb_timer == TIMER_WIDTH'(APB_TIMEOUT - 1)) begin
                        // Give up. Dropping psel mid-transfer breaks the APB protocol,
                        // but a stuck peripheral shouldn't lock up the console.
                        resp  <= R_ERR;
                        state <= S_RESP;
                    end
                end

                S_RESP: begin
                    if (resp_fire && resp_last)
                        state <= S_CMD;
                end

                default: begin
                    if (rx_fire) begin
                        last_cr <= rx_char == CR;

                        if (rx_char == LF && last_cr) begin
                            // LF of a CR LF pair: the CR already ended the line

                        end else if (rx_eol) begin
                            // End of the line: run the command, or report the error
                            state <= S_RESP;
                            resp  <= R_ERR;
                            if (state == S_CMD)
                                resp <= R_NEWLINE; // Empty line
                            else if (state == S_TRAIL || (state == S_FIELD && last_field && field_ok))
                                state <= S_APB_SETUP;

                        end else begin
                            case (state)
                                S_CMD: begin
                                    field <= '0;
                                    if (rx_char == "r" || rx_char == "R") begin
                                        is_write <= 1'b0;
                                        state    <= S_SEP;
                                    end else if (rx_char == "w" || rx_char == "W") begin
                                        is_write <= 1'b1;
                                        state    <= S_SEP;
                                    end else if (rx_char != " ") begin
                                        state    <= S_DISCARD;
                                    end
                                end

                                S_SEP: state <= rx_char == " " ? S_GAP : S_DISCARD;

                                S_GAP: begin
                                    if (is_hex(rx_char)) begin
                                        acc     <= ACC_WIDTH'(hex_val(rx_char));
                                        acc_ovf <= 1'b0;
                                        state   <= S_FIELD;
                                    end else if (rx_char != " ") begin
                                        state   <= S_DISCARD;
                                    end
                                end

                                S_FIELD: begin
                                    if (is_hex(rx_char)) begin
                                        acc     <= {acc[ACC_WIDTH-5:0], hex_val(rx_char)};
                                        acc_ovf <= acc_ovf || acc[ACC_WIDTH-1 -: 4] != '0;
                                    end else if (rx_char == " " && field_ok) begin
                                        if (last_field) begin
                                            state <= S_TRAIL;
                                        end else begin
                                            field <= field + 1'b1;
                                            state <= S_GAP;
                                        end
                                    end else begin
                                        state <= S_DISCARD;
                                    end
                                end

                                S_TRAIL: begin
                                    if (rx_char != " ")
                                        state <= S_DISCARD;
                                end

                                default: ; // S_DISCARD
                            endcase
                        end
                    end
                end
            endcase
        end
    end

    // APB outputs come straight from the state, so they hold for the whole transfer
    always_comb begin
        for (int i = 0; i < APB_SEL_WIDTH; i++)
            apb.psel[i] = (state == S_APB_SETUP || state == S_APB_ACCESS) && sel_r == SEL_IDX_WIDTH'(i);
    end
    assign apb.penable = state == S_APB_ACCESS;
    assign apb.pwrite  = is_write;
    assign apb.paddr   = addr_r;
    assign apb.pwdata  = data_r;

    // ------------------------------------------------------------------
    // Response
    // ------------------------------------------------------------------
    int body_len;

    always_comb begin
        int k;
        case (resp)
            R_NEWLINE: body_len = 0;
            R_ERR:     body_len = 3;
            R_OK:      body_len = 2;
            default:   body_len = NUM_DATA_CHARS;
        endcase

        // CR LF, then the body, then CR LF if there is a body
        k = int'(resp_idx) - 2;
        if (resp_idx == 0 || k == body_len)
            resp_char = CR;
        else if (resp_idx == 1 || k == body_len + 1)
            resp_char = LF;
        else begin
            case (resp)
                R_ERR:   resp_char = k == 0 ? "E" : "R";
                R_OK:    resp_char = k == 0 ? "O" : "K";
                default: resp_char = hex_char(4'(data_r >> (4 * (NUM_DATA_CHARS - 1 - k))));
            endcase
        end

        resp_last = int'(resp_idx) == (body_len == 0 ? 1 : body_len + 3);
    end

    assign uart_tx_inputs[1].tvalid = state == S_RESP;
    assign uart_tx_inputs[1].tdata  = resp_char;
    assign resp_fire = uart_tx_inputs[1].tvalid && uart_tx_inputs[1].tready;

    always_ff @(posedge clk) begin
        if (state != S_RESP)
            resp_idx <= '0;
        else if (resp_fire)
            resp_idx <= resp_idx + 1'b1;
    end

endmodule
