// RMII receiver (100 Mbit/s). Strips the preamble and SFD and outputs the frame
// (destination address through FCS) one byte per beat on axis_out, tlast on the last.
//
// Framing (LAN8720A datasheet 3.4.1.1):
//   - CRS_DV rises with the carrier. RXD reads 00 until the PHY has locked, then
//     preamble (01 dibits), then the SFD, which ends with an 11 dibit. Data follows,
//     LSB dibit first.
//   - If the carrier ends while the PHY's FIFO still holds data, CRS_DV toggles: low
//     on the first dibit of each nibble, high on the second. So the frame only ends
//     when CRS_DV is low on a nibble's second dibit. A trailing partial byte is dropped.
//
// Errors:
//   - RXER before the SFD (bad SSD, false carrier, ...) is ignored.
//   - RXER after the SFD aborts the frame. If any of the frame has already gone out
//     on axis_out, the held byte (below) goes out with tlast, ending the frame early
//     so the FCS check downstream drops it. Otherwise nothing of the frame is output.
//     The rest of the carrier is ignored.
//
// Each byte is held back until the next one arrives or the frame ends, as only then
// is it known whether it is the last. RMII has no flow control: axis_out must take
// each beat within one byte time (4 cycles). If it doesn't, the frame is cut short
// the same way as for RXER.
//
// 10 Mbit/s (each dibit repeated for 10 cycles) is not supported.
module rmii_rx (
    input  logic       clk,
    input  logic       rstn,

    input  logic       rmii_crs_dv,
    input  logic [1:0] rmii_rxd,
    input  logic       rmii_rxerr,

    axis_tlast_if.mst  axis_out
);

    localparam AXIS_WIDTH = 8;

    localparam logic [1:0] PREAMBLE_DIBIT = 2'b01;
    localparam logic [1:0] SFD_LAST_DIBIT = 2'b11; // SFD 0xD5 is 01 01 01 11 on RXD


    // Register the RMII inputs before any logic: they are timed against clk at the pins
    // (constraints.sdc). Not reset, so they follow the line during reset (see DROP below).
    logic       crs_dv, crs_dv_prev;
    logic [1:0] rxd, rxd_prev;
    logic       rxerr;

    always_ff @(posedge clk) begin
        crs_dv      <= rmii_crs_dv;
        rxd         <= rmii_rxd;
        rxerr       <= rmii_rxerr;
        crs_dv_prev <= crs_dv;
        rxd_prev    <= rxd;
    end


    typedef enum {
        IDLE,     // No carrier
        PREAMBLE, // Carrier, waiting for the SFD
        DATA,     // Receiving the frame
        DROP      // Ignoring the rest of the carrier
    } state_t;
    state_t state, next_state;

    logic [1:0] dibit_counter; // Position of the current dibit in its byte
    logic [AXIS_WIDTH-1:0] byte_sr; // Dibits of the byte so far, shifted in from the top
    logic [AXIS_WIDTH-1:0] rx_byte; // The completed byte, valid with byte_done

    logic sfd;        // Last dibit of the SFD
    logic rx_error;   // RXER during the frame
    logic frame_end;  // Carrier gone: CRS_DV low on the second dibit of a nibble
    logic byte_done;  // Last dibit of a byte
    logic overflow;   // A byte arrived with the held byte still waiting for axis_out

    // Output side: one byte is held back in pend_data, see top
    logic pend_valid;
    logic pend_last;  // pend_data is the frame's last byte: send it with tlast
    logic [AXIS_WIDTH-1:0] pend_data;
    logic started;    // Some of this frame has been put on axis_out
    logic out_free;   // axis_out can take a new beat this cycle

    assign sfd       = crs_dv && !rxerr && rxd == SFD_LAST_DIBIT && rxd_prev == PREAMBLE_DIBIT;
    assign rx_error  = state == DATA && rxerr;
    assign frame_end = state == DATA && !rxerr && !crs_dv && dibit_counter[0];
    assign byte_done = state == DATA && !rxerr && !frame_end && dibit_counter == 3;
    assign rx_byte   = {rxd, byte_sr[AXIS_WIDTH-1:2]};
    assign out_free  = !axis_out.tvalid || axis_out.tready;
    assign overflow  = byte_done && pend_valid && !out_free;


    // State machine
    always_comb begin
        next_state = state;
        if(~rstn) begin
            next_state = DROP; // A frame may already be in progress
        end else begin
            case(state)
            IDLE: if(crs_dv) begin
                next_state = PREAMBLE;
            end

            PREAMBLE: begin
                if(!crs_dv) begin
                    next_state = DROP; // CRS_DV may toggle as the carrier ends
                end else if(sfd) begin
                    // The previous frame's last byte still hasn't gone out: no room
                    next_state = pend_valid ? DROP : DATA;
                end
            end

            DATA: begin
                if(rx_error || overflow) begin
                    next_state = DROP;
                end else if(frame_end) begin
                    next_state = IDLE;
                end
            end

            DROP: if(!crs_dv && !crs_dv_prev) begin
                next_state = IDLE;
            end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        state <= next_state;
    end


    // Dibit counter and byte shift register
    always_ff @(posedge clk) begin
        if(~rstn || state != DATA) begin
            dibit_counter <= '0;
        end else begin
            dibit_counter <= dibit_counter + 1;
        end
    end

    always_ff @(posedge clk) begin
        if(~rstn) begin
            byte_sr <= '0;
        end else if(state == DATA) begin
            byte_sr <= rx_byte;
        end
    end


    // Output: hold each byte in pend_data until it is known whether it is the last
    always_ff @(posedge clk) begin
        if(~rstn) begin
            axis_out.tvalid <= 1'b0;
            axis_out.tdata  <= '0;
            axis_out.tlast  <= 1'b0;
            pend_valid      <= 1'b0;
            pend_last       <= 1'b0;
            pend_data       <= '0;
            started         <= 1'b0;

        end else begin
            if(axis_out.tready) begin
                axis_out.tvalid <= 1'b0;
            end

            if(state == PREAMBLE && sfd) begin
                started <= 1'b0;
            end

            if(byte_done) begin
                if(!pend_valid) begin
                    // First byte of the frame
                    pend_valid <= 1'b1;
                    pend_data  <= rx_byte;
                end else if(out_free) begin
                    // Another byte arrived, so the held one isn't the last
                    axis_out.tvalid <= 1'b1;
                    axis_out.tdata  <= pend_data;
                    axis_out.tlast  <= 1'b0;
                    pend_data       <= rx_byte;
                    started         <= 1'b1;
                end else if(started) begin
                    pend_last <= 1'b1; // Overflow: end the frame on the held byte
                end else begin
                    pend_valid <= 1'b0; // Overflow before any output: drop the frame
                end
            end

            if(frame_end) begin
                pend_last <= pend_valid;
            end

            if(rx_error) begin
                if(started) begin
                    pend_last <= 1'b1;
                end else begin
                    pend_valid <= 1'b0;
                end
            end

            // Send the last byte. Never in the same cycle as the events above: they only
            // happen in DATA, and pend_last is only set on leaving it.
            if(pend_valid && pend_last && out_free) begin
                axis_out.tvalid <= 1'b1;
                axis_out.tdata  <= pend_data;
                axis_out.tlast  <= 1'b1;
                pend_valid      <= 1'b0;
                pend_last       <= 1'b0;
            end
        end
    end

endmodule
