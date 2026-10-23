module eth_smi #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 5,
    parameter MDC_DIV = 10
) (
    input  logic clk,
    input  logic rstn,

    // APB interface
    apb_if.slv apb,

    // SMI interface
    output logic mdc,
    input  logic mdio_IN,
    output logic mdio_OUT,
    output logic mdio_OE
);


    localparam logic [1:0] START_CODE = 2'b01;
    localparam logic [1:0] OP_READ     = 2'b10;
    localparam logic [1:0] OP_WRITE    = 2'b01;
    localparam logic [1:0] TA_WRITE    = 2'b10; // Turnaround driven during a write
    localparam logic [4:0] PHY_ADDR    = 5'h00;
    localparam PREAMBLE_BITS = 32; // Ones sent before every frame

    // Clock rising edge and falling edge
    logic mdc_fe, mdc_re;

    // SMI state machine
    typedef enum {
        IDLE,
        ERROR,
        PREAMBLE,
        START,
        OP,
        PHY,
        REG,
        TURNAROUND,
        DATA
    } smi_state_t;
    smi_state_t state, next_state;
    logic write_active;
    logic two_counter;
    logic [$clog2(ADDR_WIDTH)-1:0] addr_counter;
    logic [$clog2(DATA_WIDTH)-1:0] data_counter;
    logic [$clog2(PREAMBLE_BITS)-1:0] pre_counter;

    logic [ADDR_WIDTH-1:0] reg_addr;
    logic [DATA_WIDTH-1:0] write_data;
    logic [DATA_WIDTH-1:0] read_data;
    logic address_valid;

    // A transfer is waiting: access phase, but not the cycle it completes in (psel is
    // still high then, and would start the same transfer again)
    logic request;
    assign request = apb.psel[0] && apb.penable && !apb.pready;


    always_comb begin
        next_state = state; // Hold unless a transition below applies
        if(~rstn) begin
            next_state = IDLE;
        end else begin
            case(state)
            IDLE: begin
                if(request) begin
                    if(address_valid) begin
                        next_state = PREAMBLE;
                    end else begin
                        next_state = ERROR;
                    end
                end
            end

            ERROR: begin
                next_state = IDLE;
            end

            PREAMBLE: begin
                if(pre_counter == PREAMBLE_BITS-1) begin
                    next_state = START;
                end
            end

            START: begin
                if(two_counter) begin
                    next_state = OP;
                end
            end

            OP: begin
                if(two_counter) begin
                    next_state = PHY;
                end
            end

            PHY: begin
                if(addr_counter == ADDR_WIDTH-1) begin
                    next_state = REG;
                end
            end

            REG: begin
                if(addr_counter == ADDR_WIDTH-1) begin
                    next_state = TURNAROUND;
                end
            end

            TURNAROUND: begin
                if(two_counter) begin
                    next_state = DATA;
                end
            end

            DATA: begin
                if(data_counter == DATA_WIDTH-1) begin
                    next_state = IDLE;
                end
            end
            endcase

        end
    end

    // Check for valid addressing
    // Valid addresses are 0-6, 17, 18, 26, 27, 29, 30, 31 (decimal)
    assign address_valid = apb.paddr <= 6 ||
                           apb.paddr == 17 ||
                           apb.paddr == 18 ||
                           apb.paddr == 26 ||
                           apb.paddr == 27 ||
                           apb.paddr == 29 ||
                           apb.paddr == 30 ||
                           apb.paddr == 31;

    // Assert pslverr on error
    always_ff @(posedge clk) begin
        if(~rstn) begin
            apb.pslverr <= 1'b0;

        end else begin
            apb.pslverr <= state == ERROR && mdc_fe;
        end
    end
    

    // State machine logic is run based on mdc, transitions are done on falling edge of mdc
    always_ff @(posedge clk) begin
        if(~rstn) begin
            state <= IDLE;
        end else if(mdc_fe) begin
            state <= next_state;
        end
    end

    // Preamble counter - counts the ones sent before the start of frame
    always_ff @(posedge clk) begin
        if(~rstn) begin
            pre_counter <= 0;
        end else if(state == PREAMBLE && mdc_fe) begin
            pre_counter <= pre_counter + 1;
        end
    end

    // Two counter - counts any parts of the SMI frame that are 2 bits long (start, op, turnaround)
    always_ff @(posedge clk) begin
        if(~rstn) begin
            two_counter <= 1'b0;
        end else if((state == START || state == OP || state == TURNAROUND) && mdc_fe) begin
            two_counter <= ~two_counter;
        end
    end

    // Write active flag - indicates that the SMI frame is a write operation
    always_ff @(posedge clk) begin
        if(~rstn) begin
            write_active <= 1'b0;
        end else if(state == IDLE && apb.psel[0]) begin
            write_active <= apb.pwrite;
        end
    end

    // Address counter - counts the number of address bits sent/received (PHY Address and Register Address)
    always_ff @(posedge clk) begin
        if(~rstn) begin
            addr_counter <= 0;
        end else if((state == PHY || state == REG) && mdc_fe) begin
            if(addr_counter == ADDR_WIDTH-1) begin
                addr_counter <= 0;
            end else begin
                addr_counter <= addr_counter + 1;
            end
        end
    end

    // Data counter - counts the number of data bits sent/received
    always_ff @(posedge clk) begin
        if(~rstn) begin
            data_counter <= 0;
        end else if(state == DATA && mdc_fe) begin
            if(data_counter == DATA_WIDTH-1) begin
                data_counter <= 0;
            end else begin
                data_counter <= data_counter + 1;
            end
        end
    end

    // Latch register address and write data from APB interface
    always_ff @(posedge clk) begin
        if(~rstn) begin
            reg_addr <= 0;
            write_data <= 0;
        end else if(state == IDLE && apb.psel[0]) begin
            reg_addr <= apb.paddr;
            write_data <= apb.pwdata;
        end
    end

    // MDIO output enable - driven from the preamble through the register address, and for the
    // turnaround and data of a write. Released otherwise: the PHY drives read data, and IDLE is
    // high-Z (pulled up), which leaves the PHY up to 300 ns to release MDIO after a read.
    always_ff @(posedge clk) begin
        if(~rstn) begin
            mdio_OE <= 1'b0;
        end else begin
            mdio_OE <= state == PREAMBLE || state == START || state == OP || state == PHY || state == REG ||
                       (write_active && (state == TURNAROUND || state == DATA));
        end
    end

    // MDIO output
    always_ff @(posedge clk) begin
        if(~rstn) begin
            mdio_OUT <= 1'b0;

        end else begin
            case(state)
            START: begin
                mdio_OUT <= START_CODE[~two_counter];
            end

            OP: begin
                mdio_OUT <= write_active ? OP_WRITE[~two_counter] : OP_READ[~two_counter];
            end

            PHY: begin
                mdio_OUT <= PHY_ADDR[ADDR_WIDTH-1-addr_counter];
            end

            REG: begin
                mdio_OUT <= reg_addr[ADDR_WIDTH-1-addr_counter];
            end

            TURNAROUND: begin
                mdio_OUT <= TA_WRITE[~two_counter]; // Only driven for writes
            end

            DATA: begin
                mdio_OUT <= write_data[DATA_WIDTH-1-data_counter];
            end

            default:
                mdio_OUT <= 1'b1;
            endcase
        end
    end

    // Generate MDC clock
    logic [$clog2(MDC_DIV)-1:0] mdc_counter;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            mdc_counter <= '0;
            mdc <= 1'b0;
            mdc_fe <= 1'b0;
            mdc_re <= 1'b0;

        end else begin
            if(mdc_counter == MDC_DIV-1) begin
                mdc_counter <= '0;

            end else begin
                mdc_counter <= mdc_counter + 1;
            end

            mdc <= mdc_counter >= MDC_DIV/2;
            mdc_fe <= mdc_counter == MDC_DIV-1;
            mdc_re <= mdc_counter == MDC_DIV/2 - 1;
            
        end
    end

    // Shift in read data
    always_ff @(posedge clk) begin
        if(~rstn) begin
            read_data <= '0;

        end else begin
            if(state == DATA && mdc_re) begin
                read_data <= {read_data[DATA_WIDTH-2:0], mdio_IN};
            end
        end
    end

    // Generate APB response: at the end of the last data bit, or of an error
    logic transfer_done;
    assign transfer_done = mdc_fe && ((state == DATA && next_state == IDLE) || state == ERROR);
    always_ff @(posedge clk) begin
        if(~rstn) begin
            apb.prdata <= '0;
            apb.pready <= 1'b0;

        end else begin
            apb.pready <= transfer_done; // Assert pready on last cycle of transfer
            if(transfer_done) begin
                apb.prdata <= read_data;
            end
        end
    end
endmodule