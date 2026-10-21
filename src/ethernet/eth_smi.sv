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
    localparam logic [4:0] PHY_ADDR    = 5'h00;

    logic mdc_fe;

    // SMI state machine
    typedef enum {
        IDLE,
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

    logic [ADDR_WIDTH-1:0] reg_addr;
    logic [DATA_WIDTH-1:0] write_data;


    always_comb begin
        if(~rstn) begin
            next_state = IDLE;
        end else begin
            case(state)
            IDLE: begin
                if(apb.psel) begin
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

    // State machine logic is run based on mdc, transitions are done on falling edge of mdc
    always_ff @(posedge clk) begin
        if(mdc_fe) begin
            state <= next_state;
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
        end else if(state == IDLE && apb.psel) begin
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
        end else if(state == IDLE && apb.psel) begin
            reg_addr <= apb.paddr;
            write_data <= apb.pwdata;
        end
    end

    // MDIO output enable - enabled at all times except for read operations during data/turnaround
    always_ff @(posedge clk) begin
        if(~rstn) begin
            mdio_OE <= 1'b0;
        end else begin
            mdio_OE <= ~(~write_active && (state == DATA || state == TURNAROUND));
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

            DATA: begin
                mdio_OUT <= write_data[DATA_WIDTH-1-data_counter];
            end

            default:
                mdio_OUT <= 1'b1;
            endcase
        end
    end



endmodule