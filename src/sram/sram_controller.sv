module sram_controller #(
    parameter MAX_BURST = 256
) (
    input logic clk,
    input logic rstn,

    // QSPI SRAM interface
    // In SPI mode:
    //   SIO0: MOSI
    //   SIO1: MISO
    //   SIO2: N/A
    //   SIO3: HOLD
    input  logic [3:0]  qspi_sio_IN,
    output logic [3:0]  qspi_sio_OUT,
    output logic [3:0]  qspi_sio_OE,
    output logic        qspi_cs_n,

    // RAM interface
    // Write interface
    // Address is latched on wr_addr_valid, and data is written on subsequent wr_valid cycles.
    // Wait until busy is low before asserting wr_addr_valid
    input  logic [23:0] wr_addr,
    input  logic wr_addr_valid,
    input  logic [7:0] wr_data,
    input  logic wr_valid,
    output logic wr_ready,
    input  logic wr_last,

    // Latches addr on rd_addr_valid. Reads rd_burst_len bytes from memory and streams it out on rd_data.
    // Wait until busy is low before asserting rd_addr_valid
    input  logic [23:0] rd_addr,
    input  logic rd_addr_valid,
    output logic [7:0] rd_data,
    output logic rd_valid,
    input  logic rd_ready,
    output logic rd_last,
    input  logic [$clog2(MAX_BURST)-1:0] rd_burst_len,

    output logic busy
);

    localparam logic [7:0] CMD_EQIO = 8'h38; // Enter Quad I/O mode
    localparam logic [7:0] CMD_READ = 8'h0B; // High-speed read
    localparam logic [7:0] CMD_WRITE = 8'h02; // Write

    typedef enum {
        RESET,      // Initial state coming out of reset
        RESET_IO,   // Resets SRAM chip to SPI mode to ensure its in a known state
        RESET_IO_PAUSE, // Brings CS back high for a cycle before initiating next command
        ENTER_QUAD, // Enters quad SPI mode
        IDLE,       // Idle state, waiting for commands
        READ,       // Performing a read operation
        WRITE       // Performing a write operation
    } state_t;

    logic read_done, write_done, reset_io_done, enter_quad_done;

    state_t state;

    // State machine
    always_ff @(posedge clk) begin
        if(~rstn) begin
            state <= RESET;
        end else begin
            case(state)
            RESET: begin
                // Transition to RESET_IO state
                state <= RESET_IO;
            end
            RESET_IO: begin
                // Transition to ENTER_QUAD state
                if(reset_io_done) begin
                    state <= RESET_IO_PAUSE;
                end
            end
            RESET_IO_PAUSE: begin
                // Transition to ENTER_QUAD state
                state <= ENTER_QUAD;
            end
            ENTER_QUAD: begin
                // Transition to IDLE state
                if(enter_quad_done) begin
                    state <= IDLE;
                end
            end
            IDLE: begin
                // Wait for read or write commands
                if(rd_addr_valid) begin
                    state <= READ;
                end else if(wr_addr_valid) begin
                    state <= WRITE;
                end
            end
            READ: begin
                // Perform read operation and transition back to IDLE when done
                if(read_done) begin
                    state <= IDLE;
                end
            end
            WRITE: begin
                // Perform write operation and transition back to IDLE when done
                if(write_done) begin
                    state <= IDLE;
                end
            end
            endcase
        end
    end

    // Busy
    assign busy = (state != IDLE);

    // Write done waits 2 clock cyles to allow data to transfer
    always_ff @(posedge clk) begin
        if(~rstn) begin
            write_done <= 1'b0;
        end else begin
            write_done <= $past(wr_valid && wr_ready && wr_last);
        end
    end


    // CS
    assign qspi_cs_n = (state == RESET) || (state == RESET_IO_PAUSE) || (state == IDLE);

    // SPI mode bit counter
    logic [$clog2(8)-1:0] spi_bit_count;
    assign enter_quad_done = spi_bit_count == 7;
    always_ff @(posedge clk) begin
        if(~rstn || (state != ENTER_QUAD)) begin
            spi_bit_count <= '0;
        end else begin
            spi_bit_count <= spi_bit_count + 1'b1;
        end
    end

    // QSPI mode bit counter
    // QSPI mode transfers 4 bits at a time, so this actually stores the number of bits sent /4
    logic [$clog2((8 + 3*8 + MAX_BURST*8)/4)-1:0] qspi_bit_count;
    logic [$clog2((8 + 3*8 + MAX_BURST*8)/8)-1:0] qspi_byte_count;
    assign qspi_byte_count = qspi_bit_count >> 1;
    always_ff @(posedge clk) begin
        if(!rstn || ((state != READ) && (state != WRITE))) begin
            qspi_bit_count <= '0;
        end else begin
            qspi_bit_count <= qspi_bit_count + 1'b1;
        end
    end

    // Latch the address for read operations
    logic [23:0] read_addr_latched;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            read_addr_latched <= '0;
        end else begin
            if(rd_addr_valid && (state == IDLE)) begin
                read_addr_latched <= rd_addr;
            end
        end
    end

    // Latch the data for write operations
    logic [23:0] write_addr_latched;
    always_ff @(posedge clk) begin
        if(~rstn) begin
            write_addr_latched <= '0;
        end else begin
            if(wr_addr_valid && (state == IDLE)) begin
                write_addr_latched <= wr_data;
            end
        end
    end


    // Latch incoming wr data
    logic [7:0] wr_data_latched;
    assign wr_ready = (state == WRITE) && (qspi_byte_count >= 1 + 3 - 1) && qspi_bit_count[0] && ~write_done; // Requests new byte on second 4-bit pulse after address is transferred
    always_ff @(posedge clk) begin
        if(~rstn) begin
            wr_data_latched <= '0;

        end else begin
            if(wr_valid && wr_ready) begin
                wr_data_latched <= wr_data;
            end
        end
    end


    // SIO outputs
    always_comb begin
        case(state)
        RESET_IO: begin
            // Reset sequence sets all SIO lines high for 8 clocks
            // Do this by tri-stating the SIO lines to let the pull ups pull them high
            qspi_sio_OUT = 4'b1111;
            qspi_sio_OE = 4'b0000;
        end

        ENTER_QUAD: begin
            // SPI mode. HOLD held high, MOSI sends instruction, MISO is tri-stated
            qspi_sio_OUT = {3'b111, CMD_EQIO[7 - spi_bit_count]};
            qspi_sio_OE = 4'b1001; // Only drive MOSI and HOLD
        end

        READ: begin
            if(qspi_byte_count == '0) begin
                qspi_sio_OUT = qspi_bit_count[0] ? (CMD_READ[3:0]) : (CMD_READ[7:4]);
                qspi_sio_OE = 4'b1111; // Drive all SIO lines
            end else if(qspi_byte_count < 1 + 3) begin // Next 3 bytes are address
                qspi_sio_OUT = read_addr_latched[23 - (qspi_bit_count - 2)*4 -: 4];
                qspi_sio_OE = 4'b1111; // Drive all SIO lines
            end else begin
                qspi_sio_OUT = '0;
                qspi_sio_OE = 4'b0000; // Tri-state all SIO lines
            end
        end

        WRITE: begin
            if(qspi_byte_count == '0) begin
                qspi_sio_OUT = qspi_bit_count[0] ? (CMD_WRITE[3:0]) : (CMD_WRITE[7:4]);
                qspi_sio_OE = 4'b1111; // Drive all SIO lines
            end else if(qspi_byte_count < 1 + 3) begin // Next 3 bytes are address
                qspi_sio_OUT = write_addr_latched[23 - (qspi_bit_count - 2)*4 -: 4];
                qspi_sio_OE = 4'b1111; // Drive all SIO lines
            end else begin
                qspi_sio_OUT = qspi_bit_count[0] ? (wr_data_latched[3:0]) : (wr_data_latched[7:4]);
                qspi_sio_OE = 4'b1111; // Drive all SIO lines
            end
        end

        default: begin
            qspi_sio_OUT = '0;
            qspi_sio_OE = 4'b0000; // Tri-state all SIO lines
        end
        endcase
    end

    
    // Read in data
    always_ff @(posedge clk) begin
        if(~rstn) begin
            rd_data <= '0;

        end else begin
            // 1 CMD byte, 3 address, 3 dummy
            if(qspi_byte_count >= 1 + 3 + 3) begin
                if(~qspi_bit_count[0]) begin // first transfer, MSBs
                    rd_data[7:4] <= qspi_sio_IN;
                end else begin
                    rd_data[3:0] <= qspi_sio_IN;
                end
            end
        end
    end

    // Handle read valids and last
    always_ff @(posedge clk) begin
        if(~rstn) begin
            rd_valid <= 1'b0;
            rd_last <= 1'b0;

        end else begin
            if(qspi_byte_count >= 1 + 3 + 3 && qspi_bit_count[0]) begin
                rd_valid <= 1'b1;
                rd_last <= qspi_byte_count - (1 + 3 + 3) + 1 >= rd_burst_len;
            end else begin
                rd_valid <= 1'b0;
                rd_last <= 1'b0;
            end
        end
    end

endmodule