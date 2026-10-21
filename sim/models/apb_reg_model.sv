`timescale 1ns/1ps
`include "tb_macros.svh"

// APB peripheral with 8-bit registers, for testing APB masters and bridges.
//   - registers at 0..SIZE-1, random initial contents (read them through `mem`)
//   - PSLVERR at SIZE and above
//   - pready never rises at HANG_FROM and above, to test a master's timeout
//   - random wait states per transfer
// Checks the APB protocol: penable low in the setup phase, and paddr/pwrite/pwdata
// held through the access phase. A master may drop psel before pready only at a
// HANG_FROM address.
module apb_reg_model #(
    parameter int ADDR_WIDTH = 8,
    parameter int SIZE       = 16,               // registers at 0..SIZE-1, PSLVERR above
    parameter int MAX_WAIT   = 0,                // wait states: random 0..MAX_WAIT per transfer
    parameter int HANG_FROM  = 1 << ADDR_WIDTH   // from this address up, pready never rises
) (
    input logic clk,
    input logic rstn,
    apb_if.slv  apb
);
    import tb_pkg::*;

    logic [7:0] mem [SIZE];
    initial foreach (mem[a]) mem[a] = 8'($urandom);

    int unsigned           wait_left = 0;
    bit                    active = 1'b0; // between the setup phase and the end of the transfer
    logic [ADDR_WIDTH-1:0] t_addr;
    logic                  t_write;
    logic [7:0]            t_wdata;

    logic in_range, hang;
    assign in_range = int'(apb.paddr) < SIZE;
    assign hang     = int'(apb.paddr) >= HANG_FROM;

    assign apb.pready  = apb.psel[0] && apb.penable && wait_left == 0 && !hang;
    assign apb.pslverr = apb.pready && !in_range;
    assign apb.prdata  = (apb.pready && in_range) ? mem[int'(apb.paddr) % SIZE] : 8'h00;

    always @(posedge clk) begin
        if (!rstn) begin
            active     = 1'b0;
            wait_left <= 0;
        end else if (apb.psel[0]) begin
            if (!active) begin
                // Setup phase
                `CHECK_EQ(apb.penable, 1'b0, "APB: penable low in the setup phase")
                active     = 1'b1;
                t_addr     = apb.paddr;
                t_write    = apb.pwrite;
                t_wdata    = apb.pwdata;
                wait_left <= $urandom_range(MAX_WAIT, 0);
            end else begin
                // Access phase
                if (!apb.penable || apb.paddr !== t_addr || apb.pwrite !== t_write ||
                    (t_write && apb.pwdata !== t_wdata))
                    tb_error_once("apb access", $sformatf(
                        "%m: access phase to 0x%02h: penable=%b, or paddr/pwrite/pwdata changed since the setup phase",
                        t_addr, apb.penable));
                if (apb.pready) begin
                    if (apb.pwrite && in_range)
                        mem[int'(apb.paddr) % SIZE] <= apb.pwdata;
                    active = 1'b0;
                end else if (wait_left > 0) begin
                    wait_left <= wait_left - 1;
                end
            end
        end else if (active) begin
            // psel dropped before pready: only allowed where pready never rises
            `CHECK(int'(t_addr) >= HANG_FROM, $sformatf("%m: transfer to 0x%02h abandoned", t_addr))
            active = 1'b0;
        end
    end
endmodule
