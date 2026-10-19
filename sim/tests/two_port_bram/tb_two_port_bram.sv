`timescale 1ns/1ps
`include "tb_macros.svh"

// two_port_bram test: one RAM per ordered pair of the design's clock domains.
//
// Both ports are checked against a shared memory model: reads have one cycle of
// latency, return the old data when writing the same address (read-first), and
// dout holds while en is low. Ports issue operations on random cycles.
//   1. port A writes every address, then port B reads them all back
//   2. port B writes every address, then port A reads them all back
//   3. both ports do random reads/writes at once, A on even and B on odd
//      addresses (same-address access from both ports at once is undefined)
//   4. both ports read every address at once
module two_port_bram_pair #(
    parameter real   FREQ_A   = 100.0e6,
    parameter real   FREQ_B   = 100.0e6,
    parameter string NAME     = "",
    parameter int    WIDTH    = 16,
    parameter int    AWIDTH   = 5,
    parameter int    N_RANDOM = 300  // ops per port in phase 3
) (
    input  logic clka,
    input  logic clkb,
    output logic done
);
    import tb_pkg::*;

    localparam int DEPTH = 1 << AWIDTH;

    typedef enum { IDLE, WRITE_ALL, READ_ALL, RANDOM } mode_t;

    logic              clk  [2];
    logic              en   [2];
    logic              we   [2];
    logic [AWIDTH-1:0] addr [2];
    logic [WIDTH-1:0]  din  [2];
    logic [WIDTH-1:0]  dout [2];

    assign clk[0] = clka;
    assign clk[1] = clkb;

    two_port_bram #(
        .WIDTH(WIDTH),
        .AWIDTH(AWIDTH)
    ) dut (
        .clka(clka), .wea(we[0]), .ena(en[0]), .addra(addr[0]), .dina(din[0]), .douta(dout[0]),
        .clkb(clkb), .web(we[1]), .enb(en[1]), .addrb(addr[1]), .dinb(din[1]), .doutb(dout[1])
    );

    // Shared memory model
    logic [WIDTH-1:0] model [DEPTH];
    bit               known [DEPTH] = '{default: 1'b0};

    mode_t       mode [2] = '{IDLE, IDLE};
    int unsigned n_ops[2], n_reads[2], n_writes[2];

    function automatic string port_name(int p);
        return p == 0 ? "port A" : "port B";
    endfunction

    for (genvar p = 0; p < 2; p++) begin : g_port
        int unsigned      idx = 0;
        logic [WIDTH-1:0] exp;
        bit               exp_valid = 1'b0;

        initial begin
            en[p] = 1'b0;
            we[p] = 1'b0;
        end

        // Model: what this edge's operation should return, then apply its write
        always @(posedge clk[p]) begin
            if (en[p]) begin
                exp_valid = known[addr[p]];
                exp       = model[addr[p]];
                if (we[p]) begin
                    model[addr[p]] = din[p];
                    known[addr[p]] = 1'b1;
                    n_writes[p]++;
                end else begin
                    n_reads[p]++;
                end
            end
        end

        // Check last edge's result, then drive the next operation (on negedges, race-free)
        always @(negedge clk[p]) begin
            if (exp_valid) begin
                tb_checks++;
                if (dout[p] !== exp)
                    tb_error_once({NAME, " ", port_name(p)},
                        $sformatf("%s %s: dout 0x%0h, expected 0x%0h", NAME, port_name(p), dout[p], exp));
            end

            // Idle cycles have random addr/din/we so en gating is exercised too
            en[p]   = 1'b0;
            we[p]   = 1'($urandom);
            addr[p] = AWIDTH'($urandom);
            din[p]  = WIDTH'($urandom);
            if (mode[p] != IDLE && $urandom_range(3) != 0) begin
                en[p] = 1'b1;
                case (mode[p])
                    WRITE_ALL: begin we[p] = 1'b1; addr[p] = AWIDTH'(idx); end
                    READ_ALL:  begin we[p] = 1'b0; addr[p] = AWIDTH'(idx); end
                    RANDOM:    addr[p] = {addr[p][AWIDTH-1:1], 1'(p)}; // this port's addresses only
                    default: ;
                endcase
                idx++;
                n_ops[p]++;
                if (idx == (mode[p] == RANDOM ? N_RANDOM : DEPTH)) begin
                    mode[p] = IDLE;
                    idx     = 0;
                end
            end
        end
    end

    task automatic run(mode_t mode_a, mode_t mode_b);
        mode[0] = mode_a;
        mode[1] = mode_b;
        wait (mode[0] == IDLE && mode[1] == IDLE);
        repeat (3) @(posedge clka);
        repeat (3) @(posedge clkb);
    endtask

    initial begin
        done = 1'b0;
        repeat (4) @(posedge clka);

        run(WRITE_ALL, IDLE);
        run(IDLE,      READ_ALL);
        `CHECK_EQ(n_reads[1], DEPTH, $sformatf("%s: port B reads of port A's data", NAME))

        run(IDLE,      WRITE_ALL);
        run(READ_ALL,  IDLE);

        run(RANDOM,    RANDOM);
        run(READ_ALL,  READ_ALL);

        tb_info($sformatf("%-20s A: %0d writes, %0d reads   B: %0d writes, %0d reads",
                          NAME, n_writes[0], n_reads[0], n_writes[1], n_reads[1]));
        done = 1'b1;
    end

endmodule


module tb_two_port_bram;
    import tb_pkg::*;
    import cain_clocks_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(2_000_000) // 2 ms

    logic clk [NUM_CLKS];
    for (genvar i = 0; i < NUM_CLKS; i++) begin : g_clk
        clk_gen #(.FREQ_HZ(FREQ_HZ[i])) gen (.clk(clk[i]));
    end

    logic [NUM_PAIRS-1:0] done;
    for (genvar p = 0; p < NUM_PAIRS; p++) begin : g_pair
        two_port_bram_pair #(
            .FREQ_A(FREQ_HZ[pair_a(p)]),
            .FREQ_B(FREQ_HZ[pair_b(p)]),
            .NAME(pair_name(p))
        ) h (
            .clka(clk[pair_a(p)]),
            .clkb(clk[pair_b(p)]),
            .done(done[p])
        );
    end

    initial begin
        tb_info($sformatf("Testing two_port_bram on %0d clock pairs", NUM_PAIRS));
        wait (&done);
        tb_finish();
    end

endmodule
