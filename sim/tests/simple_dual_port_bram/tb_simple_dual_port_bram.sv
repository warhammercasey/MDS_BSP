`timescale 1ns/1ps
`include "tb_macros.svh"

// simple_dual_port_bram test: one RAM per ordered pair of the design's clock domains,
// port A (write) on clka and port B (read) on clkb.
//
// Reads are checked against a memory model: one cycle of latency, doutb holds
// while enb is low, and writes only happen when ena && wea. Idle cycles drive
// random addr/data/we so the enables are exercised.
//   1. A writes every address, then B reads them all back
//   2. A writes random addresses in one half while B reads the other half, then
//      the halves swap (a read of an address being written from the other clock
//      is undefined - Efinity's READ_UNKNOWN mode - so the halves never overlap)
//   3. B reads every address
module sdp_bram_pair #(
    parameter real   FREQ_A   = 100.0e6,
    parameter real   FREQ_B   = 100.0e6,
    parameter string NAME     = "",
    parameter int    WIDTH    = 21,
    parameter int    AWIDTH   = 5,
    parameter int    N_RANDOM = 200  // ops per port, per half, in phase 2
) (
    input  logic clka,
    input  logic clkb,
    output logic done
);
    import tb_pkg::*;

    localparam int DEPTH = 1 << AWIDTH;

    typedef enum { IDLE, ALL, RANDOM } mode_t;

    logic              ena = 1'b0, wea = 1'b0;
    logic [AWIDTH-1:0] addra;
    logic [WIDTH-1:0]  dina;
    logic              enb = 1'b0;
    logic [AWIDTH-1:0] addrb;
    logic [WIDTH-1:0]  doutb;

    simple_dual_port_bram #(
        .WIDTH(WIDTH),
        .AWIDTH(AWIDTH)
    ) dut (
        .clka(clka), .wea(wea), .ena(ena), .addra(addra), .dina(dina),
        .clkb(clkb), .enb(enb), .addrb(addrb), .doutb(doutb)
    );

    // Memory model
    logic [WIDTH-1:0] model [DEPTH];
    bit               known [DEPTH] = '{default: 1'b0};

    mode_t       mode_a = IDLE, mode_b = IDLE;
    bit          upper_a = 1'b0;    // RANDOM mode: A uses the upper half, B the lower (or swapped)
    int unsigned idx_a = 0, idx_b = 0;
    int unsigned n_writes = 0, n_reads = 0;

    function automatic logic [AWIDTH-1:0] half_addr(bit upper);
        logic [AWIDTH-1:0] a = AWIDTH'($urandom);
        a[AWIDTH-1] = upper;
        return a;
    endfunction

    // ------------------------------------------------------------------
    // Port A (clka): writes
    // ------------------------------------------------------------------
    always @(posedge clka) begin
        if (ena && wea) begin
            model[addra] = dina;
            known[addra] = 1'b1;
            n_writes++;
        end
    end

    always @(negedge clka) begin
        ena   = 1'b0;
        wea   = 1'($urandom);
        addra = AWIDTH'($urandom);
        dina  = WIDTH'({$urandom, $urandom});
        if (mode_a != IDLE) begin
            if ($urandom_range(3) != 0) begin
                // An enabled port A with wea low must not write either
                ena = 1'b1;
                wea = $urandom_range(7) != 0;
                addra = mode_a == ALL ? AWIDTH'(idx_a) : half_addr(upper_a);
                if (wea) begin
                    idx_a++;
                    if (idx_a == (mode_a == ALL ? DEPTH : N_RANDOM)) begin
                        mode_a = IDLE;
                        idx_a  = 0;
                    end
                end
            end else if (wea) begin
                // Disabled write to an address the other port may be reading: must have no effect
                addra = half_addr(!upper_a);
            end
        end
    end

    // ------------------------------------------------------------------
    // Port B (clkb): reads, checked against the model
    // ------------------------------------------------------------------
    logic [WIDTH-1:0] exp;
    bit               exp_valid = 1'b0;

    always @(posedge clkb) begin
        if (enb) begin
            exp_valid = known[addrb];
            exp       = model[addrb];
            n_reads++;
        end
    end

    always @(negedge clkb) begin
        if (exp_valid) begin
            tb_checks++;
            if (doutb !== exp)
                tb_error_once({NAME, " doutb"},
                    $sformatf("%s: doutb 0x%0h, expected 0x%0h", NAME, doutb, exp));
        end

        enb   = 1'b0;
        addrb = AWIDTH'($urandom);
        if (mode_b != IDLE && $urandom_range(3) != 0) begin
            enb   = 1'b1;
            addrb = mode_b == ALL ? AWIDTH'(idx_b) : half_addr(!upper_a);
            idx_b++;
            if (idx_b == (mode_b == ALL ? DEPTH : N_RANDOM)) begin
                mode_b = IDLE;
                idx_b  = 0;
            end
        end
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic run(mode_t a, mode_t b);
        mode_a = a;
        mode_b = b;
        wait (mode_a == IDLE && mode_b == IDLE);
        repeat (3) @(posedge clka);
        repeat (3) @(posedge clkb);
    endtask

    initial begin
        int unsigned reads_before;

        done = 1'b0;
        repeat (4) @(posedge clka);

        run(ALL, IDLE);
        reads_before = n_reads;
        run(IDLE, ALL);
        `CHECK_EQ(n_reads - reads_before, DEPTH, $sformatf("%s: reads of every address", NAME))

        upper_a = 1'b0;
        run(RANDOM, RANDOM);
        upper_a = 1'b1;
        run(RANDOM, RANDOM);
        run(IDLE, ALL);

        tb_info($sformatf("%-20s %0d writes, %0d reads", NAME, n_writes, n_reads));
        done = 1'b1;
    end

endmodule


module tb_simple_dual_port_bram;
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
        sdp_bram_pair #(
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
        tb_info($sformatf("Testing simple_dual_port_bram on %0d clock pairs", NUM_PAIRS));
        wait (&done);
        tb_finish();
    end

endmodule
