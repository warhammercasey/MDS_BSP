`timescale 1ns/1ps
`include "tb_macros.svh"

// apb_width_conv test: several width combinations, each converter in front of an
// apb_reg_model (0-2 wait states, PSLVERR above SIZE) with the downstream widths.
//
// A master runs random reads and writes. Addresses and write data are sometimes
// limited to what fits downstream and sometimes use the full upstream width, so
// truncation errors come up whenever the downstream bus is narrower. Per transfer:
//   - address or write data that doesn't fit: PSLVERR in the first access cycle,
//     and nothing happens downstream
//   - otherwise exactly one downstream transfer, with the address and write data
//     zero padded/truncated, and the model's PSLVERR passed through
//   - reads: data checked against a reference copy of the registers, and PSLVERR
//     when the register holds bits that don't fit upstream
// At the end the model's registers must match the reference.
module apb_width_conv_inst #(
    parameter int IA     = 8,   // upstream address/data widths
    parameter int ID     = 8,
    parameter int OA     = 8,   // downstream address/data widths
    parameter int OD     = 8,
    parameter int N_XFER = 400
) (
    input  logic clk,
    input  logic rstn,
    output logic done
);
    import tb_pkg::*;

    localparam int MIN_A = IA < OA ? IA : OA;
    localparam int MIN_D = ID < OD ? ID : OD;
    // Registers the model has: fewer than downstream addresses, so its PSLVERR comes up
    localparam int SIZE  = (1 << OA) > 48 ? 48 : (1 << OA) - 2;

    string NAME = $sformatf("%0d/%0d -> %0d/%0d", IA, ID, OA, OD);

    apb_if #(.ADDR_WIDTH(IA), .DATA_WIDTH(ID), .SEL_WIDTH(1)) apb_in ();
    apb_if #(.ADDR_WIDTH(OA), .DATA_WIDTH(OD), .SEL_WIDTH(1)) apb_out ();

    apb_width_conv #(
        .IN_ADDR_WIDTH(IA),
        .IN_DATA_WIDTH(ID),
        .OUT_ADDR_WIDTH(OA),
        .OUT_DATA_WIDTH(OD)
    ) dut (
        .apb_in(apb_in),
        .apb_out(apb_out)
    );

    apb_reg_model #(
        .ADDR_WIDTH(OA),
        .DATA_WIDTH(OD),
        .SIZE(SIZE),
        .MAX_WAIT(2)
    ) model (
        .clk(clk),
        .rstn(rstn),
        .apb(apb_out)
    );

    logic [63:0] ref_mem [SIZE];
    initial begin
        #1;
        foreach (ref_mem[a])
            ref_mem[a] = 64'(model.mem[a]);
    end

    function automatic logic [63:0] rand_bits(int w);
        logic [63:0] v = {$urandom, $urandom};
        return w >= 64 ? v : v & ((64'd1 << w) - 1);
    endfunction

    // ------------------------------------------------------------------
    // Downstream: count transfers, check the converted address and data
    // ------------------------------------------------------------------
    logic        cur_wr;
    logic [63:0] cur_addr, cur_wdata;
    int unsigned n_down = 0;

    always @(posedge clk) begin
        if (rstn && apb_out.psel[0] && !apb_out.penable) begin
            n_down++;
            tb_checks++;
            if (64'(apb_out.paddr) != cur_addr || apb_out.pwrite != cur_wr ||
                (cur_wr && 64'(apb_out.pwdata) != cur_wdata))
                tb_error_once({NAME, " down"}, $sformatf(
                    "%s: downstream got %s 0x%0h data 0x%0h, sent %s 0x%0h data 0x%0h", NAME,
                    apb_out.pwrite ? "write" : "read", apb_out.paddr, apb_out.pwdata,
                    cur_wr ? "write" : "read", cur_addr, cur_wdata));
        end
    end

    // ------------------------------------------------------------------
    // Master
    // ------------------------------------------------------------------
    typedef enum { M_IDLE, M_SETUP, M_ACCESS } mst_t;
    mst_t        mst = M_IDLE;
    int unsigned n_issued = 0, waited = 0, down0 = 0;
    int unsigned n_ok = 0, n_trunc = 0, n_slverr = 0, n_rd_wide = 0;

    initial begin
        apb_in.psel[0] = 1'b0;
        apb_in.penable = 1'b0;
    end

    always @(posedge clk) begin
        if (!rstn) begin
            mst = M_IDLE;
            apb_in.psel[0] <= 1'b0;
            apb_in.penable <= 1'b0;
        end else begin
            case (mst)
                M_IDLE: begin
                    if (n_issued < N_XFER) begin
                        automatic int r = $urandom_range(99);
                        cur_wr    = 1'($urandom);
                        cur_addr  = r < 50 ? 64'($urandom_range(SIZE - 1)) :
                                    r < 70 ? rand_bits(MIN_A) : rand_bits(IA);
                        cur_wdata = $urandom_range(1) == 1 ? rand_bits(MIN_D) : rand_bits(ID);
                        apb_in.psel[0] <= 1'b1;
                        apb_in.penable <= 1'b0;
                        apb_in.paddr   <= IA'(cur_addr);
                        apb_in.pwrite  <= cur_wr;
                        apb_in.pwdata  <= ID'(cur_wdata);
                        down0 = n_down;
                        n_issued++;
                        mst = M_SETUP;
                    end
                end

                M_SETUP: begin
                    apb_in.penable <= 1'b1;
                    waited = 0;
                    mst    = M_ACCESS;
                end

                M_ACCESS: begin
                    if (apb_in.pready) begin
                        automatic bit too_wide = (cur_addr >> OA) != 0 || (cur_wr && (cur_wdata >> OD) != 0);
                        tb_checks++;
                        if (too_wide) begin
                            // Rejected by the converter
                            n_trunc++;
                            if (apb_in.pslverr !== 1'b1 || waited != 0 || n_down != down0)
                                tb_error($sformatf("%s: %s 0x%0h data 0x%0h doesn't fit: pslverr=%b after %0d wait cycles, %0d downstream transfers (expected PSLVERR at once, none downstream)",
                                                   NAME, cur_wr ? "write" : "read", cur_addr, cur_wdata, apb_in.pslverr, waited, n_down - down0));
                        end else if (n_down != down0 + 1) begin
                            tb_error($sformatf("%s: %0d downstream transfers for 0x%0h (expected 1)", NAME, n_down - down0, cur_addr));
                        end else if (cur_addr >= 64'(SIZE)) begin
                            // Model's PSLVERR, passed through
                            n_slverr++;
                            if (apb_in.pslverr !== 1'b1)
                                tb_error($sformatf("%s: 0x%0h: model's PSLVERR lost", NAME, cur_addr));
                        end else if (cur_wr) begin
                            n_ok++;
                            ref_mem[int'(cur_addr)] = cur_wdata;
                            if (apb_in.pslverr !== 1'b0)
                                tb_error($sformatf("%s: write 0x%0h: unexpected PSLVERR", NAME, cur_addr));
                        end else if ((ref_mem[int'(cur_addr)] >> ID) != 0) begin
                            // Register holds bits the upstream bus can't carry
                            n_rd_wide++;
                            if (apb_in.pslverr !== 1'b1)
                                tb_error($sformatf("%s: read 0x%0h = 0x%0h doesn't fit %0d bits: no PSLVERR",
                                                   NAME, cur_addr, ref_mem[int'(cur_addr)], ID));
                        end else begin
                            n_ok++;
                            if (apb_in.pslverr !== 1'b0 || 64'(apb_in.prdata) != ref_mem[int'(cur_addr)])
                                tb_error($sformatf("%s: read 0x%0h returned 0x%0h pslverr=%b, expected 0x%0h",
                                                   NAME, cur_addr, apb_in.prdata, apb_in.pslverr, ref_mem[int'(cur_addr)]));
                        end
                        apb_in.psel[0] <= 1'b0;
                        apb_in.penable <= 1'b0;
                        mst = M_IDLE;
                    end else begin
                        waited++;
                        if (waited > 20)
                            tb_error_once({NAME, " hang"}, $sformatf("%s: no pready after 20 cycles", NAME));
                    end
                end

                default: ;
            endcase
        end
    end

    initial begin
        done = 1'b0;
        wait (n_issued == N_XFER && mst == M_IDLE);
        repeat (5) @(posedge clk);

        foreach (ref_mem[a])
            `CHECK_EQ(64'(model.mem[a]), ref_mem[a], $sformatf("%s: register 0x%0h at the end", NAME, a))

        // Every case this configuration can produce came up
        `CHECK(n_ok > 0 && n_slverr > 0, $sformatf("%s: %0d ok, %0d PSLVERR from the model", NAME, n_ok, n_slverr))
        if (IA > OA || ID > OD)
            `CHECK(n_trunc > 0, $sformatf("%s: no transfer was rejected for truncation", NAME))
        else
            `CHECK_EQ(n_trunc, 0, $sformatf("%s: rejected transfers with no truncation", NAME))
        if (OD > ID)
            `CHECK(n_rd_wide > 0, $sformatf("%s: no read returned data too wide for upstream", NAME))

        tb_info($sformatf("%-16s %0d ok, %0d rejected (truncation), %0d model PSLVERR, %0d reads too wide",
                          NAME, n_ok, n_trunc, n_slverr, n_rd_wide));
        done = 1'b1;
    end

endmodule


module tb_apb_width_conv;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(1_000_000) // 1 ms

    logic clk  = 1'b0;
    logic rstn = 1'b0;
    always #(5.0) clk = ~clk;

    // Configurations: same widths; both padded; both truncated; address padded and
    // data truncated; address truncated and data padded
    localparam int NUM_CONFIGS = 5;
    localparam int IA_LIST [NUM_CONFIGS] = '{ 8,  8, 12,  6, 16};
    localparam int ID_LIST [NUM_CONFIGS] = '{ 8,  8, 16, 16,  8};
    localparam int OA_LIST [NUM_CONFIGS] = '{ 8, 12,  6, 10,  4};
    localparam int OD_LIST [NUM_CONFIGS] = '{ 8, 16,  8,  8, 32};

    logic [NUM_CONFIGS-1:0] done;
    for (genvar c = 0; c < NUM_CONFIGS; c++) begin : g_cfg
        apb_width_conv_inst #(
            .IA(IA_LIST[c]),
            .ID(ID_LIST[c]),
            .OA(OA_LIST[c]),
            .OD(OD_LIST[c])
        ) h (
            .clk(clk),
            .rstn(rstn),
            .done(done[c])
        );
    end

    initial begin
        tb_info($sformatf("Testing apb_width_conv in %0d width configurations", NUM_CONFIGS));
        repeat (5) @(negedge clk);
        rstn = 1'b1;
        wait (&done);
        tb_finish();
    end

endmodule
