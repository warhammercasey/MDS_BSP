`timescale 1ns/1ps
`include "tb_macros.svh"

// apb_cdc test: one bridge per ordered pair of the design's clock domains, each with
// an APB master in clka and an apb_reg_model (0-3 wait states) in clkb.
//
// Each master runs random transfers, back to back or with gaps:
//   - reads and writes to registers 00-BF: responses checked against a reference
//     copy of the registers, which must also match the model at the end
//   - transfers to D0-FF: the model's PSLVERR must come back
//   - transfers to C0-CF that the master abandons (drops psel before pready, as
//     uart_controller's timeout does). These run on the peripheral side once or not
//     at all, and must not disturb the transfers after them.
// Also checked:
//   - the peripheral side runs the requested transfers, in order, each once
//   - latency: pready within MAX_LAT_NS of the access phase (unless an abandoned
//     transfer or a reset is holding the bridge)
//   - reset order: in half the pairs clkb leaves reset well after clka, while the
//     first transfer is already waiting; in the rest clka leaves reset last
module apb_cdc_pair #(
    parameter real   FREQ_A = 100.0e6,
    parameter real   FREQ_B = 100.0e6,
    parameter string NAME   = "",
    parameter bit    B_LATE = 1'b1,   // clkb leaves reset after clka
    parameter int    N_XFER = 300
) (
    input  logic clka,
    input  logic clkb,
    output logic done
);
    import tb_pkg::*;

    localparam real TA = 1.0e9 / FREQ_A;
    localparam real TB = 1.0e9 / FREQ_B;

    localparam int SIZE     = 'hD0; // registers 00-CF, PSLVERR from D0
    localparam int AB_BASE  = 'hC0; // abandoned transfers use C0-CF, the rest 00-BF
    localparam int MAX_WAIT = 3;

    // Request sync (1 clka + 3 clkb), setup + access + wait states, ack sync
    // (1 clkb + 3 clka), plus a cycle of margin each side
    localparam real MAX_LAT_NS  = (MAX_WAIT + 8) * TB + 6 * TA;
    localparam int  LAT_CYCLES  = int'($ceil(MAX_LAT_NS / TA));

    logic rstna = 1'b0;
    logic rstnb = 1'b0;

    apb_if #(.ADDR_WIDTH(8), .DATA_WIDTH(8), .SEL_WIDTH(1)) apb_a ();
    apb_if #(.ADDR_WIDTH(8), .DATA_WIDTH(8), .SEL_WIDTH(1)) apb_b ();

    apb_cdc #(
        .ADDR_WIDTH(8),
        .DATA_WIDTH(8)
    ) dut (
        .clka(clka),
        .rstna(rstna),
        .apb_a(apb_a),
        .clkb(clkb),
        .rstnb(rstnb),
        .apb_b(apb_b)
    );

    apb_reg_model #(
        .ADDR_WIDTH(8),
        .SIZE(SIZE),
        .MAX_WAIT(MAX_WAIT)
    ) model (
        .clk(clkb),
        .rstn(rstnb),
        .apb(apb_b)
    );

    logic [7:0] ref_mem [AB_BASE];
    initial begin
        #1;
        for (int a = 0; a < AB_BASE; a++)
            ref_mem[a] = model.mem[a];
    end

    // ------------------------------------------------------------------
    // Master (clka)
    // ------------------------------------------------------------------
    typedef struct packed {
        logic              wr;
        logic [7:0]        addr;
        logic [7:0]        wdata;
        logic signed [15:0] abandon; // access cycles before dropping psel; -1: never
        logic [3:0]        gap;     // idle cycles after the transfer
    } xfer_t;

    xfer_t plan[$];  // still to run
    xfer_t sent[$];  // started, not yet seen on the peripheral side

    typedef enum { M_IDLE, M_SETUP, M_ACCESS } mst_t;
    mst_t        mst = M_IDLE;
    xfer_t       cur;
    int unsigned gap_left = 0;
    int unsigned waited;
    realtime     t_access;
    realtime     t_rstnb_up = -1.0;
    bit          prev_abandoned = 1'b0;

    int unsigned n_done = 0, n_abandoned = 0, n_err = 0;
    int unsigned n_ab_planned = 0;  // transfers set to be abandoned (some finish first)
    real         lat_max = 0.0;

    initial begin
        apb_a.psel[0] = 1'b0;
        apb_a.penable = 1'b0;
    end

    task automatic start();
        cur = plan.pop_front();
        sent.push_back(cur);
        apb_a.psel[0] <= 1'b1;
        apb_a.penable <= 1'b0;
        apb_a.paddr   <= cur.addr;
        apb_a.pwrite  <= cur.wr;
        apb_a.pwdata  <= cur.wdata;
        mst = M_SETUP;
    endtask

    task automatic finish();
        if (cur.gap == 0 && plan.size() > 0) begin
            start();  // back to back
        end else begin
            apb_a.psel[0] <= 1'b0;
            apb_a.penable <= 1'b0;
            gap_left = cur.gap > 0 ? int'(cur.gap) - 1 : 0;  // the next cycle is the first idle one
            mst = M_IDLE;
        end
    endtask

    task automatic check_response();
        automatic realtime lat = $realtime - t_access;
        n_done++;
        tb_checks++;
        if (int'(cur.addr) >= SIZE) begin
            n_err++;
            if (apb_a.pslverr !== 1'b1)
                tb_error($sformatf("%s: transfer to 0x%02h: no PSLVERR", NAME, cur.addr));
        end else if (apb_a.pslverr !== 1'b0) begin
            tb_error($sformatf("%s: transfer to 0x%02h: unexpected PSLVERR", NAME, cur.addr));
        end else if (int'(cur.addr) < AB_BASE) begin
            if (cur.wr)
                ref_mem[cur.addr] = cur.wdata;
            else if (apb_a.prdata !== ref_mem[cur.addr])
                tb_error($sformatf("%s: read 0x%02h returned 0x%02h, expected 0x%02h",
                                   NAME, cur.addr, apb_a.prdata, ref_mem[cur.addr]));
        end

        // Latency, unless an abandoned transfer or clkb's reset held this one up
        if (!prev_abandoned && t_rstnb_up >= 0 && t_access > t_rstnb_up) begin
            lat_max = lat > lat_max ? lat : lat_max;
            `CHECK(lat <= MAX_LAT_NS, $sformatf("%s: transfer to 0x%02h took %0.1f ns (max %0.1f ns)",
                                                NAME, cur.addr, lat, MAX_LAT_NS))
        end
        prev_abandoned = 1'b0;
    endtask

    always @(posedge clka) begin
        if (!rstna) begin
            mst = M_IDLE;
            apb_a.psel[0] <= 1'b0;
            apb_a.penable <= 1'b0;
        end else begin
            case (mst)
                M_IDLE: begin
                    if (gap_left > 0)
                        gap_left--;
                    else if (plan.size() > 0)
                        start();
                end

                M_SETUP: begin
                    apb_a.penable <= 1'b1;
                    waited   = 0;
                    t_access = $realtime;
                    mst      = M_ACCESS;
                end

                M_ACCESS: begin
                    if (apb_a.pready) begin
                        check_response();
                        finish();
                    end else if (cur.abandon >= 0 && waited >= cur.abandon) begin
                        n_abandoned++;
                        prev_abandoned = 1'b1;
                        if (cur.gap == 0)
                            cur.gap = 1;  // psel must drop for a cycle to abandon
                        finish();
                    end else begin
                        waited++;
                    end
                end

                default: ;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Peripheral side (clkb): transfers must be the requested ones, in order
    // ------------------------------------------------------------------
    int unsigned n_b = 0, n_ab_run = 0;

    function automatic bit same(xfer_t x, logic wr, logic [7:0] addr, logic [7:0] wdata);
        return x.wr == wr && x.addr == addr && (!wr || x.wdata == wdata);
    endfunction

    always @(posedge clkb) begin
        if (rstnb && apb_b.psel[0] && apb_b.penable && apb_b.pready) begin
            n_b++;
            // Abandoned transfers may be missing (dropped before they were sent)
            while (sent.size() > 0 && sent[0].abandon >= 0 &&
                   !same(sent[0], apb_b.pwrite, apb_b.paddr, apb_b.pwdata))
                void'(sent.pop_front());
            tb_checks++;
            if (sent.size() == 0 || !same(sent[0], apb_b.pwrite, apb_b.paddr, apb_b.pwdata)) begin
                tb_error_once({NAME, " order"}, $sformatf(
                    "%s: peripheral side ran %s 0x%02h, which is not the next requested transfer",
                    NAME, apb_b.pwrite ? "write to" : "read from", apb_b.paddr));
            end else begin
                if (sent[0].abandon >= 0)
                    n_ab_run++;
                void'(sent.pop_front());
            end
        end
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    // Abandoned transfers come in bursts of 1-3. The first is dropped at a random
    // point; the others are dropped within 2 cycles, mostly while the first is still
    // in flight, so they never reach the peripheral.
    function automatic void plan_xfers();
        while (plan.size() < N_XFER) begin
            xfer_t x;
            automatic int kind = $urandom_range(99);
            x.wr      = 1'($urandom);
            x.wdata   = 8'($urandom);
            x.abandon = -1;
            x.gap     = $urandom_range(3) == 0 ? 4'($urandom_range(6, 1)) : 4'd0;
            if (kind < 85) begin
                x.addr = kind < 75 ? 8'($urandom_range(AB_BASE - 1)) : 8'($urandom_range(255, SIZE));
                plan.push_back(x);
            end else begin
                automatic int burst = $urandom_range(3, 1);
                for (int b = 0; b < burst; b++) begin
                    x.wr      = 1'($urandom);
                    x.wdata   = 8'($urandom);
                    x.addr    = 8'($urandom_range(AB_BASE + 15, AB_BASE));
                    x.abandon = 16'(b == 0 ? $urandom_range(LAT_CYCLES) : $urandom_range(2));
                    plan.push_back(x);
                    n_ab_planned++;
                end
            end
        end
    endfunction

    initial begin
        automatic int unsigned n_planned;
        done = 1'b0;
        plan_xfers();
        n_planned = plan.size();

        // Leave reset one domain after the other. The master starts as soon as clka
        // is out of reset.
        if (B_LATE) begin
            repeat (8)  @(negedge clka);
            rstna = 1'b1;
            repeat (40) @(negedge clkb);
            rstnb = 1'b1;
        end else begin
            repeat (8)  @(negedge clkb);
            rstnb = 1'b1;
            repeat (40) @(negedge clka);
            rstna = 1'b1;
        end
        t_rstnb_up = $realtime;

        wait (plan.size() == 0 && mst == M_IDLE);
        repeat (int'($ceil(4 * MAX_LAT_NS / TA))) @(posedge clka);

        `CHECK(n_done + n_abandoned == n_planned, $sformatf("%s: %0d transfers finished of %0d", NAME, n_done + n_abandoned, n_planned))
        foreach (sent[i])
            `CHECK(sent[i].abandon >= 0, $sformatf("%s: transfer to 0x%02h never ran on the peripheral side", NAME, sent[i].addr))
        for (int a = 0; a < AB_BASE; a++)
            `CHECK_EQ(model.mem[a], ref_mem[a], $sformatf("%s: register 0x%02h at the end", NAME, a))

        // Of the transfers set to be abandoned: some finish before the master gives up
        // (counted as done), the rest are abandoned; those that never reach the
        // peripheral were dropped while an earlier abandoned transfer was in flight
        `CHECK(n_abandoned > 0 && n_ab_planned > n_ab_run,
               $sformatf("%s: abandoned transfers not exercised (%0d abandoned, %0d never ran)",
                         NAME, n_abandoned, n_ab_planned - n_ab_run))
        tb_info($sformatf("%-20s %0d done (%0d PSLVERR), %0d abandoned (%0d never ran); latency max %0.1f ns (limit %0.1f)",
                          NAME, n_done, n_err, n_abandoned, n_ab_planned - n_ab_run, lat_max, MAX_LAT_NS));
        done = 1'b1;
    end

endmodule


module tb_apb_cdc;
    import tb_pkg::*;
    import cain_clocks_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(5_000_000) // 5 ms

    logic clk [NUM_CLKS];
    for (genvar i = 0; i < NUM_CLKS; i++) begin : g_clk
        clk_gen #(.FREQ_HZ(FREQ_HZ[i])) gen (.clk(clk[i]));
    end

    logic [NUM_PAIRS-1:0] done;
    for (genvar p = 0; p < NUM_PAIRS; p++) begin : g_pair
        apb_cdc_pair #(
            .FREQ_A(FREQ_HZ[pair_a(p)]),
            .FREQ_B(FREQ_HZ[pair_b(p)]),
            .NAME(pair_name(p)),
            .B_LATE(p % 2 == 0)
        ) h (
            .clka(clk[pair_a(p)]),
            .clkb(clk[pair_b(p)]),
            .done(done[p])
        );
    end

    initial begin
        tb_info($sformatf("Testing apb_cdc on %0d clock pairs", NUM_PAIRS));
        wait (&done);
        tb_finish();
    end

endmodule
