`timescale 1ns/1ps
`include "tb_macros.svh"

// eth_smi test: an APB master in the testbench, and a LAN8720A model on MDC/MDIO
// that logs every SMI frame it receives.
//
// Each APB transfer is checked against the frames the PHY saw while it ran:
//   - registers the LAN8720A has (0-6, 17, 18, 26, 27, 29-31): one frame, to PHY
//     address 0, with the transfer's opcode, register and write data. A read
//     returns the data the PHY sent. No PSLVERR.
//   - any other register: PSLVERR, and no frame
//   - pready within one frame time (32-bit preamble + 32 bits) of the access
//     phase, and not while a frame is still in progress
//   - no frames between transfers
// Reads are also checked against the datasheet's reset values, and against
// patterns written to a read/write register. The model checks the frame format
// and the SMI timing; the testbench flags MDIO contention (the FPGA driving MDIO
// during the PHY's slot of a read).
//
// The bus is 16-bit data and 5-bit addresses (eth_smi's defaults: one APB address
// per PHY register), on clk_io as cain_top's APB bus is. MDC_DIV is the smallest
// even divider that meets the LAN8720A's 400 ns minimum MDC period at that clock.
module tb_eth_smi;
    import tb_pkg::*;

    localparam int  CLK_FREQ      = 73_571_400; // cain_top CLK_IO_FREQ (the APB bus clock)
    localparam real CLK_PERIOD_NS = 1.0e9 / CLK_FREQ;
    localparam real MDC_MIN_NS    = 400.0;      // LAN8720A minimum MDC period (Table 5-13)
    localparam int  MDC_DIV       = 2 * int'($ceil(MDC_MIN_NS / (2.0 * CLK_PERIOD_NS)));

    localparam int ADDR_WIDTH = 5;
    localparam int DATA_WIDTH = 16;

    // Clock cycles from the access phase to pready: up to an MDC period to line up
    // with MDC, 64 bits (preamble and frame), and up to a period to finish
    localparam int MAX_LAT     = 66 * MDC_DIV + 4;
    localparam int APB_TIMEOUT = 4 * MAX_LAT;

    localparam real PHY_RESET_NS = 101.0e3;     // nRST low >= 100 us (Table 5-9)
    localparam int  GAP          = 2;           // idle cycles after directed transfers

    localparam int N_BURST  = 30;
    localparam int N_RANDOM = 300;

    // The LAN8720A's registers (Table 4-2)
    localparam int N_REGS = 14;
    localparam int REGS [N_REGS] = '{0, 1, 2, 3, 4, 5, 6, 17, 18, 26, 27, 29, 30, 31};

    // Every data bit gets a distinct code across these, so a stuck, swapped or
    // shifted bit shows up
    localparam logic [15:0] PATTERNS [10] = '{16'h0000, 16'hFFFF, 16'hAAAA, 16'h5555, 16'hCCCC,
                                              16'h3333, 16'hF0F0, 16'h0F0F, 16'hFF00, 16'h00FF};

    `TB_INIT
    `TB_WATCHDOG(50_000_000) // 50 ms

    // ------------------------------------------------------------------
    // DUT + PHY model
    // ------------------------------------------------------------------
    logic clk      = 1'b0;
    logic rstn     = 1'b0;
    logic phy_nrst = 1'b1;

    always #(CLK_PERIOD_NS/2) clk = ~clk;

    apb_if #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .SEL_WIDTH(1)) apb ();

    logic mdc;
    logic mdio;          // the MDIO line
    logic mdio_OUT;
    logic mdio_OE;
    logic phy_mdio_out;
    logic phy_mdio_oe;

    eth_smi #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .MDC_DIV(MDC_DIV)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .apb(apb),
        .mdc(mdc),
        .mdio_IN(mdio),
        .mdio_OUT(mdio_OUT),
        .mdio_OE(mdio_OE)
    );

    lan8720a_model phy (
        .nrst(phy_nrst),
        .mdc(mdc),
        .mdio(mdio),
        .mdio_out(phy_mdio_out),
        .mdio_oe(phy_mdio_oe)
    );

    // MDIO: whichever side drives it, else the pull-up
    assign mdio = mdio_OE ? mdio_OUT : phy_mdio_oe ? phy_mdio_out : 1'b1;

    always @(mdio_OE or phy_mdio_oe)
        if (mdio_OE && phy_mdio_oe)
            tb_error_once("mdio contention", {"MDIO contention: the FPGA drove MDIO while the PHY was sending ",
                                              "read data (turnaround bit 2 to D0, and up to 300 ns after the last MDC rising edge)"});

    // ------------------------------------------------------------------
    // Scoreboard
    // ------------------------------------------------------------------
    typedef struct packed {
        logic        wr;
        logic [4:0]  addr;
        logic [15:0] wdata;
        logic [15:0] exp_mask;  // read: bits to compare with exp_data
        logic [15:0] exp_data;
        logic [15:0] gap;       // idle cycles after the transfer; 0: the next follows back to back
    } xfer_t;

    int not_regs[$];            // addresses with no register

    function automatic bit is_reg(int a);
        for (int i = 0; i < N_REGS; i++)
            if (REGS[i] == a)
                return 1'b1;
        return 1'b0;
    endfunction

    initial
        for (int a = 0; a < 32; a++)
            if (!is_reg(a))
                not_regs.push_back(a);

    // The first failure of each kind is printed; tb_finish() counts the rest
    function automatic void check(bit ok, string kind, string msg);
        tb_checks++;
        if (!ok)
            tb_error_once(kind, msg);
    endfunction

    function automatic string xfer_str(xfer_t x);
        if (x.wr)
            return $sformatf("APB write 0x%04h to reg %0d", x.wdata, x.addr);
        return $sformatf("APB read of reg %0d", x.addr);
    endfunction

    function automatic string frame_str(int i);
        return $sformatf("%s reg %0d at PHY address %0d (data 0x%04h)",
                         phy.frames[i].write ? "write to" : "read of",
                         phy.frames[i].regad, phy.frames[i].phyad, phy.frames[i].data);
    endfunction

    // Frames that started before t, which no transfer asked for
    function automatic void drop_stray_frames(realtime t);
        while (phy.frames.size() > 0 && phy.frames[0].t_start < t) begin
            tb_checks++;
            tb_error_once("stray frame", $sformatf("SMI frame with no APB transfer running: %s, started at %.1f ns",
                                                   frame_str(0), phy.frames[0].t_start));
            void'(phy.frames.pop_front());
        end
    endfunction

    function automatic void drop_frames();
        while (phy.frames.size() > 0)
            void'(phy.frames.pop_front());
    endfunction

    // ------------------------------------------------------------------
    // APB master
    // ------------------------------------------------------------------
    xfer_t plan[$];  // still to run

    typedef enum { M_IDLE, M_SETUP, M_ACCESS } mst_t;
    mst_t        mst = M_IDLE;
    xfer_t       cur;
    int unsigned gap_left = 0;
    int unsigned waited;
    realtime     t_setup;

    int unsigned n_xfer  = 0;
    int unsigned n_noreg = 0;
    int unsigned n_cut   = 0;  // cut short by a reset
    int unsigned lat_max = 0;

    initial begin
        apb.psel[0] = 1'b0;
        apb.penable = 1'b0;
        apb.pwrite  = 1'b0;
        apb.paddr   = '0;
        apb.pwdata  = '0;
    end

    // pready seen: check the response and the frames the PHY received during the transfer
    function automatic void check_xfer();
        automatic string       what = xfer_str(cur);
        automatic int unsigned n;

        n_xfer++;
        drop_stray_frames(t_setup);
        n = phy.frames.size();

        lat_max = waited > lat_max ? waited : lat_max;
        check(waited <= MAX_LAT, "latency", $sformatf("%s: pready after %0d cycles (max %0d)", what, waited, MAX_LAT));
        check(!phy.in_frame, "pready mid-frame", $sformatf("%s: pready while the PHY was still receiving a frame", what));

        if (is_reg(int'(cur.addr))) begin
            check(apb.pslverr === 1'b0, "pslverr", $sformatf("%s: unexpected PSLVERR", what));
            check(n == 1, "frame count", $sformatf("%s: the PHY received %0d SMI frames, expected 1", what, n));
            if (n > 0) begin
                check(phy.frames[0].write == cur.wr && phy.frames[0].regad == cur.addr && phy.frames[0].phyad == 5'd0,
                      "frame", $sformatf("%s: the PHY received a %s", what, frame_str(0)));
                if (cur.wr)
                    check(phy.frames[0].data === cur.wdata, "write data",
                          $sformatf("%s: the PHY received 0x%04h", what, phy.frames[0].data));
                else
                    check(apb.prdata === phy.frames[0].data, "read data",
                          $sformatf("%s: prdata 0x%04h, but the PHY sent 0x%04h", what, apb.prdata, phy.frames[0].data));
            end
            if (!cur.wr && cur.exp_mask != '0)
                `CHECK_EQ(apb.prdata & cur.exp_mask, cur.exp_data,
                          $sformatf("%s: datasheet value (bits 0x%04h)", what, cur.exp_mask))
        end else begin
            n_noreg++;
            check(apb.pslverr === 1'b1, "no pslverr",
                  $sformatf("%s: no PSLVERR, but the LAN8720A has no register %0d", what, cur.addr));
            check(n == 0, "frame for no register",
                  $sformatf("%s: the PHY received %0d SMI frames, expected none", what, n));
        end

        // Any others also started during this transfer; the frame count covered them
        drop_frames();
    endfunction

    task automatic start();
        cur = plan.pop_front();
        t_setup = $realtime;
        apb.psel[0] <= 1'b1;
        apb.penable <= 1'b0;
        apb.paddr   <= cur.addr;
        apb.pwrite  <= cur.wr;
        apb.pwdata  <= cur.wdata;
        mst = M_SETUP;
    endtask

    task automatic finish(bit force_gap);
        if (cur.gap == 0 && !force_gap && plan.size() > 0) begin
            start();  // back to back
        end else begin
            apb.psel[0] <= 1'b0;
            apb.penable <= 1'b0;
            gap_left = cur.gap > 0 ? int'(cur.gap) - 1 : 0;  // the next cycle is the first idle one
            mst = M_IDLE;
        end
    endtask

    always @(posedge clk) begin
        if (!rstn) begin
            if (mst != M_IDLE)
                n_cut++;
            mst      = M_IDLE;
            gap_left = 0;
            apb.psel[0] <= 1'b0;
            apb.penable <= 1'b0;
        end else begin
            case (mst)
                M_IDLE: begin
                    if (gap_left > 0)
                        gap_left--;
                    else if (plan.size() > 0)
                        start();
                end

                M_SETUP: begin
                    apb.penable <= 1'b1;
                    waited = 0;
                    mst    = M_ACCESS;
                end

                M_ACCESS: begin
                    if (apb.pready) begin
                        check_xfer();
                        finish(1'b0);
                    end else if (waited >= APB_TIMEOUT) begin
                        n_xfer++;
                        tb_checks++;
                        tb_error_once("timeout", $sformatf("%s: no pready after %0d cycles; transfer abandoned",
                                                           xfer_str(cur), APB_TIMEOUT));
                        drop_frames();
                        finish(1'b1);
                    end else begin
                        waited++;
                    end
                end

                default: ;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Sequence helpers
    // ------------------------------------------------------------------
    function automatic void q_xfer(bit wr, int addr, logic [15:0] wdata, int gap,
                                   logic [15:0] exp_mask = '0, logic [15:0] exp_data = '0);
        xfer_t x;
        x.wr       = wr;
        x.addr     = 5'(addr);
        x.wdata    = wdata;
        x.exp_mask = exp_mask;
        x.exp_data = exp_data;
        x.gap      = 16'(gap);
        plan.push_back(x);
    endfunction

    // A read; the bits in `mask` must match `exp`
    function automatic void q_read(int addr, logic [15:0] exp = '0, logic [15:0] mask = '0);
        q_xfer(1'b0, addr, 16'h0000, GAP, mask, exp);
    endfunction

    function automatic void q_write(int addr, logic [15:0] data);
        q_xfer(1'b1, addr, data, GAP);
    endfunction

    // Idle cycles after a transfer: back to back, short, up to two MDC periods, or
    // long enough for the bus to settle
    function automatic int rand_gap();
        case ($urandom_range(9))
            0, 1, 2: return 0;
            3, 4, 5: return $urandom_range(8, 1);
            6, 7:    return $urandom_range(2 * MDC_DIV, 9);
            default: return $urandom_range(80 * MDC_DIV, 40 * MDC_DIV);
        endcase
    endfunction

    // A random read or write, mostly to registers that exist
    function automatic void q_random(int gap, bit regs_only = 1'b0);
        automatic bit          wr   = 1'($urandom);
        automatic logic [15:0] data = 16'($urandom);
        automatic int          addr;
        if (regs_only || $urandom_range(9) < 8)
            addr = REGS[$urandom_range(N_REGS - 1)];
        else
            addr = not_regs[$urandom_range(not_regs.size() - 1)];
        if (addr == 18)
            data[4:0] = 5'd0;  // PHYAD: keep the PHY at address 0, where eth_smi sends
        q_xfer(wr, addr, data, gap);
    endfunction

    task automatic run();
        wait (plan.size() == 0 && mst == M_IDLE && gap_left == 0);
    endtask

    // Reset the DUT, the APB master and the PHY together, holding them for nRST's
    // minimum time
    task automatic reset_all();
        @(negedge clk);
        rstn     = 1'b0;
        phy_nrst = 1'b0;
        plan.delete();
        repeat (int'($ceil(PHY_RESET_NS / CLK_PERIOD_NS))) @(negedge clk);
        drop_frames();  // the transfer cut short is gone
        rstn     = 1'b1;
        phy_nrst = 1'b1;
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    initial begin
        tb_info($sformatf("MDC_DIV %0d: MDC period %.1f ns", MDC_DIV, MDC_DIV * CLK_PERIOD_NS));
        repeat (20) @(negedge clk);
        rstn = 1'b1;

        // Straight after reset. Masks leave out reserved bits and those set by MODE (X).
        tb_info("Phase 1: every register's reset value (datasheet section 4.2)");
        q_read(0,  16'h1000, 16'hDE00);  // auto-negotiation on, not isolated (MODE 111)
        q_read(1,  16'h7809, 16'hFF3F);  // 10/100 half/full able, AN able, no link
        q_read(2,  16'h0007, 16'hFFFF);  // PHY ID 1
        q_read(3,  16'hC0F0, 16'hFFF0);  // PHY ID 2 (revision varies)
        q_read(4,  16'h01E1, 16'h2DFF);  // advertise 10/100 half/full, IEEE 802.3
        q_read(5,  16'h0001, 16'hE7FF);  // no link partner
        q_read(6,  16'h0000, 16'h001F);
        q_read(17, 16'h0002, 16'h2243);  // ENERGYON
        q_read(18, 16'h40E0, 16'h40FF);  // MODE 111, PHYAD 0
        q_read(26, 16'h0000, 16'hFFFF);  // no symbol errors
        q_read(27, 16'h0000, 16'hA810);
        q_read(29);                      // interrupt flags: INT7 may be set at power-up
        q_read(30, 16'h0000, 16'h00FE);
        q_read(31, 16'h0040, 16'h1FE0);
        run();

        tb_info("Phase 2: write and read back");
        // PHY ID 2 is read/write (4.2.4)
        foreach (PATTERNS[i]) begin
            q_write(3, PATTERNS[i]);
            q_read(3, PATTERNS[i], 16'hFFFF);
        end
        q_write(3, 16'hC0F1);
        // Read-only and reserved bits
        q_write(1,  16'hFFFF);
        q_read(1,   16'h7809, 16'hFF3F);  // basic status is read-only
        q_write(30, 16'hFFFF);
        q_read(30,  16'h00FE, 16'h00FF);  // interrupt mask: bits 7:1
        q_write(30, 16'h0000);
        q_read(30,  16'h0000, 16'h00FF);
        run();

        tb_info($sformatf("Phase 3: the %0d addresses with no register", not_regs.size()));
        foreach (not_regs[i]) begin
            q_read(not_regs[i]);
            q_write(not_regs[i], 16'($urandom));
        end
        run();

        tb_info($sformatf("Phase 4: %0d transfers back to back", N_BURST));
        for (int i = 0; i < N_BURST; i++)
            q_random(i < N_BURST - 1 ? 0 : GAP);
        run();

        tb_info($sformatf("Phase 5: %0d random transfers and gaps", N_RANDOM));
        repeat (N_RANDOM)
            q_random(rand_gap());
        run();

        tb_info("Phase 6: reset in the middle of a transfer");
        repeat (4) begin
            q_random(GAP, 1'b1);
            wait (mst == M_ACCESS);
            repeat ($urandom_range(60 * MDC_DIV)) @(negedge clk);
            reset_all();
            // Transfers straight after the reset
            q_read(2, 16'h0007, 16'hFFFF);  // the PHY was reset too
            repeat (4)
                q_random(rand_gap() % (4 * MDC_DIV));
            run();
        end

        tb_info("Idle: no frames without a transfer");
        repeat (3 * MAX_LAT) @(posedge clk);
        drop_stray_frames($realtime);
        `CHECK(!phy.in_frame, "the PHY is not in a frame at the end")

        tb_info($sformatf("%0d transfers (%0d to no register), %0d cut by a reset; pready after at most %0d cycles (limit %0d)",
                          n_xfer, n_noreg, n_cut, lat_max, MAX_LAT));
        phy.report();
        tb_finish();
    end

endmodule
