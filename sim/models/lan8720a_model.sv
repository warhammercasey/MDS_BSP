`timescale 1ns/1ps

// Behavioral model of the Microchip LAN8720A/LAN8720Ai Ethernet PHY: the serial
// management interface (SMI: MDC/MDIO) and the register file behind it. The RMII
// data path and the line side are not modelled.
//
// Behavior per datasheet DS00002165C, and IEEE 802.3 clause 22, which its SMI
// follows (note 5-19):
//  - A frame is a preamble of 32 ones, start 01, opcode 10 (read) or 01 (write),
//    the PHY and register addresses (5 bits each), a turnaround, and 16 data bits.
//    Fields are MSB first. MDIO is sampled on MDC rising edges.
//  - Write: the STA drives turnaround 10, then the data. Read: the STA releases
//    MDIO for the turnaround; the PHY drives 0 in its second bit, then the data.
//    The PHY changes MDIO 0-300 ns (tval) after MDC rising edges, and releases it
//    the same way after the last data bit (the IDLE bit after a frame is high-Z).
//  - Only frames to this PHY's address are answered: the PHYAD field of register
//    18, which resets to the PHYAD0 strap.
//  - Registers 0-6, 17, 18, 26, 27 and 29-31 (Table 4-2), with their defaults for
//    MODE[2:0] = 111 (the strap default) and their writable bits. Unsupported
//    registers read FFFF and ignore writes (section 3.5). Soft reset (0.15) restores
//    the defaults except the NASR bits; it and restart auto-negotiate (0.9)
//    self-clear at once. There is no link, so status and interrupt bits stay at
//    their reset values.
//  - nRST restores every register and drops any frame in progress.
//  - Timing checks: MDC period >= 400 ns, high and low >= 160 ns; MDIO from the
//    STA set up and held >= 10 ns around MDC rising; nRST low >= 100 us.
//
// A frame with a short preamble, or a write turnaround other than 10, is reported
// and then handled as if it were valid, so one bug doesn't hide the others. A bad
// start or opcode drops the frame.
//
// The real MDIO pin is open drain (VOD8), with a pull-up for ones. Here the PHY
// drives its whole slot on `mdio_out` + `mdio_oe` instead, so the testbench can
// resolve the line and flag an STA that drives MDIO during the PHY's slot.
//
// Each well-formed frame, to any PHY address, is pushed to `frames` once its last
// data bit is sampled, for the testbench scoreboard to consume.
module lan8720a_model #(
    parameter bit       PHYAD0   = 1'b0, // PHYAD0 strap (RXER pin): SMI address 0 or 1
    parameter bit [3:0] REVISION = 4'd1  // PHY ID 2 revision number (varies with the silicon)
)(
    input  logic nrst,      // hardware reset, active low
    input  logic mdc,
    input  logic mdio,      // the resolved MDIO line
    output logic mdio_out,
    output logic mdio_oe
);
    import tb_pkg::*;

    localparam real T_CLKP_MIN_NS  = 400.0;
    localparam real T_CLKH_MIN_NS  = 160.0;
    localparam real T_CLKL_MIN_NS  = 160.0;
    localparam real T_VAL_MAX_NS   = 300.0;
    localparam real T_SU_MIN_NS    = 10.0;
    localparam real T_IHOLD_MIN_NS = 10.0;
    localparam real T_RSTIA_MIN_NS = 100.0e3;
    localparam int  PREAMBLE_BITS  = 32;

    typedef struct {
        bit          write;
        logic [4:0]  phyad;
        logic [4:0]  regad;
        logic [15:0] data;      // as written; as read, what this PHY sent (else the line)
        bit          answered;  // addressed to this PHY
        int unsigned preamble;  // ones before the start bits
        realtime     t_start;   // first start bit sampled
        realtime     t_end;     // last data bit sampled
    } frame_t;

    frame_t      frames[$];     // completed frames, consumed by the testbench
    int unsigned n_frames  = 0;
    int unsigned n_reads   = 0;
    int unsigned n_writes  = 0;
    int unsigned n_dropped = 0;
    real         mdc_min_period = 0.0;

    // Timing and protocol violations are reported once, then counted
    function automatic void violation(string kind, string msg);
        tb_error_once({"LAN8720A ", kind}, {"LAN8720A model: ", msg});
    endfunction

    function automatic void report();
        tb_info($sformatf("LAN8720A model: %0d frames (%0d reads, %0d writes), %0d dropped; MDC period min %.1f ns (%.2f MHz)",
                          n_frames, n_reads, n_writes, n_dropped, mdc_min_period,
                          mdc_min_period > 0.0 ? 1.0e3 / mdc_min_period : 0.0));
    endfunction

    // ------------------------------------------------------------------
    // Registers
    // ------------------------------------------------------------------
    logic [15:0] regs [32];

    function automatic logic [15:0] reg_default(int r);
        case (r)
            0:       return 16'h3100; // 100 Mbps, auto-negotiation enabled, full duplex (MODE 111: X10X)
            1:       return 16'h7809; // 10/100 half/full able, auto-negotiation able, no link
            2:       return 16'h0007; // PHY ID 1
            3:       return {6'b110000, 6'b001111, REVISION}; // PHY ID 2: C0Fx
            4:       return 16'h01E1; // advertise 10/100 half/full, IEEE 802.3
            5:       return 16'h0001; // no link partner
            6:       return 16'h0000;
            17:      return 16'h0002; // ENERGYON
            18:      return {1'b0, 1'b1, 6'b0, 3'b111, 4'b0, PHYAD0}; // MODE 111, PHYAD from the strap
            26:      return 16'h0000;
            27:      return 16'h0000;
            29:      return 16'h0000;
            30:      return 16'h0000;
            31:      return 16'h0040; // bits 11:5 reset to 0000010b
            default: return 16'hFFFF; // no register here
        endcase
    endfunction

    // Writable bits
    function automatic logic [15:0] reg_wmask(int r);
        case (r)
            0:       return 16'hFF00;
            2, 3:    return 16'hFFFF;
            4:       return 16'h2DFF;
            17:      return 16'h2241;
            18:      return 16'h40FF;
            27:      return 16'hA800;
            30:      return 16'h00FE;
            31:      return 16'h0FE0;
            default: return 16'h0000;
        endcase
    endfunction

    // Bits a soft reset leaves alone (NASR)
    function automatic logic [15:0] reg_nasr(int r);
        case (r)
            17:      return 16'h0002;
            18:      return 16'h40FF;
            27:      return 16'h0800;
            default: return 16'h0000;
        endcase
    endfunction

    function automatic void hw_reset();
        for (int r = 0; r < 32; r++)
            regs[r] = reg_default(r);
    endfunction

    function automatic void soft_reset();
        for (int r = 0; r < 32; r++)
            regs[r] = (regs[r] & reg_nasr(r)) | (reg_default(r) & ~reg_nasr(r));
    endfunction

    function automatic void write_reg(int r, logic [15:0] d);
        regs[r] = (regs[r] & ~reg_wmask(r)) | (d & reg_wmask(r));
        if (r == 0) begin
            if (d[15])
                soft_reset();
            regs[0][15] = 1'b0;  // self-clearing
            regs[0][9]  = 1'b0;
        end
    endfunction

    initial hw_reset();

    // ------------------------------------------------------------------
    // Frame receiver (all visible in waves)
    // ------------------------------------------------------------------
    typedef enum logic [2:0] { F_IDLE, F_START, F_OP, F_PHYAD, F_REGAD, F_TA, F_DATA } fstate_t;

    fstate_t     fstate     = F_IDLE;
    logic        in_frame;          // between the first start bit and the last data bit
    int unsigned ones       = 0;    // ones while idle: the preamble
    int unsigned nbits      = 0;    // bits of the current field
    logic [15:0] shift      = '0;
    logic        f_write    = 1'b0;
    logic [4:0]  f_phyad    = '0;
    logic [4:0]  f_regad    = '0;
    logic [15:0] f_data     = '0;
    logic        f_ours     = 1'b0; // addressed to this PHY
    int unsigned f_preamble = 0;
    realtime     f_t_start  = 0.0;
    logic        phy_slot   = 1'b0; // MDIO is the PHY's: read turnaround up to the edge after the data
    logic        slot_end   = 1'b0; // the PHY released MDIO after the last data bit

    assign in_frame = fstate != F_IDLE;

    // MDIO output. on_rise() requests a change, which lands tval after the edge.
    // Gated by phy_slot, which a reset clears, so a change still pending at a reset
    // can't drive the line afterwards.
    logic oe_q      = 1'b0;
    logic out_q     = 1'b1;
    logic drive_req = 1'b0;
    logic drive_en  = 1'b0;
    logic drive_val = 1'b1;

    assign mdio_oe  = oe_q && phy_slot && nrst;
    assign mdio_out = out_q;

    function automatic void drive(logic en, logic val);
        drive_req = 1'b1;
        drive_en  = en;
        drive_val = val;
    endfunction

    // 0, tval max, or in between
    function automatic real out_delay();
        case ($urandom_range(3))
            0:       return 0.0;
            1:       return T_VAL_MAX_NS;
            default: return real'($urandom_range(int'(T_VAL_MAX_NS)));
        endcase
    endfunction

    function automatic void end_frame();
        fstate = F_IDLE;
        ones   = 0;
        nbits  = 0;
    endfunction

    function automatic void log_frame();
        frame_t f;
        f.write    = f_write;
        f.phyad    = f_phyad;
        f.regad    = f_regad;
        f.data     = f_data;
        f.answered = f_ours;
        f.preamble = f_preamble;
        f.t_start  = f_t_start;
        f.t_end    = $realtime;
        frames.push_back(f);
        n_frames++;
        if (f_write)
            n_writes++;
        else
            n_reads++;
    endfunction

    // One bit, sampled on an MDC rising edge
    function automatic void on_rise(logic b);
        case (fstate)
            F_IDLE: begin
                if (b) begin
                    if (ones < 'hFFFF)
                        ones++;
                end else begin
                    // First start bit
                    if (ones < PREAMBLE_BITS)
                        violation("preamble", $sformatf("frame started after %0d preamble ones (%0d required); handled anyway",
                                                        ones, PREAMBLE_BITS));
                    f_preamble = ones;
                    f_t_start  = $realtime;
                    fstate     = F_START;
                end
            end

            F_START: begin
                if (b) begin
                    fstate = F_OP;
                    nbits  = 0;
                end else begin
                    violation("start", "start of frame was 00, not 01; frame dropped");
                    n_dropped++;
                    end_frame();
                end
            end

            F_OP: begin
                shift = {shift[14:0], b};
                nbits++;
                if (nbits == 2) begin
                    if (shift[1:0] == 2'b10 || shift[1:0] == 2'b01) begin
                        f_write = shift[1:0] == 2'b01;
                        fstate  = F_PHYAD;
                        nbits   = 0;
                    end else begin
                        violation("opcode", $sformatf("opcode %02b is neither read (10) nor write (01); frame dropped",
                                                      shift[1:0]));
                        n_dropped++;
                        end_frame();
                    end
                end
            end

            F_PHYAD, F_REGAD: begin
                shift = {shift[14:0], b};
                nbits++;
                if (nbits == 5) begin
                    nbits = 0;
                    if (fstate == F_PHYAD) begin
                        f_phyad = shift[4:0];
                        fstate  = F_REGAD;
                    end else begin
                        f_regad = shift[4:0];
                        f_ours  = f_phyad == regs[18][4:0];
                        if (!f_write && f_ours)
                            f_data = regs[f_regad];
                        fstate  = F_TA;
                    end
                end
            end

            F_TA: begin
                shift = {shift[14:0], b};
                nbits++;
                if (!f_write && f_ours) begin
                    // After the first turnaround bit, drive the second (0); after it, the MSB
                    if (nbits == 1) begin
                        phy_slot = 1'b1;
                        drive(1'b1, 1'b0);
                    end else begin
                        drive(1'b1, f_data[15]);
                    end
                end
                if (nbits == 2) begin
                    if (f_write && shift[1:0] != 2'b10)
                        violation("turnaround", $sformatf("write turnaround was %02b, not 10; handled anyway", shift[1:0]));
                    fstate = F_DATA;
                    nbits  = 0;
                end
            end

            F_DATA: begin
                shift = {shift[14:0], b};
                nbits++;
                if (!f_write && f_ours) begin
                    if (nbits < 16) begin
                        drive(1'b1, f_data[15 - nbits]);
                    end else begin
                        drive(1'b0, 1'b1);
                        slot_end = 1'b1;
                    end
                end
                if (nbits == 16) begin
                    if (f_write || !f_ours)
                        f_data = shift;
                    if (f_write && f_ours)
                        write_reg(int'(f_regad), f_data);
                    log_frame();
                    end_frame();
                end
            end

            default: end_frame();
        endcase
    endfunction

    // ------------------------------------------------------------------
    // MDC/MDIO, with timing checks
    // ------------------------------------------------------------------
    realtime t_rise      = -1.0e9;
    realtime t_fall      = -1.0e9;
    realtime t_mdio      = -1.0e9;
    realtime t_nrst_fall = -1.0e9;

    always @(posedge mdc) begin
        if (nrst) begin
            automatic realtime now = $realtime;

            if (now - t_rise < T_CLKP_MIN_NS)
                violation("tclkp", $sformatf("MDC period %.1f ns (min %.0f ns)", now - t_rise, T_CLKP_MIN_NS));
            if (now - t_fall < T_CLKL_MIN_NS)
                violation("tclkl", $sformatf("MDC low for %.1f ns (min %.0f ns)", now - t_fall, T_CLKL_MIN_NS));
            if (!phy_slot && now - t_mdio < T_SU_MIN_NS)
                violation("tsu", $sformatf("MDIO changed %.1f ns before MDC rose (setup min %.0f ns)",
                                           now - t_mdio, T_SU_MIN_NS));
            if (t_rise > 0.0 && (mdc_min_period == 0.0 || now - t_rise < mdc_min_period))
                mdc_min_period = now - t_rise;
            t_rise = now;

            if (slot_end) begin
                phy_slot = 1'b0;
                slot_end = 1'b0;
            end
            drive_req = 1'b0;
            on_rise(mdio);
            if (drive_req) begin
                automatic realtime d = out_delay();
                out_q <= #(d) drive_val;
                oe_q  <= #(d) drive_en;
            end
        end
    end

    always @(negedge mdc) begin
        if (nrst) begin
            automatic realtime now = $realtime;
            if (now - t_rise < T_CLKH_MIN_NS)
                violation("tclkh", $sformatf("MDC high for %.1f ns (min %.0f ns)", now - t_rise, T_CLKH_MIN_NS));
            if (now - t_fall < T_CLKP_MIN_NS)
                violation("tclkp", $sformatf("MDC period %.1f ns (min %.0f ns)", now - t_fall, T_CLKP_MIN_NS));
            t_fall = now;
        end
    end

    // Changes while the PHY owns MDIO are its own, so they aren't checked
    always @(mdio) begin
        if (nrst && !phy_slot && $realtime - t_rise < T_IHOLD_MIN_NS)
            violation("tihold", $sformatf("MDIO changed %.1f ns after MDC rose (hold min %.0f ns)",
                                          $realtime - t_rise, T_IHOLD_MIN_NS));
        t_mdio = $realtime;
    end

    always @(negedge nrst) begin
        t_nrst_fall = $realtime;
        hw_reset();
        end_frame();
        phy_slot = 1'b0;
        slot_end = 1'b0;
    end

    always @(posedge nrst) begin
        if ($realtime - t_nrst_fall < T_RSTIA_MIN_NS)
            violation("trstia", $sformatf("nRST low for %.1f us (min %.0f us)",
                                          ($realtime - t_nrst_fall) / 1.0e3, T_RSTIA_MIN_NS / 1.0e3));
        end_frame();
    end

endmodule
