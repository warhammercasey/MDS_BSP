`timescale 1ns/1ps
`include "tb_macros.svh"

// rmii_rx test. A PHY model drives RMII at 100 Mbit/s the way the LAN8720A does
// (datasheet 3.4.1.1): carrier with 00 dibits until locked, preamble, SFD, data LSB
// dibit first, and optionally CRS_DV toggling as the carrier ends ahead of the data.
// After each frame and its gap, the bytes that came out of axis_out are checked:
//   - good frame: exactly the frame, tlast on the last byte only
//   - RXER in the data: a prefix of the frame ending in tlast, shorter than the frame
//     and no longer than the bytes completed before the error. Once two bytes are in
//     (the receiver holds one back), the prefix may not be empty.
//   - noise (false carrier, bad SSD, no SFD, empty frame): nothing
// The sink checks tvalid/tdata/tlast hold while stalled.
// Phases:
//   directed      one of each case above, and RXER before the SFD (ignored)
//   random        random frames, errors, toggling, dribble nibbles; sink always ready
//   backpressure  same, sink ready 50% but never stalled more than 2 cycles
//   overflow      sink stalls 8-30 cycles at a time: frames come out whole or cut
//                 short with tlast, never corrupted, and later frames are unaffected
//   reset         reset released mid-frame: that frame is ignored, the next is fine
module tb_rmii_rx;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(50_000_000) // 50 ms

    localparam real CLK_PERIOD_NS = 20.0; // 50 MHz RMII REF_CLK

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    logic       rstn = 1'b0;
    logic       crs_dv = 1'b0;
    logic [1:0] rxd = 2'b00;
    logic       rxerr = 1'b0;

    axis_tlast_if #(.WIDTH(8)) axis_out ();

    rmii_rx dut (
        .clk(clk),
        .rstn(rstn),
        .rmii_crs_dv(crs_dv),
        .rmii_rxd(rxd),
        .rmii_rxerr(rxerr),
        .axis_out(axis_out)
    );

    // ------------------------------------------------------------------
    // Sink: tready pattern, stability check, and collects what comes out
    // ------------------------------------------------------------------
    int unsigned ready_pct  = 100; // chance of tready each cycle...
    int unsigned max_stall  = 0;   // ...but never low more than this many cycles in a row
    int unsigned burst_pct  = 0;   // per mille chance per cycle of starting a long stall
    int unsigned burst_min  = 0, burst_max = 0;

    int unsigned stall_run  = 0;
    int unsigned burst_left = 0;

    byte unsigned got[$];
    bit           got_last[$];

    bit         stalled = 1'b0;
    logic [7:0] stalled_data;
    logic       stalled_last;

    always @(posedge clk) begin
        bit r;
        if (rstn) begin
            if (stalled) begin
                tb_checks++;
                if (axis_out.tvalid !== 1'b1 || axis_out.tdata !== stalled_data || axis_out.tlast !== stalled_last)
                    tb_error_once("stability", $sformatf("axis_out changed while stalled: tvalid=%b tdata 0x%02h -> 0x%02h tlast %b -> %b",
                                                         axis_out.tvalid, stalled_data, axis_out.tdata, stalled_last, axis_out.tlast));
            end
            if (axis_out.tvalid && axis_out.tready) begin
                got.push_back(axis_out.tdata);
                got_last.push_back(axis_out.tlast);
            end
            stalled      = axis_out.tvalid && !axis_out.tready;
            stalled_data = axis_out.tdata;
            stalled_last = axis_out.tlast;
        end else begin
            stalled = 1'b0;
        end

        if (burst_left > 0) begin
            burst_left--;
            r = 1'b0;
        end else if (burst_pct > 0 && $urandom_range(999) < burst_pct) begin
            burst_left = $urandom_range(burst_max, burst_min) - 1;
            r = 1'b0;
        end else begin
            r = $urandom_range(99) < ready_pct || stall_run >= max_stall;
        end
        stall_run = r ? 0 : stall_run + 1;
        axis_out.tready <= r;
    end

    // ------------------------------------------------------------------
    // PHY model
    // ------------------------------------------------------------------
    // One dibit: set up after a rising edge, sampled on the next one
    task automatic drive(logic dv, logic [1:0] d, logic er = 1'b0);
        crs_dv <= dv;
        rxd    <= d;
        rxerr  <= er;
        @(posedge clk);
    endtask

    task automatic idle(int unsigned n);
        repeat (n) drive(1'b0, 2'b00);
    endtask

    // A frame as the PHY presents it.
    //   err_dibit  index into the data dibits where RXER starts (-1: none); RXER lasts
    //              err_len dibits, with garbage on RXD
    //   toggle     data nibbles at the end sent after the carrier dropped (CRS_DV toggling)
    //   dribble    an extra nibble after the last byte
    //   pre_zero   00 dibits before the preamble (PHY not yet locked)
    //   pre_len    preamble dibits (01) before the SFD
    //   pre_err    RXER on this preamble dibit (-1: none)
    task automatic phy_frame(byte unsigned data[$], int err_dibit = -1, int err_len = 1,
                             int toggle = 0, bit dribble = 1'b0,
                             int pre_zero = 4, int pre_len = 28, int pre_err = -1);
        logic [1:0] d[$];
        int n_nib;

        d = {}; // Needed: in this simulator, an automatic queue can keep its contents between calls from another task
        repeat (pre_zero) drive(1'b1, 2'b00);
        for (int i = 0; i < pre_len; i++) drive(1'b1, 2'b01, i == pre_err);
        drive(1'b1, 2'b01); drive(1'b1, 2'b01); drive(1'b1, 2'b01); drive(1'b1, 2'b11); // SFD

        foreach (data[i])
            for (int k = 0; k < 4; k++) d.push_back(data[i][2*k +: 2]);
        if (dribble) begin
            d.push_back(2'($urandom));
            d.push_back(2'($urandom));
        end
        n_nib = d.size() / 2;

        foreach (d[i]) begin
            automatic logic dv = (i / 2 >= n_nib - toggle) ? logic'(i % 2) : 1'b1;
            automatic bit   er = err_dibit >= 0 && i >= err_dibit && i < err_dibit + err_len;
            drive(dv, er ? 2'($urandom) : d[i], er);
        end
        drive(1'b0, 2'b00);
    endtask

    // Carrier that never reaches an SFD: false carrier (RXD 10 with RXER), or a bad
    // SSD (PHY drives nibble 1110 with RXER, then the carrier goes)
    task automatic phy_false_carrier(int unsigned n);
        repeat (n) drive(1'b1, 2'b10, 1'b1);
        drive(1'b0, 2'b00);
    endtask

    task automatic phy_bad_ssd();
        drive(1'b1, 2'b00); drive(1'b1, 2'b00);
        drive(1'b1, 2'b10, 1'b1); drive(1'b1, 2'b11, 1'b1);
        repeat (6) drive(1'b1, 2'b01);
        drive(1'b0, 2'b00);
    endtask

    // ------------------------------------------------------------------
    // Checking
    // ------------------------------------------------------------------
    int unsigned n_frames = 0, n_exact = 0, n_cut = 0, n_dropped = 0;

    function automatic void rand_frame(output byte unsigned data[$], input int unsigned len);
        data = {};
        repeat (len) data.push_back(8'($urandom));
    endfunction

    // tlast on the final beat and nowhere else
    function automatic void check_tlast(string name);
        foreach (got_last[i]) begin
            tb_checks++;
            if (got_last[i] !== (i == got_last.size() - 1))
                tb_error($sformatf("%s: tlast=%b on byte %0d of %0d", name, got_last[i], i, got_last.size()));
        end
    endfunction

    function automatic bit is_prefix(byte unsigned data[$]);
        if (got.size() > data.size()) return 1'b0;
        foreach (got[i]) if (got[i] !== data[i]) return 1'b0;
        return 1'b1;
    endfunction

    function automatic string show(byte unsigned q[$]);
        string s = "";
        foreach (q[i]) begin
            if (i == 8) return {s, " ..."};
            s = {s, $sformatf(" %02h", q[i])};
        end
        return s;
    endfunction

    // The whole frame, unless cut short is allowed (overflow phase)
    function automatic void check_good(string name, byte unsigned data[$], bit allow_cut = 1'b0);
        n_frames++;
        check_tlast(name);
        tb_checks++;
        if (got.size() == data.size() && is_prefix(data)) begin
            n_exact++;
        end else if (allow_cut && is_prefix(data)) begin
            if (got.size() == 0) n_dropped++; else n_cut++;
        end else begin
            tb_error($sformatf("%s: got %0d bytes {%s }, expected %0d bytes {%s }",
                               name, got.size(), show(got), data.size(), show(data)));
        end
        got = {};
        got_last = {};
    endfunction

    // RXER at data dibit err_dibit: cut short with tlast, or nothing if too early
    function automatic void check_error(string name, byte unsigned data[$], int err_dibit);
        int unsigned b = err_dibit / 4; // bytes complete before the error
        n_frames++;
        check_tlast(name);
        tb_checks++;
        if (!is_prefix(data) || got.size() > b || got.size() >= data.size())
            tb_error($sformatf("%s: RXER after %0d of %0d bytes, got %0d bytes {%s } (expected a shorter prefix of {%s })",
                               name, b, data.size(), got.size(), show(got), show(data)));
        else if (b >= 2 && got.size() == 0)
            tb_error($sformatf("%s: RXER after %0d bytes, but the frame wasn't ended with tlast", name, b));
        if (got.size() == 0) n_dropped++; else n_cut++;
        got = {};
        got_last = {};
    endfunction

    function automatic void check_nothing(string name);
        n_frames++;
        `CHECK(got.size() == 0, $sformatf("%s: expected no output, got %0d bytes {%s }", name, got.size(), show(got)))
        got = {};
        got_last = {};
    endfunction

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    // Random frame with random PHY behaviour; gap after it, then check
    task automatic random_frame(string name, int unsigned gap_min, int unsigned gap_max,
                                int unsigned err_pct, bit allow_cut = 1'b0);
        byte unsigned data[$];
        int unsigned  len    = $urandom_range(9) == 0 ? $urandom_range(1518, 60) : $urandom_range(64, 1);
        int           err    = $urandom_range(99) < err_pct ? int'($urandom_range(4 * len - 1)) : -1;
        int           toggle = $urandom_range(4);
        bit           drib   = $urandom_range(9) == 0;
        int           pzero  = 2 * $urandom_range(5);
        int           plen   = 2 * $urandom_range(14, 1);

        rand_frame(data, len);
        if (toggle > 2 * len) toggle = 2 * len;
        phy_frame(data, err, $urandom_range(3, 1), toggle, drib, pzero, plen);
        idle($urandom_range(gap_max, gap_min));
        if (err >= 0) check_error(name, data, err);
        else          check_good(name, data, allow_cut);
    endtask

    initial begin
        byte unsigned data[$];

        tb_info("Testing rmii_rx");
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        repeat (3) @(posedge clk);
        `CHECK_EQ(axis_out.tvalid, 1'b0, "tvalid low after reset")

        // --- Directed ---
        tb_info("Directed cases");
        rand_frame(data, 64);  phy_frame(data);                     idle(48); check_good("64 byte frame", data);
        rand_frame(data, 1);   phy_frame(data);                     idle(48); check_good("1 byte frame", data);
        rand_frame(data, 2);   phy_frame(data);                     idle(48); check_good("2 byte frame", data);
        rand_frame(data, 60);  phy_frame(data, .toggle(4));         idle(48); check_good("CRS_DV toggling, 4 nibbles", data);
        rand_frame(data, 1);   phy_frame(data, .toggle(2));         idle(48); check_good("1 byte, all data after carrier drop", data);
        rand_frame(data, 30);  phy_frame(data, .dribble(1));        idle(48); check_good("dribble nibble", data);
        rand_frame(data, 30);  phy_frame(data, .dribble(1), .toggle(3)); idle(48); check_good("dribble nibble, toggling", data);
        rand_frame(data, 40);  phy_frame(data, .pre_err(10));       idle(48); check_good("RXER in preamble", data);
        rand_frame(data, 40);  phy_frame(data, .pre_zero(0), .pre_len(2)); idle(48); check_good("short preamble", data);
        rand_frame(data, 0);   phy_frame(data);                     idle(48); check_nothing("SFD with no data");
        rand_frame(data, 64);  phy_frame(data, .err_dibit(2));      idle(48); check_error("RXER in byte 0", data, 2);
        rand_frame(data, 64);  phy_frame(data, .err_dibit(4));      idle(48); check_error("RXER in byte 1", data, 4);
        rand_frame(data, 64);  phy_frame(data, .err_dibit(8));      idle(48); check_error("RXER in byte 2", data, 8);
        rand_frame(data, 64);  phy_frame(data, .err_dibit(150), .err_len(3)); idle(48); check_error("RXER mid-frame", data, 150);
        rand_frame(data, 64);  phy_frame(data, .err_dibit(255), .toggle(2));  idle(48); check_error("RXER on the last dibit", data, 255);
        phy_false_carrier(20);                                      idle(48); check_nothing("false carrier");
        phy_bad_ssd();                                              idle(48); check_nothing("bad SSD");
        repeat (16) drive(1'b1, 2'b01); drive(1'b0, 2'b00);         idle(48); check_nothing("preamble, no SFD");
        rand_frame(data, 64);  phy_frame(data);                     idle(12); check_good("frame after noise", data);

        // --- Random, sink always ready, gaps down to 12 cycles ---
        tb_info("Random frames, sink always ready");
        repeat (300) random_frame("random", 12, 60, 20);

        // --- Backpressure within what the receiver can absorb ---
        tb_info("Random frames, sink ready 50% (stalls <= 2 cycles)");
        ready_pct = 50;
        max_stall = 2;
        repeat (300) random_frame("backpressure", 12, 60, 20);

        // --- Long stalls: frames may be cut short, never corrupted ---
        tb_info("Random frames, sink stalls 8-30 cycles");
        ready_pct = 100;
        max_stall = 0;
        burst_pct = 10; // 1% per cycle
        burst_min = 8;
        burst_max = 30;
        begin
            int unsigned c0 = n_cut + n_dropped, e0 = n_exact;
            repeat (300) random_frame("overflow", 60, 80, 0, 1'b1);
            `CHECK(n_cut + n_dropped > c0, "overflow phase: no frame was cut short (stalls never hit)")
            `CHECK(n_exact > e0, "overflow phase: no frame got through whole")
        end
        burst_pct = 0;
        repeat (32) @(posedge clk);
        rand_frame(data, 64);  phy_frame(data);                     idle(48); check_good("frame after overflow phase", data);

        // --- Reset released mid-frame ---
        tb_info("Reset mid-frame");
        rand_frame(data, 64);
        fork
            phy_frame(data);
            begin
                repeat (100) @(posedge clk);
                rstn = 1'b0;
                repeat (5) @(posedge clk);
                `CHECK_EQ(axis_out.tvalid, 1'b0, "tvalid low in reset")
                got = {}; // what came out before the reset doesn't count
                got_last = {};
                rstn = 1'b1;
            end
        join
        idle(48);
        check_nothing("rest of frame after reset");
        rand_frame(data, 64);  phy_frame(data);                     idle(48); check_good("frame after reset", data);

        tb_info($sformatf("%0d frames: %0d whole, %0d cut short with tlast, %0d dropped",
                          n_frames, n_exact, n_cut, n_dropped));
        tb_finish();
    end

endmodule
