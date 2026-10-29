`timescale 1ns/1ps
`include "tb_macros.svh"

// rmii_tx test. Frames go in on axis_in; a PHY-side monitor decodes TXEN/TXD the way
// the LAN8720A samples them (rising REF_CLK edge, TXD[0] first, LSB dibit first) and
// checks each frame on the wire:
//   - preamble 7 x 0x55, SFD 0xD5, then exactly the frame's bytes
//   - TXEN high for whole bytes only, TXD 00 whenever TXEN is low
//   - at least 12 bytes (48 cycles) of TXEN low between frames
// Phases:
//   directed      1 byte frame, 2 byte frame, a 1518 byte frame
//   random        random lengths, source keeps tvalid up for the whole frame
//   gappy         source drops tvalid for 0-3 cycles after each beat (rmii_tx asks
//                 for a byte every 4 cycles, so this never starves it)
//   back-to-back  the next frame is always waiting: the gap should be the minimum
module tb_rmii_tx;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(50_000_000) // 50 ms

    localparam real CLK_PERIOD_NS = 20.0; // 50 MHz RMII REF_CLK
    localparam int  IPG_CYCLES = 48;      // 96 bit times

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    logic       rstn = 1'b0;
    logic [1:0] txd;
    logic       txen;

    axis_tlast_if #(.WIDTH(8)) axis_in ();

    rmii_tx dut (
        .clk(clk),
        .rstn(rstn),
        .rmii_txd(txd),
        .rmii_txen(txen),
        .axis_in(axis_in)
    );

    // ------------------------------------------------------------------
    // Source
    // ------------------------------------------------------------------
    // Beats waiting to go out; the head is the one on the bus while tvalid is high.
    // Driven from an always block so handshakes are sampled like the DUT sees them.
    byte unsigned expected[$][$]; // frames handed to the source, in order
    byte unsigned beat_data[$];
    bit           beat_last[$];
    int unsigned  beat_gap_max = 0; // idle cycles after each beat within a frame
    int unsigned  gap_left = 0;

    always @(posedge clk) begin
        bit hs;
        if (!rstn) begin
            axis_in.tvalid <= 1'b0;
            axis_in.tdata  <= '0;
            axis_in.tlast  <= 1'b0;
        end else begin
            hs = axis_in.tvalid && axis_in.tready;
            if (hs) begin
                gap_left = beat_last[0] ? 0 : $urandom_range(beat_gap_max);
                void'(beat_data.pop_front());
                void'(beat_last.pop_front());
            end
            if (!axis_in.tvalid || hs) begin
                if (gap_left > 0) begin
                    gap_left--;
                    axis_in.tvalid <= 1'b0;
                end else if (beat_data.size() > 0) begin
                    axis_in.tvalid <= 1'b1;
                    axis_in.tdata  <= beat_data[0];
                    axis_in.tlast  <= beat_last[0];
                end else begin
                    axis_in.tvalid <= 1'b0;
                end
            end
        end
    end

    function automatic void send_frame(byte unsigned data[$]);
        expected.push_back(data);
        foreach (data[i]) begin
            beat_data.push_back(data[i]);
            beat_last.push_back(i == data.size() - 1);
        end
    endfunction

    function automatic void rand_frame(output byte unsigned data[$], input int unsigned len);
        data = {};
        repeat (len) data.push_back(8'($urandom));
    endfunction

    // ------------------------------------------------------------------
    // PHY-side monitor
    // ------------------------------------------------------------------
    int unsigned n_frames = 0;
    int unsigned min_gap = '1, max_gap_b2b = 0;
    bit          in_frame = 1'b0, seen_frame = 1'b0;
    int unsigned idle_cycles = 0;
    bit          b2b = 1'b0; // back-to-back phase: record the gaps
    logic [1:0]  dibits[$];

    function automatic string show(byte unsigned q[$]);
        string s = "";
        foreach (q[i]) begin
            if (i == 10) return {s, " ..."};
            s = {s, $sformatf(" %02h", q[i])};
        end
        return s;
    endfunction

    function automatic void check_frame();
        byte unsigned on_wire[$], exp[$], payload[$];
        bit           ok;
        n_frames++;

        tb_checks++;
        if (dibits.size() % 4 != 0)
            tb_error($sformatf("frame %0d: TXEN high for %0d dibits, not a whole number of bytes", n_frames, dibits.size()));
        for (int i = 0; i + 3 < dibits.size(); i += 4)
            on_wire.push_back({dibits[i+3], dibits[i+2], dibits[i+1], dibits[i]});

        tb_checks++;
        if (expected.size() == 0) begin
            tb_error($sformatf("frame %0d: frame on the wire that was never sent: {%s }", n_frames, show(on_wire)));
            return;
        end
        exp = expected.pop_front();

        tb_checks++;
        ok = on_wire.size() >= 8;
        for (int i = 0; i < 8 && i < on_wire.size(); i++)
            if (on_wire[i] != (i == 7 ? 8'hD5 : 8'h55)) ok = 1'b0;
        if (!ok) begin
            tb_error($sformatf("frame %0d: expected preamble 55 x7 and SFD D5, wire has {%s }", n_frames, show(on_wire)));
            return;
        end
        for (int i = 8; i < on_wire.size(); i++) payload.push_back(on_wire[i]);

        tb_checks++;
        ok = payload.size() == exp.size();
        foreach (exp[i]) if (ok && payload[i] != exp[i]) ok = 1'b0;
        if (!ok)
            tb_error($sformatf("frame %0d: sent %0d bytes {%s }, wire has %0d {%s }",
                               n_frames, exp.size(), show(exp), payload.size(), show(payload)));
    endfunction

    always @(posedge clk) begin
        if (rstn) begin
            tb_checks++;
            if (txen === 1'b1) begin
                if (!in_frame) begin
                    if (seen_frame) begin
                        tb_checks++;
                        if (idle_cycles < IPG_CYCLES)
                            tb_error($sformatf("frame %0d started %0d cycles after the last one ended (IPG is %0d)",
                                               n_frames + 1, idle_cycles, IPG_CYCLES));
                        min_gap = idle_cycles < min_gap ? idle_cycles : min_gap;
                        if (b2b) max_gap_b2b = idle_cycles > max_gap_b2b ? idle_cycles : max_gap_b2b;
                    end
                    in_frame = 1'b1;
                    dibits = {};
                end
                dibits.push_back(txd);
            end else if (txen === 1'b0) begin
                if (txd !== 2'b00)
                    tb_error_once("txd idle", $sformatf("TXD = %b while TXEN is low", txd));
                if (in_frame) begin
                    check_frame();
                    in_frame    = 1'b0;
                    seen_frame  = 1'b1;
                    idle_cycles = 0;
                end
                idle_cycles++;
            end else begin
                tb_error_once("txen x", $sformatf("TXEN = %b after reset", txen));
            end
        end
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic wait_sent(int unsigned timeout = 100_000);
        while ((expected.size() > 0 || in_frame) && timeout > 0) begin
            @(posedge clk);
            timeout--;
        end
        `CHECK(expected.size() == 0 && !in_frame, $sformatf("%0d frames never made it onto the wire", expected.size()))
    endtask

    initial begin
        byte unsigned data[$];

        tb_info("Testing rmii_tx");
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        repeat (3) @(posedge clk);
        `CHECK_EQ(txen, 1'b0, "TXEN low after reset")
        `CHECK_EQ(axis_in.tready, 1'b0, "tready low when idle")

        tb_info("Directed frames");
        rand_frame(data, 1);    send_frame(data); wait_sent();
        rand_frame(data, 2);    send_frame(data); wait_sent();
        rand_frame(data, 1518); send_frame(data); wait_sent();

        tb_info("Random frames, continuous source");
        repeat (200) begin
            rand_frame(data, $urandom_range(9) == 0 ? $urandom_range(1518, 60) : $urandom_range(100, 1));
            send_frame(data);
            wait (beat_data.size() == 0);
            repeat ($urandom_range(100)) @(posedge clk);
        end
        wait_sent();

        tb_info("Random frames, source gaps 0-3 cycles between beats");
        beat_gap_max = 3;
        repeat (200) begin
            rand_frame(data, $urandom_range(100, 1));
            send_frame(data);
            wait (beat_data.size() == 0);
            repeat ($urandom_range(100)) @(posedge clk);
        end
        wait_sent();

        tb_info("Back-to-back frames");
        beat_gap_max = 0;
        b2b = 1'b1;
        repeat (50) begin // all queued at once: each frame is offered right after the last
            rand_frame(data, $urandom_range(100, 1));
            send_frame(data);
        end
        wait_sent();
        b2b = 1'b0;
        `CHECK(max_gap_b2b <= IPG_CYCLES + 2, $sformatf("back-to-back frames had gaps of up to %0d cycles (IPG is %0d)", max_gap_b2b, IPG_CYCLES))

        tb_info($sformatf("%0d frames; smallest gap %0d cycles, largest back-to-back gap %0d", n_frames, min_gap, max_gap_b2b));
        tb_finish();
    end

endmodule
