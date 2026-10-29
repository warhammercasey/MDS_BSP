`timescale 1ns/1ps
`include "tb_macros.svh"

// rtp_packetizer test, for several sample sizes and channel counts.
//
// A reference model here turns the samples sent into the packets that should come
// out, byte for byte: header (version 2, marker, payload type, sequence number,
// timestamp, SSRC), then the samples big endian and left justified. Every packet out
// is compared against it, and the output must hold still while stalled.
//
//   start   the stream starts halfway through a frame: those samples are thrown away,
//           the first packet starts on channel 0, is marked, and has timestamp 0
//   steady  whole frames with gaps on the input and stalls on the output: timestamps
//           go up by FRAMES_PER_PACKET, sequence numbers by 1, no marker
//   glitch  a sample goes missing now and then: the packet it was in comes out the
//           right length anyway, the next one restarts on channel 0, is marked, and
//           its timestamp counts the frames since reset
module rtp_packetizer_inst #(
    parameter int          SAMPLE_BITS       = 18,
    parameter int          CHANNELS          = 8,
    parameter int          ID_WIDTH          = 3,
    parameter int          FRAMES_PER_PACKET = 3,
    parameter logic [31:0] SSRC              = 32'h1234_5678,
    parameter logic [6:0]  PAYLOAD_TYPE      = 7'd97
) (
    input  logic clk,
    output logic done
);
    import tb_pkg::*;

    localparam int SAMPLE_BYTES = (SAMPLE_BITS + 7) / 8;
    localparam int SAMPLES      = FRAMES_PER_PACKET * CHANNELS;
    localparam int PACKET_BYTES = 12 + SAMPLES * SAMPLE_BYTES;

    logic rstn = 1'b0;

    axis_tid_if   #(.WIDTH(SAMPLE_BITS), .ID_WIDTH(ID_WIDTH)) axis_in ();
    axis_tlast_if #(.WIDTH(8))                                axis_out ();

    rtp_packetizer #(
        .SSRC(SSRC),
        .PAYLOAD_TYPE(PAYLOAD_TYPE),
        .SAMPLE_BITS(SAMPLE_BITS),
        .CHANNELS(CHANNELS),
        .ID_WIDTH(ID_WIDTH),
        .FRAMES_PER_PACKET(FRAMES_PER_PACKET)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .axis_in(axis_in),
        .axis_out(axis_out)
    );

    string NAME = $sformatf("%0d bit x %0d ch", SAMPLE_BITS, CHANNELS);

    // ------------------------------------------------------------------
    // Reference model, run on samples as they are queued (they all go in, in order)
    // ------------------------------------------------------------------
    byte unsigned expected[$][$];
    byte unsigned m_pkt[$];
    bit           m_in_packet = 1'b0;
    bit           m_resync    = 1'b1;
    int unsigned  m_frames    = 0;
    int unsigned  m_seq       = 0;
    int unsigned  m_channel   = 0;
    int unsigned  m_count     = 0;
    int unsigned  m_marked    = 0;  // Packets expected with the marker set

    function automatic void model(int unsigned tid, logic [SAMPLE_BITS-1:0] data);
        logic [SAMPLE_BYTES*8-1:0] s;
        if (!m_in_packet) begin
            if (tid != 0) begin
                m_resync = 1'b1;   // Thrown away
                return;
            end
            m_pkt = {};
            m_pkt.push_back(8'h80);
            m_pkt.push_back({m_resync, PAYLOAD_TYPE});
            for (int k = 1; k >= 0; k--) m_pkt.push_back(8'(m_seq >> (8 * k)));
            for (int k = 3; k >= 0; k--) m_pkt.push_back(8'(m_frames >> (8 * k)));
            for (int k = 3; k >= 0; k--) m_pkt.push_back(SSRC[8*k +: 8]);
            m_marked   += m_resync;
            m_resync    = 1'b0;
            m_in_packet = 1'b1;
            m_channel   = 0;
            m_count     = 0;
        end
        s = (SAMPLE_BYTES*8)'(data) << (SAMPLE_BYTES*8 - SAMPLE_BITS);
        for (int k = SAMPLE_BYTES - 1; k >= 0; k--) m_pkt.push_back(s[8*k +: 8]);
        if (tid != m_channel) m_resync = 1'b1;
        if (tid == 0) m_frames++;
        m_channel = m_channel == CHANNELS - 1 ? 0 : m_channel + 1;
        m_count++;
        if (m_count == SAMPLES) begin
            expected.push_back(m_pkt);
            m_seq       = (m_seq + 1) % 65536;
            m_in_packet = 1'b0;
        end
    endfunction

    // ------------------------------------------------------------------
    // Source: {tid, data} from a queue
    // ------------------------------------------------------------------
    logic [ID_WIDTH+SAMPLE_BITS-1:0] src_q[$];
    int unsigned                     gap_pct = 0;

    always @(posedge clk) begin
        bit hs;
        if (!rstn) begin
            axis_in.tvalid <= 1'b0;
            axis_in.tdata  <= '0;
            axis_in.tid    <= '0;
        end else begin
            hs = axis_in.tvalid && axis_in.tready;
            if (hs) void'(src_q.pop_front());
            if (!axis_in.tvalid || hs) begin
                if (src_q.size() > 0 && $urandom_range(99) >= gap_pct) begin
                    axis_in.tvalid <= 1'b1;
                    {axis_in.tid, axis_in.tdata} <= src_q[0];
                end else begin
                    axis_in.tvalid <= 1'b0;
                end
            end
        end
    end

    function automatic void send(int unsigned tid);
        logic [SAMPLE_BITS-1:0] d = SAMPLE_BITS'({$urandom, $urandom});
        src_q.push_back({ID_WIDTH'(tid), d});
        model(tid, d);
    endfunction

    // Whole frames; `skip` channels are left out (-1: none)
    function automatic void send_frames(int unsigned n, int skip = -1);
        repeat (n) for (int c = 0; c < CHANNELS; c++) if (c != skip) send(c);
    endfunction

    // Frames until the model is between packets
    function automatic void finish_packet();
        while (m_in_packet) send_frames(1);
    endfunction

    // ------------------------------------------------------------------
    // Sink: packets, compared with the model
    // ------------------------------------------------------------------
    int unsigned  sink_rate = 100;
    int unsigned  n_recv = 0;
    int unsigned  n_marked = 0;
    byte unsigned cur[$];
    bit           stalled = 1'b0;
    logic [8:0]   stalled_beat;

    function automatic string show(byte unsigned q[$]);
        string s = "";
        foreach (q[i]) begin
            if (i == 20) return {s, " ..."};
            s = {s, $sformatf(" %02h", q[i])};
        end
        return s;
    endfunction

    function automatic void packet_done();
        byte unsigned exp[$];
        bit           ok;
        tb_checks++;
        if (expected.size() == 0) begin
            tb_error_once({NAME, " extra"}, $sformatf("%s: unexpected packet {%s }", NAME, show(cur)));
            return;
        end
        exp = expected.pop_front();
        ok  = exp.size() == cur.size();
        foreach (exp[i]) if (ok && exp[i] != cur[i]) ok = 1'b0;
        if (!ok)
            tb_error_once({NAME, " packet"},
                $sformatf("%s: packet %0d is %0d bytes {%s }\n    expected %0d bytes {%s }",
                          NAME, n_recv, cur.size(), show(cur), exp.size(), show(exp)));
        if (cur.size() > 1 && cur[1][7]) n_marked++;
    endfunction

    always @(posedge clk) begin
        if (rstn) begin
            if (stalled) begin
                tb_checks++;
                if (axis_out.tvalid !== 1'b1 || {axis_out.tlast, axis_out.tdata} !== stalled_beat)
                    tb_error_once({NAME, " stability"}, $sformatf("%s: output changed while stalled", NAME));
            end
            if (axis_out.tvalid && axis_out.tready) begin
                cur.push_back(axis_out.tdata);
                if (axis_out.tlast) begin
                    packet_done();
                    cur = {};
                    n_recv++;
                end
            end
            stalled      = axis_out.tvalid && !axis_out.tready;
            stalled_beat = {axis_out.tlast, axis_out.tdata};
        end
        axis_out.tready <= $urandom_range(99) < sink_rate;
    end

    // ------------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------------
    task automatic drain(string name);
        int unsigned timeout = 0;
        while ((expected.size() > 0 || src_q.size() > 0) && timeout < 100_000) begin
            @(posedge clk);
            timeout++;
        end
        repeat (20) @(posedge clk);
        `CHECK(expected.size() == 0, $sformatf("%s, %s: %0d packets never came out", NAME, name, expected.size()))
        `CHECK(cur.size() == 0, $sformatf("%s, %s: %0d bytes of a packet left over", NAME, name, cur.size()))
    endtask

    initial begin
        int unsigned n0;
        done = 1'b0;
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        @(posedge clk);
        `CHECK_EQ(axis_out.tvalid, 1'b0, $sformatf("%s: tvalid low after reset", NAME))

        // Start partway through a frame (no partial frame with one channel)
        for (int c = (CHANNELS + 1) / 2; c < CHANNELS; c++) send(c);
        send_frames(FRAMES_PER_PACKET);
        drain("start");
        `CHECK(n_recv == 1 && n_marked == 1, $sformatf("%s, start: %0d packets, %0d marked (expected 1, 1)", NAME, n_recv, n_marked))

        // Steady: no more markers
        n0 = n_marked;
        gap_pct   = 30;
        sink_rate = 40;
        send_frames(FRAMES_PER_PACKET * 20);
        drain("steady");
        `CHECK(n_marked == n0, $sformatf("%s, steady: %0d packets marked", NAME, n_marked - n0))

        // Glitches: a sample missing from a random frame
        repeat (8) begin
            send_frames($urandom_range(2 * FRAMES_PER_PACKET, 1));
            send_frames(1, $urandom_range(CHANNELS - 1));
            send_frames($urandom_range(2 * FRAMES_PER_PACKET));
        end
        finish_packet();
        drain("glitch");
        `CHECK(n_marked == m_marked, $sformatf("%s: %0d packets marked, expected %0d", NAME, n_marked, m_marked))

        tb_info($sformatf("%s: %0d packets of %0d bytes, %0d marked", NAME, n_recv, PACKET_BYTES, n_marked));
        done = 1'b1;
    end

endmodule


module tb_rtp_packetizer;
    import tb_pkg::*;

    `TB_INIT
    `TB_WATCHDOG(50_000_000) // 50 ms

    localparam real CLK_PERIOD_NS = 20.0; // clk_50

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    logic [3:0] done;
    rtp_packetizer_inst #(.SAMPLE_BITS(18), .CHANNELS(8), .ID_WIDTH(3), .FRAMES_PER_PACKET(3),
                          .SSRC(32'h4D49_4330), .PAYLOAD_TYPE(7'd97)) g_mic (.clk(clk), .done(done[0]));
    rtp_packetizer_inst #(.SAMPLE_BITS(8),  .CHANNELS(8), .ID_WIDTH(3), .FRAMES_PER_PACKET(4),
                          .SSRC(32'h4144_4330), .PAYLOAD_TYPE(7'd96)) g_adc (.clk(clk), .done(done[1]));
    rtp_packetizer_inst #(.SAMPLE_BITS(12), .CHANNELS(3), .ID_WIDTH(2), .FRAMES_PER_PACKET(2),
                          .SSRC(32'hDEAD_BEEF), .PAYLOAD_TYPE(7'd127)) g_12b (.clk(clk), .done(done[2]));
    rtp_packetizer_inst #(.SAMPLE_BITS(24), .CHANNELS(1), .ID_WIDTH(1), .FRAMES_PER_PACKET(5),
                          .SSRC(32'h0000_0001), .PAYLOAD_TYPE(7'd100)) g_mono (.clk(clk), .done(done[3]));

    initial begin
        tb_info("Testing rtp_packetizer: 18 bit x 8, 8 bit x 8, 12 bit x 3, 24 bit x 1");
        wait (&done);
        tb_finish();
    end

endmodule
