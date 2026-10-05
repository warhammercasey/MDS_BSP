`timescale 1ns/1ps
`include "tb_macros.svh"

// UART sanity test: send and receive a few messages in each direction.
//
//   TX path: bytes pushed into axis_tx are decoded off the tx pin by a serial monitor.
//   RX path: bytes driven onto the rx pin by a serial driver are collected from axis_rx.
//
// The serial driver/monitor use the exact baud period (not the DUT's integer
// clocks-per-bit), so the test also covers the DUT's baud rounding error.
module tb_uart;
    import tb_pkg::*;

    localparam int  CLK_FREQ      = 73_571_400; // cain_top CLK_IO_FREQ
    localparam int  BAUD          = 115_200;
    localparam real CLK_PERIOD_NS = 1.0e9 / CLK_FREQ;
    localparam real BIT_PERIOD_NS = 1.0e9 / BAUD;

    typedef logic [7:0] byte_q_t[$];

    `TB_INIT
    `TB_WATCHDOG(50_000_000) // 50 ms

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    logic clk  = 1'b0;
    logic rstn = 1'b0;
    logic rx   = 1'b1; // serial line into the DUT
    logic tx;          // serial line out of the DUT

    always #(CLK_PERIOD_NS/2) clk = ~clk;

    axis_if #(.WIDTH(8)) axis_tx ();
    axis_if #(.WIDTH(8)) axis_rx ();

    uart #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD(BAUD)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .rx(rx),
        .tx(tx),
        .axis_tx(axis_tx),
        .axis_rx(axis_rx)
    );

    // ------------------------------------------------------------------
    // Scoreboards: bytes still expected on each output
    // ------------------------------------------------------------------
    byte_q_t tx_expected; // expected on the tx pin
    byte_q_t rx_expected; // expected on axis_rx
    int unsigned tx_count = 0;
    int unsigned rx_count = 0;

    function automatic byte_q_t str_to_bytes(string s);
        byte_q_t q;
        for (int i = 0; i < s.len(); i++)
            q.push_back(s[i]);
        return q;
    endfunction

    function automatic string fmt_byte(logic [7:0] b);
        if (b >= 8'h20 && b < 8'h7f)
            return $sformatf("0x%02h '%c'", b, b);
        return $sformatf("0x%02h", b);
    endfunction

    // ------------------------------------------------------------------
    // AXI-Stream source -> DUT axis_tx
    // Drives on negedge so tready is sampled race-free between posedges.
    // ------------------------------------------------------------------
    task automatic axis_send(input logic [7:0] b);
        @(negedge clk);
        axis_tx.tdata  = b;
        axis_tx.tvalid = 1'b1;
        while (!axis_tx.tready) @(negedge clk);
        @(negedge clk); // handshake happened on the posedge in between
        axis_tx.tvalid = 1'b0;
    endtask

    task automatic send_tx(input byte_q_t msg);
        foreach (msg[i]) begin
            tx_expected.push_back(msg[i]);
            axis_send(msg[i]);
        end
    endtask

    // ------------------------------------------------------------------
    // Serial monitor on the DUT tx pin (8N1, LSB first)
    // ------------------------------------------------------------------
    initial begin : tx_monitor
        logic [7:0] data;
        wait (rstn);
        forever begin
            @(negedge tx);
            #(BIT_PERIOD_NS/2);
            `CHECK_EQ(tx, 1'b0, "tx start bit")
            for (int i = 0; i < 8; i++) begin
                #(BIT_PERIOD_NS);
                data[i] = tx;
            end
            #(BIT_PERIOD_NS);
            `CHECK_EQ(tx, 1'b1, "tx stop bit")

            if (tx_expected.size() == 0) begin
                tb_error($sformatf("tx: unexpected byte %s", fmt_byte(data)));
            end else begin
                logic [7:0] exp = tx_expected.pop_front();
                `CHECK_EQ(data, exp, $sformatf("tx byte %0d", tx_count))
            end
            tx_count++;
        end
    end

    // ------------------------------------------------------------------
    // Serial driver -> DUT rx pin (8N1, LSB first)
    // ------------------------------------------------------------------
    task automatic serial_send(input logic [7:0] b);
        rx = 1'b0;
        #(BIT_PERIOD_NS);
        for (int i = 0; i < 8; i++) begin
            rx = b[i];
            #(BIT_PERIOD_NS);
        end
        rx = 1'b1;
        #(BIT_PERIOD_NS);
    endtask

    task automatic send_rx(input byte_q_t msg);
        foreach (msg[i]) begin
            rx_expected.push_back(msg[i]);
            serial_send(msg[i]);
        end
    endtask

    // ------------------------------------------------------------------
    // AXI-Stream sink <- DUT axis_rx (always ready)
    // ------------------------------------------------------------------
    initial axis_rx.tready = 1'b1;

    always @(negedge clk) begin : rx_monitor
        if (rstn && axis_rx.tvalid && axis_rx.tready) begin
            if (rx_expected.size() == 0) begin
                tb_error($sformatf("rx: unexpected byte %s", fmt_byte(axis_rx.tdata)));
            end else begin
                logic [7:0] exp = rx_expected.pop_front();
                `CHECK_EQ(axis_rx.tdata, exp, $sformatf("rx byte %0d", rx_count))
            end
            rx_count++;
        end
    end

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    // Wait for both scoreboards to drain, then a little idle time.
    task automatic wait_idle();
        while (tx_expected.size() != 0 || rx_expected.size() != 0)
            @(posedge clk);
        #(2*BIT_PERIOD_NS);
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    initial begin
        axis_tx.tvalid = 1'b0;
        axis_tx.tdata  = '0;

        repeat (10) @(posedge clk);
        rstn = 1'b1;
        repeat (10) @(posedge clk);
        `CHECK_EQ(tx, 1'b1, "tx idles high after reset")

        tb_info("Test 1: TX message");
        send_tx(str_to_bytes("Hello, CAIN!\n"));
        wait_idle();

        tb_info("Test 2: RX message");
        send_rx(str_to_bytes("UART rx ok\n"));
        wait_idle();

        tb_info("Test 3: bit patterns, both directions");
        send_tx('{8'h00, 8'hFF, 8'h55, 8'hAA, 8'h01, 8'h80});
        wait_idle();
        send_rx('{8'h00, 8'hFF, 8'h55, 8'hAA, 8'h01, 8'h80});
        wait_idle();

        tb_info("Test 4: full duplex");
        fork
            send_tx(str_to_bytes("ping from fpga"));
            send_rx(str_to_bytes("pong from host"));
        join
        wait_idle();

        tb_info($sformatf("Transferred %0d tx bytes, %0d rx bytes", tx_count, rx_count));
        tb_finish();
    end

endmodule
