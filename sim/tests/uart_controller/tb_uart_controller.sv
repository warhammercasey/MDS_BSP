`timescale 1ns/1ps
`include "tb_macros.svh"

// uart_controller test: commands typed on the rx pin, output decoded off the tx pin,
// and three APB register models behind an apb_splitter:
//   p0  16 registers, no wait states, PSLVERR above that
//   p1  256 registers, 0-4 wait states
//   p2  4 registers, 0-2 wait states, PSLVERR at 04-EF, and pready never rises at
//       F0-FF, so the controller must time out and print ERR
//
// The expected output of each line (its echo, then the response) comes from a
// reference copy of the registers. Also checked: the number of APB transfers per line
// (none for a malformed line), the APB protocol, and the registers' final contents.
//
// The bus has cain_top's 16-bit data width, so reads print 4 hex digits. The baud
// rate is 921600 rather than cain_top's 115200 to keep the sim short. The
// controller's logic doesn't depend on it, and the uart test covers 115200.

module tb_uart_controller;
    import tb_pkg::*;

    localparam int  CLK_FREQ      = 73_571_400; // cain_top CLK_IO_FREQ
    localparam int  BAUD          = 921_600;
    localparam real CLK_PERIOD_NS = 1.0e9 / CLK_FREQ;
    localparam real BIT_PERIOD_NS = 1.0e9 / BAUD;

    localparam int ADDR_WIDTH  = 8;
    localparam int DATA_WIDTH  = 16;            // cain_top APB_DATA_WIDTH
    localparam int DATA_MAX    = (1 << DATA_WIDTH) - 1;
    localparam int DATA_CHARS  = (DATA_WIDTH + 3) / 4; // hex digits in a read response
    localparam int NUM_PERIPH  = 3;
    localparam int APB_TIMEOUT = 256;
    localparam int SIZE      [NUM_PERIPH] = '{16, 256, 4};
    localparam int MAX_WAIT  [NUM_PERIPH] = '{0, 4, 2};
    localparam int HANG_FROM [NUM_PERIPH] = '{256, 256, 'hF0};

    localparam int N_RANDOM = 60;

    localparam logic [7:0] CR = 8'h0D;
    localparam logic [7:0] LF = 8'h0A;
    localparam string      EOL_CR   = "\015";
    localparam string      EOL_LF   = "\n";
    localparam string      EOL_CRLF = "\015\n";

    typedef logic [7:0] byte_q_t[$];

    `TB_INIT
    `TB_WATCHDOG(100_000_000) // 100 ms

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    logic clk  = 1'b0;
    logic rstn = 1'b0;
    logic rx   = 1'b1;
    logic tx;

    always #(CLK_PERIOD_NS/2) clk = ~clk;

    apb_if #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .SEL_WIDTH(NUM_PERIPH)) apb ();
    apb_if #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .SEL_WIDTH(1)) apb_periph [NUM_PERIPH] ();

    uart_controller #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD(BAUD),
        .APB_ADDR_WIDTH(ADDR_WIDTH),
        .APB_DATA_WIDTH(DATA_WIDTH),
        .APB_SEL_WIDTH(NUM_PERIPH),
        .APB_TIMEOUT(APB_TIMEOUT)
    ) dut (
        .clk(clk),
        .rstn(rstn),
        .rx(rx),
        .tx(tx),
        .apb(apb)
    );

    apb_splitter #(
        .DATA_WIDTH(DATA_WIDTH),
        .N(NUM_PERIPH)
    ) splitter (
        .apb_in(apb),
        .apb_out(apb_periph)
    );

    // ------------------------------------------------------------------
    // Peripherals and the reference copy of their registers
    // ------------------------------------------------------------------
    logic [DATA_WIDTH-1:0] ref_mem [NUM_PERIPH][256];
    bit         check_mem = 1'b0;

    for (genvar i = 0; i < NUM_PERIPH; i++) begin : g_p
        apb_reg_model #(
            .ADDR_WIDTH(ADDR_WIDTH),
            .DATA_WIDTH(DATA_WIDTH),
            .SIZE(SIZE[i]),
            .MAX_WAIT(MAX_WAIT[i]),
            .HANG_FROM(HANG_FROM[i])
        ) model (
            .clk(clk),
            .rstn(rstn),
            .apb(apb_periph[i])
        );

        initial begin
            #1;
            for (int a = 0; a < SIZE[i]; a++)
                ref_mem[i][a] = model.mem[a];
            wait (check_mem);
            for (int a = 0; a < SIZE[i]; a++)
                `CHECK_EQ(model.mem[a], ref_mem[i][a], $sformatf("p%0d register 0x%02h at the end", i, a))
        end
    end

    // Master side: count transfers (setup phases), check psel/penable
    int unsigned n_apb = 0;

    always @(posedge clk) begin
        if (rstn) begin
            automatic int n_sel = 0;
            for (int i = 0; i < NUM_PERIPH; i++)
                n_sel += int'(apb.psel[i]);
            tb_checks++;
            if (n_sel > 1)
                tb_error_once("apb psel", "APB: more than one psel high");
            if (apb.penable && n_sel == 0)
                tb_error_once("apb penable", "APB: penable high without psel");
            if (n_sel == 1 && !apb.penable)
                n_apb++;
        end
    end

    // ------------------------------------------------------------------
    // Serial monitor on the DUT tx pin (8N1, LSB first)
    // ------------------------------------------------------------------
    byte_q_t     tx_expected;
    string       cur_line;     // line being typed, for messages
    int unsigned tx_count = 0;

    function automatic string fmt_byte(logic [7:0] b);
        if (b >= 8'h20 && b < 8'h7f)
            return $sformatf("0x%02h '%c'", b, b);
        return $sformatf("0x%02h", b);
    endfunction

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
                tb_error($sformatf("Line \"%s\": unexpected output %s", cur_line, fmt_byte(data)));
            end else begin
                automatic logic [7:0] exp = tx_expected.pop_front();
                tb_checks++;
                if (data !== exp)
                    tb_error($sformatf("Line \"%s\": output %s, expected %s", cur_line, fmt_byte(data), fmt_byte(exp)));
            end
            tx_count++;
        end
    end

    // ------------------------------------------------------------------
    // Typing
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

    int unsigned n_lines = 0;

    // Type `line` then `eol`, and check the output and the number of APB transfers.
    // `body` is the expected response; "" means the line only gets a new line.
    task automatic send_line(string line, string eol, string body, int unsigned napb);
        automatic int unsigned apb0 = n_apb;
        cur_line = line;

        for (int i = 0; i < line.len(); i++)
            tx_expected.push_back(line[i]);   // echo
        tx_expected.push_back(CR);
        tx_expected.push_back(LF);
        if (body.len() > 0) begin
            for (int i = 0; i < body.len(); i++)
                tx_expected.push_back(body[i]);
            tx_expected.push_back(CR);
            tx_expected.push_back(LF);
        end

        // Typed with random gaps of 0 to 1.5 characters; the LF of a CR LF follows
        // the CR immediately, while the response is still printing
        for (int i = 0; i < line.len(); i++) begin
            serial_send(line[i]);
            #(BIT_PERIOD_NS * 5 * $urandom_range(3));
        end
        for (int i = 0; i < eol.len(); i++)
            serial_send(eol[i]);

        while (tx_expected.size() != 0)
            @(posedge clk);
        #(3 * BIT_PERIOD_NS);
        `CHECK_EQ(n_apb - apb0, napb, $sformatf("APB transfers for line \"%s\"", line))
        n_lines++;
    endtask

    // Response to a well-formed command, and the number of APB transfers it makes.
    // Updates the reference registers for a write.
    function automatic string expect_cmd(bit wr, int sel, int addr, int data, output int unsigned napb);
        string s;
        napb = 0;
        if (sel >= NUM_PERIPH || addr >= 256 || (wr && data > DATA_MAX))
            return "ERR";   // field doesn't fit: no transfer
        napb = 1;
        if (addr >= HANG_FROM[sel] || addr >= SIZE[sel])
            return "ERR";   // timeout or PSLVERR
        if (wr) begin
            ref_mem[sel][addr] = DATA_WIDTH'(data);
            return "OK";
        end
        s = $sformatf("%0h", ref_mem[sel][addr]);
        while (s.len() < DATA_CHARS)
            s = {"0", s};
        return s.toupper();
    endfunction

    // A well-formed command
    task automatic cmd(string line, bit wr, int sel, int addr, int data = 0, string eol = EOL_CR);
        int unsigned napb;
        string       body;
        body = expect_cmd(wr, sel, addr, data, napb);
        send_line(line, eol, body, napb);
    endtask

    // A malformed line
    task automatic bad(string line, string eol = EOL_CR);
        send_line(line, eol, "ERR", 0);
    endtask

    // ------------------------------------------------------------------
    // Random commands
    // ------------------------------------------------------------------
    function automatic string spaces(int lo, int hi);
        string s = "";
        repeat ($urandom_range(hi, lo)) s = {s, " "};
        return s;
    endfunction

    // Hex in random case, with 0-2 leading zeros
    function automatic string rand_hex(int v);
        string s = $sformatf("%0h", v);
        repeat ($urandom_range(2)) s = {"0", s};
        return $urandom_range(1) == 1 ? s.toupper() : s;
    endfunction

    function automatic string fmt_cmd(bit wr, int sel, int addr, int data);
        string s = wr ? "w" : "r";
        if ($urandom_range(1) == 1)
            s = s.toupper();
        s = {spaces(0, 1), s, spaces(1, 2), rand_hex(sel), spaces(1, 2), rand_hex(addr)};
        if (wr)
            s = {s, spaces(1, 2), rand_hex(data)};
        return {s, spaces(0, 1)};
    endfunction

    function automatic string rand_eol();
        case ($urandom_range(2))
            0:       return EOL_CR;
            1:       return EOL_LF;
            default: return EOL_CRLF;
        endcase
    endfunction

    task automatic random_line();
        automatic bit    wr   = 1'($urandom);
        automatic int    sel  = $urandom_range(NUM_PERIPH - 1);
        automatic int    addr = $urandom_range(SIZE[sel] - 1);
        automatic int    data = $urandom_range(DATA_MAX);
        automatic int    kind = $urandom_range(9);
        automatic string eol  = rand_eol();
        automatic string bad_chars = "Gz#-,\177";
        string line;

        case (kind)
            6: addr = $urandom_range(255);  // anywhere: may be PSLVERR or a timeout
            7: begin                        // a field too big for the bus
                case ($urandom_range(2))
                    0:       sel  = $urandom_range(255, NUM_PERIPH);
                    1:       addr = $urandom_range(4095, 256);
                    default: begin wr = 1'b1; data = $urandom_range(16 * DATA_MAX + 15, DATA_MAX + 1); end
                endcase
            end
            default: ;
        endcase

        if (kind == 8) begin
            // One character replaced by one that is never valid
            automatic int c = bad_chars[$urandom_range(bad_chars.len() - 1)];
            line = fmt_cmd(wr, sel, addr, data);
            line.putc($urandom_range(line.len() - 1), c);
            bad(line, eol);
        end else if (kind == 9) begin
            // Wrong number of fields
            line = wr ? {"w ", rand_hex(sel), " ", rand_hex(addr)}
                      : {"r ", rand_hex(sel), " ", rand_hex(addr), " ", rand_hex(data)};
            bad(line, eol);
        end else begin
            cmd(fmt_cmd(wr, sel, addr, data), wr, sel, addr, data, eol);
        end
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    initial begin
        repeat (10) @(posedge clk);
        rstn = 1'b1;
        repeat (10) @(posedge clk);
        `CHECK_EQ(tx, 1'b1, "tx idles high after reset")

        tb_info("Test 1: reads and writes");
        cmd("w 0 03 DE", 1, 0, 'h03, 'hDE);
        cmd("r 0 03",    0, 0, 'h03);
        cmd("r 0 0f",    0, 0, 'h0F);
        cmd("w 1 ff 5a", 1, 1, 'hFF, 'h5A);
        cmd("r 1 FF",    0, 1, 'hFF);
        cmd("r 2 3",     0, 2, 'h3);
        cmd("w 2 0 81",  1, 2, 'h0, 'h81);
        cmd("r 2 0",     0, 2, 'h0);
        cmd("w 0 05 BEEF", 1, 0, 'h05, 'hBEEF);
        cmd("r 0 05",      0, 0, 'h05);
        cmd("w 1 80 8001", 1, 1, 'h80, 'h8001);
        cmd("r 1 80",      0, 1, 'h80);
        cmd("w 1 81 7ffe", 1, 1, 'h81, 'h7FFE);
        cmd("r 1 81",      0, 1, 'h81);

        tb_info("Test 2: formatting");
        cmd("R 0 3",                    0, 0, 'h3);
        cmd("W 1 0A b7",                1, 1, 'h0A, 'hB7);
        cmd("r 1 0a",                   0, 1, 'h0A);
        cmd("  r   1   0A   ",          0, 1, 'h0A);
        cmd("w 00 00000000003 000000FE", 1, 0, 'h3, 'hFE);
        cmd("r 0 3",                    0, 0, 'h3);

        tb_info("Test 3: line endings and empty lines");
        cmd("r 0 3",    0, 0, 'h3, 0, EOL_LF);
        cmd("r 0 3",    0, 0, 'h3, 0, EOL_CRLF);
        cmd("w 0 4 44", 1, 0, 'h4, 'h44, EOL_CRLF);
        // (an LF straight after a CR is the end of a CR LF, so LF goes first)
        send_line("",     EOL_LF,   "", 0);
        send_line("",     EOL_LF,   "", 0);
        send_line("",     EOL_CR,   "", 0);
        send_line("",     EOL_CRLF, "", 0);
        send_line("    ", EOL_CR,   "", 0);
        cmd("r 0 4",    0, 0, 'h4, 0, EOL_LF);

        tb_info("Test 4: malformed lines");
        bad("x 0 03");
        bad("r");
        bad("r ");
        bad("r 0");
        bad("r 0 ");
        bad("w 0 03");
        bad("w 0 03 ", EOL_CRLF);
        bad("r 0 03 DE");
        bad("w 0 03 DE 01");
        bad("r0 03");
        bad("rr 0 03");
        bad("read 0 03");
        bad("r 0 0G");
        bad("r 0 03 x");
        bad("r, 0, 03");
        bad("- r 0 03");
        bad("r 0 0\1773");          // backspace (DEL): no line editing
        cmd("r 0 3", 0, 0, 'h3);    // still works

        tb_info("Test 5: fields too big for the bus");
        cmd("r 3 00",     0, 3,  'h0);
        cmd("r F 00",     0, 15, 'h0);
        cmd("r 0 100",      0, 0,  'h100);
        cmd("w 0 00 FFFF",  1, 0,  'h0, 'hFFFF);    // the widest data that fits
        cmd("r 0 00",       0, 0,  'h0);
        cmd("w 0 00 0FFFF", 1, 0,  'h0, 'hFFFF);
        cmd("w 0 00 10000", 1, 0,  'h0, 'h10000);
        cmd("w 0 00 1FFFF", 1, 0,  'h0, 'h1FFFF);
        cmd("r 0 00",       0, 0,  'h0);            // unchanged by the two above
        // Overflows that would wrap to a valid value if the dropped digits were ignored
        bad("r 0 100000000000003");
        bad("r 1000000000000000000 3");
        bad("w 0 3 10000000000000000DE");
        bad("w 0 3 100000000000000BEEF");
        cmd("r 0 3", 0, 0, 'h3);

        tb_info("Test 6: PSLVERR and timeout");
        cmd("r 0 10",    0, 0, 'h10);
        cmd("w 0 10 55", 1, 0, 'h10, 'h55);
        cmd("r 2 04",    0, 2, 'h04);
        cmd("w 2 EF 01", 1, 2, 'hEF, 'h01);
        cmd("r 2 F0",    0, 2, 'hF0);
        cmd("w 2 FF 12", 1, 2, 'hFF, 'h12);
        cmd("r 2 1",     0, 2, 'h1);  // p2 still works after a timeout

        tb_info($sformatf("Test 7: %0d random lines", N_RANDOM));
        repeat (N_RANDOM) random_line();

        check_mem = 1'b1;
        #1;
        tb_info($sformatf("%0d lines, %0d APB transfers, %0d characters out", n_lines, n_apb, tx_count));
        tb_finish();
    end

endmodule
