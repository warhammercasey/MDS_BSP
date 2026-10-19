// The design's clock domains (mirror the CLK_*_FREQ localparams in src/cain_top.sv).
// CDC tests instantiate one DUT per ordered pair of these, NUM_PAIRS in total.
//
// Pair p runs from clock pair_a(p) to clock pair_b(p):
//    p   0..2   clk_50   -> core, sram, io
//    p   3..5   clk_core -> 50,   sram, io
//    p   6..8   clk_sram -> 50,   core, io
//    p   9..11  clk_io   -> 50,   core, sram
package cain_clocks_pkg;

    localparam int  NUM_CLKS  = 4;
    localparam int  NUM_PAIRS = NUM_CLKS * (NUM_CLKS - 1);
    localparam real FREQ_HZ [NUM_CLKS] = '{
        50.0e6,      // clk_50
        200.0e6,     // clk_core
        100.0e6,     // clk_sram
        73.5714e6    // clk_io
    };

    function automatic string clk_name(int i);
        case (i)
            0:       return "clk_50";
            1:       return "clk_core";
            2:       return "clk_sram";
            default: return "clk_io";
        endcase
    endfunction

    function automatic int pair_a(int p);
        return p / (NUM_CLKS - 1);
    endfunction

    function automatic int pair_b(int p);
        int b = p % (NUM_CLKS - 1);
        return b >= pair_a(p) ? b + 1 : b;
    endfunction

    function automatic string pair_name(int p);
        return {clk_name(pair_a(p)), " -> ", clk_name(pair_b(p))};
    endfunction

endpackage
