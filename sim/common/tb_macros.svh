`ifndef TB_MACROS_SVH
`define TB_MACROS_SVH

// Record a check; logs an error with file/line if ACT !== EXP.
`define CHECK_EQ(ACT, EXP, MSG) \
    begin \
        tb_pkg::tb_checks++; \
        if ((ACT) !== (EXP)) \
            tb_pkg::tb_error($sformatf("%s: got 0x%0h, expected 0x%0h (%s:%0d)", \
                                       MSG, (ACT), (EXP), `__FILE__, `__LINE__)); \
    end

// Record a check; logs an error with file/line if COND is false.
`define CHECK(COND, MSG) \
    begin \
        tb_pkg::tb_checks++; \
        if (!(COND)) \
            tb_pkg::tb_error($sformatf("%s (%s:%0d)", MSG, `__FILE__, `__LINE__)); \
    end

// Standard testbench setup; place once in every top-level TB module.
//  - log timestamps in ns
//  - dump waves when the runner passes +trace=<file> (FST/VCD chosen at build time)
`define TB_INIT \
    initial begin \
        string tb_trace_file; \
        $timeformat(-9, 1, " ns", 0); \
        if ($value$plusargs("trace=%s", tb_trace_file)) begin \
            $dumpfile(tb_trace_file); \
            $dumpvars(0); \
        end \
    end

// Fail the test if it is still running after TIMEOUT (in time units of the caller).
`define TB_WATCHDOG(TIMEOUT) \
    initial begin \
        #(TIMEOUT); \
        tb_pkg::tb_error("Watchdog timeout - simulation did not finish"); \
        tb_pkg::tb_finish(); \
    end

`endif
