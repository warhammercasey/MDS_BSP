// Shared testbench utilities. Compiled automatically into every sim by sim/run.py.
//
// The runner decides PASS/FAIL from the final verdict line printed by tb_finish(),
// so every testbench must end by calling tb_finish().
package tb_pkg;
    timeunit 1ns;
    timeprecision 1ps;

    int unsigned tb_errors = 0;
    int unsigned tb_checks = 0;

    function automatic void tb_info(string msg);
        $display("[%t] INFO:  %s", $realtime, msg);
    endfunction

    function automatic void tb_warn(string msg);
        $display("[%t] WARN:  %s", $realtime, msg);
    endfunction

    function automatic void tb_error(string msg);
        tb_errors++;
        $display("[%t] ERROR: %s", $realtime, msg);
    endfunction

    // Error the first time `key` occurs; later occurrences (from anywhere, e.g. several
    // instances of a device model) are only counted and summarized by tb_finish().
    int unsigned tb_once_counts[string];

    function automatic void tb_error_once(string key, string msg);
        if (tb_once_counts.exists(key) == 0) begin
            tb_once_counts[key] = 0;
            tb_error({msg, " (repeats are counted, not printed)"});
        end
        tb_once_counts[key]++;
    endfunction

    // Print the verdict line the runner looks for, then end the sim.
    function automatic void tb_finish();
        if (tb_checks == 0)
            tb_error("No checks were performed");

        foreach (tb_once_counts[key])
            if (tb_once_counts[key] > 1)
                tb_info($sformatf("'%s' occurred %0d times", key, tb_once_counts[key]));

        if (tb_errors == 0)
            $display("TEST PASSED (%0d checks)", tb_checks);
        else
            $display("TEST FAILED (%0d errors, %0d checks)", tb_errors, tb_checks);
        $finish;
    endfunction

endpackage
