//=============================================================================
// tb_key_debounce.v -- self-checking testbench for mm_key_debounce
//
// A button does not close once: it chatters.  What matters is that the module
// turns that into *one* press and a clean level, and that it does not believe
// anything until the contact has held still -- the tests below are exactly that
// claim, in the order a real press goes through them:
//
//   1. a bouncing edge produces nothing while it is still bouncing
//   2. it produces exactly one press once the contact holds, and `level` moves
//      with it -- and not a cycle before the hold is up
//   3. holding the button down for many debounce times does not press again
//   4. a bouncing release does not press at all, and the level follows it
//   5. the same sequence on a pin that reads low when pressed (INVERT) behaves
//      identically, which is what the board's buttons need
//=============================================================================
`timescale 1ns / 100ps

module tb_key_debounce;

    localparam integer STABLE = 100;    // debounce time, in clock cycles

    reg  clk = 1'b0;
    reg  rst = 1'b1;
    reg  raw = 1'b0;                    // the pin: high when pressed
    reg  raw_n = 1'b1;                  // the same button, active low

    wire level, press;
    wire level_n, press_n;

    integer errors = 0;
    integer pulses = 0, pulses_n = 0;
    integer i;

    always #5 clk = ~clk;               // 10 ns period

    mm_key_debounce #(
        .STABLE_CYCLES (STABLE)
    ) dut (
        .clk   (clk),
        .rst   (rst),
        .raw   (raw),
        .level (level),
        .press (press)
    );

    mm_key_debounce #(
        .STABLE_CYCLES (STABLE),
        .INVERT        (1)
    ) dut_n (
        .clk   (clk),
        .rst   (rst),
        .raw   (raw_n),
        .level (level_n),
        .press (press_n)
    );

    always @(posedge clk) begin
        if (!rst && press)   pulses   = pulses + 1;
        if (!rst && press_n) pulses_n = pulses_n + 1;
    end

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    // Drive the pin (and its inverted twin) and let the line settle nothing:
    // the caller decides how long to hold.
    task set_raw;
        input v;
        begin
            raw   = v;
            raw_n = ~v;
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        // ---- 1. bouncing: nothing may come out ---------------------------
        for (i = 0; i < 40; i = i + 1) begin
            set_raw(1'b1);  repeat (3) @(posedge clk);
            set_raw(1'b0);  repeat (3) @(posedge clk);
        end
        if (level !== 1'b0 || pulses !== 0) begin
            $display("  FAIL: a bouncing edge got through (level=%b pulses=%0d)",
                     level, pulses);
            errors = errors + 1;
        end else begin
            $display("  bounce       : PASS (40 edges in, nothing out)");
        end

        // ---- 2. one clean press -----------------------------------------
        set_raw(1'b1);
        repeat (STABLE - 2) @(posedge clk);
        if (level !== 1'b0)
            fail("the level moved before the contact had held");
        // it needs the hold plus its own two input flops to have gone through
        repeat (8) @(posedge clk);
        if (level !== 1'b1 || pulses !== 1) begin
            $display("  FAIL: one press gave level=%b pulses=%0d", level, pulses);
            errors = errors + 1;
        end else begin
            $display("  press        : PASS (one pulse, level high)");
        end

        // ---- 3. held down is still one press ----------------------------
        repeat (5 * STABLE) @(posedge clk);
        if (pulses !== 1) begin
            $display("  FAIL: a held button pressed %0d times", pulses);
            errors = errors + 1;
        end else begin
            $display("  hold         : PASS (still one press)");
        end

        // ---- 4. a bouncing release --------------------------------------
        for (i = 0; i < 20; i = i + 1) begin
            set_raw(1'b0);  repeat (2) @(posedge clk);
            set_raw(1'b1);  repeat (2) @(posedge clk);
        end
        set_raw(1'b0);
        repeat (STABLE + 8) @(posedge clk);
        if (level !== 1'b0 || presses_bad(pulses)) begin
            $display("  FAIL: the release did not settle clean (level=%b pulses=%0d)",
                     level, pulses);
            errors = errors + 1;
        end else begin
            $display("  release      : PASS (level low, no extra press)");
        end

        // ---- 5. the active-low pin behaved the same all along -----------
        if (pulses_n !== pulses || level_n !== level) begin
            $display("  FAIL: INVERT instance disagreed (%0d vs %0d pulses)",
                     pulses_n, pulses);
            errors = errors + 1;
        end else begin
            $display("  active low   : PASS (%0d press, same as the active high pin)",
                     pulses_n);
        end

        if (errors == 0)
            $display("RESULT: PASS -- key debounce verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    function presses_bad;
        input integer p;
        begin
            presses_bad = (p != 1);
        end
    endfunction

    initial begin
        #2_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
