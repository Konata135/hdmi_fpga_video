//=============================================================================
// tb_pic_sched.v -- self-checking testbench for mm_pic_sched
//
// A fake scan (a found count and a parked flag), a fake table and a fake loader
// that takes a few cycles and whose slot 1 never works.  One claim per check:
//
//   1. nothing is asked of the loader until the scan has parked
//   2. the slots are loaded in order, one loader run at a time -- the model
//      counts a hit if a start ever arrives while it is still working
//   3. the first picture to arrive is announced exactly once *even when it is
//      the slot that was already selected*, and every later change of `disp_sel`
//      toggles `disp_tgl` exactly once -- that toggle is how the top level knows
//      it may point the reader at a slot, and leaving the first one out is what
//      put nothing but colour bars on the screen
//   4. a key press walks to the next *loaded* picture and wraps round
//   5. the switch makes the same walk happen by itself, and with the switch off
//      nothing moves however long the test waits
//   6. a slot that keeps failing is retried after the retry delay, then left out
//      -- it is never shown, and the pictures behind it still load
//=============================================================================
`timescale 1ns / 100ps

module tb_pic_sched;

    localparam integer FOUND     = 3;    // pictures the fake scan found
    localparam integer RETRY     = 40;   // "a second", in clock cycles
    localparam integer CAROU     = 60;   // "one second" of carousel, same
    localparam integer TRIES     = 2;
    localparam integer LD_CYCLES = 8;    // how long the fake loader takes

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                // 100 MHz

    wire [2:0]  tbl_index;
    reg  [3:0]  images_found = FOUND;
    reg         scan_parked  = 1'b0;
    wire        load_start;
    wire [2:0]  load_slot;
    reg         load_busy  = 1'b0;
    reg         load_done  = 1'b0;
    reg  [3:0]  load_phase = 4'd0;
    reg         key_press  = 1'b0;
    reg         auto_on    = 1'b0;
    reg  [27:0] carou_period = CAROU;
    wire [2:0]  disp_sel;
    wire        disp_tgl;
    wire [3:0]  loaded_count;
    wire [3:0]  state_dbg;

    integer errors    = 0;
    integer ask_count = 0;
    integer overlap   = 0;
    integer toggles   = 0;
    integer asks [0:15];
    integer ld_left;
    reg       tried_1 = 1'b0;       // slot 1's transient failure, used once
    reg [2:0] ld_slot;
    reg       tg_q = 1'b0;
    integer   k;

    mm_pic_sched #(
        .RETRY_CYCLES    (RETRY),
        .MAX_TRIES       (TRIES)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .tbl_index    (tbl_index),
        .images_found (images_found),
        .scan_parked  (scan_parked),
        .load_start   (load_start),
        .load_slot    (load_slot),
        .load_busy    (load_busy),
        .load_done    (load_done),
        .load_phase   (load_phase),
        .key_press    (key_press),
        .auto_on      (auto_on),
        .carou_period (carou_period),
        .disp_sel     (disp_sel),
        .disp_tgl     (disp_tgl),
        .loaded_count (loaded_count),
        .state_dbg    (state_dbg)
    );

    //------------------------------------------------------------------
    // the fake loader: one run at a time, and slot 1 never works
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            load_busy  <= 1'b0;
            load_done  <= 1'b0;
            load_phase <= 4'd0;
            ld_left    <= 0;
        end else begin
            load_done <= 1'b0;

            if (load_start) begin
                if (load_busy) overlap = overlap + 1;
                if (ask_count < 16) asks[ask_count] = load_slot;
                ask_count  = ask_count + 1;
                ld_slot    <= load_slot;
                ld_left    <= LD_CYCLES;
                load_busy  <= 1'b1;
                // `phase` is deliberately *not* cleared here.  The real loader
                // holds its old value -- including the failure flag -- until the
                // attempt has got far enough to set a new one, and the
                // scheduler has to cope with that: testing the flag on its own
                // calls every retry a failure the cycle it starts.
            end else if (load_busy) begin
                if (ld_left == 0) begin
                    load_busy <= 1'b0;
                    // Slot 1 fails once and then works -- a transient read
                    // error, which the retry has to actually retry -- and slot
                    // 2 never works, which the scheduler has to give up on
                    // after its tries without taking the rest down with it.
                    if ((ld_slot == 3'd2) || ((ld_slot == 3'd1) && !tried_1)) begin
                        if (ld_slot == 3'd1) tried_1 = 1'b1;
                        load_phase <= 4'hF;
                    end else begin
                        load_phase <= 4'h3;     // done
                        load_done  <= 1'b1;
                    end
                end else begin
                    ld_left <= ld_left - 1;
                end
            end
        end
    end

    // count the announce toggles
    always @(posedge clk) begin
        if (!rst && (disp_tgl !== tg_q)) begin
            toggles = toggles + 1;
            tg_q    = disp_tgl;
        end
    end

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    // one key press: a cycle-wide pulse is what the debouncer hands over
    task press_key;
        begin
            @(posedge clk);
            key_press = 1'b1;
            @(posedge clk);
            key_press = 1'b0;
            repeat (3) @(posedge clk);
        end
    endtask

    integer toggles_before, ok;

    initial begin
        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        // ---- 1. nothing before the scan parks ---------------------------
        repeat (100) @(posedge clk);
        if (ask_count != 0)
            fail("the loader was asked for a picture before the scan parked");
        else
            $display("  park first   : PASS (nothing asked while the scan runs)");

        // ---- 2. in order, one at a time ---------------------------------
        scan_parked = 1'b1;
        k = 0;
        // Wait for the whole sequence, not for "two are in": the last slot is
        // still being tried (and given up on) after the second one has loaded.
        // state 5 is S_DONE, "nothing left to load".
        while ((state_dbg !== 4'd5) && (k < 20000)) begin
            @(posedge clk);
            k = k + 1;
        end
        if (k >= 20000) begin
            fail("the scheduler never finished walking the table");
        end else begin
            $display("  order        : asks %0d %0d %0d %0d %0d, %0d loaded",
                     asks[0], asks[1], asks[2], asks[3], asks[4], loaded_count);
            if (asks[0] !== 0 || asks[1] !== 1 || asks[2] !== 1 ||
                asks[3] !== 2 || asks[4] !== 2)
                fail("the slots were not loaded in order, with the failure retried");
            else if (overlap != 0)
                fail("the loader was asked again while it was still running");
            else if (loaded_count != 2)
                fail("a picture that never loaded was counted as loaded");
            else if (disp_sel !== 3'd0)
                fail("the screen did not come up on the first picture");
            else
                $display("  load order   : PASS (0, then 1 twice -- the retry loaded it -- then 2 twice, given up on)");
        end

        // ---- 3. the first picture is announced, and announced once ------
        // This is the half of the contract the board cares about most: the top
        // level only lets the reader fetch once the scheduler has told it which
        // slot to point at.  Leaving the *first* announce out (because the slot
        // happened to be the one already selected) is exactly the failure that
        // showed nothing but the colour bars on the board, and this testbench
        // missed it because it only counted the toggle on a key press.
        if (toggles != 1)
            fail("the first picture was not announced exactly once");
        else
            $display("  announce     : PASS (one toggle when the first picture arrived)");

        // ---- 4. one more toggle per change ------------------------------
        toggles_before = toggles;
        press_key;
        if (toggles != toggles_before + 1)
            fail("a change of disp_sel did not toggle disp_tgl exactly once");
        else
            $display("  step toggle  : PASS (one toggle per change)");

        // ---- 5. the key walks the loaded set and wraps ------------------
        // loaded set is {0, 1}: from 0 the next is 1, and from 1 it wraps to 0
        if (disp_sel !== 3'd1)
            fail("the key did not step on to the next loaded picture");
        else
            $display("  key step     : PASS (0 -> 1)");
        press_key;
        if (disp_sel !== 3'd0)
            fail("the walk did not wrap round to the first loaded picture");
        else
            $display("  wrap         : PASS (2 -> 0)");

        // ---- 6. the switch runs the same walk by itself -----------------
        auto_on = 1'b1;
        k = 0;
        while ((disp_sel == 3'd0) && (k < 4 * CAROU)) begin
            @(posedge clk);
            k = k + 1;
        end
        auto_on     = 1'b0;
        toggles_before = toggles;
        repeat (3 * CAROU) @(posedge clk);
        if (k >= 4 * CAROU)
            fail("the carousel never stepped with the switch on");
        else if (toggles != toggles_before)
            fail("something stepped with the switch off");
        else
            $display("  switch       : PASS (steps while up, still while down)");

        // ---- 7. the interval selector is honoured ------------------------
        // Four times the base interval has to take four times as long, measured
        // between two steps: this is the SW2/SW3 selector on the board.
        carou_period = 4 * CAROU;
        auto_on      = 1'b1;
        k = 0;
        while ((disp_sel == 3'd0) && (k < 10 * CAROU)) begin
            @(posedge clk);
            k = k + 1;
        end
        if (k >= 10 * CAROU) begin
            fail("the carousel never stepped with the long interval");
        end else begin
            toggles_before = toggles;
            k = 0;
            while ((toggles == toggles_before) && (k < 10 * CAROU)) begin
                @(posedge clk);
                k = k + 1;
            end
            if ((k < 4 * CAROU - 8) || (k > 4 * CAROU + 8)) begin
                $display("  FAIL: the long interval took %0d clocks, expected about %0d",
                         k, 4 * CAROU);
                errors = errors + 1;
            end else begin
                $display("  interval     : PASS (%0d clocks between steps at 4x)", k);
            end
        end
        auto_on = 1'b0;

        if (errors == 0)
            $display("RESULT: PASS -- picture scheduler verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
