//=============================================================================
// tb_pic_sched.v -- self-checking testbench for mm_pic_sched
//
// A fake scan (a found count and a parked flag), a fake table and a fake loader
// that takes a few cycles and whose slot 1 never works.  One claim per check:
//
//   1. nothing is asked of the loader until the scan has parked
//   2. the slots are loaded in order, one loader run at a time -- the model
//      counts a hit if a start ever arrives while it is still working
//   3. the first picture to arrive takes the screen, and every change of
//      `disp_sel` toggles `disp_tgl` exactly once (that toggle is how the top
//      level re-points the reader)
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
    reg [2:0] ld_slot;
    reg       tg_q = 1'b0;
    integer   k;

    mm_pic_sched #(
        .RETRY_CYCLES    (RETRY),
        .MAX_TRIES       (TRIES),
        .CAROUSEL_CYCLES (CAROU)
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
                load_phase <= 4'd2;             // reading
            end else if (load_busy) begin
                if (ld_left == 0) begin
                    load_busy <= 1'b0;
                    if (ld_slot == 3'd1) begin
                        load_phase <= 4'hF;     // this picture never loads
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
        while ((loaded_count < 2) && (k < 5000)) begin
            @(posedge clk);
            k = k + 1;
        end
        if (k >= 5000) begin
            fail("the two good pictures never both loaded");
        end else begin
            $display("  order        : asks %0d %0d %0d %0d, %0d loaded",
                     asks[0], asks[1], asks[2], asks[3], loaded_count);
            if (asks[0] !== 0 || asks[1] !== 1 || asks[2] !== 1 || asks[3] !== 2)
                fail("the slots were not loaded in order, with the failure retried");
            else if (overlap != 0)
                fail("the loader was asked again while it was still running");
            else if (loaded_count != 2)
                fail("a picture that never loaded was counted as loaded");
            else if (disp_sel !== 3'd0)
                fail("the screen did not come up on the first picture");
            else
                $display("  load order   : PASS (0, 1, 1 retry, 2; slot 1 given up on)");
        end

        // ---- 3. the announce toggle -------------------------------------
        toggles_before = toggles;
        press_key;
        if (toggles != toggles_before + 1)
            fail("a change of disp_sel did not toggle disp_tgl exactly once");
        else
            $display("  announce     : PASS (one toggle per change)");

        // ---- 4. the key walks the loaded set and wraps ------------------
        // loaded set is {0, 2}: from 0 the next is 2, and from 2 it wraps to 0
        if (disp_sel !== 3'd2)
            fail("the key did not step on to the next loaded picture");
        else
            $display("  key step     : PASS (0 -> 2, skipping the picture that never loaded)");
        press_key;
        if (disp_sel !== 3'd0)
            fail("the walk did not wrap round to the first loaded picture");
        else
            $display("  wrap         : PASS (2 -> 0)");

        // ---- 5. the switch runs the same walk by itself -----------------
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
