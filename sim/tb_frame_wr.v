//=============================================================================
// tb_frame_wr.v -- self-checking testbench for mm_frame_wr
//
// The whole write path is exercised end to end: bytes go in on the 50 MHz
// side, and the testbench then looks *inside the SDRAM model's array* to see
// what actually landed.  That checks the packing order (first byte of a group
// into bits [7:0]), the address sequence, and that a second frame neither
// disturbs the first nor inherits stale words from an abandoned one.
//
//  1. 482 words of a known pattern: every word in the store matches
//  2. a second frame at another base leaves the first one alone
//  3. a frame that is abandoned half way is flushed, so the next frame starts
//     at its own base instead of inheriting the leftovers
//  4. the packing FIFO never overflowed and frame_done came exactly once
//=============================================================================
`timescale 1ns / 100ps

module tb_frame_wr;

    reg         clk, clk_byte, clk_sft;
    reg         rst, rst_byte;
    reg         start;
    reg  [20:0] base_word;
    reg  [17:0] words_total;
    reg         byte_valid;
    reg  [7:0]  byte_data;
    wire [7:0]  overflow_count;
    wire        busy;
    wire [20:0] wr_addr;
    wire [31:0] wr_data;
    wire        wr_req;
    wire        wr_done;
    wire        ready;
    wire        frame_done;
wire        frame_done_tgl;
    wire [17:0] words_written;

    integer errors;
    integer i, k;
    integer done_pulses;
    reg [31:0] expected;
    reg [7:0]  b;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial clk      = 1'b0;
    initial clk_byte = 1'b0;
    initial clk_sft  = 1'b1;            // the device clock: 125 MHz, 180 degrees
always #8  clk      = ~clk;             // 62.5 MHz memory side
    always #10 clk_byte = ~clk_byte;
always #8  clk_sft  = ~clk_sft;         // 62.5 MHz, 180 degrees, device side

    // the byte pattern the testbench feeds; the checker recomputes it
    function [7:0] pat;
        input integer n;
        begin
            pat = (n * 7 + 11) ^ 8'hA5;
        end
    endfunction

    mm_sdram #(
        .CAS_LAT        (3),
        .T_RP           (3),
        .T_RCD          (3),
        .T_RFC          (9),
        .T_MRD          (2),
        .T_WR           (2),
        .INIT_WAIT      (50),
        .REFRESH_PERIOD (1953)
    ) u_ctrl (
        .clk        (clk),
        .clk_dev   (clk),
        .clk_sft    (clk_sft),
        .rst        (rst),
        .ready      (ready),
        .rd_adj    (/*the sample code the probe finds; the model wants one cycle later*/ 3'd1),
        .br_adj    ( 3'd1),
        .wr_req     (wr_req),
        .wr_addr    (wr_addr),
        .wr_data    (wr_data),
        .wr_dm      (4'h0),
        .wr_done    (wr_done),
        .rd_req     (1'b0),
        .rd_addr    (21'd0),
        .rd_dv      (),
        .rd_data    (),
        .init_done  (),
        .state_dbg  (),
        .echo_ok   (),
        .br_req     (1'b0),
        .br_addr    (21'd0),
        .br_len     (9'd0),
        .br_dv      (),
        .br_data    (),
        .br_busy    ()
    );

    mm_frame_wr #(
        .AFIFO_AW (6)                    // 64 words is ample when fed at SPI pace
    ) dut (
        .clk_byte       (clk_byte),
        .rst_byte       (rst_byte),
        .start          (start),
        .base_word      (base_word),
        .words_total    (words_total),
        .byte_valid     (byte_valid),
        .byte_data      (byte_data),
        .overflow_count (overflow_count),
        .busy           (busy),
        .clk            (clk),
        .rst            (rst),
        .wr_addr        (wr_addr),
        .wr_data        (wr_data),
        .wr_req         (wr_req),
        .wr_done        (wr_done),
        .ready          (ready),
        .frame_done     (frame_done),
        .frame_done_tgl (frame_done_tgl),
        .words_written  (words_written)
    );

    // count frame_done pulses
    always @(posedge clk)
        if (frame_done)
            done_pulses = done_pulses + 1;

    // one byte, paced like the SPI stream (about 200 ns apart)
    task feed;
        input [7:0] value;
        begin
            @(negedge clk_byte);
            byte_data  = value;
            byte_valid = 1'b1;
            @(negedge clk_byte);
            byte_valid = 1'b0;
            repeat (9) @(negedge clk_byte);
        end
    endtask

    task feed_frame;
        input integer first_byte;
        input integer n_bytes;
        begin
            for (i = 0; i < n_bytes; i = i + 1)
                feed(pat(first_byte + i));
        end
    endtask

    integer pulses_before;

    task wait_done;
        input integer limit;
        begin
            k = 0;
            // frame_done is one cycle wide and can fire while the bytes are
            // still being fed, so wait for the *count* to move, not for the
            // level to be high at the moment we happen to look
            while ((done_pulses == pulses_before) && (k < limit)) begin
                @(posedge clk);
                k = k + 1;
            end
            if (done_pulses == pulses_before) fail("frame_done never came");
            repeat (40) @(posedge clk);      // let the last write land
        end
    endtask

    // check a whole frame inside the device model's array
    task check_frame;
        input integer base;
        input integer first_byte;
        input integer n_words;
        begin
            for (k = 0; k < n_words; k = k + 1) begin
                expected = {pat(first_byte + 4*k + 3), pat(first_byte + 4*k + 2),
                            pat(first_byte + 4*k + 1), pat(first_byte + 4*k + 0)};
                if (u_ctrl.u_sdram.mem[base + k] !== expected) begin
                    errors = errors + 1;
                    $display("  FAIL: word %0d at %0d holds %h, expected %h",
                             k, base, u_ctrl.u_sdram.mem[base + k], expected);
                    if (errors > 6) k = n_words;     // do not flood the log
                end
            end
        end
    endtask

    integer errors_before;
    integer base_a, base_b;
    integer words_a, words_b;

    initial begin
        errors      = 0;
        done_pulses = 0;
        rst         = 1'b1;
        rst_byte    = 1'b1;
        start       = 1'b0;
        base_word   = 21'd0;
        words_total = 18'd0;
        byte_valid  = 1'b0;
        byte_data   = 8'd0;

        repeat (4) @(posedge clk);
        repeat (4) @(posedge clk_byte);
        rst      = 1'b0;
        rst_byte = 1'b0;
        repeat (200) @(posedge clk);        // let the SDRAM finish its init

        base_a  = 21'h0_0100;
        words_a = 482;                      // one row of 480 plus two words

        // ---- 1. a frame of 482 words -----------------------------------
        errors_before = errors;
        pulses_before = done_pulses;
        base_word   = base_a;
        words_total = words_a;
        @(negedge clk_byte);
        start = 1'b1;
        @(negedge clk_byte);
        start = 1'b0;
        feed_frame(0, words_a * 4);
        wait_done(200000);
        check_frame(base_a, 0, words_a);
        if (words_written !== words_a) begin
            $display("  FAIL: words_written = %0d, expected %0d", words_written, words_a);
            errors = errors + 1;
        end
        if (errors == errors_before)
            $display("  frame 0      : PASS (%0d words, packing and addresses right)",
                     words_a);

        // ---- 2. a second frame must not touch the first ----------------
        errors_before = errors;
        base_b  = 21'h0_2000;
        words_b = 100;
        pulses_before = done_pulses;
        base_word   = base_b;
        words_total = words_b;
        @(negedge clk_byte);
        start = 1'b1;
        @(negedge clk_byte);
        start = 1'b0;
        feed_frame(1000, words_b * 4);
        wait_done(200000);
        check_frame(base_b, 1000, words_b);
        check_frame(base_a, 0, words_a);            // the first frame is intact
        if (errors == errors_before)
            $display("  frame 1      : PASS (%0d words, frame 0 untouched)", words_b);

        // ---- 3. an abandoned frame is flushed --------------------------
        errors_before = errors;
        pulses_before = done_pulses;
        base_word   = 21'h0_3000;
        words_total = 50;
        @(negedge clk_byte);
        start = 1'b1;
        @(negedge clk_byte);
        start = 1'b0;
        feed_frame(2000, 9 * 4);                    // 9 words, then walk away
        repeat (200) @(posedge clk);
        // now start a fresh frame somewhere else and check it starts at its base
        pulses_before = done_pulses;
        base_word   = 21'h0_4000;
        words_total = 20;
        @(negedge clk_byte);
        start = 1'b1;
        @(negedge clk_byte);
        start = 1'b0;
        feed_frame(3000, 20 * 4);
        wait_done(200000);
        check_frame(21'h0_4000, 3000, 20);
        if (errors == errors_before)
            $display("  abandoned    : PASS (leftovers flushed, new frame starts clean)");

        // ---- 4. health -------------------------------------------------
        errors_before = errors;
        if (overflow_count !== 8'd0) begin
            $display("  FAIL: the packing FIFO overflowed %0d times", overflow_count);
            errors = errors + 1;
        end
        if (done_pulses !== 3) begin
            $display("  FAIL: frame_done pulsed %0d times, expected 3", done_pulses);
            errors = errors + 1;
        end
        if (u_ctrl.u_sdram.errors !== 0) begin
            $display("  FAIL: the SDRAM model reported %0d protocol violations",
                     u_ctrl.u_sdram.errors);
            errors = errors + 1;
        end
        if (errors == errors_before)
            $display("  health       : PASS (no FIFO overflow, 3 frame_done, no SDRAM violation)");

        if (errors == 0)
            $display("RESULT: PASS -- frame write path verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
