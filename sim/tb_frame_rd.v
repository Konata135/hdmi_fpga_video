//=============================================================================
// tb_frame_rd.v -- self-checking testbench for mm_frame_rd
//
// A small picture (4 rows x 16 pixels) is written straight into the SDRAM
// model's array in BMP order, with each byte carrying the thing it stands for:
//
//   byte 0 of a pixel (B) = 0x10 + stored row
//   byte 1 of a pixel (G) = 0x20 + column
//   byte 2 of a pixel (R) = 0xA5
//
// The testbench then plays the part of the video timing and collects what the
// reader hands out.  A correct reader must produce, for display row v,
//
//   {0xA5, 0x20 + column, 0x10 + (ROWS - 1 - v)}
//
// i.e. the marker in the top byte, the column in the middle, and the *flipped*
// row in the bottom byte, which nails the row order (BMP is bottom up), the
// column order and the B,G,R -> R,G,B swap in one comparison.
//
// Crucially the check is on the *position* as well as the order: the pixel this
// module hands out at display position n of a frame has to be pixel n of the
// picture.  The order-only version of this check (collect the pixels it hands
// out and compare them one by one) cannot see an offset at all -- a picture
// shifted sideways within the line, or down by whole lines, hands out exactly
// the same sequence.  Position is the *display's*, counted in this testbench's
// own de cycles, so the picture is judged against the screen and not against
// itself.
//
// The top level's takeover rule is modelled here too, so the test covers the
// whole sequence the board runs and not just the reader: the picture becoming
// the source is a pulse into the reader's `restart`, and the unmask is a
// registered decision taken on the blanking pixel *before* a frame boundary
// (frame_next) out of the reader's "aligned and primed" signal.  Taking it on
// frame_first instead would put the frame's first pixel in the pattern, and the
// position check is what notices that.
//
//  1. a picture that becomes the source part way through a frame -- the
//     condition that used to leave it offset, by a different amount after every
//     reset -- is on the screen from the frame's first pixel, in the right
//     place, with nothing missing and no underrun
//  2. it stays aligned frame after frame: the walk goes past exactly one picture
//     per display frame, or the next frame's positions would be out
//  3. a restart in the middle of a frame re-aligns it again, three times, landing
//     at a different point in the frame each time
//  4. switching to another frame at another base reads that frame
//  5. the sequence the top level uses on a switch -- the base change and the
//     restart toggle, together -- leaves it walking on the new picture
//  6. with `active` low the reader stops fetching
//  7. the SDRAM model saw no protocol violation
//=============================================================================
`timescale 1ns / 100ps

module tb_frame_rd;

    localparam integer ROW_WORDS = 12;      // 48 bytes = 16 pixels per row
    localparam integer ROWS      = 4;
    localparam integer PIXELS    = ROW_WORDS * 4 / 3;
    localparam integer FRAME_PIX = ROWS * PIXELS;   // one whole display frame
    localparam integer BLANK     = 4;               // blank pixels between lines

    reg         clk, clk_sft, pix_clk;
    reg         rst, pix_rst;
    reg  [20:0] base_word;
    reg         loaded;             // the picture is in the store: this is the
                                    // top level's frame_display, the thing that
                                    // starts all of this off
    reg         sw_restart;         // this side asking for a restart that is not
                                    // a load -- what M6 will do on a switch
    reg         pix_de;
    reg         pix_frame_next;     // one pixel clock before a frame's first
                                    // active pixel, as the timing generator has it
    wire        rd_br_req;
    wire [20:0] rd_br_addr;
    wire [8:0]  rd_br_len;
    wire        rd_br_dv;
    wire [31:0] rd_br_data;
    wire        rd_br_busy;
    wire        ready;
    wire        rd_flush_done;
    wire [31:0] words_fetched;
    wire        pix_ready;
    wire [23:0] pix_rgb;
    wire        pix_valid;
    wire [15:0] pix_underrun;

    integer errors = 0, i, r, c, w, k;
    // how often the reader asked the pixel side for a drain: a request that is
    // never answered has to be asked again, so this is the counter the resend
    // test watches
    integer resync_toggles = 0;
    reg     rs_q = 1'b0;
    integer count_fetched_before, errors_before;
    integer clean_before, shown_before;

    always @(posedge clk) begin
        if (!rst && (dut.resync !== rs_q)) begin
            resync_toggles = resync_toggles + 1;
            rs_q           = dut.resync;
        end
    end
    integer base_a, base_b;

    // display frame accounting, and the checks that go with it
    integer dpos = 0;               // which pixel of the display frame is on the
                                    // wire right now
    integer frames_played = 0;
    integer shown_frames  = 0;      // frames the picture was on the screen for,
                                    // end to end
    integer clean_frames  = 0;      // ... and every pixel of them was the right
                                    // one, in the right place
    integer shown_in_frame = 0;
    integer bad_in_frame   = 0;
    integer miss_in_frame  = 0;
    reg     skipped = 1'b0;         // the reader was re-arming during this frame,
                                    // so nothing is claimed about what was on it

    // what the display side should see: R,G,B with the marker, the column and
    // the flipped row.  A function is used so the field widths stay 8 bits.
    function [23:0] expect_rgb;
        input integer v;
        input integer cc;
        reg [7:0] rbyte, gbyte, bbyte;
        reg [7:0] flipped;
        begin
            flipped = ROWS - 1 - v;             // stored row this line needs
            rbyte   = 8'hA5;
            gbyte   = 8'h20 + cc[7:0];
            bbyte   = 8'h10 + flipped;
            expect_rgb = {rbyte, gbyte, bbyte};
        end
    endfunction

    // what display position n of a frame must show
    function [23:0] expect_pos;
        input integer n;
        begin
            expect_pos = expect_rgb(n / PIXELS, n % PIXELS);
        end
    endfunction

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial clk      = 1'b0;
    initial clk_sft  = 1'b1;
    initial pix_clk  = 1'b0;
always #8  clk     = ~clk;              // 62.5 MHz memory side
always #8  clk_sft = ~clk_sft;          // ... 180 degrees, to the device
    // The two clocks are deliberately not phase-aligned: the handshakes here are
    // pulses a few nanoseconds wide crossing between them, and whether a
    // sampling edge lands inside one is a phase question.  Having them start
    // together is the easy case, so the pixel clock starts half a period late.
    always begin #10; forever #20 pix_clk = ~pix_clk; end   // 25 MHz, offset

    // what a stored byte should be
    function [7:0] stored_byte;
        input integer row;
        input integer j;
        integer cc, kk;
        begin
            cc = j / 3;
            kk = j % 3;
            if (kk == 0)      stored_byte = 8'h10 + row;    // B
            else if (kk == 1) stored_byte = 8'h20 + cc;     // G
            else              stored_byte = 8'hA5;          // R
        end
    endfunction

    mm_sdram #(
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
        .wr_req     (1'b0),
        .wr_addr    (21'd0),
        .wr_data    (32'd0),
        .wr_dm      (4'h0),
        .wr_done    (),
        .rd_req     (1'b0),
        .rd_addr    (21'd0),
        .rd_dv      (),
        .rd_data    (),
        .init_done  (),
        .state_dbg  (),
        .echo_ok   (),
        .br_req     (rd_br_req),
        .br_addr    (rd_br_addr),
        .br_len     (rd_br_len),
        .br_dv      (rd_br_dv),
        .br_data    (rd_br_data),
        .br_busy    (rd_br_busy)
    );

    //------------------------------------------------------------------
    // the top level's takeover rule, modelled here so the test covers the whole
    // sequence the board runs rather than the reader on its own
    //
    // `loaded` is frame_display: two flops into the memory domain and the rise of
    // *that* asks the reader to restart (a level is no use as an edge), two flops
    // into the pixel domain, and the unmask is decided on frame_next -- the
    // blanking pixel before the boundary -- out of the reader's own "aligned and
    // primed" answer.
    //------------------------------------------------------------------
    reg  active_m1, active_m2, active_d;
    reg  sw_restart_d;
    reg  restart;
    reg  loaded_s1, loaded_s2, ready_s1, ready_s2, show;

    always @(posedge clk) begin
        if (rst) begin
            active_m1    <= 1'b0;
            active_m2    <= 1'b0;
            active_d     <= 1'b0;
            sw_restart_d <= 1'b0;
            restart      <= 1'b0;
        end else begin
            active_m1    <= loaded;
            active_m2    <= active_m1;
            active_d     <= active_m2;
            sw_restart_d <= sw_restart;
            if ((active_m2 && !active_d) || (sw_restart && !sw_restart_d))
                restart <= ~restart;
        end
    end

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            loaded_s1 <= 1'b0;
            loaded_s2 <= 1'b0;
            ready_s1  <= 1'b0;
            ready_s2  <= 1'b0;
            show      <= 1'b0;
        end else begin
            loaded_s1 <= loaded;
            loaded_s2 <= loaded_s1;
            ready_s1  <= pix_ready;
            ready_s2  <= ready_s1;
            if (pix_frame_next)         // decide before the boundary, not on it
                show <= loaded_s2 && ready_s2;
        end
    end

    mm_frame_rd #(
        .ROW_WORDS (ROW_WORDS),
        .ROWS      (ROWS),
        .FIFO_AW   (5),
        .TOPUP     (20),
        .BURST_MAX (16)
    ) dut (
        .clk           (clk),
        .rst           (rst),
        .base_word     (base_word),
        .active        (active_m2),
        .restart       (restart),
        .br_req        (rd_br_req),
        .br_addr       (rd_br_addr),
        .br_len        (rd_br_len),
        .br_dv         (rd_br_dv),
        .br_data       (rd_br_data),
        .br_busy       (rd_br_busy),
        .ready         (ready),
        .flush_done    (rd_flush_done),
        .words_fetched (words_fetched),
        .pix_ready     (pix_ready),
        .pix_clk       (pix_clk),
        .pix_rst       (pix_rst),
        .pix_de        (pix_de),
        .pix_frame_next(pix_frame_next),
        .pix_rgb       (pix_rgb),
        .pix_valid     (pix_valid),
        .pix_underrun  (pix_underrun)
    );

    //------------------------------------------------------------------
    // the checks: what is on the wire at each display position of each frame
    //
    // Frames are closed at the testbench's own frame boundaries, so the picture
    // is judged against the screen.  A frame where the picture was on screen from
    // its first pixel to its last is counted, and it has to have been right
    // everywhere: the pixel at each display position, and a pixel at all (a
    // missing one repeats the previous pixel, which shifts everything after it).
    //------------------------------------------------------------------
    always @(posedge pix_clk) begin
        if (pix_rst) begin
            dpos           = 0;
            shown_in_frame = 0;
            bad_in_frame   = 0;
            miss_in_frame  = 0;
            skipped        = 1'b0;
        end else if (pix_frame_next) begin
            // the last blank pixel of the frame that is ending: close it
            frames_played = frames_played + 1;
            if (skipped) begin
                $display("  frame %0d: realigning", frames_played);
            end else if (shown_in_frame == 0) begin
                $display("  frame %0d: pattern", frames_played);
            end else if (shown_in_frame != FRAME_PIX) begin
                $display("  FAIL: the picture took the screen part way through frame %0d (%0d of %0d pixels)",
                         frames_played, shown_in_frame, FRAME_PIX);
                errors = errors + 1;
            end else begin
                shown_frames = shown_frames + 1;
                if ((bad_in_frame == 0) && (miss_in_frame == 0)) begin
                    clean_frames = clean_frames + 1;
                    $display("  frame %0d: picture, every pixel in place", frames_played);
                end else begin
                    $display("  FAIL: frame %0d was shown but %0d pixels were wrong and %0d missing",
                             frames_played, bad_in_frame, miss_in_frame);
                    errors = errors + 1;
                end
            end
            dpos           = 0;
            shown_in_frame = 0;
            bad_in_frame   = 0;
            miss_in_frame  = 0;
            skipped        = 1'b0;
        end else if (pix_de) begin
            if (show) begin
                shown_in_frame = shown_in_frame + 1;
                if (dpos >= FRAME_PIX) begin
                    $display("  FAIL: the display frame is longer than this testbench plays");
                    errors = errors + 1;
                end else if (!pix_valid) begin
                    miss_in_frame = miss_in_frame + 1;
                end else if (pix_rgb !== expect_pos(dpos)) begin
                    if (bad_in_frame < 4)
                        $display("  FAIL: display position %0d (row %0d col %0d) shows %h, expected %h",
                                 dpos, dpos / PIXELS, dpos % PIXELS, pix_rgb, expect_pos(dpos));
                    bad_in_frame = bad_in_frame + 1;
                    errors = errors + 1;
                end
            end
            if (dut.align_wait)
                skipped = 1'b1;
            dpos = dpos + 1;
        end
    end

    // fill a stored picture, in BMP order, into the device model's array
    task fill_frame;
        input integer base;
        input integer marker;
        reg [31:0] word;
        begin
            for (r = 0; r < ROWS; r = r + 1) begin
                for (w = 0; w < ROW_WORDS; w = w + 1) begin
                    word = 32'd0;
                    for (k = 0; k < 4; k = k + 1)
                        word[8*k +: 8] = stored_byte(r, 4*w + k) ^ marker;
                    u_ctrl.u_sdram.mem[base + r*ROW_WORDS + w] = word;
                end
            end
        end
    endtask

    // Play the display: the blanking pixel that says the next pixel clock starts
    // a frame, then a line of active pixels and the blank gap after it.  (The
    // reader hands out one byte per pixel clock and a pixel is three bytes, so
    // the shifter is what turns those bytes into one pixel per active clock; the
    // timing here only has to look like the real thing in that the frame boundary
    // arrives in the blanking, one clock before the first active pixel.)
    task frame_boundary;
        begin
            @(negedge pix_clk);
            pix_frame_next = 1'b1;
            @(negedge pix_clk);
            pix_frame_next = 1'b0;
        end
    endtask

    task play_line;
        begin
            for (c = 0; c < PIXELS; c = c + 1) begin
                @(negedge pix_clk);
                pix_de = 1'b1;
            end
            @(negedge pix_clk);
            pix_de = 1'b0;
            repeat (BLANK) @(negedge pix_clk);
        end
    endtask

    task play_frame;
        begin
            frame_boundary;
            for (i = 0; i < ROWS; i = i + 1)
                play_line;
        end
    endtask

    // A restart that is not a load, i.e. what M6 will do when it switches
    // picture: the reader's `restart` is a toggle, so this is one memory clock
    // wide.
    task pulse_restart;
        begin
            @(negedge clk);
            sw_restart = 1'b1;
            @(negedge clk);
            sw_restart = 1'b0;
        end
    endtask

    initial begin
        errors         = 0;
        rst            = 1'b1;
        pix_rst        = 1'b1;
        base_word      = 21'd0;
        loaded         = 1'b0;
        sw_restart     = 1'b0;
        pix_de         = 1'b0;
        pix_frame_next = 1'b0;

        repeat (4) @(posedge clk);
        repeat (4) @(posedge pix_clk);
        rst     = 1'b0;
        pix_rst = 1'b0;
        repeat (200) @(posedge clk);            // let the SDRAM initialise

        base_a = 21'h0_0100;
        base_b = 21'h0_0800;
        fill_frame(base_a, 0);
        fill_frame(base_b, 8'h40);              // a visibly different picture

        // ---- 1. the picture becomes the source part way through a frame ----
        // That is the condition which used to leave it offset: `loaded` rises in
        // the middle of a display frame, exactly as it does on the board when the
        // load finishes, and only the reader's own realignment can make the
        // picture line up from there.  The frames played here are also test 2:
        // the position check is what says the walk goes past exactly one picture
        // per display frame, because if it did not, the frames after the first
        // one would be out of place.
        base_word = base_a;
        play_frame;                     // a frame of pattern
        frame_boundary;                 // ... and part way into the next one
        play_line;
        repeat (PIXELS / 2) begin
            @(negedge pix_clk);
            pix_de = 1'b1;
        end
        loaded = 1'b1;
        @(negedge pix_clk);
        pix_de = 1'b0;
        repeat (4) play_frame;

        if (shown_frames < 2) begin
            $display("  FAIL: the picture reached the screen only %0d times in those frames",
                     shown_frames);
            errors = errors + 1;
        end else if (clean_frames < shown_frames) begin
            $display("  FAIL: %0d of %0d shown frames were not the picture, in place",
                     shown_frames - clean_frames, shown_frames);
            errors = errors + 1;
        end else begin
            $display("  takeover     : PASS (%0d frames shown, all in place, from each frame's first pixel)",
                     clean_frames);
        end

        if (pix_underrun !== 16'd0) begin
            $display("  FAIL: %0d pixel underruns", pix_underrun);
            errors = errors + 1;
        end

        // ---- 2. a restart in the middle of a frame re-aligns it again ------
        // Wherever it lands, the picture has to be back on the screen in place
        // within a couple of frames: that is what M6's picture switch will need.
        for (r = 0; r < 3; r = r + 1) begin
            errors_before = errors;
            clean_before  = clean_frames;
            shown_before  = shown_frames;
            repeat (3 + 11 * r) @(negedge pix_clk);  // land it at a different
            pulse_restart;                           // point in the frame each time
            repeat (5) play_frame;
            if ((clean_frames - clean_before) < 2) begin
                $display("  FAIL: restart %0d: %0d clean frames after it (%0d shown)",
                         r, clean_frames - clean_before, shown_frames - shown_before);
                errors = errors + 1;
            end else begin
                $display("  restart %0d    : PASS (realigned, %0d clean frames in the 5 that followed)",
                         r, clean_frames - clean_before);
            end
        end

        // ---- 3. switching picture: the next fetch must be the new one ------
        // Checked at the memory side, which is where it matters: after a base
        // change the very next fetch has to come from the new picture's last
        // row, not from wherever the old walk had got to.  (The display side is
        // only expected to be mid-frame during a switch when the top level
        // switches at a frame boundary, which M6 does.)
        errors_before = errors;
        loaded    = 1'b0;
        repeat (200) @(posedge clk);        // let the reader settle to idle
        base_word = base_b;
        loaded    = 1'b1;                   // ... and let it read the new one
        // wait for a request and look at where it points
        k = 0;
        while (!rd_br_req && (k < 20000)) begin
            @(posedge clk);
            k = k + 1;
        end
        if (k >= 20000) begin
            fail("no fetch after switching picture");
        end else if (rd_br_addr !== (base_b + (ROWS - 1) * ROW_WORDS)) begin
            $display("  FAIL: first fetch after the switch is %h, expected %h",
                     rd_br_addr, base_b + (ROWS - 1) * ROW_WORDS);
            errors = errors + 1;
        end else if (dut.dbg_flush_words == 0) begin
            // the old picture's bytes must have been thrown away, not handed
            // to the display as if they belonged to the new one
            fail("the switch did not flush the old picture out of the buffer");
        end else begin
            $display("  switch       : PASS (flushed %0d stale bytes, the next fetch is the new row)",
                     dut.dbg_flush_words);
        end

        // ---- 4. the switch sequence the top level actually uses -------------
        // A switch changes the base *and* toggles `restart` in the same cycle,
        // so this module is armed once and has to come back up on the new
        // picture: re-point the walk, drain, refill and hand out pixels again.
        // (This is also the sequence that would break it if the two ever became
        // two separate arms: the second `resync` toggle would cancel the first
        // inside the pixel side's synchroniser and the reader would sit in
        // M_FLUSH for ever -- see the note in mm_frame_rd.)
        errors_before = errors;
        clean_before  = clean_frames;
        shown_before  = shown_frames;
        loaded        = 1'b0;
        repeat (200) @(posedge clk);        // let it settle
        @(negedge clk);
        base_word  = base_a;                // the base change ...
        sw_restart = 1'b1;                  // ... and the restart, together
        loaded     = 1'b1;
        @(negedge clk);
        sw_restart = 1'b0;
        repeat (4) play_frame;
        if ((clean_frames - clean_before) < 2) begin
            $display("  FAIL: a base change and a restart a clock apart left the reader stuck (%0d clean frames)",
                     clean_frames - clean_before);
            errors = errors + 1;
        end else begin
            $display("  arm pair     : PASS (%0d clean frames after the base change and the restart)",
                     clean_frames - clean_before);
        end

        // ---- 5. not the active source: fetching stops ----------------------
        errors_before = errors;
        loaded = 1'b0;
        repeat (400) @(posedge clk);
        count_fetched_before = words_fetched;
        repeat (4000) @(posedge clk);
        if (words_fetched !== count_fetched_before) begin
            $display("  FAIL: still fetching with active low (%0d -> %0d)",
                     count_fetched_before, words_fetched);
            errors = errors + 1;
        end
        if (errors == errors_before)
            $display("  idle         : PASS (no fetching while the frame is unused)");

        // ---- 6. a drain that nobody answers is asked again ----------------
        // The memory side waits in M_FLUSH for the pixel side's
        // acknowledgement.  If that never comes it must not wait for ever: the
        // request is re-issued every couple of milliseconds.  Holding the pixel
        // side in reset is the only way to create that case here -- the real one
        // always answers, which is why the board failure it protects against was
        // so hard to catch.
        errors_before = errors;
        loaded        = 1'b0;
        repeat (200) @(posedge clk);
        base_word = base_b;
        loaded    = 1'b1;
        @(posedge clk);
        pix_rst   = 1'b1;                   // this side will not answer
        resync_toggles = 0;
        repeat (25000) @(posedge clk);      // 0.4 ms: see it waiting
        if (resync_toggles != 1) begin
            $display("  FAIL: expected exactly one drain request, saw %0d", resync_toggles);
            errors = errors + 1;
        end
        repeat (300000) @(posedge clk);     // 4.8 ms: two timeouts due
        if (resync_toggles < 2) begin
            $display("  FAIL: an unanswered drain request was not re-issued (%0d requests)",
                     resync_toggles);
            errors = errors + 1;
        end else begin
            $display("  resend       : PASS (%0d drain requests while nothing answered)",
                     resync_toggles);
        end
        pix_rst = 1'b0;
        loaded  = 1'b0;
        repeat (200) @(posedge clk);

        // ---- 7. the memory model saw nothing illegal -----------------------
        if (u_ctrl.u_sdram.errors !== 0) begin
            $display("  FAIL: the SDRAM model reported %0d protocol violations",
                     u_ctrl.u_sdram.errors);
            errors = errors + 1;
        end else begin
            $display("  protocol     : PASS");
        end

        $display("  [state] frames=%0d shown=%0d clean=%0d fetched=%0d buffered=%0d underruns=%0d",
                 frames_played, shown_frames, clean_frames, words_fetched,
                 dut.u_word_fifo.wused, pix_underrun);

        if (errors == 0)
            $display("RESULT: PASS -- frame read path verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
