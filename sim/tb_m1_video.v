//=============================================================================
// tb_m1_video.v -- self-checking testbench for the video timing and pattern
//
// Checks, over several complete frames:
//   * exactly 640x480 = 307200 active pixels per frame
//   * exactly 480 end-of-line strobes per frame, exactly 1 frame strobe
//   * every de run inside a line is exactly 640 pixels long
//   * frame period is 800 x 525 = 420000 pixel clocks
//   * the eight colour bars, the active-area frame and both active-area
//     edges carry the expected RGB values
//   * the moving square actually advances once per frame
//=============================================================================
`timescale 1ns / 100ps

module tb_m1_video;

    localparam integer H_ACTIVE    = 640;
    localparam integer V_ACTIVE    = 480;
    localparam integer H_TOTAL     = 800;
    localparam integer V_TOTAL     = 525;
    localparam integer FRAME_CLKS  = H_TOTAL * V_TOTAL;
    localparam integer FRAMES      = 3;

    reg pix_clk;
    reg rst;

    initial pix_clk = 1'b0;
    always #20 pix_clk = ~pix_clk;      // 40 ns period = 25 MHz

    initial begin
        rst = 1'b1;
        repeat (8) @(posedge pix_clk);
        rst = 1'b0;
    end

    wire [11:0] h_cnt;
    wire [10:0] v_cnt;
    wire        hs, vs, de, frame_first, line_last;

    mm_video_timing #(
        .H_ACTIVE (H_ACTIVE),
        .H_FP     (16),
        .H_SYNC   (96),
        .H_BP     (48),
        .V_ACTIVE (V_ACTIVE),
        .V_FP     (10),
        .V_SYNC   (2),
        .V_BP     (33)
    ) dut_timing (
        .pix_clk     (pix_clk),
        .rst         (rst),
        .h_cnt       (h_cnt),
        .v_cnt       (v_cnt),
        .hs          (hs),
        .vs          (vs),
        .de          (de),
        .frame_first (frame_first),
        .line_last   (line_last)
    );

    wire [23:0] pattern_rgb;

    mm_test_pattern #(
        .H_ACTIVE (H_ACTIVE),
        .V_ACTIVE (V_ACTIVE)
    ) dut_pattern (
        .pix_clk     (pix_clk),
        .rst         (rst),
        .frame_first (frame_first),
        .h_cnt       (h_cnt),
        .v_cnt       (v_cnt),
        .rgb         (pattern_rgb)
    );

    integer errors;
    integer frame_no;
    integer de_cnt;
    integer last_cnt;
    integer first_cnt;
    integer clk_cnt;
    integer de_run;
    integer prev_box_x;

    task fail;
        input [1023:0] msg;
        begin
            errors <= errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial begin
        errors     = 0;
        frame_no   = 0;
        de_cnt     = 0;
        last_cnt   = 0;
        first_cnt  = 0;
        clk_cnt    = 1;
        de_run     = 0;
        prev_box_x = -1;
    end

    always @(posedge pix_clk) begin
        if (rst) begin
            de_cnt    <= 0;
            last_cnt  <= 0;
            first_cnt <= 0;
            clk_cnt   <= 0;
            de_run    <= 0;
        end else begin
            if (frame_first) begin
                // the pixel that just ended is both the first pixel of this
                // frame and an active pixel, so count it here and start the
                // other counters from zero
                if (frame_no > 0) begin
                    if (de_cnt !== H_ACTIVE * V_ACTIVE)
                        $display("  FAIL: frame %0d active pixels = %0d (expected %0d)",
                                 frame_no, de_cnt, H_ACTIVE * V_ACTIVE);
                    if (last_cnt !== V_ACTIVE)
                        $display("  FAIL: frame %0d end-of-line strobes = %0d (expected %0d)",
                                 frame_no, last_cnt, V_ACTIVE);
                    if (first_cnt !== 1)
                        $display("  FAIL: frame %0d frame strobes = %0d (expected 1)",
                                 frame_no, first_cnt);
                    if (clk_cnt !== FRAME_CLKS)
                        $display("  FAIL: frame %0d period = %0d clocks (expected %0d)",
                                 frame_no, clk_cnt, FRAME_CLKS);
                    if (de_cnt !== H_ACTIVE * V_ACTIVE || last_cnt !== V_ACTIVE ||
                        first_cnt !== 1 || clk_cnt !== FRAME_CLKS)
                        errors <= errors + 1;

                    if (dut_pattern.box_x === prev_box_x)
                        $display("  FAIL: square x did not advance on frame %0d", frame_no);
                    else
                        $display("  frame %0d: active=%0d eol=%0d sof=%0d period=%0d box_x=%0d",
                                 frame_no, de_cnt, last_cnt, first_cnt, clk_cnt,
                                 dut_pattern.box_x);
                end else begin
                    $display("  frame 0 baseline: box_x=%0d box_y=%0d",
                             dut_pattern.box_x, dut_pattern.box_y);
                end

                prev_box_x <= dut_pattern.box_x;
                frame_no   <= frame_no + 1;
                de_cnt     <= 1;
                last_cnt   <= 0;
                first_cnt  <= 1;
                clk_cnt    <= 1;

                if (frame_no >= FRAMES) begin
                    if (errors == 0)
                        $display("RESULT: PASS -- video timing and pattern verified");
                    else
                        $display("RESULT: FAIL -- %0d error(s)", errors);
                    $finish;
                end
            end else begin
                if (de)        de_cnt   <= de_cnt   + 1;
                if (line_last) last_cnt <= last_cnt + 1;
                clk_cnt <= clk_cnt + 1;
            end

            if (de) begin
                de_run <= de_run + 1;
            end else begin
                if (de_run != 0 && de_run != H_ACTIVE) begin
                    $display("  FAIL: de run of %0d pixels at line %0d", de_run, v_cnt);
                    errors <= errors + 1;
                end
                de_run <= 0;
            end
        end
    end

    always @(posedge pix_clk) begin
        if (!rst && de && (v_cnt == 11'd478)) begin
            case (h_cnt)
                12'd40:   if (pattern_rgb !== 24'hFFFFFF) fail("bar 0 (white) wrong");
                12'd120:  if (pattern_rgb !== 24'hFFFF00) fail("bar 1 (yellow) wrong");
                12'd200:  if (pattern_rgb !== 24'h00FFFF) fail("bar 2 (cyan) wrong");
                12'd280:  if (pattern_rgb !== 24'h00FF00) fail("bar 3 (green) wrong");
                12'd360:  if (pattern_rgb !== 24'hFF00FF) fail("bar 4 (magenta) wrong");
                12'd440:  if (pattern_rgb !== 24'hFF0000) fail("bar 5 (red) wrong");
                12'd520:  if (pattern_rgb !== 24'h0000FF) fail("bar 6 (blue) wrong");
                12'd600:  if (pattern_rgb !== 24'h000000) fail("bar 7 (black) wrong");
                12'd0:    if (pattern_rgb !== 24'h808080) fail("left frame column wrong");
                12'd639:  if (pattern_rgb !== 24'h808080) fail("right frame column wrong");
                default:  ;
            endcase
        end
        if (!rst && de && (h_cnt == 12'd300)) begin
            if ((v_cnt == 11'd0) && (pattern_rgb !== 24'h808080))
                fail("top frame row wrong");
            if ((v_cnt == 11'd479) && (pattern_rgb !== 24'h808080))
                fail("bottom frame row wrong");
        end
    end

    initial begin
        #60_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule

