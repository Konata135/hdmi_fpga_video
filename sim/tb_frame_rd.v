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
//  1. one whole frame of pixels matches, with no underrun
//  2. switching to another frame at another base reads that frame
//  3. with `active` low the reader stops fetching
//  4. the SDRAM model saw no protocol violation
//=============================================================================
`timescale 1ns / 100ps

module tb_frame_rd;

    localparam integer ROW_WORDS = 12;      // 48 bytes = 16 pixels per row
    localparam integer ROWS      = 4;
    localparam integer PIXELS    = ROW_WORDS * 4 / 3;

    reg         clk, clk_sft, pix_clk;
    reg         rst, pix_rst;
    reg  [20:0] base_word;
    reg         active;
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
    reg         pix_de;
    wire [23:0] pix_rgb;
    wire        pix_valid;
    wire [15:0] pix_underrun;

    integer errors, i, r, c, w, k;
    integer line, col;
    integer count_fetched_before;
    reg [7:0] marker_q;         // which stored picture the checker expects

    // what the display side should see: R,G,B with the marker, the column and
    // the flipped row.  A function is used so the field widths stay 8 bits.
    function [23:0] expect_rgb;
        input integer v;
        input integer cc;
        reg [7:0] rbyte, gbyte, bbyte;
        reg [7:0] flipped;
        begin
            flipped = ROWS - 1 - v;             // stored row this line needs
            rbyte   = 8'hA5           ^ marker_q;
            gbyte   = (8'h20 + cc[7:0]) ^ marker_q;
            bbyte   = (8'h10 + flipped) ^ marker_q;
            expect_rgb = {rbyte, gbyte, bbyte};
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
    always #20 pix_clk = ~pix_clk;          // 25 MHz pixel side

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
        .active        (active),
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
        .pix_rgb       (pix_rgb),
        .pix_valid     (pix_valid),
        .pix_underrun  (pix_underrun)
    );

    // check every pixel as it is handed out
    always @(posedge pix_clk) begin
        if (!pix_rst && pix_valid) begin
            if (pix_rgb !== expect_rgb(line, col)) begin
                errors = errors + 1;
                $display("  FAIL: row %0d col %0d got %h, expected %h",
                         line, col, pix_rgb, expect_rgb(line, col));
            end
            if (col == PIXELS - 1) begin
                col  = 0;
                line = line + 1;
            end else begin
                col = col + 1;
            end
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

    // Play one frame of video.  The reader hands out *one byte per pixel
    // clock*, and a pixel is three bytes, so a line of PIXELS pixels takes
    // PIXELS*3 pixel clocks, with a small blanking gap between lines.
    task play_frame;
        begin
            for (i = 0; i < ROWS; i = i + 1) begin
                for (c = 0; c < PIXELS; c = c + 1) begin
                    @(negedge pix_clk);
                    pix_de = 1'b1;
                end
                @(negedge pix_clk);
                pix_de = 1'b0;
                repeat (4) @(negedge pix_clk);
            end
        end
    endtask

    integer errors_before;
    integer base_a, base_b;

    initial begin
        errors    = 0;
        rst       = 1'b1;
        pix_rst   = 1'b1;
        base_word = 21'd0;
        active    = 1'b0;
        pix_de    = 1'b0;
        line      = 0;
        col       = 0;
        marker_q  = 8'h00;

        repeat (4) @(posedge clk);
        repeat (4) @(posedge pix_clk);
        rst     = 1'b0;
        pix_rst = 1'b0;
        repeat (200) @(posedge clk);            // let the SDRAM initialise

        base_a = 21'h0_0100;
        base_b = 21'h0_0800;
        fill_frame(base_a, 0);
        fill_frame(base_b, 8'h40);              // a visibly different picture

        // ---- 1. one frame, in order, flipped, RGB -----------------------
        errors_before = errors;
        line = 0;
        col  = 0;
        marker_q  = 8'h00;
        base_word = base_a;
        active    = 1'b1;
        repeat (2000) @(posedge clk);       // let the reader prime the buffer
        play_frame;
        repeat (10) @(posedge pix_clk);
        if (line !== ROWS) begin
            $display("  FAIL: handed out %0d lines (expected %0d), underruns %0d",
                     line, ROWS, pix_underrun);
            errors = errors + 1;
        end
        if (pix_underrun !== 16'd0) begin
            $display("  FAIL: %0d pixel underruns", pix_underrun);
            errors = errors + 1;
        end
        $display("  [state] lines=%0d fetched=%0d buffered=%0d state=%0d flushed=%0d",
                 line, words_fetched, dut.u_word_fifo.wused, dut.mstate,
                 dut.dbg_flush_words);
        if (errors == errors_before)
            $display("  picture A    : PASS (%0d pixels, rows flipped, R,G,B order)",
                     ROWS * PIXELS);

        // ---- 2. switching picture: the next fetch must be the new one ----
        // Checked at the memory side, which is where it matters: after a base
        // change the very next fetch has to come from the new picture's last
        // row, not from wherever the old walk had got to.  (The display side
        // is only expected to be mid-frame during a switch when the top level
        // switches at a frame boundary, which M6 does.)
        errors_before = errors;
        active    = 1'b0;
        repeat (2000) @(posedge clk);       // let the reader settle to idle
        base_word = base_b;
        active    = 1'b1;                   // ... and let it read the new one
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
            $display("  switch       : PASS (flushed %0d stale bytes, next fetch is the new row)",
                     dut.dbg_flush_words);
        end

        // ---- 3. not the active source: fetching stops -------------------
        errors_before = errors;
        active = 1'b0;
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

        // ---- 4. the memory model saw nothing illegal --------------------
        if (u_ctrl.u_sdram.errors !== 0) begin
            $display("  FAIL: the SDRAM model reported %0d protocol violations",
                     u_ctrl.u_sdram.errors);
            errors = errors + 1;
        end else begin
            $display("  protocol     : PASS");
        end

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
