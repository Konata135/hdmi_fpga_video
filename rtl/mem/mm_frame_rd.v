//=============================================================================
// mm_frame_rd.v -- frame store in, one whole pixel on every pixel clock
//
// Sits between the SDRAM controller (125 MHz) and the pixel domain (25 MHz):
//
//   * pulls the picture in bursts -- the burst read port hands over one word
//     per cycle, which is exactly the rate the word buffer wants
//   * walks the stored rows *backwards*: a BMP is bottom up, so the first line
//     of the display wants the last row of the store
//   * turns the stored byte stream into R,G,B pixels, one per pixel clock
//
// Two things about the pull rate matter and are easy to get wrong:
//
//   * the display eats three bytes per pixel clock it has a pixel to show, so
//     the buffer is refilled at three bytes per pixel; a burst that asks for
//     more words than the buffer has room for is not harmless -- a word handed
//     to a full buffer is simply lost, and a lost word shifts the 3-byte pixel
//     phase for the rest of the frame -- so the burst length is capped by the
//     free space as well as by what is left of the row
//   * a picture switch must throw the old picture's bytes away *on the read
//     side*, because the read pointer lives in the pixel clock domain: the
//     memory side stops, asks, waits for the acknowledgement, and only then
//     starts the walk on the new picture
//
// Why the bytes are regrouped on the pixel side: a word is four bytes and a
// pixel is three, so they never line up.  On this side there is one clock per
// pixel to spend: two word registers make an eight byte window, the pixel being
// presented is the three bytes at the current offset in that window (which may
// straddle the two words), and sliding the window along by four bytes happens
// in the same clock as an emission -- it never costs a pixel clock, which
// matters because the display timing cannot pause.  A row is 480 words =
// 1920 bytes = 640 whole pixels, so the offset is back to zero at every row
// start, and it is reset explicitly when the picture changes.
//
// `active` and `base_word` are memory domain signals: the top level
// synchronises the user's picture choice into this clock domain, and that only
// ever changes between frames.
//=============================================================================
`timescale 1ns / 100fs

module mm_frame_rd #(
    parameter integer ROW_WORDS = 480,      // 640 x 3 bytes / 4
    parameter integer ROWS      = 480,
    parameter integer FIFO_AW   = 8,        // word buffer depth = 2**FIFO_AW
    parameter integer TOPUP     = 0,        // top up to here; 0 = three quarters
    parameter integer BURST_MAX = 64,       // words per burst request (<= 256)
    // Words per SDRAM device row.  A burst run has to stay inside one device
    // row (mm_sdram issues one READ per cycle from a column and never
    // activates mid-run), so a run is also cut short at the next row boundary.
    // A picture line is 480 words -- wider than the device row -- so a walk
    // that ignored this would read the wrong words every 256 words, which is
    // exactly what the small testbench could not see and the 48x48 one did.
    parameter integer SDRAM_COLS = 256
)(
    // memory side, 125 MHz
    input  wire        clk,
    input  wire        rst,
    input  wire [20:0] base_word,
    input  wire        active,
    output reg         br_req,
    output reg  [20:0] br_addr,
    output reg  [8:0]  br_len,
    input  wire        br_dv,
    input  wire [31:0] br_data,
    input  wire        br_busy,
    input  wire        ready,
    input  wire        flush_done,          // pixel side: old bytes discarded
    output reg  [31:0] words_fetched,       // diagnostics
    // How many times the walk has stepped on to the next stored row.  A live
    // counter rather than the pointer itself: the pointer is a multi bit value
    // changing every 35 us, and watching *that* across clock domains is exactly
    // how this project once read a value that contradicted itself.  "Does it
    // move at all" only needs a count, and a count that keeps counting cannot
    // be mistaken for a stuck one.
    output reg  [15:0] rows_seen,           // diagnostics
    // internal state, for the display: which side is waiting on which.  The
    // FIFO's used count says "full" (the pixel side stopped taking words) or
    // "empty" (the memory side stopped supplying them), and the two handshake
    // bits say whether this side is even allowed to ask.
    output wire [8:0]  dbg_used,            // words in the word buffer
    output wire        dbg_app_ok,          // there is room for a whole run
    output wire [1:0]  dbg_mstate,          // M_IDLE / M_BURST / M_FLUSH
    output wire        pix_ready,           // buffer primed: safe to unmask

    // pixel side, 25 MHz
    input  wire        pix_clk,
    input  wire        pix_rst,
    input  wire        pix_de,              // active pixel: present one pixel
    output wire [23:0] pix_rgb,
    output wire        pix_valid,           // this active clock carries a pixel
    output reg  [15:0] pix_underrun,        // saturating, for diagnostics
    output wire [31:0] dbg_flush_words      // words thrown away by a switch
);

    localparam [1:0] M_IDLE  = 2'd0,
                     M_BURST = 2'd1,
                     M_FLUSH = 2'd2;

    localparam integer LAST_ROW_OFFSET = (ROWS - 1) * ROW_WORDS;
    localparam integer FIFO_DEPTH      = 1 << FIFO_AW;
    // The buffer is topped up to three quarters, never to the brim.  The
    // writer's `full` flag is computed from the read pointer synchronised into
    // the memory clock domain, so it reads full a few words early; running the
    // buffer right up against the top is what turns that lag into a dropped
    // word, and a dropped word shifts the 3-byte pixel phase for the rest of
    // the frame.  A quarter of the buffer is plenty of margin.
    localparam integer TOPUP_LEVEL    = (TOPUP > 0) ? TOPUP : (FIFO_DEPTH * 3) / 4;

    //------------------------------------------------------------------
    // word buffer between the two clock domains
    //------------------------------------------------------------------
    wire        fifo_full;
    wire        fifo_empty;
    wire [31:0] fifo_dout;
    wire [FIFO_AW:0] fifo_used;
    reg         fifo_winc;
    reg  [31:0] fifo_wdata;
    wire        fifo_rinc;

    //------------------------------------------------------------------
    // memory side
    //------------------------------------------------------------------
    reg  [1:0]  mstate;
    reg  [18:0] row_ptr;            // which stored row, walks backwards
    reg  [17:0] col_ptr;            // next word within the row
    reg  [20:0] row_base;           // first word of the current row
    reg  [20:0] base_q;             // which frame the walk belongs to
    reg  [8:0]  br_left;            // words still to arrive from the burst
    reg         resync;             // tells the pixel side to realign
    reg         ack_ref;            // flush_done as it was when the switch began
    reg         fd_s1, fd_s2;

    // A run is a fixed length, cut short by the end of the picture row and by
    // the end of the SDRAM device row.  Working the length out from the free
    // space (as this did) put a gray decode, two subtractors, a comparison and
    // a four way minimum on one path: 11.6 ns in an 8 ns period, which the
    // place and route stage reported as -3.9 ns.  It also bought nothing --
    // the buffer is only ever topped up to TOPUP_LEVEL, so the peak occupancy
    // is bounded either way.  What is left is one registered flag saying "there
    // is room for a whole run"; while no burst is in flight the occupancy can
    // only fall, so that registered view is conservative.
    localparam integer RUN      = (BURST_MAX < (FIFO_DEPTH / 4)) ? BURST_MAX
                                                                 : (FIFO_DEPTH / 4);
    localparam integer RUN_ROOM = TOPUP_LEVEL - RUN;

    localparam [8:0]   RUN9     = RUN;
    localparam [FIFO_AW:0] RUN_ROOM_W = RUN_ROOM;

    reg app_ok;                     // the buffer has room for a whole run

    always @(posedge clk) begin
        if (rst) app_ok <= 1'b0;
        else     app_ok <= (fifo_used <= RUN_ROOM_W);
    end

    wire [18:0] row_left  = ROW_WORDS - col_ptr;        // words to the row end
    // the address this run would start from, and how much of the device row is
    // left from there: the column field is the low eight bits of the address
    wire [20:0] fetch_word = row_base + col_ptr;
    wire [8:0]  dev_left   = SDRAM_COLS - fetch_word[7:0];
    wire [8:0]  take_row   = (row_left < RUN9) ? row_left[8:0] : RUN9;
    // never walk past the end of the device row: the burst port issues one READ
    // per cycle from a column and never activates mid-run
    wire [8:0]  burst_len  = (dev_left < take_row) ? dev_left : take_row;
    wire        can_burst  = !br_busy && app_ok && (burst_len != 9'd0);
    wire        flush_done_s = fd_s2;

    assign pix_ready = (fifo_used >= (FIFO_DEPTH / 2));
    assign dbg_used   = fifo_used;
    assign dbg_app_ok = app_ok;
    assign dbg_mstate = mstate;

    always @(posedge clk) begin
        if (rst) begin
            mstate        <= M_IDLE;
            row_ptr       <= 19'd0;
            col_ptr       <= 18'd0;
            row_base      <= 21'd0;
            base_q        <= 21'h1F_FFFF;   // a value no frame can have
            br_req        <= 1'b0;
            br_addr       <= 21'd0;
            br_len        <= 9'd0;
            br_left       <= 9'd0;
            resync        <= 1'b0;
            ack_ref       <= 1'b0;
            words_fetched <= 32'd0;
            rows_seen     <= 16'd0;
            fifo_winc     <= 1'b0;
            fifo_wdata    <= 32'd0;
        end else begin
            br_req    <= 1'b0;
            fifo_winc <= 1'b0;

            // A change of picture always wins: with the buffer full and nobody
            // consuming it would otherwise never see the new base at all.
            if (base_word != base_q) begin
                base_q        <= base_word;
                row_ptr       <= ROWS - 1;
                row_base      <= base_word + LAST_ROW_OFFSET;
                col_ptr       <= 18'd0;
                br_left       <= 9'd0;
                words_fetched <= 32'd0;
                resync        <= ~resync;
                ack_ref       <= flush_done_s;   // the drain we are about to ask for
                mstate        <= M_FLUSH;
            end else case (mstate)
                M_IDLE: begin
                    // the burst port wants the request in a ready cycle, one
                    // cycle wide, exactly like the single word port
                    if (active && ready && can_burst) begin
                        br_addr <= row_base + col_ptr;
                        br_len  <= burst_len;
                        br_left <= burst_len;
                        br_req  <= 1'b1;
                        mstate  <= M_BURST;
                    end
                end

                M_BURST: begin
                    // Keep asking until the controller takes the run.  A
                    // request is only sampled in a ready cycle, and the cycle
                    // this side saw `ready` is not necessarily the cycle its
                    // one-cycle-wide request reaches the controller: a refresh
                    // (or another port) can take the ready cycle in between.
                    // A dropped request left this state machine in M_BURST
                    // waiting for words that were never coming -- which is
                    // exactly what pinned the picture to a single colour while
                    // the buffer sat empty and the controller reported no
                    // burst in flight.  The writer already re-asserts until
                    // `wr_done`; this is the same discipline for the burst.
                    if (!br_busy && (br_left == br_len) && ready)
                        br_req <= 1'b1;
                    if (br_dv) begin
                        fifo_wdata    <= br_data;
                        fifo_winc     <= 1'b1;
                        words_fetched <= words_fetched + 32'd1;
                        if (br_left <= 9'd1) begin
                            // last word of the run: step the walk on
                            if ((col_ptr + br_len) >= ROW_WORDS) begin
                                col_ptr <= 18'd0;
                                rows_seen <= rows_seen + 16'd1;
                                if (row_ptr == 0) begin
                                    row_ptr  <= ROWS - 1;
                                    row_base <= base_q + LAST_ROW_OFFSET;
                                end else begin
                                    row_ptr  <= row_ptr - 19'd1;
                                    row_base <= row_base - ROW_WORDS;
                                end
                            end else begin
                                col_ptr <= col_ptr + br_len;
                            end
                            br_left <= 9'd0;
                            mstate  <= M_IDLE;
                        end else begin
                            br_left <= br_left - 9'd1;
                        end
                    end
                end

                // Switching picture.  The pixel side owns the read pointer, so
                // it does the throwing away; this side just waits for its
                // acknowledgement before walking the new picture (writing
                // before the drain is finished would have the drain eat the
                // first words of the new picture).
                default: begin
                    if (flush_done_s != ack_ref)
                        mstate <= M_IDLE;
                end
            endcase
        end
    end

    // the pixel side's acknowledgement, brought into the memory clock domain
    always @(posedge clk) begin
        if (rst) begin
            fd_s1 <= 1'b0;
            fd_s2 <= 1'b0;
        end else begin
            fd_s1 <= flush_done;
            fd_s2 <= fd_s1;
        end
    end

    mm_afifo #(
        .DW (32),
        .AW (FIFO_AW)
    ) u_word_fifo (
        .wclk  (clk),
        .wrst  (rst),
        .winc  (fifo_winc),
        .wdata (fifo_wdata),
        .wfull (fifo_full),
        .wused (fifo_used),
        .rclk  (pix_clk),
        .rrst  (pix_rst),
        .rinc  (fifo_rinc),
        .rdata (fifo_dout),
        .rempty(fifo_empty),
        .rused ()
    );

    //------------------------------------------------------------------
    // pixel side: a byte shifter
    //
    // Words arrive four bytes at a time and a pixel is three, so the phases
    // drift.  Keeping the bytes in a shifter removes the problem entirely: a
    // word is appended at whatever byte position the shifter is holding,
    // three bytes are taken for the pixel being presented, and the rest slides
    // down.  No alignment constraint, and the top-up happens in the same clock
    // as a pixel so no pixel clock is ever lost.
    //------------------------------------------------------------------
    reg  [95:0] hold;               // pending bytes, first byte in [7:0]
    reg  [3:0]  hcnt;               // how many of them are valid, 0..12
    reg  [23:0] last_rgb;           // repeated if the memory falls behind
    reg  [15:0] dbg_underruns_seen;

    // picture switch bookkeeping.  These sit above everything that uses them:
    // an identifier read before its declaration turns into an implicit wire
    // here, which this project has already been bitten by once.
    reg         flushing;           // emptying the buffer of the old picture
    reg         flush_done_r;       // toggle: the old picture has gone
    reg  [31:0] flush_words;        // how many old words were thrown away

    assign dbg_flush_words = flush_words;
    assign flush_done      = flush_done_r;

    // the memory side's request, synchronised and turned into an edge
    reg         rs_s1, rs_s2, rs_prev;
    wire        resync_pulse = rs_s2 ^ rs_prev;

    // appended word, at the current byte position
    reg  [95:0] appended;
    always @(*) begin
        case (hcnt)
            4'd0:    appended = {64'b0, fifo_dout};
            4'd1:    appended = {56'b0, fifo_dout, hold[7:0]};
            4'd2:    appended = {48'b0, fifo_dout, hold[15:0]};
            4'd3:    appended = {40'b0, fifo_dout, hold[23:0]};
            4'd4:    appended = {32'b0, fifo_dout, hold[31:0]};
            4'd5:    appended = {24'b0, fifo_dout, hold[39:0]};
            default: appended = {16'b0, fifo_dout, hold[47:0]};  // hcnt == 6
        endcase
    end

    wire        can_topup  = !flushing && !fifo_empty && (hcnt <= 4'd6);
    wire        have_pixel = (hcnt >= 4'd3);
    wire        take_byte  = pix_de && have_pixel && !flushing;

    // one combined decision, so `hold` has a single driver
    wire [95:0] hold_next  = can_topup ? (appended >> (take_byte ? 24 : 0))
                                       : (hold     >> (take_byte ? 24 : 0));
    wire [3:0]  hcnt_next  = (can_topup ? (hcnt + 4'd4) : hcnt) - (take_byte ? 4'd3 : 4'd0);

    // B,G,R in the store, R,G,B on the wire
    wire [23:0] rgb_now    = {hold[23:16], hold[15:8], hold[7:0]};

    assign pix_rgb   = take_byte ? rgb_now : last_rgb;
    assign pix_valid = take_byte;
    // while a switch is being thrown away the buffer is emptied a word at a
    // time; the words never reach the shifter
    assign fifo_rinc = flushing ? 1'b1 : can_topup;

    //------------------------------------------------------------------
    // picture switch: empty the buffer of the old picture
    //
    // Only this side can do it, because the read pointer is here.  The memory
    // side asked (resync) and will not write again until it sees flush_done, so
    // nothing of the new picture can be thrown away by mistake.
    //------------------------------------------------------------------
    always @(posedge pix_clk) begin
        if (pix_rst) begin
            flushing     <= 1'b0;
            flush_done_r <= 1'b0;
            flush_words  <= 32'd0;
        end else if (flushing) begin
            if (!fifo_empty)
                flush_words <= flush_words + 32'd1;
            if (fifo_empty) begin
                flushing     <= 1'b0;
                flush_done_r <= ~flush_done_r;
            end
        end else if (resync_pulse) begin
            flushing    <= 1'b1;
            flush_words <= 32'd0;
        end
    end

    // the memory side's request: two flops in, then the edge detector above
    always @(posedge pix_clk) begin
        if (pix_rst) begin
            rs_s1   <= 1'b0;
            rs_s2   <= 1'b0;
            rs_prev <= 1'b0;
        end else begin
            rs_s1   <= resync;
            rs_s2   <= rs_s1;
            rs_prev <= rs_s2;
        end
    end

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            hold          <= 96'd0;
            hcnt          <= 4'd0;
            last_rgb      <= 24'd0;
            pix_underrun  <= 16'd0;
        end else if (resync_pulse) begin
            // a fresh picture starts at a row boundary, which is also a whole
            // number of pixels, so the byte phase starts at zero
            hold <= 96'd0;
            hcnt <= 4'd0;
        end else begin
            hold <= hold_next;
            hcnt <= hcnt_next;
            if (take_byte) begin
                last_rgb <= rgb_now;
            end else if (pix_de && (pix_underrun != 16'hFFFF)) begin
                // an active pixel with nothing to show: hold the last one and
                // count it, so a starved stream is visible in the diagnostics
                pix_underrun <= pix_underrun + 16'd1;
            end
        end
    end

endmodule
