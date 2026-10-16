//=============================================================================
// mm_frame_wr.v -- byte stream in, packed words into the frame store
//
// The picture arrives as bytes (SPI clocked, in the 50 MHz domain) and the
// frame store wants 32 bit words, so this module
//
//   * packs four bytes at a time, first byte into bits [7:0]
//   * crosses into the 125 MHz memory domain through mm_afifo
//   * writes the words at base_word + 0, 1, 2, ... one at a time
//
// Row alignment is what makes the packing clean: a row of the picture is
// 640 x 3 = 1920 bytes = exactly 480 words, so the byte phase restarts at
// every row and no bookkeeping is needed across rows (see
// doc/SDRAM内存映射.md, section 2).  The loader is responsible for not
// feeding the 54 byte header and for handing over exactly
// 4 x words_total bytes; a partial word at the end is dropped, and a stray
// "start" flushes whatever is still in the FIFO so a half finished frame
// cannot shift the next one.
//
// The byte side is a push interface: the SPI stream cannot be held back, so
// there is no ready signal.  At 25 MHz SPI that is 2.5 MB/s against the
// memory's 45 MB/s, hence the FIFO only has to cover short bursts; overflows
// are counted so the testbench (and the on-board display, later) can tell.
//=============================================================================
`timescale 1ns / 100fs

module mm_frame_wr #(
    parameter integer AFIFO_AW = 10          // 1024 words of packing buffer
)(
    // byte side: 50 MHz, the SD sector stream
    input  wire        clk_byte,
    input  wire        rst_byte,
    input  wire        start,                // pulse: begin a frame
    input  wire [20:0] base_word,            // where this frame lives
    input  wire [17:0] words_total,          // words expected this frame
    input  wire        byte_valid,
    input  wire [7:0]  byte_data,
    output reg  [7:0]  overflow_count,       // saturating, for diagnostics
    output reg         busy,

    // word side: 125 MHz, the SDRAM controller
    input  wire        clk,
    input  wire        rst,
    output reg  [20:0] wr_addr,
    output reg  [31:0] wr_data,
    output reg         wr_req,
    input  wire        wr_done,
    input  wire        ready,
    output reg         frame_done,           // one cycle pulse
    // The pulse above is easy for a caller in the other clock domain to miss:
    // the words can all be in before the caller even starts looking, which is
    // exactly what happened to the picture loader (the CRC bytes of the last
    // sector take longer to clock out than the write takes).  This toggle
    // cannot be missed -- a caller records it when the frame starts and waits
    // for it to differ, the same trick `start` uses in the other direction.
    output reg         frame_done_tgl,
    output reg  [17:0] words_written
);

    localparam [3:0] W_WAIT  = 4'd0,
                     W_POP   = 4'd1,
                     W_REQ   = 4'd2,
                     W_FLUSH = 4'd3;

    // `start` arrives in the byte clock domain and is needed in the word
    // clock domain, so it crosses as a toggle (see the block at the end).
    reg  start_toggle;
    reg  st_s1, st_s2, st_prev;
    wire start_pulse = st_s2 ^ st_prev;

    //------------------------------------------------------------------
    // byte side: pack four bytes into a word and push it into the FIFO
    //------------------------------------------------------------------
    // The word is assembled little endian in stream order: the first byte of
    // the group ends up in bits [7:0], so dumping the frame store reads like
    // the file itself.
    reg  [2:0]  byte_cnt;
    reg  [31:0] packed;

    wire [31:0] word_full = {byte_data, packed[31:8]};
    wire        word_push = busy && byte_valid && (byte_cnt == 3'd3);
    wire        fifo_full;
    wire        fifo_empty;
    wire [31:0] fifo_dout;
    wire        fifo_rinc;

    always @(posedge clk_byte) begin
        if (rst_byte) begin
            byte_cnt       <= 3'd0;
            packed         <= 32'd0;
            overflow_count <= 8'd0;
            busy           <= 1'b0;
        end else if (start) begin
            // a new frame always starts a fresh byte phase
            byte_cnt <= 3'd0;
            packed   <= 32'd0;
            busy     <= 1'b1;
        end else if (busy && byte_valid) begin
            packed   <= {byte_data, packed[31:8]};
            byte_cnt <= (byte_cnt == 3'd3) ? 3'd0 : (byte_cnt + 3'd1);
            if (word_push && fifo_full && (overflow_count != 8'hFF))
                overflow_count <= overflow_count + 8'd1;
        end
    end

    mm_afifo #(
        .DW (32),
        .AW (AFIFO_AW)
    ) u_pack_fifo (
        .wclk  (clk_byte),
        .wrst  (rst_byte),
        .winc  (word_push),
        .wdata (word_full),
        .wfull (fifo_full),
        .wused (),
        .rclk  (clk),
        .rrst  (rst),
        .rinc  (fifo_rinc),
        .rdata (fifo_dout),
        .rempty(fifo_empty),
        .rused ()
    );

    //------------------------------------------------------------------
    // word side: drain the FIFO into the frame store
    //------------------------------------------------------------------
    reg  [3:0]  wstate;
    reg  [20:0] wr_base;
    reg  [17:0] wr_total;
    reg  [17:0] wr_idx;
    reg         pop_req;        // registered to be high for exactly one cycle

    assign fifo_rinc = pop_req;

    always @(posedge clk) begin
        if (rst) begin
            wstate        <= W_WAIT;
            wr_base       <= 21'd0;
            wr_total      <= 18'd0;
            wr_idx        <= 18'd0;
            pop_req       <= 1'b0;
            wr_req        <= 1'b0;
            wr_addr       <= 21'd0;
            wr_data       <= 32'd0;
            frame_done    <= 1'b0;
            frame_done_tgl <= 1'b0;
            words_written <= 18'd0;
        end else begin
            frame_done <= 1'b0;
            pop_req    <= 1'b0;

            // A start always wins, whatever the state: an abandoned frame must
            // not wedge the drain FSM waiting for words that will never come.
            // Anything still in the FIFO belongs to the dead frame and gets
            // thrown away before the new one begins.
            if (start_pulse) begin
                wr_base       <= base_word;
                wr_total      <= words_total;
                wr_idx        <= 18'd0;
                words_written <= 18'd0;
                wr_req        <= 1'b0;
                wstate        <= fifo_empty ? W_POP : W_FLUSH;
            end else begin
            case (wstate)
                // idle between frames
                W_WAIT: begin
                    wr_req <= 1'b0;
                end

                // discard stale words so the next frame starts at its own base
                W_FLUSH: begin
                    if (fifo_empty)
                        wstate <= W_POP;
                    else
                        pop_req <= 1'b1;
                end

                // take one word out of the FIFO and write it into the store
                W_POP: begin
                    if (!fifo_empty && (wr_idx != wr_total) && ready) begin
                        wr_data <= fifo_dout;               // the word we pop now
                        wr_addr <= wr_base + wr_idx;
                        pop_req <= 1'b1;
                        wstate  <= W_REQ;
                    end
                end

                W_REQ: begin
                    wr_req <= 1'b1;
                    if (wr_done) begin
                        wr_req        <= 1'b0;
                        wr_idx        <= wr_idx + 18'd1;
                        words_written <= wr_idx + 18'd1;
                        if ((wr_idx + 18'd1) == wr_total) begin
                            frame_done     <= 1'b1;
                            frame_done_tgl <= ~frame_done_tgl;
                            wstate     <= W_WAIT;     // frame complete, next start
                        end else begin
                            wstate <= W_POP;          // more words to move
                        end
                    end
                end

                default: wstate <= W_WAIT;
            endcase
            end
        end
    end

    // the toggle that carries `start` across: the word side sees the edge and
    // acts on it once.  base_word and words_total must be stable for a few
    // clocks around the pulse (the loader sets them before starting).
    always @(posedge clk_byte) begin
        if (rst_byte)
            start_toggle <= 1'b0;
        else if (start)
            start_toggle <= ~start_toggle;
    end

    always @(posedge clk) begin
        if (rst) begin
            st_s1   <= 1'b0;
            st_s2   <= 1'b0;
            st_prev <= 1'b0;
        end else begin
            st_s1   <= start_toggle;
            st_s2   <= st_s1;
            st_prev <= st_s2;
        end
    end

endmodule
