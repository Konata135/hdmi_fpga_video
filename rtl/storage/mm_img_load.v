//=============================================================================
// mm_img_load.v -- stream one BMP file into the frame store
//
// The scan (mm_sd_scan) has already accepted the file and said where it starts
// and how long it is.  This module turns that into a frame in the SDRAM:
//
//   * reads the file's sectors in order, one at a time, through the same
//     sector engine the scan uses (a second client of one SPI bus, not a
//     second SPI controller)
//   * throws away the first HEADER_BYTES bytes -- the scan has already checked
//     the header, so the only thing left to do with it is skip it
//   * hands the pixels to mm_frame_wr as they arrive, which packs them into
//     32 bit words and writes them to the store
//
// Nothing is flipped here.  A BMP stores its rows bottom up, the frame store
// keeps them in file order, and the reader walks the stored rows backwards, so
// "stored row 0" is the bottom line of the picture and the display gets it the
// right way up (see doc/SDRAM内存映射.md).
//
// 640x480 24 bit is what the scan accepts, and 640 x 3 = 1920 bytes is a whole
// number of words, so the byte phase restarts every row and no padding or row
// bookkeeping is needed.  1920 x 480 = 921600 bytes = 230400 words fits the
// 18 bit word count with room to spare.
//
// A sector that comes back with an error is retried a few times before giving
// up, and giving up is reported rather than hidden: the top level restarts the
// whole sequence, which is also what happens when the card is pulled out.
//=============================================================================
`timescale 1ns / 100fs

module mm_img_load #(
    parameter integer HEADER_BYTES = 54,     // 14 byte file header + 40 byte info
    parameter integer READ_TRIES   = 3,
    // (2**18 - 1) words is the most words_total can carry
    parameter integer MAX_PIXEL_BYTES = 230400 * 4
)(
    input  wire        clk,             // 50 MHz, the SD side
    input  wire        rst,

    // start: load the picture whose {start_lba, file_bytes} the table is
    // presenting.  All three of these and frame_base have to be stable from the
    // start pulse until the frame is in: they are sampled when the header has
    // been checked and the frame is about to be written.
    input  wire        start,
    input  wire [31:0] start_lba,
    input  wire [31:0] file_bytes,
    input  wire        entry_valid,
    // which slot of the frame store this picture goes into; the carousel gives
    // each picture its own, so it cannot be a parameter any more
    input  wire [20:0] frame_base,

    // the shared sector engine: pulse rd_start with rd_lba, wait for rd_done,
    // and read the bytes off rd_valid / rd_index / rd_byte as they stream by
    output reg  [31:0] rd_lba,
    output reg         rd_start,
    input  wire        rd_busy,
    input  wire        rd_done,
    input  wire [3:0]  rd_error,
    input  wire [8:0]  rd_index,
    input  wire [7:0]  rd_byte,
    input  wire        rd_valid,

    // the frame writer
    output reg         frame_start,
    output reg  [20:0] base_word,
    output reg  [17:0] words_total,
    output reg         byte_valid,
    output reg  [7:0]  byte_data,
    // The writer's "frame is in" signal is a toggle, not a pulse: the words
    // can all have been written before this side even gets to its drain state
    // (the last sector's CRC bytes take longer to clock out than the write
    // takes), and a one cycle pulse would simply be missed.  The toggle value
    // is recorded when the load starts and waited on to differ.
    input  wire        frame_done_tgl,

    // status
    output reg         busy,
    output reg         done,            // one cycle pulse: the frame is in
    output reg  [3:0]  error_code,
    output reg  [3:0]  phase,           // for the on-board display
    output reg  [31:0] bytes_sent,
    output reg  [15:0] sectors_read,
    // What the loader found at the start of the file it was told to read, and
    // what the pixel bytes it handed over look like: between them they say
    // whether a blank screen is a blank store or a blank picture.
    output wire        hdr_ok,
    output reg  [7:0]  ff_first
);

    localparam [3:0] L_IDLE  = 4'd0,
                     L_PREP  = 4'd1,   // one cycle for the size checks to settle
                     L_REQ   = 4'd2,
                     L_WAIT  = 4'd3,
                     L_DRAIN = 4'd4,
                     L_FAIL  = 4'd5;

    // what the display shows
    localparam [3:0] LP_IDLE = 4'h0,
                     LP_READ = 4'h1,
                     LP_DRAIN = 4'h2,
                     LP_DONE = 4'h3,
                     LP_FAIL = 4'hF;

    localparam [3:0] ERR_NONE = 4'd0,
                     ERR_READ = 4'd1,
                     ERR_EMPTY = 4'd2,   // nothing to load / file too short
                     ERR_BIG  = 4'd3,    // more pixels than words_total can count
                     // The scan accepted this file because it found "BM" at the
                     // first two bytes of a sector.  If the loader, reading the
                     // sector the table points at, does not see the same two
                     // bytes, then the table's start address and what is being
                     // read have come apart -- and without this check the whole
                     // frame would quietly fill with whatever is really there
                     // (0xFF from a blank area looks exactly like a white
                     // picture on the screen).
                     ERR_NOTBMP = 4'd4;

    localparam [7:0] BM_FIRST = 8'h42,   // 'B'
                     BM_SECOND = 8'h4D;  // 'M'

    // widths made explicit: comparing against a bare integer works, but a
    // part select of one does not on every tool
    localparam [8:0]  HEADER_LAST = HEADER_BYTES;
    localparam [2:0]  TRIES_MAX   = READ_TRIES;
    localparam [31:0] MAX_PIXEL   = MAX_PIXEL_BYTES;

    reg  [3:0]  state;
    reg  [31:0] pixel_bytes;        // file_bytes minus the header
    reg  [31:0] sent;               // pixel bytes handed over so far
    reg  [15:0] sector;             // sector being read, relative to the file
    reg  [2:0]  tries;
    reg         streaming;          // the bytes of this sector are wanted
    reg         hdr_bad;            // the sector we were pointed at is not a BMP
    reg         fd_s1, fd_s2;       // the writer's completion toggle, synced
    reg         fd_ref;             // ... and its value when the load began

    wire        fd_sync = fd_s2;

    assign hdr_ok = ~hdr_bad;

    // The size checks are 32 bit compares, and the word count is a subtract
    // followed by a shift.  Working them out on the same cycle as the start
    // pulse puts all of that in series with whatever produced the pulse -- in
    // the built design it was the top level's own load logic, and the path
    // missed its period by 0.97 ns.  The scan table is stable long before the
    // pulse arrives, so they are worked out while idle instead.
    wire [31:0] pixel_bytes_w = file_bytes - HEADER_BYTES;
    wire        too_short_w   = entry_valid ? (file_bytes <= HEADER_BYTES) : 1'b1;
    wire        too_big_w     = (pixel_bytes_w > MAX_PIXEL);

    reg  [31:0] pixel_bytes_q;
    reg  [17:0] words_total_q;
    reg         too_short_q, too_big_q;

    // a pixel byte is one that is past the header of the first sector
    wire in_region = (sector != 16'd0) || (rd_index >= HEADER_LAST);
    wire want_byte = rd_valid && streaming && (sent != pixel_bytes) && in_region
                     && !hdr_bad;

    always @(posedge clk) begin
        if (rst) begin
            state         <= L_IDLE;
            pixel_bytes   <= 32'd0;
            sent          <= 32'd0;
            sector        <= 16'd0;
            tries         <= 3'd0;
            streaming     <= 1'b0;
            rd_lba        <= 32'd0;
            rd_start      <= 1'b0;
            frame_start   <= 1'b0;
            base_word     <= frame_base;
            words_total   <= 18'd0;
            byte_valid    <= 1'b0;
            byte_data     <= 8'd0;
            busy          <= 1'b0;
            done          <= 1'b0;
            error_code    <= ERR_NONE;
            phase         <= LP_IDLE;
            bytes_sent    <= 32'd0;
            sectors_read  <= 16'd0;
            fd_s1         <= 1'b0;
            fd_s2         <= 1'b0;
            fd_ref        <= 1'b0;
            pixel_bytes_q <= 32'd0;
            words_total_q <= 18'd0;
            too_short_q   <= 1'b1;
            too_big_q     <= 1'b0;
            hdr_bad       <= 1'b0;
            ff_first      <= 8'd0;
        end else begin
            rd_start   <= 1'b0;
            frame_start <= 1'b0;
            byte_valid <= 1'b0;
            done       <= 1'b0;

            fd_s1 <= frame_done_tgl;
            fd_s2 <= fd_s1;

            pixel_bytes_q <= pixel_bytes_w;
            words_total_q <= pixel_bytes_w >> 2;
            too_short_q   <= too_short_w;
            too_big_q     <= too_big_w;

            // The first sector has to start with the two bytes the scan found:
            // anything else means the read is not landing where the table says,
            // and nothing that follows is worth handing to the frame writer.
            if (rd_valid && (sector == 16'd0)) begin
                if ((rd_index == 9'd0) && (rd_byte != BM_FIRST))
                    hdr_bad <= 1'b1;
                if ((rd_index == 9'd1) && (rd_byte != BM_SECOND))
                    hdr_bad <= 1'b1;
            end

            // Start the writer only once the header verdict is in -- the second
            // byte is the last one that can spoil it, and by index 2 the result
            // has settled.  A sector that is not a BMP then never starts a
            // frame at all, and the frame's pixel bytes (which begin at index
            // 54) all still arrive after it.
            if (rd_valid && streaming && (sector == 16'd0)
                && (rd_index == 9'd2) && !hdr_bad) begin
                frame_start <= 1'b1;
                fd_ref      <= fd_sync;
            end

            // pixels stream out of the sector as they go by; everything before
            // the end of the header is dropped on the floor
            if (want_byte) begin
                byte_valid <= 1'b1;
                byte_data  <= rd_byte;
                // a sample of the first bytes: a real picture has almost none
                // of them at 0xFF, a blank area is nothing else
                if ((sent < 32'd255) && (rd_byte == 8'hFF) && (ff_first != 8'hFF))
                    ff_first <= ff_first + 8'd1;
                sent       <= sent + 32'd1;
                bytes_sent <= bytes_sent + 32'd1;
            end

            // A start always wins, in any state: that is what makes a load
            // that gave up retryable, and what keeps a stalled load from
            // wedging the sequence.  It only arms the load though -- the
            // decisions are taken a cycle later in L_PREP, because the flags
            // they read are registered and would otherwise still be describing
            // the cycle before the entry became valid.
            if (start && !busy) begin
                streaming <= 1'b0;
                busy      <= 1'b1;
                state     <= L_PREP;
            end else case (state)
                L_IDLE: begin
                    streaming <= 1'b0;
                    busy      <= 1'b0;
                end

                L_PREP: begin
                    if (too_short_q) begin
                        error_code <= ERR_EMPTY;
                        phase      <= LP_FAIL;
                        busy       <= 1'b0;
                        state      <= L_FAIL;
                    end else if (too_big_q) begin
                        error_code <= ERR_BIG;
                        phase      <= LP_FAIL;
                        busy       <= 1'b0;
                        state      <= L_FAIL;
                    end else begin
                        // tell the writer what is coming before the bytes do: a
                        // start always wins inside mm_frame_wr too, and
                        // base/words are held steady for the whole frame
                        pixel_bytes  <= pixel_bytes_q;
                        words_total  <= words_total_q;
                        base_word    <= frame_base;
                        sent         <= 32'd0;
                        bytes_sent   <= 32'd0;
                        sector       <= 16'd0;
                        sectors_read <= 16'd0;
                        hdr_bad      <= 1'b0;
                        ff_first     <= 8'd0;
                        tries        <= 3'd0;
                        error_code   <= ERR_NONE;
                        phase        <= LP_READ;
                        rd_lba       <= start_lba;
                        fd_ref       <= fd_sync;   // a completion from before
                                                   // this load must not count
                        state        <= L_REQ;
                    end
                end

                // ask for the next sector; the engine is idle here by
                // construction (the scan has finished and parked)
                L_REQ: begin
                    if (!rd_busy) begin
                        rd_lba    <= start_lba + {16'd0, sector};
                        rd_start  <= 1'b1;
                        streaming <= 1'b1;
                        state     <= L_WAIT;
                    end
                end

                L_WAIT: begin
                    if (rd_done) begin
                        streaming <= 1'b0;
                        if (rd_error != 4'd0) begin
                            if (tries == TRIES_MAX) begin
                                error_code <= ERR_READ;
                                phase      <= LP_FAIL;
                                busy       <= 1'b0;
                                state      <= L_FAIL;
                            end else begin
                                tries <= tries + 3'd1;
                                state <= L_REQ;         // same sector again
                            end
                        end else begin
                            tries        <= 3'd0;
                            sectors_read <= sectors_read + 16'd1;
                            // reading the wrong place is worth stopping for, not
                            // filling the frame store with whatever is there
                            if ((sector == 16'd0) && hdr_bad) begin
                                error_code <= ERR_NOTBMP;
                                phase      <= LP_FAIL;
                                busy       <= 1'b0;
                                state      <= L_FAIL;
                            end else if (sent == pixel_bytes) begin
                                phase <= LP_DRAIN;
                                state <= L_DRAIN;
                            end else if (sector == 16'd65535) begin
                                error_code <= ERR_READ;
                                phase      <= LP_FAIL;
                                busy       <= 1'b0;
                                state      <= L_FAIL;
                            end else begin
                                sector <= sector + 16'd1;
                                state  <= L_REQ;
                            end
                        end
                    end
                end

                // the bytes are all in the packing FIFO; wait for the memory
                // side to have written the last word before saying so
                L_DRAIN: begin
                    if (fd_sync != fd_ref) begin
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        phase <= LP_DONE;
                        state <= L_IDLE;
                    end
                end

                default: begin              // L_FAIL: stay put until restarted
                    busy <= 1'b0;
                end
            endcase
        end
    end

endmodule
