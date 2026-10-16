//=============================================================================
// mm_sd_scan.v -- sector scan for the picture files
//
// The card is not a filesystem to this design: tools/write_card.py writes the
// pictures one after another, each starting on a sector boundary, and the
// FPGA finds them by walking the partition and looking at what is at the
// start of every sector.  That is the same way the vendor example reads its
// pictures (see doc/TF卡文件格式.md, section 1).
//
// A sector is read when the position is unknown; if it starts with a valid
// header the file it belongs to is recorded in a small table and the scan
// jumps over the whole file instead of crawling through it.  Running off the
// end of the pictures is bounded two ways: a total sector budget, and a gap
// counter that gives up once nothing has been found for EMPTY_GAP_SECTORS.
//
// Two header kinds are recognised:
//
//   BMP  (magic "BM")   validated properly: 640x480, 24 bit, uncompressed,
//                       pixel data at offset 54, and a length that at least
//                       covers the pixels.  Valid ones are recorded.
//   MMAU (magic "MMAU") the audio container.  These are only skipped, using
//                       the length in the header, so that a picture behind a
//                       music track does not have to be crawled for.  M7 will
//                       turn this into a scene table of its own.
//
// The sector reads come in through a port rather than the module owning a
// second SPI controller: mm_sd_probe keeps exactly one mm_sd_sec and hands it
// to either the MBR check or this scan, one at a time.
//
// Sector read port contract:
//   * pulse rd_start with rd_lba for one clock, then wait for rd_done
//   * during the read the owner asserts rd_valid with rd_index / rd_byte
//   * rd_error is valid with rd_done; non zero aborts the scan
//=============================================================================
`timescale 1ns / 100fs

module mm_sd_scan #(
    parameter integer MAX_IMAGES        = 8,
    parameter integer MAX_SECTORS       = 200_000,   // 100 MB scan budget
    // 16 MB with nothing found means there are no more pictures.  It has to
    // stay comfortably above one audio track (a 20 s 48 kHz stereo track is
    // about 4 MB) because the scan walks over unknown data one sector at a
    // time: 32000 sectors is about 5.5 s at 25 MHz in the worst case.
    parameter integer EMPTY_GAP_SECTORS = 32_000,
    parameter integer MAX_FILE_BYTES    = 8_388_608  // 8 MiB sanity bound
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        start,
    input  wire [31:0] start_lba,        // usually the partition start
    output reg         busy,
    output reg         done,
    output reg  [3:0]  error_code,       // 0 ok, else the sector read error

    // sector read port
    output reg  [31:0] rd_lba,
    output reg         rd_start,
    input  wire        rd_done,
    input  wire [3:0]  rd_error,
    input  wire [8:0]  rd_index,
    input  wire [7:0]  rd_byte,
    input  wire        rd_valid,

    // results
    output reg  [3:0]  images_found,
    output reg  [31:0] sectors_used,     // sectors read plus sectors skipped
    output reg  [31:0] known_lba,        // just past the last recognised file

    // table read port: entry 0 is the first picture found
    input  wire [2:0]  tbl_index,
    output wire [63:0] tbl_entry         // {start_lba, length_in_bytes}
);

    localparam [2:0] ST_IDLE  = 3'd0,
                     ST_REQ   = 3'd1,
                     ST_WAIT  = 3'd2,
                     ST_JUDGE = 3'd3,
                     ST_DONE  = 3'd4;

    localparam [31:0] PIXEL_BYTES = 32'd54 + 32'd921_600;   // 640 x 480 x 3
    localparam [31:0] MAX_AUDIO   = 32'h0200_0000;          // 32 MiB per track

    reg [2:0]  state;
    reg [31:0] lba;
    reg [31:0] budget;

    // header fields, captured one byte at a time as the sector streams past
    reg [31:0] magic;        // first four bytes, big endian
    reg [31:0] f_size;       // byte 2  file length      (BMP)
    reg [31:0] f_off;        // byte 10 pixel data offset
    reg [31:0] f_w;          // byte 18 width
    reg [31:0] f_h;          // byte 22 height
    reg [15:0] f_depth;      // byte 28 bits per pixel
    reg [31:0] f_comp;       // byte 30 compression
    reg [15:0] f_ver;        // byte 4  container version (MMAU)
    reg [31:0] f_abytes;     // byte 20 audio data bytes  (MMAU)
    reg [31:0] f_adoff;      // byte 24 audio data offset (MMAU)

    reg [31:0] tbl_lba [0:MAX_IMAGES-1];
    reg [31:0] tbl_len [0:MAX_IMAGES-1];

    assign tbl_entry = {tbl_lba[tbl_index], tbl_len[tbl_index]};

    // A picture has to look exactly like what tools/prepare_images.py writes:
    // a 640x480 24 bit uncompressed BMP whose length covers the pixels.
    wire bmp_ok = (magic[31:16] == 16'h424D) &&
                  (f_off  == 32'd54)  && (f_w == 32'd640) && (f_h == 32'd480) &&
                  (f_depth == 16'd24) && (f_comp == 32'd0) &&
                  (f_size >= PIXEL_BYTES) && (f_size <= MAX_FILE_BYTES);

    // An audio track only has to be skippable: magic, version, data offset
    // and a sane data length.
    wire aud_ok = (magic == 32'h4D4D_4155) &&
                  (f_ver == 16'd1) && (f_adoff == 32'd512) &&
                  (f_abytes != 32'd0) && (f_abytes <= MAX_AUDIO);

    wire [31:0] bmp_skip = (f_size   + 32'd511)  >> 9;              // whole file
    wire [31:0] aud_skip = (f_abytes + 32'd1023) >> 9;              // header + data
    wire [31:0] step     = bmp_ok ? bmp_skip : (aud_ok ? aud_skip : 32'd1);

    //------------------------------------------------------------------
    // sequence
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state        <= ST_IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            error_code   <= 4'd0;
            rd_lba       <= 32'd0;
            rd_start     <= 1'b0;
            lba          <= 32'd0;
            budget       <= 32'd0;
            images_found <= 4'd0;
            sectors_used <= 32'd0;
            known_lba    <= 32'd0;
            magic        <= 32'd0;
            f_size       <= 32'd0;
            f_off        <= 32'd0;
            f_w          <= 32'd0;
            f_h          <= 32'd0;
            f_depth      <= 16'd0;
            f_comp       <= 32'd0;
            f_ver        <= 16'd0;
            f_abytes     <= 32'd0;
            f_adoff      <= 32'd0;
        end else begin
            rd_start <= 1'b0;
            done     <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy         <= 1'b1;
                        error_code   <= 4'd0;
                        images_found <= 4'd0;
                        sectors_used <= 32'd0;
                        known_lba    <= start_lba;
                        budget       <= MAX_SECTORS;
                        lba          <= start_lba;
                        state        <= ST_REQ;
                    end
                end

                // ask for one sector: the header fields are cleared here, so
                // nothing from the previous sector can leak into the report
                ST_REQ: begin
                    rd_lba   <= lba;
                    rd_start <= 1'b1;
                    magic    <= 32'd0;
                    f_size   <= 32'd0;
                    f_off    <= 32'd0;
                    f_w      <= 32'd0;
                    f_h      <= 32'd0;
                    f_depth  <= 16'd0;
                    f_comp   <= 32'd0;
                    f_ver    <= 16'd0;
                    f_abytes <= 32'd0;
                    f_adoff  <= 32'd0;
                    state    <= ST_WAIT;
                end

                ST_WAIT: begin
                    // the fields are gathered as they stream past; byte 2..3
                    // feed both the four byte magic and the BMP file length
                    if (rd_valid) begin
                        if (rd_index <= 9'd3)
                            magic <= {magic[23:0], rd_byte};
                        if (rd_index >= 9'd2 && rd_index <= 9'd5)
                            f_size <= {rd_byte, f_size[31:8]};
                        if (rd_index >= 9'd4 && rd_index <= 9'd5)
                            f_ver <= {rd_byte, f_ver[15:8]};
                        if (rd_index >= 9'd10 && rd_index <= 9'd13)
                            f_off <= {rd_byte, f_off[31:8]};
                        if (rd_index >= 9'd18 && rd_index <= 9'd21)
                            f_w <= {rd_byte, f_w[31:8]};
                        if (rd_index >= 9'd22 && rd_index <= 9'd25)
                            f_h <= {rd_byte, f_h[31:8]};
                        if (rd_index >= 9'd28 && rd_index <= 9'd29)
                            f_depth <= {rd_byte, f_depth[15:8]};
                        if (rd_index >= 9'd30 && rd_index <= 9'd33)
                            f_comp <= {rd_byte, f_comp[31:8]};
                        if (rd_index >= 9'd20 && rd_index <= 9'd23)
                            f_abytes <= {rd_byte, f_abytes[31:8]};
                        if (rd_index >= 9'd24 && rd_index <= 9'd27)
                            f_adoff <= {rd_byte, f_adoff[31:8]};
                    end

                    if (rd_done) begin
                        if (rd_error != 4'd0) begin
                            error_code <= rd_error;
                            state      <= ST_DONE;
                        end else begin
                            state <= ST_JUDGE;
                        end
                    end
                end

                ST_JUDGE: begin
                    if (bmp_ok && images_found < MAX_IMAGES) begin
                        tbl_lba[images_found] <= lba;
                        tbl_len[images_found] <= f_size;
                        images_found <= images_found + 4'd1;
                    end

                    // anything recognised moves the "known" boundary past it,
                    // so the gap below measures unknown data only: a skip
                    // over a recognised file must not look like empty space
                    if (bmp_ok || aud_ok)
                        known_lba <= lba + step;

                    // stop when the table is full, when the budget is about
                    // to run out, or when nothing has been found for a while
                    if ((bmp_ok && (images_found + 4'd1) >= MAX_IMAGES) ||
                        (budget <= step) ||
                        ((lba - known_lba) >= EMPTY_GAP_SECTORS)) begin
                        state <= ST_DONE;
                    end else begin
                        budget       <= budget - step;
                        lba          <= lba + step;
                        sectors_used <= sectors_used + step;
                        state        <= ST_REQ;
                    end
                end

                default: begin      // ST_DONE
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
