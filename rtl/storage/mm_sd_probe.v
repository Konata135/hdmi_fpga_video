//=============================================================================
// mm_sd_probe.v -- M3 bring-up chain: initialise the card, read LBA0, check
//                  the MBR
//
// This is the acceptance path for the SPI storage milestone.  It takes the
// card through its whole start-up and then proves the data path:
//
//   mm_sd_init   CMD0 -> CMD8 -> CMD55+ACMD41 -> CMD58
//   mm_sd_sec    CMD17 on block 0, one 512 byte sector
//
// and checks what came back:
//
//   bytes 510, 511    0x55 0xAA       the MBR signature
//   byte  450         partition type  (0x0B, 0x0C, ... are FAT32)
//   bytes 454..457    first LBA       (little endian)
//   bytes 458..461    sector count    (little endian) -> capacity in GiB
//
// The whole read takes about 170 us at 25 MHz, far quicker than the display
// scans, so the phases inside mm_sd_sec are only ever visible on the display
// when a card is stuck half way through.
//
// Failure is never fatal.  A failed read is retried READ_TRIES times, then
// the card is initialised again and the whole cycle repeats for as long as it
// takes.  The last outcome stays latched in error_code so that whatever went
// wrong is what the display shows.
//
// Error codes, on digit 2:
//   0  no error
//   1  the card never answered CMD17
//   2  CMD17 answered with an error bit set
//   3  the card never offered a data token
//   4  the card offered a data error token (it could not read the sector)
//   5  the MBR signature is not 0x55AA
//   6  the scan found no usable picture file
//   7  the card did not initialise
//
// What this module reports for the display (top.v turns it into pages):
//
//   disp_phase    sequence step: 0..7 initialisation as mm_sd_init reports
//                 it, 8 waiting for R1, 9 waiting for the data block,
//                 A MBR read and verified, B scanning, C scan finished,
//                 F failed
//   images_found  live count of pictures the scan has accepted
//   capacity_gib  partition size in GiB, rounded up
//   tbl_entry     one table entry at a time, {start_lba, length_in_bytes}
//=============================================================================
`timescale 1ns / 100fs

module mm_sd_probe #(
    parameter integer INIT_POWER_CYCLES = 100_000,    // 2 ms at 50 MHz
    parameter integer INIT_ACMD41_TRIES = 2000,
    parameter integer INIT_WAIT_CYCLES  = 50_000_000, // 1 s between attempts
    parameter integer READ_TRIES        = 3,          // per initialisation
    parameter integer READ_WAIT_CYCLES  = 50_000,     // 1 ms between retries
    parameter integer DATA_CLK_DIV      = 1,          // 25 MHz SPI data phase
    parameter integer SCAN_MAX_SECTORS  = 200_000,    // 100 MB scan budget
    parameter integer SCAN_EMPTY_GAP    = 32_000      // 16 MB with no picture
)(
    input  wire        clk,
    input  wire        rst,

    // where the sequence has got to; top.v turns this into a display page
    output reg  [3:0]  disp_phase,

    // what the MBR says, once it has been read
    output reg         sig_ok,
    output reg  [7:0]  part_type,
    output reg  [31:0] part_start_lba,
    output reg  [31:0] part_sectors,
    output reg  [7:0]  capacity_gib,

    // what the scan found
    output wire [3:0]  images_found,
    input  wire [2:0]  tbl_index,
    output wire [63:0] tbl_entry,        // {start_lba, length_in_bytes}

    output reg  [3:0]  error_code,
    output reg  [7:0]  error_count,
    output reg  [7:0]  read_attempts,

    // The sector engine is shared with the picture loader (M5), which reads a
    // whole file through it once the sequence has parked.  Only one client
    // ever has a request outstanding, so the engine is simply lent out: these
    // are the engine's own handshake and byte stream, brought to the surface.
    input  wire        aux_rd_start,
    input  wire [31:0] aux_rd_lba,
    output wire        aux_sec_busy,
    output wire        aux_sec_done,
    output wire [3:0]  aux_sec_error,
    output wire        aux_sec_valid,
    output wire [8:0]  aux_sec_index,
    output wire [7:0]  aux_sec_byte,

    // card
    output wire        sd_ncs,
    output wire        sd_sclk,
    output wire        sd_mosi,
    input  wire        sd_miso
);

    // Nine states, so the register has to be four bits wide.  With a three
    // bit state register ST_DONE = 3'd8 truncates to 0, which is ST_INIT_WAIT:
    // the sequence would restart from scratch right after finishing instead of
    // sitting still.  (TD warns about exactly this: HDL-5007.)
    localparam [3:0] ST_INIT_WAIT = 4'd0,
                     ST_INIT_GO   = 4'd1,
                     ST_INIT_RUN  = 4'd2,
                     ST_READ_WAIT = 4'd3,
                     ST_READ_GO   = 4'd4,
                     ST_READ_RUN  = 4'd5,
                     ST_SCAN_GO   = 4'd6,
                     ST_SCAN_RUN  = 4'd7,
                     ST_DONE      = 4'd8;

    localparam [3:0] PH_READ_CMD  = 4'h8,
                     PH_READ_DATA = 4'h9,
                     PH_OK        = 4'hA,
                     PH_SCAN      = 4'hB,
                     PH_SCAN_DONE = 4'hC,
                     PH_FAIL      = 4'hF;

    localparam [3:0] ERR_NONE = 4'd0,
                     ERR_SIG  = 4'd5,
                     ERR_NONE_FOUND = 4'd6,
                     ERR_INIT = 4'd7;

    localparam [15:0] DATA_DIV = DATA_CLK_DIV;

    reg [3:0]  state;
    reg [31:0] wait_cnt;
    reg [3:0]  read_try;
    reg        high_capacity;
    reg [7:0]  sig_hi, sig_lo;

    // initialisation sequence
    reg         init_start;
    wire        init_busy, init_done, init_error;
    wire [3:0]  init_phase;
    wire [31:0] init_ocr;
    wire [2:0]  init_type;
    wire        init_hc;
    wire        init_ncs, init_sclk, init_mosi;

    // sector read
    reg         sec_start;
    wire        sec_busy, sec_done, sec_waiting;
    wire [3:0]  sec_error;
    wire [7:0]  sec_r1, sec_token;
    wire [15:0] sec_crc;
    wire [8:0]  sec_index;
    wire [7:0]  sec_byte;
    wire        sec_valid;
    wire        sec_ncs, sec_sclk, sec_mosi;

    // saturating counter, so a card that never behaves cannot wrap a display
    function [7:0] bump;
        input [7:0] v;
        begin
            bump = (v == 8'hFF) ? v : (v + 8'd1);
        end
    endfunction

    // The partition's sector count, rounded up to whole GiB: an "8 GB" card
    // reports 7.3 GiB of sectors, and 8 is what the display should say.
    wire [32:0] gib_sum = {1'b0, part_sectors} + 33'h0_001F_FFFF;
    wire [11:0] gib_v   = gib_sum[32:21];
    wire [7:0]  gib_cap = (gib_v > 12'd255) ? 8'hFF : gib_v[7:0];

    // sector reads are shared: the probe reads LBA0 itself to check the MBR,
    // the scan asks for one sector at a time through this port.  Only one of
    // them ever has a request outstanding, so a plain mux does the job.
    wire        scan_rd_start;
    wire [31:0] scan_rd_lba;
    wire        sec_start_any = sec_start | scan_rd_start | aux_rd_start;
    wire [31:0] sec_lba_any   = scan_rd_start ? scan_rd_lba :
                                (aux_rd_start  ? aux_rd_lba  : 32'd0);

    // the loader's view of the engine: same handshake, same byte stream
    assign aux_sec_busy  = sec_busy;
    assign aux_sec_done  = sec_done;
    assign aux_sec_error = sec_error;
    assign aux_sec_valid = sec_valid;
    assign aux_sec_index = sec_index;
    assign aux_sec_byte  = sec_byte;

    //------------------------------------------------------------------
    // three engines, one SPI bus
    //
    // Only one is ever active: the sequence initialises the card, reads a
    // sector, scans the volume, and starts over if any step will not pass.
    // An idle engine leaves the bus alone (select high, clock low), so
    // switching between them cannot glitch the clock or the data line.
    //------------------------------------------------------------------
    mm_sd_init #(
        .POWER_CYCLES (INIT_POWER_CYCLES),
        .ACMD41_TRIES (INIT_ACMD41_TRIES)
    ) u_init (
        .clk           (clk),
        .rst           (rst),
        .start         (init_start),
        .busy          (init_busy),
        .done          (init_done),
        .error         (init_error),
        .state_code    (init_phase),
        .high_capacity (init_hc),
        .ocr           (init_ocr),
        .card_type     (init_type),
        .sd_ncs        (init_ncs),
        .sd_sclk       (init_sclk),
        .sd_mosi       (init_mosi),
        .sd_miso       (sd_miso)
    );

    mm_sd_sec #(
        .MAX_POLL_BYTES  (100_000),
        .MAX_TOKEN_BYTES (400_000)
    ) u_sec (
        .clk            (clk),
        .rst            (rst),
        .clk_div        (DATA_DIV),
        .start          (sec_start_any),
        .block_addr     (sec_lba_any),
        .byte_addressed (~high_capacity),   // v1 cards address bytes
        .busy           (sec_busy),
        .done           (sec_done),
        .error_code     (sec_error),
        .r1             (sec_r1),
        .err_token      (sec_token),
        .crc16          (sec_crc),
        .poll_bytes     (),
        .token_poll     (),
        .data_index     (sec_index),
        .data_byte      (sec_byte),
        .data_valid     (sec_valid),
        .waiting_data   (sec_waiting),
        .sd_ncs         (sec_ncs),
        .sd_sclk        (sec_sclk),
        .sd_mosi        (sec_mosi),
        .sd_miso        (sd_miso)
    );

    assign sd_ncs  = sec_busy ? sec_ncs  : init_ncs;
    assign sd_sclk = sec_busy ? sec_sclk : init_sclk;
    assign sd_mosi = sec_busy ? sec_mosi : init_mosi;

    //------------------------------------------------------------------
    // picture scan, started once the MBR has been read
    //------------------------------------------------------------------
    reg         scan_start;
    wire        scan_busy, scan_done;
    wire [3:0]  scan_error;
    wire [3:0]  scan_images;
    wire [31:0] scan_sectors;
    wire [31:0] scan_known;

    assign images_found = scan_images;      // live count, even mid scan

    //------------------------------------------------------------------
    // every failure goes through here: count it, then either try the
    // sequence again after a short pause or start over from initialisation
    //------------------------------------------------------------------
    task retry_or_reinit;
        begin
            error_count <= bump(error_count);
            disp_phase  <= PH_FAIL;
            if (read_try >= READ_TRIES) begin
                read_try <= 4'd0;
                wait_cnt <= 32'd0;
                state    <= ST_INIT_WAIT;
            end else begin
                wait_cnt <= 32'd0;
                state    <= ST_READ_WAIT;
            end
        end
    endtask

    mm_sd_scan #(
        .MAX_IMAGES        (8),
        .MAX_SECTORS       (SCAN_MAX_SECTORS),
        .EMPTY_GAP_SECTORS (SCAN_EMPTY_GAP),
        .MAX_FILE_BYTES    (8_388_608)
    ) u_scan (
        .clk          (clk),
        .rst          (rst),
        .start        (scan_start),
        .start_lba    (part_start_lba),     // the partition, not the raw card
        .busy         (scan_busy),
        .done         (scan_done),
        .error_code   (scan_error),
        .rd_lba       (scan_rd_lba),
        .rd_start     (scan_rd_start),
        .rd_done      (sec_done),
        .rd_error     (sec_error),
        .rd_index     (sec_index),
        .rd_byte      (sec_byte),
        .rd_valid     (sec_valid),
        .images_found (scan_images),
        .sectors_used (scan_sectors),
        .known_lba    (scan_known),
        .tbl_index    (tbl_index),
        .tbl_entry    (tbl_entry)
    );

    //------------------------------------------------------------------
    // sequence: initialise, read, check, repeat
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state          <= ST_INIT_WAIT;
            wait_cnt       <= 32'd0;
            read_try       <= 4'd0;
            high_capacity  <= 1'b0;
            init_start     <= 1'b0;
            sec_start      <= 1'b0;
            scan_start     <= 1'b0;
            sig_ok         <= 1'b0;
            sig_hi         <= 8'h00;
            sig_lo         <= 8'h00;
            part_type      <= 8'h00;
            part_start_lba <= 32'd0;
            part_sectors   <= 32'd0;
            capacity_gib   <= 8'h00;
            error_code     <= ERR_NONE;
            error_count    <= 8'd0;
            read_attempts  <= 8'd0;
            disp_phase     <= 4'h0;
        end else begin
            init_start <= 1'b0;
            sec_start  <= 1'b0;
            scan_start <= 1'b0;

            case (state)
                // wait before each attempt, so a card inserted later is
                // still picked up and a broken one is not hammered
                ST_INIT_WAIT: begin
                    if (wait_cnt == INIT_WAIT_CYCLES - 1) begin
                        wait_cnt <= 32'd0;
                        state    <= ST_INIT_GO;
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                ST_INIT_GO: begin
                    init_start <= 1'b1;
                    state      <= ST_INIT_RUN;
                end

                ST_INIT_RUN: begin
                    // digit 4 follows the sequence, but the CMD55 step (4)
                    // alternates with ACMD41 every few hundred microseconds,
                    // far faster than the display scans, so it is skipped
                    if (init_phase != 4'h0 && init_phase != 4'h4)
                        disp_phase <= init_phase;

                    if (init_done) begin
                        high_capacity <= init_hc;
                        read_try      <= 4'd0;
                        error_code    <= ERR_NONE;
                        state         <= ST_READ_GO;
                    end else if (init_error) begin
                        error_code  <= ERR_INIT;
                        error_count <= bump(error_count);
                        disp_phase  <= PH_FAIL;
                        state       <= ST_INIT_WAIT;
                    end
                end

                ST_READ_GO: begin
                    sec_start      <= 1'b1;
                    sig_ok         <= 1'b0;
                    sig_hi         <= 8'h00;
                    sig_lo         <= 8'h00;
                    part_type      <= 8'h00;
                    part_start_lba <= 32'd0;
                    part_sectors   <= 32'd0;
                    read_try       <= read_try + 4'd1;
                    read_attempts  <= bump(read_attempts);
                    state          <= ST_READ_RUN;
                end

                ST_READ_RUN: begin
                    // the sector streams past one byte at a time; pick out
                    // the fields the check needs
                    if (sec_valid) begin
                        if (sec_index == 9'd450)
                            part_type <= sec_byte;
                        if (sec_index >= 9'd454 && sec_index <= 9'd457)
                            part_start_lba <= {sec_byte, part_start_lba[31:8]};
                        if (sec_index >= 9'd458 && sec_index <= 9'd461)
                            part_sectors <= {sec_byte, part_sectors[31:8]};
                        if (sec_index == 9'd510)
                            sig_hi <= sec_byte;
                        if (sec_index == 9'd511)
                            sig_lo <= sec_byte;
                    end

                    if (sec_done) begin
                        if (sec_error != 4'd0) begin
                            // the card refused, or never sent the block
                            error_code <= sec_error;
                            retry_or_reinit;
                        end else if (sig_hi != 8'h55 || sig_lo != 8'hAA) begin
                            // a sector came back, but it is not an MBR
                            error_code <= ERR_SIG;
                            retry_or_reinit;
                        end else begin
                            // signature verified: keep the capacity, then go
                            // looking for the pictures themselves
                            sig_ok       <= 1'b1;
                            error_code   <= ERR_NONE;
                            capacity_gib <= (part_type != 8'h00 &&
                                             part_sectors != 32'd0) ? gib_cap : 8'h00;
                            read_try     <= 4'd0;
                            disp_phase   <= PH_OK;
                            state        <= ST_SCAN_GO;
                        end
                    end else begin
                        disp_phase <= sec_waiting ? PH_READ_DATA : PH_READ_CMD;
                    end
                end

                // brief pause before trying the sector again
                ST_READ_WAIT: begin
                    if (wait_cnt == READ_WAIT_CYCLES - 1) begin
                        wait_cnt <= 32'd0;
                        state    <= ST_READ_GO;
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                // walk the partition looking for the picture files
                ST_SCAN_GO: begin
                    scan_start <= 1'b1;
                    disp_phase <= PH_SCAN;
                    state      <= ST_SCAN_RUN;
                end

                ST_SCAN_RUN: begin
                    if (scan_done) begin
                        if (scan_error != 4'd0) begin
                            error_code <= scan_error;
                            retry_or_reinit;
                        end else if (scan_images == 4'd0) begin
                            // the volume holds no picture this design can use
                            error_code <= ERR_NONE_FOUND;
                            retry_or_reinit;
                        end else begin
                            error_code <= ERR_NONE;
                            read_try   <= 4'd0;
                            disp_phase <= PH_SCAN_DONE;
                            state      <= ST_DONE;
                        end
                    end
                end

                default: begin      // ST_DONE: found and verified, nothing to do
                    read_try <= 4'd0;
                end
            endcase
        end
    end

endmodule
