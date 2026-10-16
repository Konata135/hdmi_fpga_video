//=============================================================================
// tb_sd_probe.v -- self-checking testbench for mm_sd_probe
//
// A behavioural card serves the whole start-up sequence and then CMD17 with a
// made up volume (an MBR and two picture files), and the testbench checks what
// the two storage milestones are about:
//
//   1. a clean card: the MBR signature is found, the partition entry is
//      decoded (type, first LBA, sector count), the capacity in GiB is what
//      that sector count means, and the scan that follows records both
//      pictures in the fake volume with their start LBA and length
//   2. a card whose first read fails once, three ways: it drops the block
//      half way, it answers an error R1, and it returns a wrong signature.
//      Each one ends in a retry rather than a stall.  What is checked here is
//      the retry policy, not the individual error code: which code a
//      particular kind of failure produces is pinned down one at a time in
//      tb_sd_sec, where each failure can be produced on its own.
//   3. a card that fails three reads in a row is initialised again from
//      scratch, and a card that never becomes ready keeps being retried
//      without ever hanging
//   4. a volume with no picture file in it: the scan reports error 6 and the
//      sequence keeps retrying rather than giving up or hanging
//
// The MBR used here describes a 7.28 GiB partition (0x00E90000 sectors), which
// is what an "8 GB" card reports, so the capacity page says 08.  The pictures
// sit at LBA 2050 and 3856, the second one a small crawl after the first
// picture's 1801 sectors.
//=============================================================================
`timescale 1ns / 100ps

module tb_sd_probe;

    reg         clk;
    reg         rst;
    wire [3:0]  disp_phase;
    wire        sig_ok;
    wire [7:0]  part_type;
    wire [31:0] part_start_lba;
    wire [31:0] part_sectors;
    wire [7:0]  capacity_gib;
    wire [3:0]  images_found;
    reg  [2:0]  tbl_index;
    wire [63:0] tbl_entry;
    wire [3:0]  error_code;
    wire [7:0]  error_count;
    wire [7:0]  read_attempts;
    wire        sd_ncs;
    wire        sd_sclk;
    wire        sd_mosi;
    wire        sd_miso;

    integer errors;
    integer spin;
    reg     spin_timed_out;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial clk = 1'b0;
    always #5 clk = ~clk;           // 10 ns clock

    mm_sd_probe #(
        .INIT_POWER_CYCLES (50),
        .INIT_ACMD41_TRIES (6),
        .INIT_WAIT_CYCLES  (50),
        .READ_TRIES        (3),
        .READ_WAIT_CYCLES  (50),
        .DATA_CLK_DIV      (2),
        .SCAN_MAX_SECTORS  (5000),
        .SCAN_EMPTY_GAP    (6)
    ) dut (
        .clk            (clk),
        .rst            (rst),
        .disp_phase     (disp_phase),
        .sig_ok         (sig_ok),
        .part_type      (part_type),
        .part_start_lba (part_start_lba),
        .part_sectors   (part_sectors),
        .capacity_gib   (capacity_gib),
        .images_found   (images_found),
        .tbl_index      (tbl_index),
        .tbl_entry      (tbl_entry),
        .error_code     (error_code),
        .error_count    (error_count),
        .read_attempts  (read_attempts),
        // the picture loader's side of the shared sector engine: tied off here,
        // it is exercised by tb_img_load and tb_frame_chain
        .aux_rd_start   (1'b0),
        .aux_rd_lba     (32'd0),
        .aux_sec_busy   (),
        .aux_sec_done   (),
        .aux_sec_error  (),
        .aux_sec_valid  (),
        .aux_sec_index  (),
        .aux_sec_byte   (),
        .sd_ncs         (sd_ncs),
        .sd_sclk        (sd_sclk),
        .sd_mosi        (sd_mosi),
        .sd_miso        (sd_miso)
    );

    //------------------------------------------------------------------
    // behavioural card
    //
    // fail_no_token_n  the first n reads are answered but the block never
    //                  arrives: the model stops driving real data part way
    //                  through.  Which error code the DUT latches for this
    //                  depends on how far into the block it gets, so the
    //                  check below only requires that the attempt failed and
    //                  the retry succeeded (see the header note)
    // r1_err_n         the first n reads are answered with R1 = 0x40
    // bad_sig_n        the first n reads return a sector whose signature is
    //                  wrong (0x55 0x00 instead of 0x55 0xAA)
    // never_ready      ACMD41 never reports the card as ready
    //------------------------------------------------------------------
    reg [7:0]  cmd_buf [0:5];
    integer    bytes_seen;
    integer    rx_bit;
    reg [7:0]  rx_shift;
    reg [2:0]  tx_bit;
    reg [11:0] tx_byte_idx;

    integer cmd0_count;
    integer cmd17_count;
    integer acmd41_rounds;

    integer fail_no_token_n;
    integer r1_err_n;
    integer bad_sig_n;
    reg     never_ready;
    reg     card_is_v2;
    reg     vol_has_images;         // clear to test "the scan finds nothing"

    wire [5:0] seen_cmd  = cmd_buf[0][5:0];
    wire       ready_now = !never_ready && (acmd41_rounds >= 2);

    // where the two pictures live in the fake partition (the MBR says the
    // partition starts at 2048, and the scan starts there)
    localparam integer IMG0_LBA = 2050;
    localparam integer IMG1_LBA = 3856;     // 2050 + 1801 sectors + a small gap
    localparam integer BMP_BYTES = 921654;  // 54 + 640 x 480 x 3

    // the address the current CMD17 is asking for
    wire [31:0] sec_lba = {cmd_buf[1], cmd_buf[2], cmd_buf[3], cmd_buf[4]};

    // what one picture looks like at the start of its first sector
    function [7:0] bmp_hdr_byte;
        input [15:0] off;
        begin
            case (off)
                16'd0:  bmp_hdr_byte = 8'h42;              // 'B'
                16'd1:  bmp_hdr_byte = 8'h4D;              // 'M'
                16'd2:  bmp_hdr_byte = BMP_BYTES[7:0];     // length, little endian
                16'd3:  bmp_hdr_byte = BMP_BYTES[15:8];
                16'd4:  bmp_hdr_byte = BMP_BYTES[23:16];
                16'd5:  bmp_hdr_byte = BMP_BYTES[31:24];
                16'd10: bmp_hdr_byte = 8'd54;              // pixel data offset
                16'd18: bmp_hdr_byte = 8'h80;              // width 640
                16'd19: bmp_hdr_byte = 8'h02;
                16'd22: bmp_hdr_byte = 8'hE0;              // height 480
                16'd23: bmp_hdr_byte = 8'h01;
                16'd28: bmp_hdr_byte = 8'd24;              // bits per pixel
                16'd30: bmp_hdr_byte = 8'd0;               // no compression
                default: bmp_hdr_byte = 8'h00;
            endcase
        end
    endfunction

    // the sector the card serves: an MBR with one FAT32 partition
    function [7:0] mbr_byte;
        input [15:0] off;
        begin
            case (off)
                16'd446: mbr_byte = 8'h80;      // bootable
                16'd450: mbr_byte = 8'h0C;      // FAT32 with LBA
                16'd454: mbr_byte = 8'h00;      // first LBA = 2048
                16'd455: mbr_byte = 8'h08;
                16'd456: mbr_byte = 8'h00;
                16'd457: mbr_byte = 8'h00;
                16'd458: mbr_byte = 8'h00;      // 0x00E90000 sectors
                16'd459: mbr_byte = 8'h00;
                16'd460: mbr_byte = 8'hE9;
                16'd461: mbr_byte = 8'h00;
                16'd510: mbr_byte = 8'h55;      // MBR signature
                16'd511: mbr_byte = (cmd17_count <= bad_sig_n) ? 8'h00 : 8'hAA;
                default: mbr_byte = 8'h00;
            endcase
        end
    endfunction

    // one sector of the fake volume: LBA0 is the MBR, the two picture
    // positions carry a BMP header, everything else reads as unwritten space
    function [7:0] vol_byte;
        input [31:0] lba;
        input [15:0] off;
        begin
            if (lba == 32'd0)
                vol_byte = mbr_byte(off);
            else if (vol_has_images && (lba == IMG0_LBA || lba == IMG1_LBA))
                vol_byte = bmp_hdr_byte(off);
            else
                vol_byte = 8'h00;
        end
    endfunction

    function [7:0] card_byte;
        input [5:0]  cmd;
        input [15:0] idx;
        begin
            card_byte = 8'hFF;
            if (idx > 6) begin            // 48 bit command plus one NCR byte
                case (cmd)
                    6'd0:  card_byte = (idx == 7) ? 8'h01 : 8'hFF;
                    6'd8:  case (idx)
                               7:  card_byte = card_is_v2 ? 8'h01 : 8'h05;
                               8:  card_byte = 8'h00;
                               9:  card_byte = 8'h00;
                               10: card_byte = 8'h01;
                               11: card_byte = 8'hAA;
                               default: card_byte = 8'hFF;
                           endcase
                    6'd55: card_byte = (idx == 7) ? 8'h01 : 8'hFF;
                    6'd41: card_byte = (idx == 7) ? (ready_now ? 8'h00 : 8'h01)
                                                  : 8'hFF;
                    6'd58: case (idx)
                               7:  card_byte = 8'h00;
                               8:  card_byte = 8'hC0;
                               9:  card_byte = 8'hFF;
                               10: card_byte = 8'h80;
                               11: card_byte = 8'h00;
                               default: card_byte = 8'hFF;
                           endcase
                    6'd17: begin
                               if (idx == 7)
                                   card_byte = (cmd17_count <= r1_err_n) ? 8'h40 : 8'h00;
                               else if (cmd17_count <= fail_no_token_n)
                                   card_byte = 8'hFF;      // response, then silence
                               else if (idx == 10)
                                   card_byte = 8'hFE;      // data token
                               else if (idx >= 11 && idx <= 522)
                                   card_byte = vol_byte(sec_lba, idx - 16'd11);
                           end
                    default: ;
                endcase
            end
        end
    endfunction

    wire [7:0] card_out = card_byte(seen_cmd, tx_byte_idx);
    assign sd_miso = card_out[3'd7 - tx_bit];

    // a new chip select transaction: restart the byte counters
    always @(negedge sd_ncs) begin
        bytes_seen  = 0;
        rx_bit      = 0;
        rx_shift    = 8'd0;
        tx_bit      = 0;
        tx_byte_idx = 0;
    end

    always @(posedge sd_sclk) begin
        rx_shift = {rx_shift[6:0], sd_mosi};
        rx_bit   = rx_bit + 1;
        if (rx_bit == 8) begin
            rx_bit = 0;
            if (bytes_seen < 6)
                cmd_buf[bytes_seen] = rx_shift;
            bytes_seen = bytes_seen + 1;
        end
    end

    always @(negedge sd_sclk) begin
        if (tx_bit == 3'd7) begin
            tx_bit      = 3'd0;
            tx_byte_idx = tx_byte_idx + 12'd1;
            // the command is known once its first poll byte starts
            if (tx_byte_idx == 12'd6) begin
                if (seen_cmd == 6'd0)  cmd0_count  = cmd0_count  + 1;
                if (seen_cmd == 6'd17) cmd17_count = cmd17_count + 1;
                if (seen_cmd == 6'd41) acmd41_rounds = acmd41_rounds + 1;
            end
        end else begin
            tx_bit = tx_bit + 3'd1;
        end
    end

    //------------------------------------------------------------------
    // helpers
    //------------------------------------------------------------------
    task pulse_reset;
        begin
            rst = 1'b1;
            repeat (4) @(posedge clk);
            rst = 1'b0;
            repeat (4) @(posedge clk);
        end
    endtask

    task clear_card;
        begin
            cmd0_count    = 0;
            cmd17_count   = 0;
            acmd41_rounds = 0;
            never_ready   = 1'b0;
        end
    endtask

    task wait_sig;
        input integer limit;
        begin
            spin           = 0;
            spin_timed_out = 1'b0;
            while (!sig_ok && (spin < limit)) begin
                @(posedge clk);
                spin = spin + 1;
            end
            if (spin >= limit) begin
                spin_timed_out = 1'b1;
                fail("the MBR check never passed");
            end
        end
    endtask

    task wait_cmd0_count;
        input integer want;
        input integer limit;
        begin
            spin = 0;
            while ((cmd0_count < want) && (spin < limit)) begin
                @(posedge clk);
                spin = spin + 1;
            end
            if (cmd0_count < want)
                fail("the card was never initialised again");
        end
    endtask

    // the scan runs after the MBR check, so wait for the phase that means
    // "MBR read, scan finished, pictures recorded"
    task wait_scan_done;
        input integer limit;
        begin
            spin           = 0;
            spin_timed_out = 1'b0;
            while ((disp_phase !== 4'hC) && (spin < limit)) begin
                @(posedge clk);
                spin = spin + 1;
            end
            if (spin >= limit) begin
                spin_timed_out = 1'b1;
                fail("the scan never completed");
            end
        end
    endtask

    task check_table;
        input integer n;
        input integer exp_lba;
        input integer exp_len;
        begin
            tbl_index = n[2:0];
            #1;
            if (tbl_entry[63:32] !== exp_lba[31:0]) begin
                $display("  FAIL: picture %0d starts at %0d, expected %0d",
                         n, tbl_entry[63:32], exp_lba);
                errors = errors + 1;
            end
            if (tbl_entry[31:0] !== exp_len[31:0]) begin
                $display("  FAIL: picture %0d length %0d, expected %0d",
                         n, tbl_entry[31:0], exp_len);
                errors = errors + 1;
            end
        end
    endtask

    // The last failure code seen, so each scenario can say which error the
    // retry actually recovered from.  This is a level sensitive monitor on
    // error_code only: it never touches the card model's byte framing.
    reg [3:0] err_latch;

    always @(posedge clk) begin
        if (rst)
            err_latch <= 4'd0;
        else if (error_code != 4'd0)
            err_latch <= error_code;
    end

    //------------------------------------------------------------------
    // scenarios
    //------------------------------------------------------------------
    integer errors_before;

    initial begin
        errors          = 0;
        rst             = 1'b1;
        bytes_seen      = 0;
        rx_bit          = 0;
        rx_shift        = 8'd0;
        tx_bit          = 0;
        tx_byte_idx     = 0;
        card_is_v2      = 1'b1;
        vol_has_images  = 1'b1;
        fail_no_token_n = 0;
        r1_err_n        = 0;
        bad_sig_n       = 0;
        tbl_index       = 3'd0;
        clear_card;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (4) @(posedge clk);

        // ---- 1. a clean card -------------------------------------------
        errors_before = errors;
        clear_card;
        fail_no_token_n = 0;
        r1_err_n        = 0;
        bad_sig_n       = 0;
        vol_has_images  = 1'b1;
        pulse_reset;
        wait_sig(2_000_000);
        wait_scan_done(4_000_000);
        // the sequence has to *stay* finished.  A state register that is one
        // bit too narrow makes ST_DONE alias ST_INIT_WAIT, so the whole chain
        // silently restarts (that is what the board showed on 2026-10-13:
        // the display went back to "b" every few seconds).
        if (!spin_timed_out) begin
            spin = 0;
            while ((spin < 600) && (cmd0_count == 1)) begin
                @(posedge clk);
                spin = spin + 1;
            end
            if (cmd0_count !== 1)
                fail("the sequence restarted by itself after finishing");
            if (disp_phase !== 4'hC)
                fail("the phase did not stay on C after finishing");
            if (images_found !== 4'd2)
                fail("the picture count changed after finishing");
        end
        if (!spin_timed_out) begin
            if (part_type !== 8'h0C) fail("the partition type was not captured");
            if (part_start_lba !== 32'd2048) begin
                $display("  FAIL: first LBA is %0d, expected 2048", part_start_lba);
                errors = errors + 1;
            end
            if (part_sectors !== 32'h00E9_0000) begin
                $display("  FAIL: sector count is %08x, expected 00E90000", part_sectors);
                errors = errors + 1;
            end
            if (capacity_gib !== 8'd8) begin
                $display("  FAIL: capacity is %0d GiB, expected 8", capacity_gib);
                errors = errors + 1;
            end
            if (read_attempts !== 8'd1) fail("the clean read did not succeed first time");
            if (cmd0_count !== 1) fail("a clean card was initialised more than once");
            if (error_code !== 4'd0) fail("a good read left an error code behind");
            if (images_found !== 4'd2) fail("the scan did not find both pictures");
            check_table(0, 2050, BMP_BYTES);
            check_table(1, 3856, BMP_BYTES);
            tbl_index = 3'd0;
            if (errors == errors_before)
                $display("  clean card   : PASS (8 GiB partition, 2 pictures at 2050 and 3856)");
        end

        // ---- 2. the first read drops out part way through ---------------
        errors_before = errors;
        clear_card;
        fail_no_token_n = 1;
        r1_err_n        = 0;
        bad_sig_n       = 0;
        pulse_reset;
        wait_sig(2_000_000);
        wait_scan_done(4_000_000);
        if (!spin_timed_out) begin
            if (read_attempts !== 8'd2) fail("a dropped read was not retried exactly once");
            if (cmd0_count !== 1) fail("a dropped read should not need a new init");
            if (error_code !== 4'd0) fail("the retry did not clear the error code");
            if (err_latch == 4'd0) fail("the dropped read latched no failure at all");
            if (images_found !== 4'd2) fail("the retried sequence did not reach the pictures");
            if (errors == errors_before)
                $display("  dropped read : PASS (retried after error %0d, read on attempt 2)",
                         err_latch);
        end

        // ---- 3. the first read is answered with an error R1 ------------
        errors_before = errors;
        clear_card;
        fail_no_token_n = 0;
        r1_err_n        = 1;
        bad_sig_n       = 0;
        pulse_reset;
        wait_sig(2_000_000);
        wait_scan_done(4_000_000);
        if (!spin_timed_out) begin
            if (read_attempts !== 8'd2) fail("an error R1 was not retried exactly once");
            if (error_code !== 4'd0) fail("the retry did not clear the error code");
            if (err_latch !== 4'd2) fail("an error R1 did not latch error 2");
            if (images_found !== 4'd2) fail("the retried sequence did not reach the pictures");
            if (errors == errors_before)
                $display("  error R1     : PASS (retried after error %0d, read on attempt 2)",
                         err_latch);
        end

        // ---- 4. three bad signatures: initialise again, then read ------
        errors_before = errors;
        clear_card;
        fail_no_token_n = 0;
        r1_err_n        = 0;
        bad_sig_n       = 3;
        pulse_reset;
        wait_sig(2_000_000);
        wait_scan_done(4_000_000);
        if (!spin_timed_out) begin
            if (read_attempts !== 8'd4) begin
                $display("  FAIL: %0d read attempts, expected 4", read_attempts);
                errors = errors + 1;
            end
            if (cmd0_count !== 2) fail("three bad signatures did not re-initialise the card");
            if (error_code !== 4'd0) fail("the re-init did not clear the error code");
            if (err_latch !== 4'd5) fail("a wrong signature did not latch error 5");
            if (images_found !== 4'd2) fail("the re-initialised sequence did not reach the pictures");
            if (errors == errors_before)
                $display("  bad signature: PASS (re-initialised after error %0d, read on attempt 4)",
                         err_latch);
        end

        // ---- 5. a card that never leaves idle --------------------------
        errors_before = errors;
        clear_card;
        never_ready = 1'b1;
        pulse_reset;
        wait_cmd0_count(3, 2_000_000);
        // the display alternates between the latched failure and the start of
        // the next attempt, so wait for the failure to be latched again
        spin = 0;
        while ((disp_phase !== 4'hF) && (spin < 2_000_000)) begin
            @(posedge clk);
            spin = spin + 1;
        end
        if (disp_phase !== 4'hF) begin
            fail("digit 4 never showed F for a card that cannot initialise");
        end else begin
            if (read_attempts !== 8'd0) fail("a card that never initialised was read anyway");
            if (error_code !== 4'd7) fail("an initialisation failure did not report error 7");
            if (errors == errors_before)
                $display("  never ready  : PASS (kept retrying, no lock up)");
        end

        // ---- 6. a volume that holds no picture file --------------------
        errors_before = errors;
        clear_card;
        never_ready    = 1'b0;
        vol_has_images = 1'b0;       // the MBR is fine, the pictures are gone
        pulse_reset;
        spin = 0;
        while ((err_latch !== 4'd6) && (spin < 4_000_000)) begin
            @(posedge clk);
            spin = spin + 1;
        end
        if (err_latch !== 4'd6) begin
            fail("a volume with no picture file did not report error 6");
        end else begin
            if (sig_ok !== 1'b1) fail("the MBR check should still have passed");
            if (read_attempts < 8'd2) fail("the empty volume was not retried");
            if (errors == errors_before)
                $display("  no pictures  : PASS (error 6, kept retrying, no lock up)");
        end
        vol_has_images = 1'b1;

        if (errors == 0)
            $display("RESULT: PASS -- initialise, CMD17, MBR check and retries verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #150_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
