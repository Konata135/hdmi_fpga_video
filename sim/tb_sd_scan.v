//=============================================================================
// tb_sd_scan.v -- self-checking testbench for mm_sd_scan
//
// The card is not simulated cycle by cycle here: the scan talks to a sector
// read port, so the testbench just fakes that port with a generated volume.
// That makes it cheap to check the *policy*, which is what this module is:
//
//   1. an empty volume ends by the gap rule, with a precise number of reads
//   2. pictures are recorded with their start LBA and length, audio tracks
//      are stepped over in one go, and the sector counter shows both: a
//      volume where the scan reads 44 sectors but accounts for 9407 of them
//      can only be skipping
//   3. every way a header can be wrong is rejected, and a good header right
//      behind the bad ones is still found
//   4. the table stops at MAX_IMAGES and the ninth picture is never read
//   5. a sector read error aborts the scan with the error code
//   6. a valid but absurdly long picture cannot underflow the budget
//=============================================================================
`timescale 1ns / 100ps

module tb_sd_scan;

    reg         clk;
    reg         rst;
    reg         start;
    reg         start_b;            // the small budget instance has its own
    reg  [31:0] start_lba;
    wire        busy;
    wire        done;
    wire [3:0]  error_code;
    wire [31:0] rd_lba;
    wire        rd_start;
    reg         rd_done;
    reg  [3:0]  rd_error;
    reg  [8:0]  rd_index;
    reg  [7:0]  rd_byte;
    reg         rd_valid;
    wire [3:0]  images_found;
    wire [31:0] sectors_used;
    wire [31:0] known_lba;
    reg  [2:0]  tbl_index;
    wire [63:0] tbl_entry;

    integer errors;
    integer spin;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial clk = 1'b0;
    always #5 clk = ~clk;

    mm_sd_scan #(
        .MAX_IMAGES        (8),
        .MAX_SECTORS       (20000),
        .EMPTY_GAP_SECTORS (40),
        .MAX_FILE_BYTES    (8_388_608)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .start        (start),
        .start_lba    (start_lba),
        .busy         (busy),
        .done         (done),
        .error_code   (error_code),
        .rd_lba       (rd_lba),
        .rd_start     (rd_start),
        .rd_done      (rd_done),
        .rd_error     (rd_error),
        .rd_index     (rd_index),
        .rd_byte      (rd_byte),
        .rd_valid     (rd_valid),
        .images_found (images_found),
        .sectors_used (sectors_used),
        .known_lba    (known_lba),
        .tbl_index    (tbl_index),
        .tbl_entry    (tbl_entry)
    );

    //------------------------------------------------------------------
    // the volume: a list of files, everything else reads as zero
    //
    // kinds   1 = BMP, 2 = MMAU
    // flags   BMP  : [0] bad magic [1] width 320 [2] depth 8
    //                [3] compression 1 [4] pixel offset 40 [5] length too small
    //         MMAU : [0] bad magic [1] version 2 [2] bad data offset
    //                [3] zero data length
    //------------------------------------------------------------------
    integer   n_items;
    reg [1:0] it_kind  [0:15];
    integer   it_lba   [0:15];
    integer   it_size  [0:15];
    reg [7:0] it_flags [0:15];

    integer BMP_BYTES;      // 921654: what tools/prepare_images.py writes
    integer BMP_SECTORS;    // 1801

    function [7:0] vol_byte;
        input integer s;
        input integer o;
        integer   k;
        reg [7:0] r;
        begin
            r = 8'h00;
            for (k = 0; k < 16; k = k + 1) begin
                if (k < n_items && s == it_lba[k]) begin
                    if (it_kind[k] == 1) begin
                        if (o == 0)       r = it_flags[k][0] ? 8'h58 : 8'h42;  // 'X' / 'B'
                        else if (o == 1)  r = it_flags[k][0] ? 8'h58 : 8'h4D;  // 'X' / 'M'
                        else if (o == 2)  r = it_size[k][7:0];
                        else if (o == 3)  r = it_size[k][15:8];
                        else if (o == 4)  r = it_size[k][23:16];
                        else if (o == 5)  r = it_size[k][31:24];
                        else if (o == 10) r = it_flags[k][4] ? 8'd40 : 8'd54;
                        else if (o == 18) r = it_flags[k][1] ? 8'd64 : 8'd128;
                        else if (o == 19) r = it_flags[k][1] ? 8'd1  : 8'd2;
                        else if (o == 22) r = 8'd224;                          // 480
                        else if (o == 23) r = 8'd1;
                        else if (o == 28) r = it_flags[k][2] ? 8'd8 : 8'd24;
                        else if (o == 30) r = it_flags[k][3] ? 8'd1 : 8'd0;
                    end else begin
                        if (o == 0)       r = it_flags[k][0] ? 8'h58 : 8'h4D;  // 'M'
                        else if (o == 1)  r = 8'h4D;
                        else if (o == 2)  r = 8'h41;
                        else if (o == 3)  r = 8'h55;
                        else if (o == 4)  r = it_flags[k][1] ? 8'd2 : 8'd1;  // version 1
                        else if (o == 5)  r = 8'd0;                          // little endian
                        else if (o == 20) r = it_flags[k][3] ? 8'd0 : it_size[k][7:0];
                        else if (o == 21) r = it_size[k][15:8];
                        else if (o == 22) r = it_size[k][23:16];
                        else if (o == 23) r = it_size[k][31:24];
                        else if (o == 24) r = 8'd0;                            // 512 -> 0x00000200
                        else if (o == 25) r = it_flags[k][2] ? 8'd0 : 8'd2;
                    end
                end
            end
            vol_byte = r;
        end
    endfunction

    //------------------------------------------------------------------
    // the sector read port
    //------------------------------------------------------------------
    integer n_reads;
    integer first_read_lba;
    integer last_read_lba;
    integer fail_lba;               // -1 disables the error injection
    reg     inject_error;

    reg        rd_active;
    integer    rd_cur;
    integer    rd_off;

    // A second instance with a tiny budget shares this port, so the budget
    // guard can be exercised without starving the table-full scenario.  Only
    // one of the two is ever scanning, so the port sees at most one request
    // at a time.
    wire        rd_start_b;
    wire [31:0] rd_lba_b;
    wire        rd_start_any = rd_start | rd_start_b;
    wire [31:0] rd_lba_any   = rd_start_b ? rd_lba_b : rd_lba;

    wire        busy_b;
    wire        done_b;
    wire [3:0]  error_b;
    wire [3:0]  images_b;
    wire [31:0] sectors_b;
    wire [31:0] known_b;

    mm_sd_scan #(
        .MAX_IMAGES        (8),
        .MAX_SECTORS       (100),
        .EMPTY_GAP_SECTORS (40),
        .MAX_FILE_BYTES    (8_388_608)
    ) dut_b (
        .clk          (clk),
        .rst          (rst),
        .start        (start_b),
        .start_lba    (start_lba),
        .busy         (busy_b),
        .done         (done_b),
        .error_code   (error_b),
        .rd_lba       (rd_lba_b),
        .rd_start     (rd_start_b),
        .rd_done      (rd_done),
        .rd_error     (rd_error),
        .rd_index     (rd_index),
        .rd_byte      (rd_byte),
        .rd_valid     (rd_valid),
        .images_found (images_b),
        .sectors_used (sectors_b),
        .known_lba    (known_b),
        .tbl_index    (3'd0),
        .tbl_entry    ()
    );

    always @(posedge clk) begin
        rd_valid <= 1'b0;
        rd_done  <= 1'b0;
        rd_error <= 4'd0;

        if (rd_start_any) begin
            rd_cur   = rd_lba_any;
            rd_off   = 0;
            rd_active = 1'b1;
            n_reads  = n_reads + 1;
            if (n_reads == 1) first_read_lba = rd_lba_any;
            last_read_lba = rd_lba_any;
        end else if (rd_active) begin
            if (rd_off < 512) begin
                rd_byte  <= vol_byte(rd_cur, rd_off);
                rd_index <= rd_off[8:0];
                rd_valid <= 1'b1;
                rd_off    = rd_off + 1;
            end else begin
                rd_active = 1'b0;
                rd_done   <= 1'b1;
                if (inject_error && (rd_cur == fail_lba))
                    rd_error <= 4'd2;
            end
        end
    end

    //------------------------------------------------------------------
    // helpers
    //------------------------------------------------------------------
    integer errors_before;
    integer k;

    task clear_volume;
        begin
            n_items      = 0;
            n_reads      = 0;
            inject_error = 1'b0;
            fail_lba     = -1;
        end
    endtask

    task add_bmp;
        input integer lba;
        input integer size;
        input [7:0]   flags;
        begin
            it_kind[n_items]  = 2'd1;
            it_lba[n_items]   = lba;
            it_size[n_items]  = size;
            it_flags[n_items] = flags;
            n_items           = n_items + 1;
        end
    endtask

    task add_audio;
        input integer lba;
        input integer data_bytes;
        input [7:0]   flags;
        begin
            it_kind[n_items]  = 2'd2;
            it_lba[n_items]   = lba;
            it_size[n_items]  = data_bytes;
            it_flags[n_items] = flags;
            n_items           = n_items + 1;
        end
    endtask

    task run_scan;
        input integer lba0;
        input integer limit;
        begin
            rst = 1'b1;
            repeat (3) @(posedge clk);
            rst = 1'b0;
            repeat (2) @(posedge clk);

            @(negedge clk);
            start_lba = lba0;
            start     = 1'b1;
            @(negedge clk);
            start     = 1'b0;

            spin = 0;
            while (!done && (spin < limit)) begin
                @(posedge clk);
                spin = spin + 1;
            end
            if (spin >= limit) fail("the scan never finished");
            @(negedge clk);
            if (busy) fail("still busy after done");
        end
    endtask

    task check_entry;
        input integer n;
        input integer exp_lba;
        input integer exp_len;
        begin
            tbl_index = n[2:0];
            #1;
            if (tbl_entry[63:32] !== exp_lba[31:0]) begin
                $display("  FAIL: entry %0d lba = %0d, expected %0d",
                         n, tbl_entry[63:32], exp_lba);
                errors = errors + 1;
            end
            if (tbl_entry[31:0] !== exp_len[31:0]) begin
                $display("  FAIL: entry %0d length = %0d, expected %0d",
                         n, tbl_entry[31:0], exp_len);
                errors = errors + 1;
            end
            @(negedge clk);
        end
    endtask

    task run_scan_b;
        input integer lba0;
        input integer limit;
        begin
            rst = 1'b1;
            repeat (3) @(posedge clk);
            rst = 1'b0;
            repeat (2) @(posedge clk);

            @(negedge clk);
            start_lba = lba0;
            start_b   = 1'b1;
            @(negedge clk);
            start_b   = 1'b0;

            spin = 0;
            while (!done_b && (spin < limit)) begin
                @(posedge clk);
                spin = spin + 1;
            end
            if (spin >= limit) fail("the scan never finished on the small budget");
            @(negedge clk);
        end
    endtask

    //------------------------------------------------------------------
    // scenarios
    //------------------------------------------------------------------
    initial begin
        errors       = 0;
        rst          = 1'b1;
        start        = 1'b0;
        start_lba    = 32'd0;
        tbl_index    = 3'd0;
        rd_active    = 1'b0;
        rd_cur       = 0;
        rd_off       = 0;
        fail_lba     = -1;
        inject_error = 1'b0;
        n_items      = 0;
        n_reads      = 0;
        first_read_lba = 0;
        last_read_lba  = 0;

        BMP_BYTES   = 921654;               // 54 + 640 x 480 x 3
        BMP_SECTORS = 1801;                 // what one picture steps over

        // ---- 1. an empty volume: the gap rule ends the scan ------------
        errors_before = errors;
        clear_volume;
        run_scan(1000, 200000);
        if (images_found !== 4'd0) fail("an empty volume produced pictures");
        if (error_code !== 4'd0) fail("an empty volume reported an error");
        if (n_reads !== 41) begin
            $display("  FAIL: %0d sector reads on an empty volume, expected 41", n_reads);
            errors = errors + 1;
        end
        if (errors == errors_before)
            $display("  empty volume : PASS (%0d reads, gap rule stopped it)", n_reads);

        // ---- 2. two pictures with a music track between them -----------
        errors_before = errors;
        clear_volume;
        add_bmp  (1000, BMP_BYTES, 8'h00);
        add_audio(1000 + BMP_SECTORS, 2949120, 8'h00);              // 2801
        add_bmp  (1000 + BMP_SECTORS + 5761, BMP_BYTES, 8'h00);     // 8562
        run_scan(1000, 400000);
        if (images_found !== 4'd2) fail("did not record exactly two pictures");
        if (error_code !== 4'd0) fail("a clean scan reported an error");
        check_entry(0, 1000, BMP_BYTES);
        check_entry(1, 8562, BMP_BYTES);
        if (n_reads !== 44) begin
            $display("  FAIL: %0d sector reads, expected 44", n_reads);
            errors = errors + 1;
        end
        if (sectors_used < 32'd9000) fail("the sector counter does not show the skips");
        if (errors == errors_before)
            $display("  pictures+audio: PASS (%0d reads for %0d sectors, skips work)",
                     n_reads, sectors_used);

        // ---- 3. every kind of broken header is refused -----------------
        errors_before = errors;
        clear_volume;
        add_bmp(1000, BMP_BYTES, 8'h01);    // magic is not 'BM'
        add_bmp(1001, BMP_BYTES, 8'h02);    // width 320
        add_bmp(1002, BMP_BYTES, 8'h04);    // depth 8
        add_bmp(1003, BMP_BYTES, 8'h08);    // compressed
        add_bmp(1004, BMP_BYTES, 8'h10);    // pixel data at offset 40
        add_bmp(1005, 154,       8'h20);    // length does not cover the pixels
        add_bmp(1006, BMP_BYTES, 8'h00);    // and one good picture
        run_scan(1000, 300000);
        if (images_found !== 4'd1) fail("a broken header was accepted");
        if (error_code !== 4'd0) fail("the reject scan reported an error");
        check_entry(0, 1006, BMP_BYTES);
        if (errors == errors_before)
            $display("  broken headers: PASS (six rejected, the good one found)");

        // ---- 4. the table fills up and stops the scan ------------------
        errors_before = errors;
        clear_volume;
        for (k = 0; k < 8; k = k + 1)
            add_bmp(1000 + BMP_SECTORS * k, BMP_BYTES, 8'h00);
        add_bmp(1000 + BMP_SECTORS * 8, BMP_BYTES, 8'h00);      // a ninth
        run_scan(1000, 300000);
        if (images_found !== 4'd8) fail("the table did not fill up with 8 pictures");
        check_entry(7, 1000 + BMP_SECTORS * 7, BMP_BYTES);
        if (last_read_lba == (1000 + BMP_SECTORS * 8))
            fail("the scan read a ninth picture after the table was full");
        if (errors == errors_before)
            $display("  table full    : PASS (8 recorded, the ninth never read)");

        // ---- 5. a sector read error aborts the scan --------------------
        errors_before = errors;
        clear_volume;
        add_bmp(1000, BMP_BYTES, 8'h00);
        inject_error = 1'b1;
        fail_lba     = 1000;
        run_scan(1000, 100000);
        if (error_code !== 4'd2) fail("a sector read error was not reported");
        if (images_found !== 4'd0) fail("a failed read was recorded as a picture");
        if (errors == errors_before)
            $display("  read error    : PASS (error 2, nothing recorded)");

        // ---- 6. a very long picture cannot underflow the budget --------
        // (the small budget instance, see dut_b, makes this cheap to check)
        errors_before = errors;
        clear_volume;
        add_bmp(1000, 4194304, 8'h00);      // valid header, 8192 sector step
        run_scan_b(1000, 100000);
        if (images_b !== 4'd1) fail("the long picture was not recorded");
        if (error_b !== 4'd0) fail("stopping on the budget was reported as an error");
        if (n_reads !== 1) fail("the budget stop should have taken one read");
        if (errors == errors_before)
            $display("  budget guard  : PASS (stopped after %0d read)", n_reads);

        if (errors == 0)
            $display("RESULT: PASS -- picture scan verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
