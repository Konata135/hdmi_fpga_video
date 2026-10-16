//=============================================================================
// tb_img_load.v -- self-checking testbench for mm_img_load
//
// A fake card hands out sectors on demand.  Byte k of the file is just k mod
// 256, so the pixel stream the loader must produce is known in advance:
//
//   pixel byte k = (54 + k) mod 256        (k = 0 .. 1023)
//
// Everything past the end of the file is 0xEE, so a loader that hands over
// trailing sector filler instead of stopping at the length is caught at once.
//
//  1. exactly 1024 pixel bytes, all correct, in order
//  2. the header's 54 bytes never appear on the wire
//  3. words_total / base_word / frame_start are right
//  4. a sector that fails once is retried, and the frame still completes
//  5. done only comes after the writer says the frame has drained
//=============================================================================
`timescale 1ns / 100ps

module tb_img_load;

    localparam integer FILE_BYTES = 54 + 1024;      // header + 256 words
    localparam integer PIX_BYTES  = 1024;
    localparam integer WORDS      = PIX_BYTES / 4;
    localparam integer SECTORS    = (FILE_BYTES + 511) / 512;
    localparam [31:0]  START_LBA  = 32'd2050;
    localparam [17:0]  WORDS18    = WORDS;
    localparam [15:0]  SECTORS16  = SECTORS;

    reg        clk, rst;
    reg        start;
    reg [31:0] start_lba, file_bytes;
    reg        entry_valid;

    wire [31:0] rd_lba;
    wire        rd_start;
    wire        rd_busy;
    wire        rd_done;
    wire [3:0]  rd_error;
    wire [8:0]  rd_index;
    wire [7:0]  rd_byte;
    wire        rd_valid;

    wire        frame_start;
    wire [20:0] base_word;
    wire [17:0] words_total;
    wire        byte_valid;
    wire [7:0]  byte_data;
    reg         frame_done_tgl;

    wire        busy, done;
    wire [3:0]  error_code, phase;
    wire [31:0] bytes_sent;
    wire [15:0] sectors_read;
    wire        hdr_ok;
    wire [7:0]  ff_first;

    integer errors, i, k;
    integer nbytes, nstarts, ndone, nreq;
    reg [7:0] captured [0:4095];
    reg [20:0] cap_base;
    reg [17:0] cap_total;

    initial clk = 1'b0;
    always #10 clk = ~clk;                  // 50 MHz

    //------------------------------------------------------------------
    // fake card
    //------------------------------------------------------------------
    localparam [2:0] P_IDLE = 3'd0, P_LAT = 3'd1, P_DATA = 3'd2, P_END = 3'd3;

    reg [2:0]  pstate;
    reg [31:0] p_lba;
    reg [8:0]  p_idx;
    reg [7:0]  p_err;
    reg        injected;          // the "fails once" trick is used only once
    reg [7:0]  p_lat;

    // a switch so the last scenario can hand the loader a sector that is not a
    // BMP at all
    reg hdr_not_bmp;

    // byte k of the file, and 0xEE past its end
    function [7:0] file_byte;
        input integer kk;
        begin
            if (kk >= FILE_BYTES) file_byte = 8'hEE;
            else if (kk == 0)     file_byte = hdr_not_bmp ? 8'h11 : 8'h42;  // 'B'
            else if (kk == 1)     file_byte = hdr_not_bmp ? 8'h22 : 8'h4D;  // 'M'
            else                  file_byte = kk % 256;
        end
    endfunction

    assign rd_busy  = (pstate != P_IDLE);
    assign rd_index = p_idx;
    assign rd_byte  = file_byte((p_lba - START_LBA) * 512 + p_idx);
    assign rd_valid = (pstate == P_DATA);
    assign rd_done  = (pstate == P_END);
    assign rd_error = p_err;

    always @(posedge clk) begin
        if (rst) begin
            pstate   <= P_IDLE;
            p_lba    <= 32'd0;
            p_idx    <= 9'd0;
            p_err    <= 8'd0;
            p_lat    <= 8'd0;
            injected <= 1'b0;
        end else begin
            case (pstate)
                P_IDLE: begin
                    if (rd_start) begin
                        p_lba  <= rd_lba;
                        p_idx  <= 9'd0;
                        p_lat  <= 8'd0;
                        // the second sector of the file fails on its first try
                        if ((rd_lba == START_LBA + 32'd1) && !injected) begin
                            injected <= 1'b1;
                            p_err    <= 8'd2;
                            pstate   <= P_END;
                        end else begin
                            p_err  <= 8'd0;
                            pstate <= P_LAT;
                        end
                    end
                end

                P_LAT: begin
                    if (p_lat == 8'd3) pstate <= P_DATA;
                    else               p_lat  <= p_lat + 8'd1;
                end

                P_DATA: begin
                    if (p_idx == 9'd511) pstate <= P_END;
                    else                 p_idx  <= p_idx + 9'd1;
                end

                default: pstate <= P_IDLE;      // P_END, one cycle wide
            endcase
        end
    end

    //------------------------------------------------------------------
    // model of the frame writer's completion pulse
    //------------------------------------------------------------------
    integer wr_bytes;
    integer drain_wait;

    always @(posedge clk) begin
        if (rst) begin
            wr_bytes   <= 0;
            drain_wait <= -1;
            frame_done_tgl <= 1'b0;
        end else begin
            // like mm_frame_wr: a start resets the count for the new frame
            if (frame_start)    wr_bytes <= 0;
            else if (byte_valid) wr_bytes <= wr_bytes + 1;
            if (drain_wait >= 0) begin
                if (drain_wait == 0) begin
                    frame_done_tgl <= ~frame_done_tgl;
                    drain_wait <= -1;
                end else begin
                    drain_wait <= drain_wait - 1;
                end
            end else if ((wr_bytes == WORDS * 4) && (drain_wait < 0)) begin
                drain_wait <= 30;              // the memory side needs a moment
            end
        end
    end

    //------------------------------------------------------------------
    // the unit under test
    //------------------------------------------------------------------
    mm_img_load #(
        .HEADER_BYTES (54)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .start        (start),
        .start_lba    (start_lba),
        .file_bytes   (file_bytes),
        .frame_base   (21'd0),      // the frame store starts at word 0 here
        .entry_valid  (entry_valid),
        .rd_lba       (rd_lba),
        .rd_start     (rd_start),
        .rd_busy      (rd_busy),
        .rd_done      (rd_done),
        .rd_error     (rd_error),
        .rd_index     (rd_index),
        .rd_byte      (rd_byte),
        .rd_valid     (rd_valid),
        .frame_start  (frame_start),
        .base_word    (base_word),
        .words_total  (words_total),
        .byte_valid   (byte_valid),
        .byte_data    (byte_data),
        .frame_done_tgl (frame_done_tgl),
        .busy         (busy),
        .done         (done),
        .error_code   (error_code),
        .phase        (phase),
        .bytes_sent   (bytes_sent),
        .sectors_read (sectors_read),
        .hdr_ok       (hdr_ok),
        .ff_first     (ff_first)
    );

    // collect what the loader hands to the writer
    always @(posedge clk) begin
        if (!rst && byte_valid && (nbytes < 4096)) begin
            captured[nbytes] = byte_data;
            nbytes = nbytes + 1;
        end
    end

    always @(posedge clk) begin
        if (!rst && frame_start) begin
            nstarts   = nstarts + 1;
            cap_base  = base_word;
            cap_total = words_total;
        end
        if (!rst && done) ndone = ndone + 1;
        if (!rst && rd_start) nreq = nreq + 1;
    end

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial begin
        errors      = 0;
        rst         = 1'b1;
        start       = 1'b0;
        entry_valid = 1'b0;
        start_lba   = START_LBA;
        file_bytes  = FILE_BYTES;
        nbytes      = 0;
        nstarts     = 0;
        ndone       = 0;
        nreq        = 0;
        cap_base    = 21'd0;
        cap_total   = 18'd0;
        hdr_not_bmp = 1'b0;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (4) @(posedge clk);

        // ---- load the picture ---------------------------------------
        entry_valid = 1'b1;
        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // wait for the frame to be handed over
        k = 0;
        while (!done && (k < 200000)) begin
            @(posedge clk);
            k = k + 1;
        end
        if (k >= 200000) begin
            fail("the load never finished (watchdog)");
        end else begin
            repeat (400) @(posedge clk);        // let any straggler byte land

            if (nbytes != PIX_BYTES) begin
                $display("  FAIL: handed over %0d bytes, expected %0d",
                         nbytes, PIX_BYTES);
                errors = errors + 1;
            end else begin
                $display("  bytes        : PASS (%0d pixel bytes)", nbytes);
            end

            for (i = 0; i < PIX_BYTES; i = i + 1) begin
                if (captured[i] !== ((54 + i) % 256)) begin
                    $display("  FAIL: pixel byte %0d is %h, expected %h",
                             i, captured[i], (54 + i) % 256);
                    errors = errors + 1;
                    i = PIX_BYTES + 1;          // one report is enough
                end
            end
            if (errors == 0)
                $display("  order        : PASS (header skipped, no filler)");

            if (!hdr_ok) begin
                fail("rejected a file that does start with BM");
            end else if (ff_first !== 8'd1) begin
                $display("  FAIL: 0xFF count over the first 255 pixel bytes is %0d, expected 1",
                         ff_first);
                errors = errors + 1;
            end else begin
                $display("  header       : PASS (BM seen, %0d bytes at 0xFF)",
                         ff_first);
            end

            if (nstarts != 1) begin
                $display("  FAIL: %0d frame starts, expected 1", nstarts);
                errors = errors + 1;
            end else if (cap_total !== WORDS18) begin
                $display("  FAIL: words_total %0d, expected %0d", cap_total, WORDS);
                errors = errors + 1;
            end else if (cap_base !== 21'd0) begin
                $display("  FAIL: base_word %h, expected 0", cap_base);
                errors = errors + 1;
            end else begin
                $display("  writer       : PASS (%0d words, base 0, one start)",
                         cap_total);
            end

            if (ndone != 1) begin
                $display("  FAIL: %0d done pulses, expected 1", ndone);
                errors = errors + 1;
            end else begin
                $display("  done         : PASS (after the writer drained)");
            end

            if (nreq != SECTORS + 1) begin
                // three sectors, and the one that fails is asked for twice:
                // without the extra request the retry never happened
                $display("  FAIL: %0d read requests, expected %0d (a retry)",
                         nreq, SECTORS + 1);
                errors = errors + 1;
            end else if (sectors_read != SECTORS16) begin
                $display("  FAIL: read %0d sectors, expected %0d",
                         sectors_read, SECTORS);
                errors = errors + 1;
            end else begin
                $display("  retry        : PASS (%0d sectors, one read failed first)",
                         sectors_read);
            end
        end

        // ---- a load with nothing to load must fail, not start a frame ---
        begin : failure_case
            integer bytes_before, starts_before;
            bytes_before  = nbytes;
            starts_before = nstarts;
            entry_valid   = 1'b0;
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            repeat (20) @(posedge clk);

            if (phase !== 4'hF) begin
                $display("  FAIL: empty entry gave phase %h, expected F", phase);
                errors = errors + 1;
            end else if (error_code !== 4'd2) begin
                $display("  FAIL: empty entry gave error %0d, expected 2",
                         error_code);
                errors = errors + 1;
            end else if (busy) begin
                fail("still busy after an empty entry");
            end else if ((nbytes != bytes_before) || (nstarts != starts_before)) begin
                fail("an empty entry still started a frame");
            end else begin
                $display("  empty        : PASS (phase F, error 2, no frame)");
            end
        end

        // ---- and a start after that failure must load normally ----------
        begin : restart_case
            integer bytes_before, starts_before;
            bytes_before  = nbytes;
            starts_before = nstarts;
            entry_valid   = 1'b1;
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;

            k = 0;
            while (!done && (k < 200000)) begin
                @(posedge clk);
                k = k + 1;
            end
            repeat (200) @(posedge clk);

            if (k >= 200000) begin
                fail("the load did not restart after a failure");
            end else if (nbytes != bytes_before + PIX_BYTES) begin
                $display("  FAIL: restart handed over %0d bytes, expected %0d",
                         nbytes - bytes_before, PIX_BYTES);
                errors = errors + 1;
            end else if (nstarts != starts_before + 1) begin
                $display("  FAIL: restart issued %0d frame starts, expected 1",
                         nstarts - starts_before);
                errors = errors + 1;
            end else begin
                $display("  restart      : PASS (recovered from the failure)");
            end
        end

        // ---- a sector that is not a BMP must stop the load ---------------
        begin : notbmp_case
            integer bytes_before, starts_before;
            bytes_before  = nbytes;
            starts_before = nstarts;
            hdr_not_bmp = 1'b1;         // the card now answers with something else
            entry_valid = 1'b1;
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            repeat (20000) @(posedge clk);

            if (phase !== 4'hF) begin
                $display("  FAIL: a non-BMP sector gave phase %h, expected F",
                         phase);
                errors = errors + 1;
            end else if (error_code !== 4'd4) begin
                $display("  FAIL: a non-BMP sector gave error %0d, expected 4",
                         error_code);
                errors = errors + 1;
            end else if ((nbytes != bytes_before) || (nstarts != starts_before)) begin
                fail("a non-BMP sector still handed bytes to the writer");
            end else if (ff_first !== 8'd0) begin
                fail("a non-BMP sector still counted 0xFF bytes");
            end else begin
                $display("  not BMP      : PASS (error 4, nothing written)");
            end
        end

        if (errors == 0)
            $display("RESULT: PASS -- picture loader verified");
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
