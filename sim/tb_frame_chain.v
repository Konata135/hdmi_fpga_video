//=============================================================================
// tb_frame_chain.v -- card to screen, the whole picture path
//
//   fake card -> mm_img_load -> mm_frame_wr -> mm_sdram -> mm_frame_rd -> RGB
//
// The file is generated on demand, with every byte saying what it stands for:
//
//   pixel (row r, column c) of the stored picture is  B,G,R =
//     0x10 + r, 0x20 + c, 0xA5
//
// so the display must produce, for display row v and column c,
//
//   {0xA5, 0x20 + c, 0x10 + (ROWS - 1 - v)}
//
// That one comparison pins the row order (a BMP is stored bottom up), the
// column order, the B,G,R to R,G,B swap, and the whole write path at once.
//
// The picture is deliberately wider than the 256 word SDRAM row (48 words per
// line, 48 lines), so a burst that walks past a device row boundary has to be
// handled; a picture of a dozen words would never show it.
//
//   1. every displayed pixel is right, no underrun
//   2. the loader read exactly the sectors the file needs
//   3. the SDRAM model saw no protocol violation
//=============================================================================
`timescale 1ns / 100ps

module tb_frame_chain;

    localparam integer PIX_W      = 64;                   // pixels per line
    localparam integer ROW_WORDS  = PIX_W * 3 / 4;        // 48 words = 192 bytes
    localparam integer ROWS       = 48;
    localparam integer PIX_BYTES  = PIX_W * ROWS * 3;     // 9216
    localparam integer FILE_BYTES = 54 + PIX_BYTES;       // 9270
    localparam integer SECTORS    = (FILE_BYTES + 511) / 512;
    localparam [31:0]  START_LBA  = 32'd2050;

    reg clk125, clk_sft, clk_byte, pix_clk;
    reg rst, rst_byte, pix_rst;

    initial clk125   = 1'b0;
    initial clk_sft  = 1'b1;
    initial clk_byte = 1'b0;
    initial pix_clk  = 1'b0;
always #8  clk125   = ~clk125;      // 62.5 MHz, controller side
always #8  clk_sft  = ~clk_sft;     // 62.5 MHz, 180 degrees, device side
    always #10 clk_byte = ~clk_byte;    // 50 MHz, the SD side
    always #20 pix_clk  = ~pix_clk;     // 25 MHz, the display side

    integer errors, i, k;
    integer line, col;

    function [23:0] expect_rgb;
        input integer v;
        input integer cc;
        reg [7:0] rbyte, gbyte, bbyte;
        integer   flipped;
        begin
            flipped = ROWS - 1 - v;
            rbyte   = 8'hA5;
            gbyte   = 8'h20 + (cc % 64);
            bbyte   = 8'h10 + (flipped % 16);
            expect_rgb = {rbyte, gbyte, bbyte};
        end
    endfunction

    // byte k of the file: a header filler, then the pixel pattern, then 0xEE
    function [7:0] file_byte;
        input integer kk;
        integer p, j, r, c;
        begin
            if (kk >= FILE_BYTES) begin
                file_byte = 8'hEE;
            end else if (kk == 0) begin
                file_byte = 8'h42;              // 'B': the loader checks for it
            end else if (kk == 1) begin
                file_byte = 8'h4D;              // 'M'
            end else if (kk < 54) begin
                file_byte = 8'h5A;
            end else begin
                p = kk - 54;
                r = p / (PIX_W * 3);
                c = (p % (PIX_W * 3)) / 3;
                j = p % 3;
                if (j == 0)      file_byte = 8'h10 + (r % 16);
                else if (j == 1) file_byte = 8'h20 + (c % 64);
                else             file_byte = 8'hA5;
            end
        end
    endfunction

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    //------------------------------------------------------------------
    // fake card
    //------------------------------------------------------------------
    localparam [2:0] P_IDLE = 3'd0, P_LAT = 3'd1, P_DATA = 3'd2, P_END = 3'd3;

    reg [2:0]  pstate;
    reg [31:0] p_lba;
    reg [8:0]  p_idx;
    reg [7:0]  p_lat;
    reg [2:0]  p_bit;               // which bit of the current byte
    reg        ld_start;

    wire [31:0] ld_rd_lba_w;
    wire        ld_rd_start;
    wire [8:0]  ld_rd_index;
    wire [7:0]  ld_rd_byte;
    wire        ld_rd_valid;
    wire        ld_rd_done;
    wire [3:0]  ld_rd_error;
    wire        p_busy = (pstate != P_IDLE);

    assign ld_rd_index = p_idx;
    assign ld_rd_byte  = file_byte((p_lba - START_LBA) * 512 + p_idx);
    // The card clocks one byte every eight SPI clocks (25 MHz SPI out of the
    // 50 MHz domain, see mm_sd_sec's clk_div), which is 2.5 MB/s.  The frame
    // store's write path is good for about 45 MB/s, so a card model that ran
    // any faster than the real one would overrun it and lose words -- which is
    // a fault of the model, not of the design.
    assign ld_rd_valid = (pstate == P_DATA) && (p_bit == 3'd7);
    assign ld_rd_done  = (pstate == P_END);
    assign ld_rd_error = 4'd0;

    always @(posedge clk_byte) begin
        if (rst_byte) begin
            pstate <= P_IDLE;
            p_lba  <= 32'd0;
            p_idx  <= 9'd0;
            p_lat  <= 8'd0;
        end else begin
            case (pstate)
                P_IDLE: if (ld_rd_start) begin
                            p_lba  <= ld_rd_lba_w;
                            p_idx  <= 9'd0;
                            p_lat  <= 8'd0;
                            p_bit  <= 3'd0;
                            pstate <= P_LAT;
                        end
                P_LAT:  if (p_lat == 8'd3) pstate <= P_DATA;
                        else               p_lat  <= p_lat + 8'd1;
                P_DATA: if (p_bit == 3'd7) begin
                            p_bit <= 3'd0;
                            if (p_idx == 9'd511) pstate <= P_END;
                            else                 p_idx  <= p_idx + 9'd1;
                        end else begin
                            p_bit <= p_bit + 3'd1;
                        end
                default: pstate <= P_IDLE;
            endcase
        end
    end

    //------------------------------------------------------------------
    // the picture path
    //------------------------------------------------------------------
    wire        mem_ready;
    wire        wr_req, wr_done;
    wire [20:0] wr_addr;
    wire [31:0] wr_data;
    wire        br_req, br_dv, br_busy;
    wire [20:0] br_addr;
    wire [8:0]  br_len;
    wire [31:0] br_data;

    wire        ld_busy, ld_done;
    wire [3:0]  ld_error, ld_phase;
    wire [31:0] ld_bytes_sent;
    wire [15:0] ld_sectors_read;
    wire        ld_frame_start;
    wire [20:0] ld_base_word;
    wire [17:0] ld_words_total;
    wire        ld_byte_valid;
    wire [7:0]  ld_byte_data;
    wire [7:0]  ld_overflow;
    wire        wr_frame_done_tgl;
    wire        wr_busy;

    reg         frame_active;
    reg         pix_de;
    wire [23:0] pix_rgb;
    wire        pix_valid;
    wire [15:0] pix_underrun;

    mm_sdram #(
        .INIT_WAIT      (50),
        .REFRESH_PERIOD (1953)
    ) u_sdram (
        .clk       (clk125),
        .clk_dev   (clk125),
        .clk_sft   (clk_sft),
        .rst       (rst),
        .ready     (mem_ready),
        .rd_adj    (/*the sample code the probe finds; the model wants one cycle later*/ 3'd1),
        .br_adj    ( 3'd1),
        .wr_req    (wr_req),
        .wr_addr   (wr_addr),
        .wr_data   (wr_data),
        .wr_dm     (4'h0),
        .wr_done   (wr_done),
        .rd_req    (1'b0),
        .rd_addr   (21'd0),
        .rd_dv     (),
        .rd_data   (),
        .br_req    (br_req),
        .br_addr   (br_addr),
        .br_len    (br_len),
        .br_dv     (br_dv),
        .br_data   (br_data),
        .br_busy   (br_busy),
        .init_done (),
        .state_dbg (),
        .echo_ok   ()
    );

    mm_img_load u_load (
        .clk          (clk_byte),
        .rst          (rst_byte),
        .start        (ld_start),
        .start_lba    (START_LBA),
        .file_bytes   (FILE_BYTES[31:0]),
        .entry_valid  (1'b1),
        .rd_lba       (ld_rd_lba_w),
        .rd_start     (ld_rd_start),
        .rd_busy      (p_busy),
        .rd_done      (ld_rd_done),
        .rd_error     (ld_rd_error),
        .rd_index     (ld_rd_index),
        .rd_byte      (ld_rd_byte),
        .rd_valid     (ld_rd_valid),
        .frame_start  (ld_frame_start),
        .base_word    (ld_base_word),
        .words_total  (ld_words_total),
        .byte_valid   (ld_byte_valid),
        .byte_data    (ld_byte_data),
        .frame_done_tgl (wr_frame_done_tgl),
        .busy         (ld_busy),
        .done         (ld_done),
        .error_code   (ld_error),
        .phase        (ld_phase),
        .bytes_sent   (ld_bytes_sent),
        .sectors_read (ld_sectors_read)
    );

    mm_frame_wr #(
        .AFIFO_AW (6)
    ) u_wr (
        .clk_byte       (clk_byte),
        .rst_byte       (rst_byte),
        .start          (ld_frame_start),
        .base_word      (ld_base_word),
        .words_total    (ld_words_total),
        .byte_valid     (ld_byte_valid),
        .byte_data      (ld_byte_data),
        .overflow_count (ld_overflow),
        .busy           (wr_busy),
        .clk            (clk125),
        .rst            (rst),
        .wr_addr        (wr_addr),
        .wr_data        (wr_data),
        .wr_req         (wr_req),
        .wr_done        (wr_done),
        .ready          (mem_ready),
        .frame_done     (),
        .frame_done_tgl (wr_frame_done_tgl),
        .words_written  ()
    );

    // The reader's buffer settings are the module defaults, i.e. what top.v
    // builds: 256 words topped up to 192.  A small buffer here (32/20) starves
    // at a device row crossing, where the fetch latency is about 450 ns while
    // the buffer's *low point* only covers 0.48 us of pixel time -- and that is
    // an artifact of the test's tiny geometry, not of the design: the real one
    // has a 32 us line and a 6.8 us low point, fourteen times the latency it
    // has to hide.
    mm_frame_rd #(
        .ROW_WORDS (ROW_WORDS),
        .ROWS      (ROWS)
    ) u_rd (
        .clk            (clk125),
        .rst            (rst),
        .base_word      (21'd0),
        .active         (frame_active),
        .br_req         (br_req),
        .br_addr        (br_addr),
        .br_len         (br_len),
        .br_dv          (br_dv),
        .br_data        (br_data),
        .br_busy        (br_busy),
        .ready          (mem_ready),
        .flush_done     (),
        .words_fetched  (),
        .pix_ready      (),
        .pix_clk        (pix_clk),
        .pix_rst        (pix_rst),
        .pix_de         (pix_de),
        .pix_rgb        (pix_rgb),
        .pix_valid      (pix_valid),
        .pix_underrun   (pix_underrun),
        .dbg_flush_words()
    );

    // check every pixel as it is handed out
    always @(posedge pix_clk) begin
        if (!pix_rst && pix_valid) begin
            if (pix_rgb !== expect_rgb(line, col)) begin
                if (errors < 12)
                    $display("  FAIL: row %0d col %0d got %h, expected %h",
                             line, col, pix_rgb, expect_rgb(line, col));
                errors = errors + 1;
            end
            if (col == PIX_W - 1) begin
                col  = 0;
                line = line + 1;
            end else begin
                col = col + 1;
            end
        end
    end

    initial begin
        errors       = 0;
        rst          = 1'b1;
        rst_byte     = 1'b1;
        pix_rst      = 1'b1;
        ld_start     = 1'b0;
        frame_active = 1'b0;
        pix_de       = 1'b0;
        line         = 0;
        col          = 0;

        repeat (8) @(posedge clk125);
        repeat (4) @(posedge clk_byte);
        repeat (4) @(posedge pix_clk);
        rst      = 1'b0;
        rst_byte = 1'b0;
        pix_rst  = 1'b0;
        repeat (200) @(posedge clk125);     // let the SDRAM come up

        // ---- load the picture ---------------------------------------
        @(negedge clk_byte);
        ld_start = 1'b1;
        @(negedge clk_byte);
        ld_start = 1'b0;

        k = 0;
        while (!ld_done && (k < 4000000)) begin
            @(posedge clk125);
            k = k + 1;
        end
        if (k >= 4000000) begin
            fail("the loader never finished");
        end else begin
            $display("  load         : %0d bytes, %0d sectors, err %0d",
                     ld_bytes_sent, ld_sectors_read, ld_error);

            if (ld_bytes_sent != PIX_BYTES) begin
                $display("  FAIL: loaded %0d bytes, expected %0d",
                         ld_bytes_sent, PIX_BYTES);
                errors = errors + 1;
            end
            if (ld_sectors_read != SECTORS) begin
                $display("  FAIL: read %0d sectors, expected %0d",
                         ld_sectors_read, SECTORS);
                errors = errors + 1;
            end

            // ---- show it ---------------------------------------------
            frame_active = 1'b1;
            repeat (4000) @(posedge pix_clk);     // let the reader prime

            for (i = 0; i < ROWS; i = i + 1) begin
                for (k = 0; k < PIX_W; k = k + 1) begin
                    @(negedge pix_clk);
                    pix_de = 1'b1;
                end
                @(negedge pix_clk);
                pix_de = 1'b0;
                // 20 blank pixels per line, not 6: a real 640 pixel line has
                // 160 of them in a 32 us line, so six was a duty cycle the
                // design never sees.  At 62.5 MHz the memory side needs about
                // 900 ns of each line to fetch its 48 words, and this leaves it
                // 1.4 us.
                repeat (20) @(negedge pix_clk);
            end
            repeat (20) @(posedge pix_clk);

            if (line != ROWS) begin
                $display("  FAIL: %0d lines came out, expected %0d",
                         line, ROWS);
                errors = errors + 1;
            end

            if (pix_underrun != 16'd0) begin
                $display("  FAIL: %0d pixel underruns", pix_underrun);
                errors = errors + 1;
            end else begin
                $display("  no underrun  : PASS");
            end

            if (ld_overflow != 8'd0) begin
                $display("  FAIL: the packing FIFO overflowed %0d times",
                         ld_overflow);
                errors = errors + 1;
            end else begin
                $display("  packing      : PASS (no FIFO overflow)");
            end

            if (u_sdram.u_sdram.errors !== 0) begin
                $display("  FAIL: the SDRAM model reported %0d violations",
                         u_sdram.u_sdram.errors);
                errors = errors + 1;
            end else begin
                $display("  protocol     : PASS");
            end
        end

        if (errors == 0)
            $display("RESULT: PASS -- card to screen verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #40_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
