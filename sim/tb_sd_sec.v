//=============================================================================
// tb_sd_sec.v -- self-checking testbench for mm_sd_sec (CMD17, one sector)
//
// A behavioural card answers a sector read byte by byte.  The command frame
// is compared against the values the SD specification prescribes:
//
//   CMD17 with argument 0        -> 51 00 00 00 00 55
//   CMD17 with argument 0x0A00   -> 51 00 00 0A 00 C9   (v1 card, LBA 5)
//
// The CRC bytes are also recomputed here by polynomial division of the
// shifted command, which is a different formulation from the bit-at-a-time
// loop in the RTL, and the testbench checks its own version against the two
// published values above before using it.
//
// Then, for the data phase:
//
//   * all 512 bytes arrive once, in order, with data_index counting 0..511
//   * the two CRC bytes are clocked but never appear as data
//   * chip select is held over the whole exchange and released at the end
//
// and for the four ways a card can refuse:
//
//   * no response at all            -> error 1, after MAX_POLL_BYTES
//   * R1 with an error bit set      -> error 2, R1 kept for the display
//   * response but no data token    -> error 3, after MAX_TOKEN_BYTES
//   * a data error token (0x0D)     -> error 4, token kept
//
// None of them may leave the module busy or the card selected.
//=============================================================================
`timescale 1ns / 100ps

module tb_sd_sec;

    reg         clk;
    reg         rst;
    reg  [15:0] clk_div;
    reg         start;
    reg  [31:0] block_addr;
    reg         byte_addressed;
    wire        busy;
    wire        done;
    wire [3:0]  error_code;
    wire [7:0]  r1;
    wire [7:0]  err_token;
    wire [15:0] crc16;
    wire [15:0] poll_bytes;
    wire [19:0] token_poll;
    wire [8:0]  data_index;
    wire [7:0]  data_byte;
    wire        data_valid;
    wire        waiting_data;
    wire        sd_ncs;
    wire        sd_sclk;
    wire        sd_mosi;
    wire        sd_miso;

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
    always #5 clk = ~clk;           // 10 ns clock

    mm_sd_sec #(
        .MAX_POLL_BYTES  (12),
        .MAX_TOKEN_BYTES (12)
    ) dut (
        .clk            (clk),
        .rst            (rst),
        .clk_div        (clk_div),
        .start          (start),
        .block_addr     (block_addr),
        .byte_addressed (byte_addressed),
        .busy           (busy),
        .done           (done),
        .error_code     (error_code),
        .r1             (r1),
        .err_token      (err_token),
        .crc16          (crc16),
        .poll_bytes     (poll_bytes),
        .token_poll     (token_poll),
        .data_index     (data_index),
        .data_byte      (data_byte),
        .data_valid     (data_valid),
        .waiting_data   (waiting_data),
        .sd_ncs         (sd_ncs),
        .sd_sclk        (sd_sclk),
        .sd_mosi        (sd_mosi),
        .sd_miso        (sd_miso)
    );

    //------------------------------------------------------------------
    // behavioural card
    //
    // mode 0  a good card: R1 at byte 7, the 0xFE token at token_at, then
    //         the 512 byte sector and two dummy CRC bytes
    // mode 1  never answers anything
    // mode 2  answers R1 = 0x40 (a parameter error)
    // mode 3  answers R1 = 0x00 and then never offers a token
    // mode 4  answers R1 = 0x00 and then a data error token 0x0D
    //------------------------------------------------------------------
    reg [2:0]  mode;
    reg [15:0] token_at;

    reg [7:0]  cmd_buf [0:5];
    integer    bytes_seen;
    integer    rx_bit;
    reg [7:0]  rx_shift;
    reg [2:0]  tx_bit;
    reg [9:0]  tx_byte_idx;

    function [7:0] sector_byte;
        input [15:0] i;
        begin
            sector_byte = ((i * 16'd37) + 16'd11) ^ 16'h005A;
        end
    endfunction

    function [7:0] card_byte;
        input [15:0] idx;
        begin
            card_byte = 8'hFF;
            case (mode)
                3'd0: begin
                    if (idx == 7)
                        card_byte = 8'h00;
                    else if (idx >= token_at && idx < token_at + 16'd513)
                        card_byte = (idx == token_at) ? 8'hFE
                                                      : sector_byte(idx - token_at - 16'd1);
                end
                3'd2: if (idx == 7) card_byte = 8'h40;
                3'd3: if (idx == 7) card_byte = 8'h00;
                3'd4: begin
                    if (idx == 7)      card_byte = 8'h00;
                    else if (idx == 8) card_byte = 8'h0D;
                end
                default: ;              // mode 1: all 0xFF
            endcase
        end
    endfunction

    wire [7:0] card_out = card_byte(tx_byte_idx);
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
            tx_byte_idx = tx_byte_idx + 10'd1;
        end else begin
            tx_bit = tx_bit + 3'd1;
        end
    end

    //------------------------------------------------------------------
    // data phase observer
    //------------------------------------------------------------------
    integer data_seen;

    always @(posedge clk) begin
        if (data_valid) begin
            if (data_index !== data_seen[8:0]) begin
                errors = errors + 1;
                $display("  FAIL: byte %0d has index %0d", data_seen, data_index);
            end
            if (data_byte !== sector_byte(data_seen[15:0])) begin
                errors = errors + 1;
                $display("  FAIL: byte %0d = %02x, expected %02x",
                         data_seen, data_byte, sector_byte(data_seen[15:0]));
            end
            data_seen = data_seen + 1;
        end
    end

    //------------------------------------------------------------------
    // CRC7, recomputed as a polynomial division: the message is shifted up
    // by seven places and divided by x^7 + x^3 + 1 (0x89).
    //------------------------------------------------------------------
    function [6:0] crc7_of;
        input [5:0]  idx;
        input [31:0] arg;
        integer    i;
        reg [46:0] d;
        begin
            d = {{2'b01, idx}, arg, 7'b0};
            for (i = 46; i >= 7; i = i - 1) begin
                if (d[i])
                    d = d ^ (47'h89 << (i - 7));
            end
            crc7_of = d[6:0];
        end
    endfunction

    //------------------------------------------------------------------
    // helpers
    //------------------------------------------------------------------
    integer errors_before;

    task do_read;
        input [31:0] lba;
        input        byteaddr;
        begin
            @(negedge clk);
            block_addr     = lba;
            byte_addressed = byteaddr;
            start          = 1'b1;
            @(negedge clk);
            start          = 1'b0;

            spin = 0;
            while (!done && (spin < 20000)) begin
                @(posedge clk);
                spin = spin + 1;
            end
            if (spin >= 20000) fail("sector read never finished");
            @(negedge clk);

            if (busy) fail("still busy after done");
            if (sd_ncs !== 1'b1) fail("chip select not released after done");
        end
    endtask

    task check_frame;
        input [5:0]  idx;
        input [31:0] arg;
        reg   [7:0]  crc_byte;
        begin
            crc_byte = {crc7_of(idx, arg), 1'b1};
            if ((cmd_buf[0] !== {2'b01, idx}) || (cmd_buf[1] !== arg[31:24]) ||
                (cmd_buf[2] !== arg[23:16]) || (cmd_buf[3] !== arg[15:8]) ||
                (cmd_buf[4] !== arg[7:0])   || (cmd_buf[5] !== crc_byte)) begin
                $display("  FAIL: cmd %0d framing = %02x %02x %02x %02x %02x %02x",
                         idx, cmd_buf[0], cmd_buf[1], cmd_buf[2],
                         cmd_buf[3], cmd_buf[4], cmd_buf[5]);
                $display("        expected    %02x %02x %02x %02x %02x %02x",
                         {2'b01, idx}, arg[31:24], arg[23:16],
                         arg[15:8], arg[7:0], crc_byte);
                errors = errors + 1;
            end
        end
    endtask

    //------------------------------------------------------------------
    // scenarios
    //------------------------------------------------------------------
    initial begin
        errors         = 0;
        rst            = 1'b1;
        clk_div        = 16'd2;             // fast for simulation
        start          = 1'b0;
        block_addr     = 32'd0;
        byte_addressed = 1'b0;
        mode           = 3'd0;
        token_at       = 16'd10;
        bytes_seen     = 0;
        rx_bit         = 0;
        rx_shift       = 8'd0;
        tx_bit         = 0;
        tx_byte_idx    = 0;
        data_seen      = 0;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        // ---- 0. the testbench's own CRC7, against published frames -----
        if ({crc7_of(6'd0, 32'd0), 1'b1} !== 8'h95) begin
            $display("  FAIL: tb CRC7 for CMD0 is %02x, expected 95",
                     {crc7_of(6'd0, 32'd0), 1'b1});
            errors = errors + 1;
        end
        if ({crc7_of(6'd8, 32'h0000_01AA), 1'b1} !== 8'h87) begin
            $display("  FAIL: tb CRC7 for CMD8 is %02x, expected 87",
                     {crc7_of(6'd8, 32'h0000_01AA), 1'b1});
            errors = errors + 1;
        end

        // ---- 1. a good card, block 0 -----------------------------------
        errors_before = errors;
        mode          = 3'd0;
        token_at      = 16'd10;
        data_seen     = 0;
        do_read(32'd0, 1'b0);
        check_frame(6'd17, 32'd0);
        if (cmd_buf[5] !== 8'h55) fail("CMD17 CRC byte is not the published 0x55");
        if (error_code !== 4'd0) fail("a good read reported an error");
        if (poll_bytes !== 16'd1) begin
            $display("  FAIL: polled %0d bytes before R1, expected 1", poll_bytes);
            errors = errors + 1;
        end
        if (data_seen != 512) begin
            $display("  FAIL: %0d data bytes, expected 512", data_seen);
            errors = errors + 1;
        end
        if (crc16 !== 16'hFFFF) begin
            $display("  FAIL: CRC16 field is %04x, expected the card's FFFF", crc16);
            errors = errors + 1;
        end
        if (errors == errors_before)
            $display("  good read    : PASS (512 bytes, frame 51 00 00 00 00 55)");

        // ---- 2. a v1 card addresses bytes: LBA 5 -> 0x0A00 -------------
        errors_before = errors;
        mode          = 3'd0;
        token_at      = 16'd12;
        data_seen     = 0;
        do_read(32'd5, 1'b1);
        check_frame(6'd17, 32'h0000_0A00);
        if (cmd_buf[5] !== 8'hC9) fail("byte addressed CMD17 CRC byte is not 0xC9");
        if (error_code !== 4'd0) fail("the byte addressed read reported an error");
        if (data_seen != 512) fail("the byte addressed read did not deliver 512 bytes");
        if (errors == errors_before)
            $display("  byte address : PASS (LBA 5 -> argument 0x0A00)");

        // ---- 3. no response at all -------------------------------------
        errors_before = errors;
        mode          = 3'd1;
        data_seen     = 0;
        do_read(32'd0, 1'b0);
        if (error_code !== 4'd1) fail("a card that never answers did not report error 1");
        if (poll_bytes !== 16'd11) fail("the no response timeout is not MAX_POLL_BYTES");
        if (data_seen != 0) fail("data arrived from a card that never answered");
        if (errors == errors_before)
            $display("  no response  : PASS (error 1 after 12 poll bytes)");

        // ---- 4. R1 with an error bit set -------------------------------
        errors_before = errors;
        mode          = 3'd2;
        data_seen     = 0;
        do_read(32'd0, 1'b0);
        if (error_code !== 4'd2) fail("an error R1 did not report error 2");
        if (r1 !== 8'h40) fail("the error R1 was not kept for the display");
        if (data_seen != 0) fail("data arrived after an error R1");
        if (errors == errors_before)
            $display("  error R1     : PASS (error 2, R1 = 0x40 kept)");

        // ---- 5. response, then never a token ---------------------------
        errors_before = errors;
        mode          = 3'd3;
        data_seen     = 0;
        do_read(32'd0, 1'b0);
        if (error_code !== 4'd3) fail("a missing data token did not report error 3");
        if (token_poll !== 20'd11) fail("the token timeout is not MAX_TOKEN_BYTES");
        if (data_seen != 0) fail("data arrived without a token");
        if (errors == errors_before)
            $display("  no token     : PASS (error 3 after 12 token poll bytes)");

        // ---- 6. a data error token -------------------------------------
        errors_before = errors;
        mode          = 3'd4;
        data_seen     = 0;
        do_read(32'd0, 1'b0);
        if (error_code !== 4'd4) fail("a data error token did not report error 4");
        if (err_token !== 8'h0D) fail("the data error token was not kept");
        if (data_seen != 0) fail("data arrived after a data error token");
        if (errors == errors_before)
            $display("  error token  : PASS (error 4, token 0x0D kept)");

        // ---- 7. reset in the middle of a read --------------------------
        errors_before = errors;
        mode          = 3'd3;               // never offers a token, so it hangs
        @(negedge clk);
        block_addr     = 32'd0;
        byte_addressed = 1'b0;
        start          = 1'b1;
        @(negedge clk);
        start          = 1'b0;
        repeat (20) @(posedge clk);
        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (4) @(posedge clk);
        if (busy) fail("still busy after a reset in the middle of a read");
        if (sd_ncs !== 1'b1) fail("chip select not released by a mid read reset");
        if (errors == errors_before)
            $display("  mid read rst : PASS (busy clears, select released)");

        if (errors == 0)
            $display("RESULT: PASS -- SD sector read verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
