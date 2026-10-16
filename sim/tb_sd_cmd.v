//=============================================================================
// tb_sd_cmd.v -- self-checking testbench for mm_sd_cmd
//
// A behavioural card watches MOSI and drives MISO.  It records every command
// byte so the framing can be compared against the values the SD specification
// prescribes:
//
//   CMD0  -> 40 00 00 00 00 95
//   CMD8  -> 48 00 00 01 AA 87
//
// Those two CRCs are the only externally verifiable numbers in the command
// layer, so they are the strongest available check on the CRC7 generator.
//
// Also checked: R1 capture, the four extra bytes of R3/R7, the polling count
// when the card delays its answer, chip-select framing, and the timeout path
// when the card never answers at all.
//=============================================================================
`timescale 1ns / 100ps

module tb_sd_cmd;

    reg         clk;
    reg         rst;
    reg  [15:0] clk_div;
    reg         start;
    reg  [5:0]  cmd_index;
    reg  [31:0] cmd_arg;
    wire        busy;
    wire        done;
    wire [47:0] resp;
    wire [2:0]  resp_len;
    wire        resp_timeout;
    wire [15:0] poll_bytes;
    wire        sd_ncs;
    wire        sd_sclk;
    wire        sd_mosi;
    wire        sd_miso;

    integer errors;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial clk = 1'b0;
    always #5 clk = ~clk;           // 10 ns clock

    mm_sd_cmd #(
        .MAX_POLL_BYTES (40)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .clk_div      (clk_div),
        .start        (start),
        .cmd_index    (cmd_index),
        .cmd_arg      (cmd_arg),
        .busy         (busy),
        .done         (done),
        .resp         (resp),
        .resp_len     (resp_len),
        .resp_timeout (resp_timeout),
        .poll_bytes   (poll_bytes),
        .sd_ncs       (sd_ncs),
        .sd_sclk      (sd_sclk),
        .sd_mosi      (sd_mosi),
        .sd_miso      (sd_miso)
    );

    //------------------------------------------------------------------
    // behavioural card
    //------------------------------------------------------------------
    reg [7:0] seq [0:63];       // what the card puts on MISO, byte by byte
    reg [7:0] cmd_buf [0:5];    // command bytes the card received
    integer   bytes_seen;
    integer   rx_bit;
    reg [7:0] rx_shift;
    reg [2:0] tx_bit;
    reg [4:0] tx_byte_idx;

    assign sd_miso = seq[tx_byte_idx][7 - tx_bit];

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
            tx_byte_idx = tx_byte_idx + 1;
        end else begin
            tx_bit = tx_bit + 1;
        end
    end

    //------------------------------------------------------------------
    // helpers
    //------------------------------------------------------------------
    integer i;

    task clear_seq;
        begin
            for (i = 0; i < 64; i = i + 1)
                seq[i] = 8'hFF;
        end
    endtask

    task check_cmd;
        input [5:0]  idx;
        input [31:0] arg;
        input [7:0]  b5;
        begin
            if ((cmd_buf[0] !== {2'b01, idx}) || (cmd_buf[1] !== arg[31:24]) ||
                (cmd_buf[2] !== arg[23:16]) || (cmd_buf[3] !== arg[15:8]) ||
                (cmd_buf[4] !== arg[7:0])   || (cmd_buf[5] !== b5)) begin
                $display("  FAIL: cmd %0d framing = %02x %02x %02x %02x %02x %02x",
                         idx, cmd_buf[0], cmd_buf[1], cmd_buf[2],
                         cmd_buf[3], cmd_buf[4], cmd_buf[5]);
                $display("        expected    %02x %02x %02x %02x %02x %02x",
                         {2'b01, idx}, arg[31:24], arg[23:16], arg[15:8], arg[7:0], b5);
                errors = errors + 1;
            end
        end
    endtask

    task run_cmd;
        input [5:0]  idx;
        input [31:0] arg;
        begin
            @(negedge clk);
            cmd_index = idx;
            cmd_arg   = arg;
            start     = 1'b1;
            @(negedge clk);
            start     = 1'b0;
            @(posedge done);
            @(negedge clk);
        end
    endtask

    //------------------------------------------------------------------
    // scenarios
    //------------------------------------------------------------------
    initial begin
        errors    = 0;
        rst       = 1'b1;
        clk_div   = 16'd2;          // fast for simulation
        start     = 1'b0;
        cmd_index = 6'd0;
        cmd_arg   = 32'd0;
        clear_seq;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        // ---- 1. CMD0, card answers R1 = 0x01 (idle) after 2 poll bytes --
        clear_seq;
        seq[6] = 8'hFF;
        seq[7] = 8'hFF;
        seq[8] = 8'h01;
        run_cmd(6'd0, 32'd0);
        check_cmd(6'd0, 32'd0, 8'h95);      // specification value
        if (resp_len !== 3'd1) fail("CMD0: response length is not 1");
        if (resp[47:40] !== 8'h01) fail("CMD0: R1 not captured");
        if (resp_timeout) fail("CMD0: unexpected timeout");
        if (poll_bytes !== 16'd2) begin
            $display("  FAIL: CMD0 polled %0d bytes, expected 2", poll_bytes);
            errors = errors + 1;
        end
        if (sd_ncs !== 1'b1) fail("CMD0: chip select not released");

        // ---- 2. CMD8, R7 = 01 00 00 01 AA  (R1 plus four bytes) --------
        clear_seq;
        seq[6]  = 8'hFF;
        seq[7]  = 8'h01;
        seq[8]  = 8'h00;
        seq[9]  = 8'h00;
        seq[10] = 8'h01;
        seq[11] = 8'hAA;
        run_cmd(6'd8, 32'h0000_01AA);
        check_cmd(6'd8, 32'h0000_01AA, 8'h87);   // specification value
        if (resp_len !== 3'd5) fail("CMD8: response length is not 5");
        if (resp[47:40] !== 8'h01) fail("CMD8: R1 wrong");
        if (resp[39:32] !== 8'h00) fail("CMD8: byte 1 wrong");
        if (resp[31:24] !== 8'h00) fail("CMD8: byte 2 wrong");
        if (resp[23:16] !== 8'h01) fail("CMD8: voltage byte wrong");
        if (resp[15:8]  !== 8'hAA) fail("CMD8: echo byte wrong");

        // ---- 3. CMD58, R7 = 00 C0 FF 80 00  (R1 plus the OCR) ----------
        clear_seq;
        seq[6] = 8'h00;
        seq[7] = 8'hC0;
        seq[8] = 8'hFF;
        seq[9] = 8'h80;
        seq[10] = 8'h00;
        run_cmd(6'd58, 32'd0);
        if (resp_len !== 3'd5) fail("CMD58: response length is not 5");
        if (resp[47:40] !== 8'h00) fail("CMD58: R1 should report ready");
        if (resp[39:32] !== 8'hC0) fail("CMD58: OCR byte 0 wrong");
        if (poll_bytes !== 16'd0) begin
            $display("  FAIL: CMD58 polled %0d bytes, expected 0", poll_bytes);
            errors = errors + 1;
        end

        // ---- 4. card never answers -> timeout --------------------------
        clear_seq;                       // all 0xFF, no response
        run_cmd(6'd17, 32'd0);
        if (!resp_timeout) fail("missing response did not time out");
        if (resp_len !== 3'd0) fail("timeout should report no response bytes");
        if (sd_ncs !== 1'b1) fail("timeout: chip select not released");
        if (busy) fail("timeout: still busy after done");

        if (errors == 0)
            $display("RESULT: PASS -- SD command layer verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #500_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule

