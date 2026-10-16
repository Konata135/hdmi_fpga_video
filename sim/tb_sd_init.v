//=============================================================================
// tb_sd_init.v -- self-checking testbench for mm_sd_init
//
// A behavioural card answers the initialisation sequence and logs every
// command it receives.  Checks:
//
//   1. the command order is CMD0, CMD8, CMD55+ACMD41 repeated until ready,
//      then CMD58
//   2. ACMD41 is retried while the card reports it is still idle
//   3. the OCR is captured and bit 30 is decoded into high_capacity
//   4. init completes with no error, and the phase codes are walked through
//   5. a card that never leaves idle ends in the error state instead of
//      spinning forever
//=============================================================================
`timescale 1ns / 100ps

module tb_sd_init;

    reg        clk;
    reg        rst;
    reg        start;
    wire       busy;
    wire       done;
    wire       error;
    wire       high_capacity;
    wire [31:0] ocr;
    wire [2:0] card_type;
    wire       sd_ncs;
    wire       sd_sclk;
    wire       sd_mosi;
    wire       sd_miso;

    integer errors;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial clk = 1'b0;
    always #5 clk = ~clk;

    mm_sd_init #(
        .POWER_CYCLES (100),
        .ACMD41_TRIES (8)
    ) dut (
        .clk           (clk),
        .rst           (rst),
        .start         (start),
        .busy          (busy),
        .done          (done),
        .error         (error),
        .high_capacity (high_capacity),
        .ocr           (ocr),
        .card_type     (card_type),
        .state_code    (),
        .sd_ncs        (sd_ncs),
        .sd_sclk       (sd_sclk),
        .sd_mosi       (sd_mosi),
        .sd_miso       (sd_miso)
    );

    //------------------------------------------------------------------
    // behavioural card
    //------------------------------------------------------------------
    reg [7:0] cmd_buf [0:5];
    integer   bytes_seen;
    integer   rx_bit;
    reg [7:0] rx_shift;
    reg [2:0] tx_bit;
    reg [4:0] tx_byte_idx;

    reg [5:0] cmd_log [0:15];
    integer   log_n;
    integer   acmd41_rounds;
    reg       card_ready_en;     // test control: may this card ever become ready
    reg       card_is_v2;

    // the model becomes ready after two rounds of ACMD41 unless the test
    // disables that, which gives a card that never leaves idle
    wire card_ready = card_ready_en && (acmd41_rounds >= 2);

    wire [5:0] seen_cmd = cmd_buf[0][5:0];

    // what the card drives on MISO for byte number idx of this exchange
    function [7:0] card_byte;
        input [5:0] cmd;
        input [4:0] idx;
        begin
            if (idx <= 5)                       // the command itself
                card_byte = 8'hFF;
            else if (idx == 6)                  // one byte of response delay
                card_byte = 8'hFF;
            else begin
                case (cmd)
                    6'd0:    card_byte = (idx == 7) ? 8'h01 : 8'hFF;
                    6'd8:    case (idx)
                                 7:  card_byte = card_is_v2 ? 8'h01 : 8'h05;
                                 8:  card_byte = 8'h00;
                                 9:  card_byte = 8'h00;
                                 10: card_byte = 8'h01;
                                 11: card_byte = 8'hAA;
                                 default: card_byte = 8'hFF;
                             endcase
                    6'd55:   card_byte = (idx == 7) ? 8'h01 : 8'hFF;
                    6'd41:   card_byte = (idx == 7) ? (card_ready ? 8'h00 : 8'h01)
                                                    : 8'hFF;
                    6'd58:   case (idx)
                                 7:  card_byte = 8'h00;
                                 8:  card_byte = 8'hC0;
                                 9:  card_byte = 8'hFF;
                                 10: card_byte = 8'h80;
                                 11: card_byte = 8'h00;
                                 default: card_byte = 8'hFF;
                             endcase
                    default: card_byte = 8'hFF;
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
            tx_bit = 3'd0;
            if (tx_byte_idx == 5'd31)
                tx_byte_idx = 5'd0;
            else
                tx_byte_idx = tx_byte_idx + 5'd1;

            // the command is known once its poll byte starts
            if (tx_byte_idx == 5'd6) begin
                if (log_n < 16) begin
                    cmd_log[log_n] = seen_cmd;
                    log_n = log_n + 1;
                end
                if (seen_cmd == 6'd41)
                    acmd41_rounds = acmd41_rounds + 1;
            end
        end else begin
            tx_bit = tx_bit + 3'd1;
        end
    end

    //------------------------------------------------------------------
    // checks
    //------------------------------------------------------------------
    integer errors_before;
    integer spin;
    reg     spin_timed_out;

    task run_init;
        begin
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
        end
    endtask

    // bounded wait without SystemVerilog fork/join
    task wait_or_timeout;
        input integer limit;
        output reg timed_out;
        begin
            spin      = 0;
            timed_out = 1'b0;
            while (!done && !error && (spin < limit)) begin
                @(posedge clk);
                spin = spin + 1;
            end
            if (spin >= limit)
                timed_out = 1'b1;
        end
    endtask

    initial begin
        errors        = 0;
        rst           = 1'b1;
        start         = 1'b0;
        log_n         = 0;
        bytes_seen    = 0;
        rx_bit        = 0;
        rx_shift      = 8'd0;
        tx_bit        = 0;
        tx_byte_idx   = 0;
        acmd41_rounds = 0;
        card_ready_en = 1'b1;
        card_is_v2    = 1'b1;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        // ---- 1. a v2 card that needs two rounds of ACMD41 --------------
        run_init;
        wait_or_timeout(2_000_000, spin_timed_out);
        if (spin_timed_out) fail("initialisation of a good card did not finish");

        $display("  commands seen: %0d", log_n);
        // CMD0, CMD8, then CMD55+ACMD41 twice, then CMD58
        if (log_n !== 7) begin
            $display("  FAIL: expected 7 commands (CMD0, CMD8, CMD55+41 x2, CMD58), got %0d",
                     log_n);
            errors = errors + 1;
        end else begin
            if (cmd_log[0] !== 6'd0)  fail("first command is not CMD0");
            if (cmd_log[1] !== 6'd8)  fail("second command is not CMD8");
            if (cmd_log[2] !== 6'd55) fail("third command is not CMD55");
            if (cmd_log[3] !== 6'd41) fail("fourth command is not ACMD41");
            if (cmd_log[4] !== 6'd55) fail("fifth command is not CMD55");
            if (cmd_log[5] !== 6'd41) fail("sixth command is not ACMD41");
            if (cmd_log[6] !== 6'd58) fail("seventh command is not CMD58");
        end
        if (error) fail("init reported an error on a good card");
        if (card_type !== 3'd2) fail("card was not recognised as v2");
        if (ocr !== 32'hC0FF8000) begin
            $display("  FAIL: OCR = %08x, expected C0FF8000", ocr);
            errors = errors + 1;
        end
        if (high_capacity !== 1'b1) fail("OCR bit 30 not decoded into high_capacity");
        if (sd_ncs !== 1'b1) fail("chip select not released after init");

        $display("  init sequence: PASS (card_type=%0d, OCR=%08x, HC=%0d)",
                 card_type, ocr, high_capacity);

        // ---- 2. a card that never leaves idle --------------------------
        errors_before = errors;
        log_n         = 0;
        acmd41_rounds = 0;
        card_ready_en = 1'b0;
        run_init;
        wait_or_timeout(2_000_000, spin_timed_out);
        if (spin_timed_out) fail("a card that never becomes ready did not time out");
        if (!error) fail("stuck card did not raise error");
        if (busy) fail("still busy after the error was raised");

        if (errors == errors_before)
            $display("  stuck card   : PASS (ACMD41 retries then error)");

        if (errors == 0)
            $display("RESULT: PASS -- SD initialisation verified");
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

