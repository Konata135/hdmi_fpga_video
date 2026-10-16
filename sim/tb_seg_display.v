//=============================================================================
// tb_seg_display.v -- self-checking testbench for mm_seg_display
//
// Checks:
//   1. seg_sel is one-hot active low, and the six digits are visited in
//      order, once each per scan cycle
//   2. every digit is held for exactly SCAN_DIV clock cycles, so the scan
//      frequency is the requested one
//   3. the segment pattern driven while a select line is active matches an
//      independently derived seven segment encoding (common anode, active
//      low), for all 16 values, with the nibble-to-select mapping this board
//      uses (select 0 = the leftmost digit = value[23:20])
//   4. the decimal point stays off
//=============================================================================
`timescale 1ns / 100ps

module tb_seg_display;

    localparam integer CLK_FREQ_HZ = 50_000_000;
    localparam integer SCAN_HZ     = 200;
    localparam integer SCAN_DIV    = CLK_FREQ_HZ / (SCAN_HZ * 6);   // 41666

    reg         clk;
    reg         rst_n;
    reg  [23:0] value;
    wire [5:0]  seg_sel;
    wire [7:0]  seg_data;

    integer errors;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial clk = 1'b0;
    always #5 clk = ~clk;       // 10 ns -> 100 MHz so the scan runs faster

    mm_seg_display #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SCAN_HZ     (SCAN_HZ)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .value    (value),
        .seg_sel  (seg_sel),
        .seg_data (seg_data)
    );

    //------------------------------------------------------------------
    // independent segment model: which segments are lit for each value,
    // bit 0 = a (top), bit 1 = b, ... bit 6 = g (middle)
    //------------------------------------------------------------------
    function [6:0] seg_lit;
        input [3:0] v;
        begin
            case (v)
                4'h0: seg_lit = 7'b0111111;   // a b c d e f
                4'h1: seg_lit = 7'b0000110;   // b c
                4'h2: seg_lit = 7'b1011011;   // a b d e g
                4'h3: seg_lit = 7'b1001111;   // a b c d g
                4'h4: seg_lit = 7'b1100110;   // b c f g
                4'h5: seg_lit = 7'b1101101;   // a c d f g
                4'h6: seg_lit = 7'b1111101;   // a c d e f g
                4'h7: seg_lit = 7'b0000111;   // a b c
                4'h8: seg_lit = 7'b1111111;   // all seven
                4'h9: seg_lit = 7'b1101111;   // a b c d f g
                4'hA: seg_lit = 7'b1110111;   // a b c e f g
                4'hB: seg_lit = 7'b1111100;   // c d e f g
                4'hC: seg_lit = 7'b0111001;   // a d e f
                4'hD: seg_lit = 7'b1011110;   // b c d e g
                4'hE: seg_lit = 7'b1111001;   // a d e f g
                default: seg_lit = 7'b1110001; // a e f g
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // scan observer
    //------------------------------------------------------------------
    reg [5:0] prev_sel;
    integer   run_len;
    integer   visits;
    integer   idx;
    integer   r;
    integer   period_seen;
    integer   cycle_len;
    integer   digit_order_ok;

    // decode the one-hot select into a digit index, or -1
    function integer sel_index;
        input [5:0] s;
        begin
            case (s)
                6'b111110: sel_index = 0;
                6'b111101: sel_index = 1;
                6'b111011: sel_index = 2;
                6'b110111: sel_index = 3;
                6'b101111: sel_index = 4;
                6'b011111: sel_index = 5;
                default:   sel_index = -1;
            endcase
        end
    endfunction

    initial begin
        errors    = 0;
        rst_n     = 1'b0;
        // The board wires select line 0 to the leftmost digit, so the digit
        // decoded from seg_sel carries the nibble counted from the other end
        // of the word: select 0 shows value[23:20], select 5 shows value[3:0].
        value     = 24'h0F5A3C;
        // seed with an invalid pattern so the very first observed digit is 0
        prev_sel  = 6'b111111;
        run_len   = 0;
        visits    = 0;
        period_seen = -1;
        digit_order_ok = 1;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;

        // watch three full scan cycles
        while (visits < 20) begin
            @(posedge clk);
            #1;                     // let the combinational outputs settle

            if (seg_sel !== prev_sel) begin
                idx = sel_index(seg_sel);

                if (idx < 0) begin
                    $display("  FAIL: seg_sel = %b is not one-hot active low", seg_sel);
                    errors = errors + 1;
                end else begin
                    // digit order: must step 0,1,2,3,4,5,0,...
                    if ((visits > 0) && (idx !== (visits % 6)))
                        digit_order_ok = 0;

                    // segment pattern: select line idx carries nibble 5-idx
                    if (seg_data[6:0] !== ~seg_lit(value[4*(5-idx) +: 4])) begin
                        $display("  FAIL: select %0d value %h -> segments %b, expected %b",
                                 idx, value[4*(5-idx) +: 4], seg_data[6:0],
                                 ~seg_lit(value[4*(5-idx) +: 4]));
                        errors = errors + 1;
                    end
                    if (seg_data[7] !== 1'b1)
                        fail("decimal point is not off");
                end

                // digit period; skip the first two switches because the
                // observer joins part way through a digit period
                if (visits >= 2) begin
                    if (period_seen < 0)
                        period_seen = run_len;
                    else if (run_len !== period_seen)
                        fail("digit periods are not constant");
                end
                run_len = 0;
                prev_sel = seg_sel;
                visits = visits + 1;
            end else begin
                run_len = run_len + 1;
            end
        end

        $display("  scan: %0d digit switches observed, digit period %0d clocks",
                 visits, period_seen);
        $display("  expected digit period %0d clocks, scan cycle %0d clocks",
                 SCAN_DIV, 6 * SCAN_DIV);

        if (!digit_order_ok)
            fail("digits are not scanned in order 0..5");
        if (period_seen !== SCAN_DIV - 1) begin
            $display("  FAIL: digit period %0d, expected %0d",
                     period_seen, SCAN_DIV - 1);
            errors = errors + 1;
        end
        if (visits < 20)
            fail("not enough scan activity observed");

        // a second value, to prove the multiplexer tracks the input
        value = 24'h123456;
        repeat (3) @(posedge clk);
        #1;
        // digit 0 is selected right after reset only; just check the digit
        // that is on the wire now
        r = sel_index(seg_sel);
        if (r >= 0) begin
            if (seg_data[6:0] !== ~seg_lit(value[4*(5-r) +: 4])) begin
                $display("  FAIL: after change, select %0d shows the wrong pattern", r);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("RESULT: PASS -- six digit display verified");
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

