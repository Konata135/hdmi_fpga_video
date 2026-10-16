//=============================================================================
// mm_seg_display.v -- six digit seven segment display driver
//
// Board wiring (HX4S20C):
//   * seg_sel[5:0]  digit select, active LOW, one line pulled low at a time
//   * seg_data[7:0] segments, common anode so active LOW;
//                   seg_data[7] is the decimal point,
//                   seg_data[6:0] are {g, f, e, d, c, b, a}
//
// value[23:20] is the leftmost digit and value[3:0] the rightmost one, so the
// caller composes the whole readout in one word.  This board wires seg_sel[0]
// to the *leftmost* digit, so the multiplexer below is mirrored: digit k
// drives select bit k with nibble 5-k.  That was measured on the board on
// 2026-10-13 (the first M3 reading came out as the mirror image of the
// intended word).
//
// The display is clocked by the free running 50 MHz oscillator and reset by
// the asynchronous power-on reset, so it reports something as soon as the
// board is powered.  It does not depend on the video or storage paths, which
// makes it the only indicator still working when those are broken.
//=============================================================================
`timescale 1ns / 100fs

module mm_seg_display #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer SCAN_HZ     = 200
)(
    input  wire        clk,
    input  wire        rst_n,       // asynchronous, active low
    input  wire [23:0] value,       // six hex digits, value[23:20] is digit 5
    output reg  [5:0]  seg_sel,
    output reg  [7:0]  seg_data
);

    localparam integer SCAN_DIV = CLK_FREQ_HZ / (SCAN_HZ * 6);

    //------------------------------------------------------------------
    // hex digit to segment pattern, active low
    //------------------------------------------------------------------
    function [6:0] seg7;
        input [3:0] v;
        begin
            case (v)
                4'h0:    seg7 = 7'b1000000;
                4'h1:    seg7 = 7'b1111001;
                4'h2:    seg7 = 7'b0100100;
                4'h3:    seg7 = 7'b0110000;
                4'h4:    seg7 = 7'b0011001;
                4'h5:    seg7 = 7'b0010010;
                4'h6:    seg7 = 7'b0000010;
                4'h7:    seg7 = 7'b1111000;
                4'h8:    seg7 = 7'b0000000;
                4'h9:    seg7 = 7'b0010000;
                4'hA:    seg7 = 7'b0001000;
                4'hB:    seg7 = 7'b0000011;
                4'hC:    seg7 = 7'b1000110;
                4'hD:    seg7 = 7'b0100001;
                4'hE:    seg7 = 7'b0000110;
                default: seg7 = 7'b0001110;
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // scan counter
    //------------------------------------------------------------------
    reg [15:0] scan_cnt;
    reg [2:0]  digit;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_cnt <= 16'd0;
            digit    <= 3'd0;
        end else if (scan_cnt == SCAN_DIV - 1) begin
            scan_cnt <= 16'd0;
            digit    <= (digit == 3'd5) ? 3'd0 : (digit + 3'd1);
        end else begin
            scan_cnt <= scan_cnt + 16'd1;
        end
    end

    //------------------------------------------------------------------
    // digit multiplexer
    //
    // Digit k drives select line k, and the nibble each line shows is counted
    // from the *other* end of the word (see the note at the top).  Nothing
    // else in the design depends on which physical digit a select line
    // reaches, so if a future board revision turns out to be wired the other
    // way round, swapping the six nibbles below is the whole fix.
    //------------------------------------------------------------------
    always @(*) begin
        case (digit)
            3'd0: begin seg_sel = 6'b111110; seg_data = {1'b1, seg7(value[23:20])}; end
            3'd1: begin seg_sel = 6'b111101; seg_data = {1'b1, seg7(value[19:16])}; end
            3'd2: begin seg_sel = 6'b111011; seg_data = {1'b1, seg7(value[15:12])}; end
            3'd3: begin seg_sel = 6'b110111; seg_data = {1'b1, seg7(value[11:8])};  end
            3'd4: begin seg_sel = 6'b101111; seg_data = {1'b1, seg7(value[7:4])};   end
            default: begin seg_sel = 6'b011111; seg_data = {1'b1, seg7(value[3:0])}; end
        endcase
    end

endmodule

