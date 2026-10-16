//=============================================================================
// mm_key_debounce.v -- a button or a switch, cleaned up
//
// A mechanical contact does not settle: it opens and closes for a few
// milliseconds.  Feeding that to a counter that advances one step per edge
// turns one press into a burst of steps, so the pin is filtered here first:
// the raw input goes through two flops, and the level is only allowed to
// change once the filtered input has *held* its new value for STABLE_CYCLES.
// Ten milliseconds is comfortably longer than any bounce and far shorter than
// a human press.
//
//   level  the debounced level: 1 = pressed / switched on
//   press  one clock, when level has just become 1 (not on release: a button
//          that is held must not keep stepping, and a switch that is turned
//          on should step once)
//
// The polarity is a parameter because the board decides it, not this module:
// the pins in constraints/hx4s20c_pin.adc are buttons to ground with pull-ups
// (idle high, pressed low), so the instances pass INVERT = 1.
//=============================================================================
`timescale 1ns / 100fs

module mm_key_debounce #(
    parameter integer STABLE_CYCLES = 500_000,   // 10 ms at 50 MHz
    parameter integer INVERT        = 0          // 1: the pin reads low when on
)(
    input  wire clk,
    input  wire rst,
    input  wire raw,            // the pin, as it arrives
    output reg  level,          // debounced: 1 = pressed / switched on
    output reg  press           // one cycle: level has just become 1
);

    // A wide constant on the left of a comparison and a narrow one on the
    // right is how this project once ended up with a silently truncated
    // literal, so the bound is cut to the counter's width here.
    localparam [20:0] STABLE = STABLE_CYCLES;

    wire in_w = INVERT ? ~raw : raw;

    reg  s1, s2;                // two flops in: the pin is asynchronous
    reg  [20:0] cnt;            // how long the input has held its new value

    always @(posedge clk) begin
        if (rst) begin
            s1    <= 1'b0;
            s2    <= 1'b0;
            level <= 1'b0;
            press <= 1'b0;
            cnt   <= 21'd0;
        end else begin
            s1    <= in_w;
            s2    <= s1;
            press <= 1'b0;

            if (s2 == level) begin
                cnt <= 21'd0;               // settled: nothing to debounce
            end else if (cnt == STABLE) begin
                level <= s2;                // held long enough to believe it
                press <= s2;                // one cycle, and only on a 0 -> 1
                cnt   <= 21'd0;
            end else begin
                cnt <= cnt + 21'd1;
            end
        end
    end

endmodule
