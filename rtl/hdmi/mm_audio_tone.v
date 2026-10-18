//=============================================================================
// mm_audio_tone.v -- the audio sample source for the HDMI path
//
// Everything up to the point where the HDMI specification has to be followed
// lives here, and nothing that does:
//
//   * a 48 kHz sample enable made out of the pixel clock by a fractional
//     divider (48000/25000000, so one sample every 520.83 clocks -- a plain
//     Bresenham accumulator, no second PLL and no audio clock domain)
//   * a phase accumulator and a triangle wave: the phase increment is the note,
//     and the top of the accumulator is a signed sample, so the tone is a few
//     gates rather than a sine table
//   * the note follows the picture on screen (C4/E4/G4/C5 for slots 0..3), which
//     is the first step towards the audio/video link the extension asks for
//
// 24-bit samples, both channels the same, about -12 dBFS.  The sample rate is
// exact: 48 kHz from 25 MHz leaves no remainder over a second.
//=============================================================================
`timescale 1ns / 100fs

module mm_audio_tone #(
    parameter integer CLK_HZ    = 25_000_000,
    parameter integer SAMPLE_HZ = 48_000
)(
    input  wire        clk,             // pixel clock
    input  wire        rst,
    input  wire        enable,          // the picture is up: make a sound
    input  wire [1:0]  note_sel,        // which picture is on the screen
    output reg         sample_valid,    // one clock per audio sample
    output reg  [23:0] sample           // signed, both channels
);

    //------------------------------------------------------------------
    // 48 kHz out of the pixel clock
    //------------------------------------------------------------------
    reg  [24:0] rate_acc;

    always @(posedge clk) begin
        if (rst) begin
            rate_acc     <= 25'd0;
            sample_valid <= 1'b0;
        end else if (rate_acc + SAMPLE_HZ >= CLK_HZ) begin
            rate_acc     <= rate_acc + SAMPLE_HZ - CLK_HZ;
            sample_valid <= 1'b1;
        end else begin
            rate_acc     <= rate_acc + SAMPLE_HZ;
            sample_valid <= 1'b0;
        end
    end

    //------------------------------------------------------------------
    // the note: phase increment per sample, 2^32 = one cycle
    //   f = 48000 * inc / 2^32
    //------------------------------------------------------------------
    function [31:0] note_inc;
        input [1:0] sel;
        begin
            case (sel)
                2'd0:    note_inc = 32'd23409862;   // C4 261.63 Hz
                2'd1:    note_inc = 32'd29494578;   // E4 329.63 Hz
                2'd2:    note_inc = 32'd35075155;   // G4 392.00 Hz
                default: note_inc = 32'd46819716;   // C5 523.25 Hz
            endcase
        end
    endfunction

    wire [31:0] inc = note_inc(note_sel);

    reg  [31:0] phase;

    // a triangle from the phase: the top bit is the direction, the next 23 are
    // the position, shifted down to about a quarter of full scale
    wire        [22:0] pos      = phase[30:8];
    // signed, or the arithmetic shift on the descending half would turn into a
    // logical one and the output would never go below zero
    wire signed [23:0] ramp_pos = {1'b0, pos};
    wire signed [23:0] ramp_w   = phase[31] ? -ramp_pos : ramp_pos;
    wire        [23:0] wave     = ramp_w >>> 2;    // keep 6 dB of headroom

    always @(posedge clk) begin
        if (rst) begin
            phase  <= 32'd0;
            sample <= 24'd0;
        end else if (sample_valid) begin
            phase  <= enable ? (phase + inc) : 32'd0;
            sample <= enable ? wave : 24'd0;
        end
    end

endmodule
