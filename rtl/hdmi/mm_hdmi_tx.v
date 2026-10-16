//=============================================================================
// mm_hdmi_tx.v -- HDMI / DVI transmitter, three TMDS data pairs + clock pair
//
//   active pixels   : VIDEO DATA PERIOD, TMDS video characters
//   everything else : CONTROL PERIOD, control characters
//
//   * channel 0 carries {VSYNC, HSYNC}
//   * channels 1 and 2 carry CTL[3:0] = 0000
//
// There is deliberately no guard band and no preamble.
//
// The specification describes both, and that arrangement was implemented
// first.  On the bench this monitor showed a black screen with it, while the
// official example - same board, same cable, same monitor - displayed
// normally.  Sending plain control characters through the whole blanking
// interval is what the sink actually accepts, and it was found by making the
// line structure selectable from the board keys at run time.  The variants
// that were tried are recorded in doc/HDMI协议笔记.md.
//
// Channel mapping follows the DVI/HDMI assignment: pair 0 = blue,
// pair 1 = green, pair 2 = red.  The clock pair carries a permanent
// 5-ones-then-5-zeros character, i.e. a 25 MHz square wave for 640x480@60.
//
// The video timing only has to tell this module which pixels are active; the
// HDMI framing is derived from that single signal.
//=============================================================================
`timescale 1ns / 100fs

module mm_hdmi_tx (
    input  wire        pix_clk,
    input  wire        pix_5x_clk,
    input  wire        rst,

    input  wire        de,          // active pixel
    input  wire        hs,          // horizontal sync, active low
    input  wire        vs,          // vertical sync,   active low
    input  wire [23:0] rgb,         // {R, G, B}, one pixel per pix_clk

    output wire        tmds_clk_p,
    output wire        tmds_d0_p,    // blue
    output wire        tmds_d1_p,    // green
    output wire        tmds_d2_p     // red
);

    //------------------------------------------------------------------
    // control values during blanking
    //------------------------------------------------------------------
    wire [1:0] ctl_0 = {vs, hs};
    wire [1:0] ctl_x = 2'b00;

    //------------------------------------------------------------------
    // three TMDS encoders
    //------------------------------------------------------------------
    wire [9:0] char_b;
    wire [9:0] char_g;
    wire [9:0] char_r;

    mm_tmds_encoder u_enc_blue (
        .clk  (pix_clk),
        .rst  (rst),
        .de   (de),
        .din  (rgb[7:0]),
        .ctl  (ctl_0),
        .dout (char_b)
    );

    mm_tmds_encoder u_enc_green (
        .clk  (pix_clk),
        .rst  (rst),
        .de   (de),
        .din  (rgb[15:8]),
        .ctl  (ctl_x),
        .dout (char_g)
    );

    mm_tmds_encoder u_enc_red (
        .clk  (pix_clk),
        .rst  (rst),
        .de   (de),
        .din  (rgb[23:16]),
        .ctl  (ctl_x),
        .dout (char_r)
    );

    //------------------------------------------------------------------
    // clock pair: five ones then five zeros gives a 50 % duty 25 MHz clock
    //------------------------------------------------------------------
    wire [9:0] char_clk = 10'b1111100000;

    //------------------------------------------------------------------
    // four serialisers
    //------------------------------------------------------------------
    mm_hdmi_serializer u_ser_blue (
        .serial_clk (pix_5x_clk),
        .rst        (rst),
        .char_in    (char_b),
        .serial_out (tmds_d0_p)
    );

    mm_hdmi_serializer u_ser_green (
        .serial_clk (pix_5x_clk),
        .rst        (rst),
        .char_in    (char_g),
        .serial_out (tmds_d1_p)
    );

    mm_hdmi_serializer u_ser_red (
        .serial_clk (pix_5x_clk),
        .rst        (rst),
        .char_in    (char_r),
        .serial_out (tmds_d2_p)
    );

    mm_hdmi_serializer u_ser_clock (
        .serial_clk (pix_5x_clk),
        .rst        (rst),
        .char_in    (char_clk),
        .serial_out (tmds_clk_p)
    );

endmodule

