//=============================================================================
// mm_test_pattern.v -- bring-up test pattern
//
// Contents of the 640x480 active area:
//   * eight 80-pixel vertical colour bars, 100 % saturation
//   * a 1-pixel mid-grey frame around the active area, so a dropped or an
//     extra column / row is visible at a glance
//   * a 64x64 square that advances +8 px horizontally and +4 px vertically
//     once per frame and wraps inside the active area.  Its fill toggles
//     between white and black every 32 frames so that it stays visible on
//     top of any bar colour, and its outline always uses the opposite
//     colour.
//
// The module is a pure combinational function of the current pixel position.
// Only the square position / colour phase are registered, updated by
// frame_first.
//=============================================================================
`timescale 1ns / 100fs

module mm_test_pattern #(
    parameter integer H_ACTIVE = 640,
    parameter integer V_ACTIVE = 480,
    parameter integer BOX_W    = 64,
    parameter integer BOX_H    = 64,
    parameter integer BOX_DX   = 8,
    parameter integer BOX_DY   = 4
)(
    input  wire        pix_clk,
    input  wire        rst,
    input  wire        frame_first,
    input  wire [11:0] h_cnt,
    input  wire [10:0] v_cnt,
    output wire [23:0] rgb
);

    //------------------------------------------------------------------
    // moving square: position and colour phase
    //------------------------------------------------------------------
    reg [9:0] box_x;
    reg [9:0] box_y;
    reg [5:0] box_phase;

    wire box_x_done = (box_x + BOX_W + BOX_DX) > H_ACTIVE;
    wire box_y_done = (box_y + BOX_H + BOX_DY) > V_ACTIVE;

    always @(posedge pix_clk) begin
        if (rst) begin
            box_x     <= 10'd0;
            box_y     <= 10'd0;
            box_phase <= 6'd0;
        end else if (frame_first) begin
            box_x     <= box_x_done ? 10'd0 : (box_x + BOX_DX[9:0]);
            box_y     <= box_y_done ? 10'd0 : (box_y + BOX_DY[9:0]);
            box_phase <= box_phase + 6'd1;
        end
    end

    wire box_fill = box_phase[5];

    wire in_box_x = (h_cnt >= {2'b00, box_x}) && (h_cnt < ({2'b00, box_x} + BOX_W));
    wire in_box_y = (v_cnt >= {1'b0,  box_y}) && (v_cnt < ({1'b0,  box_y}  + BOX_H));
    wire box_on   = in_box_x && in_box_y;

    wire box_edge = box_on && ((h_cnt < ({2'b00, box_x} + 2)) ||
                               (h_cnt >= ({2'b00, box_x} + BOX_W - 2)) ||
                               (v_cnt < ({1'b0,  box_y} + 2)) ||
                               (v_cnt >= ({1'b0,  box_y}  + BOX_H - 2)));

    wire [23:0] box_colour = box_edge ? (box_fill ? 24'h000000 : 24'hFFFFFF)
                                      : (box_fill ? 24'hFFFFFF : 24'h000000);

    //------------------------------------------------------------------
    // eight colour bars
    //------------------------------------------------------------------
    reg [2:0] bar_idx;

    always @(*) begin
        if      (h_cnt < 80)  bar_idx = 3'd0;
        else if (h_cnt < 160) bar_idx = 3'd1;
        else if (h_cnt < 240) bar_idx = 3'd2;
        else if (h_cnt < 320) bar_idx = 3'd3;
        else if (h_cnt < 400) bar_idx = 3'd4;
        else if (h_cnt < 480) bar_idx = 3'd5;
        else if (h_cnt < 560) bar_idx = 3'd6;
        else                  bar_idx = 3'd7;
    end

    reg [23:0] bar_colour;

    always @(*) begin
        case (bar_idx)
            3'd0:    bar_colour = 24'hFFFFFF;   // white
            3'd1:    bar_colour = 24'hFFFF00;   // yellow
            3'd2:    bar_colour = 24'h00FFFF;   // cyan
            3'd3:    bar_colour = 24'h00FF00;   // green
            3'd4:    bar_colour = 24'hFF00FF;   // magenta
            3'd5:    bar_colour = 24'hFF0000;   // red
            3'd6:    bar_colour = 24'h0000FF;   // blue
            default: bar_colour = 24'h000000;   // black
        endcase
    end

    //------------------------------------------------------------------
    // active-area frame
    //------------------------------------------------------------------
    wire border = (h_cnt == 12'd0) || (h_cnt == H_ACTIVE - 1) ||
                  (v_cnt == 11'd0) || (v_cnt == V_ACTIVE - 1);

    assign rgb = border  ? 24'h808080 :
                 box_on  ? box_colour : bar_colour;

endmodule

