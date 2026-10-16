//=============================================================================
// mm_video_timing.v -- 640x480@60 production timing generator
//
// CEA-861 VIC 1 / VESA DMT 640x480@59.94.  Both sync pulses are negative
// polarity, which is what VIC 1 specifies.
//
//   horizontal: 640 active | 16 front porch | 96 sync | 48 back porch = 800
//   vertical  : 480 active | 10 front porch |  2 sync | 33 back porch = 525
//
// h_cnt / v_cnt are free-running registered counters.  de, frame_first and
// line_last are combinational decodes of those counters and are sampled by
// mm_axis_video_pack together with the pixel data, so stream and data stay
// aligned.
//=============================================================================
`timescale 1ns / 100fs

module mm_video_timing #(
    parameter integer H_ACTIVE = 640,
    parameter integer H_FP     = 16,
    parameter integer H_SYNC   = 96,
    parameter integer H_BP     = 48,
    parameter integer V_ACTIVE = 480,
    parameter integer V_FP     = 10,
    parameter integer V_SYNC   = 2,
    parameter integer V_BP     = 33
)(
    input  wire        pix_clk,
    input  wire        rst,
    output reg  [11:0] h_cnt,
    output reg  [10:0] v_cnt,
    output reg         hs,           // horizontal sync, active low
    output reg         vs,           // vertical sync,   active low
    output wire        de,           // data enable / active pixel
    output wire        frame_first,  // first active pixel of a frame
    output wire        frame_next,   // the cycle before it: last blanking pixel
    output wire        line_last     // last active pixel of a line
);

    localparam integer H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;   // 800
    localparam integer V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;   // 525
    localparam integer HS_BEG  = H_ACTIVE + H_FP;                   // 656
    localparam integer HS_END  = HS_BEG  + H_SYNC;                  // 752
    localparam integer VS_BEG  = V_ACTIVE + V_FP;                   // 490
    localparam integer VS_END  = VS_BEG  + V_SYNC;                  // 492

    always @(posedge pix_clk) begin
        if (rst) begin
            h_cnt <= 12'd0;
            v_cnt <= 11'd0;
        end else if (h_cnt == H_TOTAL - 1) begin
            h_cnt <= 12'd0;
            v_cnt <= (v_cnt == V_TOTAL - 1) ? 11'd0 : (v_cnt + 11'd1);
        end else begin
            h_cnt <= h_cnt + 12'd1;
        end
    end

    always @(posedge pix_clk) begin
        if (rst) begin
            hs <= 1'b1;
            vs <= 1'b1;
        end else begin
            hs <= ~((h_cnt >= HS_BEG) && (h_cnt < HS_END));
            vs <= ~((v_cnt >= VS_BEG) && (v_cnt < VS_END));
        end
    end

    assign de          = (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);
    assign frame_first = de && (h_cnt == 12'd0) && (v_cnt == 11'd0);
    assign line_last   = de && (h_cnt == H_ACTIVE - 1);

    // The last blanking pixel of the frame, i.e. the cycle in which the next
    // pixel clock is the frame's first active one.  A consumer that has to have
    // something *ready* on that first active pixel cannot use frame_first: a
    // decision it registers in that cycle only takes effect on the cycle after,
    // which leaves the frame's first pixel showing whatever was there before.
    // Both the picture mux in the top level and the frame reader's start of a
    // picture have to be settled before the boundary, so they use this.
    assign frame_next  = (h_cnt == H_TOTAL - 1) && (v_cnt == V_TOTAL - 1);

endmodule

