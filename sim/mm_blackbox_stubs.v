//=============================================================================
// mm_blackbox_stubs.v -- models for the device primitives the design uses
//
// Used only by the simulation scripts in this directory.  Behaviour follows
// the EG4 libraries guide (UG301):
//
//   EG_LOGIC_ODDR : d0 is the value emitted on the rising edge, d1 on the
//                   falling edge; rst is active high.
//
// Nothing in this file is synthesised, and it is not part of the TD project.
//=============================================================================
`timescale 1ns / 100fs

module EG_LOGIC_BUFG (
    input  i,
    output o
);
    assign o = i;
endmodule

module EG_LOGIC_ODDR #(
    parameter ASYNCRST = "ENABLE"
)(
    input  wire clk,
    input  wire rst,
    input  wire d0,      // data emitted on the rising edge
    input  wire d1,      // data emitted on the falling edge
    output wire q
);
    // Both inputs are captured on the rising edge of clk, so the two halves
    // of the output period always come from the same source word.
    reg q_rise;
    reg q_fall;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            q_rise <= 1'b0;
            q_fall <= 1'b0;
        end else begin
            q_rise <= d0;
            q_fall <= d1;
        end
    end

    assign q = clk ? q_rise : q_fall;
endmodule

module EG_PHY_PLL #(
    parameter DPHASE_SOURCE = "DISABLE",
    parameter DYNCFG        = "DISABLE",
    parameter FIN           = "50.000000",
    parameter FEEDBK_MODE   = "NORMAL",
    parameter FEEDBK_PATH   = "CLKC0_EXT",
    parameter STDBY_ENABLE  = "DISABLE",
    parameter PLLRST_ENA    = "ENABLE",
    parameter SYNC_ENABLE   = "DISABLE",
    parameter GMC_GAIN      = 2,
    parameter ICP_CURRENT   = 9,
    parameter KVCO          = 2,
    parameter LPF_CAPACITOR = 1,
    parameter LPF_RESISTOR  = 8,
    parameter REFCLK_DIV    = 2,
    parameter FBCLK_DIV     = 1,
    parameter CLKC0_ENABLE  = "ENABLE",
    parameter CLKC0_DIV     = 40,
    parameter CLKC0_CPHASE  = 39,
    parameter CLKC0_FPHASE  = 0,
    parameter CLKC1_ENABLE  = "ENABLE",
    parameter CLKC1_DIV     = 8,
    parameter CLKC1_CPHASE  = 7,
    parameter CLKC1_FPHASE  = 0,
    parameter CLKC2_ENABLE  = "DISABLE",
    parameter CLKC2_DIV     = 1,
    parameter CLKC2_CPHASE  = 0,
    parameter CLKC2_FPHASE  = 0
)(
    input  wire       refclk,
    input  wire       reset,
    input  wire       stdby,
    output wire       extlock,
    input  wire       load_reg,
    input  wire       psclk,
    input  wire       psdown,
    input  wire       psstep,
    input  wire [2:0] psclksel,
    output wire       psdone,
    input  wire       dclk,
    input  wire       dcs,
    input  wire       dwe,
    input  wire [7:0] di,
    input  wire [5:0] daddr,
    output wire [7:0] do,
    input  wire       fbclk,
    output wire [4:0] clkc
);
endmodule

