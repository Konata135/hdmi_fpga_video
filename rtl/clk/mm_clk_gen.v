//=============================================================================
// mm_clk_gen.v -- clock generation for the HX4S20C multimedia player
//
// Reference clock: 50 MHz on-board oscillator (pin R7).
//
// Clock plan for milestone M1:
//   PLL_VIDEO : C0 =  25.000 MHz -> pix_clk     (640x480@60 pixel clock)
//               C1 = 125.000 MHz -> pix_5x_clk  (TMDS serialiser clock)
//               C2 =  62.500 MHz -> mem_clk_sft (SDRAM device clock, 180 deg)
//               C4 =  62.500 MHz -> mem_clk_sft (data capture clock, 180 deg)
//               C3 =  62.500 MHz -> mem_clk     (SDRAM controller logic)
//                         VCO = 50/2 * 1 * 40 = 1000 MHz
//
// The SDRAM controller logic runs on mem_clk and the on-package SDRAM on
// mem_clk_sft, the same frequency shifted by 180 degrees -- the arrangement the
// vendor reference design uses (see its .sdc: "SDRAM controller internal
// 0-degree / 180-degree clock paths").
//
// Why 62.5 MHz and not 125: on the board the interface read back *nearly*
// right at 125 MHz -- a bit or two wrong per word, differently each time --
// which is the signature of too little timing margin, not of broken logic.
// SDRAM data is valid for a whole device clock period, so the margin from a
// sample point to the nearest transition is about half a period: 4 ns at
// 125 MHz, 8 ns at 62.5.  The picture only needs about 60 MB/s and the burst
// port delivers one word per cycle either way (62.5 M words/s here), so
// halving the clock buys margin for nothing.  The serialiser keeps its own
// 125 MHz (C1): the video path is not touched.
//
// PLL_SYS (100 MHz sd domain / 125 MHz memory domain) and PLL_AUD
// (12.295 MHz audio master clock) are added in milestones M2 and M7.
//
// The EG_PHY_PLL / EG_LOGIC_BUFG instances are direct primitive
// instantiations from the Anlogic EG4 libraries guide, not IP-Generator
// wrappers.  The feedback path follows the library reference design:
// clk0 is fed back externally through a BUFG.
//=============================================================================
`timescale 1ns / 100fs

module mm_clk_gen (
    input  wire       refclk,       // 50 MHz
    output wire       pix_clk,      // 25 MHz
    output wire       pix_5x_clk,   // 125 MHz
    output wire       mem_clk,      // 62.5 MHz (SDRAM controller logic)
    output wire       mem_clk_dev,  // 62.5 MHz, 0 deg, dedicated: the SDRAM's clock
    output wire       mem_clk_sft,  // 62.5 MHz, 180 deg: the data capture clock
    output wire       pll_lock      // high when the video PLL is locked
);

    wire       v_c0_buf;
    wire       v_c4_raw;
    wire [7:0] v_do_unused;

    // The SDRAM's clock is *its own* PLL output (C2), at the same phase as the
    // controller's (C3) but on a net with a single load, so there is almost no
    // skew between the macro's clock and its pads.  Both halves matter and each
    // was learned the hard way: sharing the fabric's clock gave 4.4 ns of skew
    // (eleven hold/setup violations inside the macro) while the shifted clock it
    // has on the board before meant, most likely, that it never latched a
    // command at all.
    //
    // The capture clock (C4, 180 degrees) goes through a global buffer because
    // it only drives fabric flops.
    EG_LOGIC_BUFG u_mem_sft_bufg (
        .i (v_c4_raw),
        .o (mem_clk_sft)
    );

    // clk0 is brought out through a BUFG and fed back into fbclk, which is
    // the "CLKC0_EXT" feedback mode used by the library reference design.
    EG_LOGIC_BUFG u_video_fb_bufg (
        .i (v_c0_buf),
        .o (pix_clk)
    );

    EG_PHY_PLL #(
        .DPHASE_SOURCE  ("DISABLE"),
        .DYNCFG         ("DISABLE"),
        .FIN            ("50.000000"),
        .FEEDBK_MODE    ("NORMAL"),
        .FEEDBK_PATH    ("CLKC0_EXT"),
        .STDBY_ENABLE   ("DISABLE"),
        .PLLRST_ENA     ("ENABLE"),
        .SYNC_ENABLE    ("DISABLE"),
        .GMC_GAIN       (2),
        .ICP_CURRENT    (9),
        .KVCO           (2),
        .LPF_CAPACITOR  (1),
        .LPF_RESISTOR   (8),
        .REFCLK_DIV     (2),
        .FBCLK_DIV      (1),
        .CLKC0_ENABLE   ("ENABLE"),
        .CLKC0_DIV      (40),
        .CLKC0_CPHASE   (39),
        .CLKC0_FPHASE   (0),
        .CLKC1_ENABLE   ("ENABLE"),
        .CLKC1_DIV      (8),
        .CLKC1_CPHASE   (7),
        .CLKC1_FPHASE   (0),
        .CLKC2_ENABLE   ("ENABLE"),
        .CLKC2_DIV      (16),       // 1000/16 = 62.5 MHz ...
        // 180 degrees, dedicated: this is the phase the tool's own timing model
        // for the on-package SDRAM expects.  Feeding it the 0 degree clock, or
        // a buffered copy of the fabric's, both made the timer report 0.5-0.7 ns
        // of hold violations *inside* the macro; with this phase the design was
        // clean (+0.102 ns, two days earlier).  The sdc the vendor ships says the
        // same thing: "SDRAM controller internal 0-degree / 180-degree paths".
        .CLKC2_CPHASE   (7),        // ... shifted 8/16 of a period = 180 deg
        .CLKC2_FPHASE   (0),
        .CLKC3_ENABLE   ("ENABLE"),
        .CLKC3_DIV      (16),       // 62.5 MHz for the controller logic
        .CLKC3_CPHASE   (15),
        .CLKC3_FPHASE   (0),
        .CLKC4_ENABLE   ("ENABLE"),
        .CLKC4_DIV      (16),       // 62.5 MHz, shifted 8/16 = 180 degrees:
        .CLKC4_CPHASE   (7),        //     the read data capture clock
        .CLKC4_FPHASE   (0)
    ) u_pll_video (
        .refclk   (refclk),
        .reset    (1'b0),
        .stdby    (1'b0),
        .extlock  (pll_lock),
        .load_reg (1'b0),
        .psclk    (1'b0),
        .psdown   (1'b0),
        .psstep   (1'b0),
        .psclksel (3'b000),
        .psdone   (),
        .dclk     (1'b0),
        .dcs      (1'b0),
        .dwe      (1'b0),
        .di       (8'h00),
        .daddr    (6'h00),
        .do       (v_do_unused),
        .fbclk    (pix_clk),
        .clkc     ({v_c4_raw, mem_clk, mem_clk_dev, pix_5x_clk, v_c0_buf})
    );

endmodule

