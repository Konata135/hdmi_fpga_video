# ============================================================================
# Timing constraints
#
# Four clocks, all from the one video PLL:
#   clk                             50 MHz, the only I/O domain (SD card, buttons,
#                                   segment display, and the whole bring-up state)
#   u_clk_gen/u_pll_video.clkc[0]   25 MHz pixel clock (video timing, HDMI)
#   u_clk_gen/u_pll_video.clkc[1]  125 MHz (the TMDS serialiser)
#   u_clk_gen/u_pll_video.clkc[3]  62.5 MHz (SDRAM controller and frame paths)
#
# The memory domain was slowed from 125 MHz to 62.5 MHz: on the board the SDRAM
# interface read back nearly right -- a bit or two wrong, differently each time
# -- which is too little timing margin rather than broken logic.  Halving the
# clock doubles the margin (SDRAM data is valid for a whole device clock
# period) and costs nothing, because the picture needs about 60 MB/s of the
# 250 MB/s the port can still deliver.  clkc[2] is the same 62.5 MHz shifted 180
# degrees and clocks the SDRAM macro only (one load, no fabric logic), so the
# only paths on it are the macro's own pins, handled at the bottom of this file.
#
# clkc[0] and clkc[1] are deliberately left related to each other: the
# serialiser's /5 counter relies on their fixed phase, so those paths must keep
# being checked.
#
# The 50 MHz domain and the memory domain are asynchronous to each other by
# design, as is the pixel domain to the memory domain: every crossing goes
# through a two flop synchroniser, a gray coded FIFO pointer or a toggle (see
# doc/路线图.md).  Without these exceptions the timer relates the domains, gives a
# crossing a zero hold budget, and reports the synchronisers themselves as
# violations -- which is what it did before these lines were added (38
# endpoints, on the probe's status registers).
# ============================================================================

create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

derive_clocks

# 50 MHz domain against the two that are not related to it
set_false_path -from [get_clocks {clk}] -to [get_clocks {u_clk_gen/u_pll_video.clkc[3]}]
set_false_path -from [get_clocks {u_clk_gen/u_pll_video.clkc[3]}] -to [get_clocks {clk}]
set_false_path -from [get_clocks {clk}] -to [get_clocks {u_clk_gen/u_pll_video.clkc[0]}]
set_false_path -from [get_clocks {u_clk_gen/u_pll_video.clkc[0]}] -to [get_clocks {clk}]

# pixel domain against the memory domain
set_false_path -from [get_clocks {u_clk_gen/u_pll_video.clkc[0]}] -to [get_clocks {u_clk_gen/u_pll_video.clkc[3]}]
set_false_path -from [get_clocks {u_clk_gen/u_pll_video.clkc[3]}] -to [get_clocks {u_clk_gen/u_pll_video.clkc[0]}]

# ============================================================================
# The SDRAM interface: the macro's clock (clkc[2], 180 degrees) against the
# fabric's (clkc[3], 0 degrees).
#
# The vendor's own sdc carries exactly this clause for this interface -- see
# "SDRAM controller internal 0-degree / 180-degree clock paths" at the end of
# usr_source/constraints_source/timing.sdc in the reference design -- because the
# SDRAM sits inside the package: what happens between the macro's pins and the
# device is the SIP's business and is characterised by the vendor, not by the
# fabric timing model.  The vendor cuts both directions; this design cuts the
# *return* one only, and on purpose:
#
# Return (macro's DQ output -> our capture flops).  Left in, the timer reports
# the macro's DQ bit arriving 15.7 ns after the macro's clock edge against a
# capture edge 8 ns later: 32 endpoints at -7.71 ns, one per DQ bit, in a path
# that no routing can fix, which also drowns out anything else that fails.  That
# 15.7 ns cannot be the real device's tAC (5-7 ns for a part that answers three
# clocks after the command) -- it belongs to the tool's SIP model.  There is no
# logic in this path to optimise either: it is a pad input.  Where the data
# really lands is what the power-up probe measures and hands to the controller
# (see doc/路线图.md), not something static analysis can settle.
#
# Forward (our command registers -> the macro's pads) stays checked: that is a
# real half-period window (8 ns), it passes with 0.8 ns to spare today, and if a
# later change eats that margin we want the report to say so.
set_false_path -from [get_clocks {u_clk_gen/u_pll_video.clkc[2]}] -to [get_clocks {u_clk_gen/u_pll_video.clkc[3]}]

