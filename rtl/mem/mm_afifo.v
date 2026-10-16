//=============================================================================
// mm_afifo.v -- asynchronous FIFO with a used-word count
//
// Gray coded pointers synchronised into the other clock domain, the usual
// arrangement: the only signal crossing a domain boundary is a gray counter,
// so at most one bit can be in flight and a two stage synchroniser is enough.
// Both sides also get a `used` count, which the frame paths need in order to
// decide whether to fetch another word (write side) or to keep a byte buffer
// topped up (read side).
//
// Depth is 2**AW words.  The frame store uses three of these: bytes from the
// SPI side into the 125 MHz memory side when writing a frame, words out of the
// memory into the byte buffer when reading one, and one more inside the
// unpacker.
//=============================================================================
`timescale 1ns / 100fs

module mm_afifo #(
    parameter integer DW = 8,
    parameter integer AW = 10
)(
    input  wire           wclk,
    input  wire           wrst,
    input  wire           winc,
    input  wire [DW-1:0]  wdata,
    output wire           wfull,
    output wire [AW:0]    wused,

    input  wire           rclk,
    input  wire           rrst,
    input  wire           rinc,
    output wire [DW-1:0]  rdata,
    output wire           rempty,
    output wire [AW:0]    rused
);

    reg [DW-1:0] mem [0:(1<<AW)-1];

    reg [AW:0] wbin, wgray;
    reg [AW:0] rbin, rgray;

    // pointers arriving from the other side
    reg [AW:0] wq1_rgray, wq2_rgray;
    reg [AW:0] rq1_wgray, rq2_wgray;

    // The flags are registered, not combinatorial: "the next pointer" itself
    // depends on the flag, so feeding the flag back combinationally would make
    // a loop that never settles.  One cycle of latency on the flags is the
    // usual price and is invisible at these rates.
    reg wfull_r, rempty_r;

    wire [AW:0] wbin_next  = wbin + {{AW{1'b0}}, (winc & ~wfull_r)};
    wire [AW:0] wgray_next = (wbin_next >> 1) ^ wbin_next;
    wire [AW:0] rbin_next  = rbin + {{AW{1'b0}}, (rinc & ~rempty_r)};
    wire [AW:0] rgray_next = (rbin_next >> 1) ^ rbin_next;

    // full when the next write pointer meets the read pointer, top two bits
    // inverted; empty when the next read pointer catches the write pointer
    assign wfull  = wfull_r;
    assign rempty = rempty_r;

    assign rdata = mem[rbin[AW-1:0]];

    // gray to binary, so each side can report how many words are in flight
    function [AW:0] gray2bin;
        input [AW:0] g;
        integer i;
        begin
            gray2bin[AW] = g[AW];
            for (i = AW - 1; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    // The counts are registered rather than worked out combinationally from
    // the synchronised pointer.  A five level gray decode followed by a
    // subtractor is fine on its own, but a consumer that then does its own
    // arithmetic on the result ends up with the whole lot in one clock period:
    // the picture reader measured 11.6 ns on an 8 ns period that way.  Moving
    // the decode and the subtract behind a flop costs one cycle of staleness,
    // which the consumers can absorb (their decisions are "shall I fetch more
    // words", and the count is only used to be conservative).
    reg [AW:0] wused_r, rused_r;

    always @(posedge wclk) begin
        if (wrst) wused_r <= {(AW+1){1'b0}};
        else      wused_r <= wbin - gray2bin(wq2_rgray);
    end

    always @(posedge rclk) begin
        if (rrst) rused_r <= {(AW+1){1'b0}};
        else      rused_r <= gray2bin(rq2_wgray) - rbin;
    end

    assign wused = wused_r;
    assign rused = rused_r;

    // ---- write side ---------------------------------------------------
    always @(posedge wclk) begin
        if (wrst) begin
            wbin <= {(AW+1){1'b0}};
            wgray <= {(AW+1){1'b0}};
            wfull_r <= 1'b0;
        end else begin
            if (winc && !wfull_r)
                mem[wbin[AW-1:0]] <= wdata;
            wbin  <= wbin_next;
            wgray <= wgray_next;
            wfull_r <= (wgray_next == {~wq2_rgray[AW:AW-1], wq2_rgray[AW-2:0]});
        end
    end

    always @(posedge wclk) begin
        if (wrst) begin
            wq1_rgray <= {(AW+1){1'b0}};
            wq2_rgray <= {(AW+1){1'b0}};
        end else begin
            wq1_rgray <= rgray;
            wq2_rgray <= wq1_rgray;
        end
    end

    // ---- read side ----------------------------------------------------
    always @(posedge rclk) begin
        if (rrst) begin
            rbin <= {(AW+1){1'b0}};
            rgray <= {(AW+1){1'b0}};
            rempty_r <= 1'b1;
        end else begin
            rbin  <= rbin_next;
            rgray <= rgray_next;
            rempty_r <= (rgray_next == rq2_wgray);
        end
    end

    always @(posedge rclk) begin
        if (rrst) begin
            rq1_wgray <= {(AW+1){1'b0}};
            rq2_wgray <= {(AW+1){1'b0}};
        end else begin
            rq1_wgray <= wgray;
            rq2_wgray <= rq1_wgray;
        end
    end

endmodule
