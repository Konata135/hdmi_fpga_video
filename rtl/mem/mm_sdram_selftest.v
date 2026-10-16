//=============================================================================
// mm_sdram_selftest.v -- ask the store where its data actually lands
//
// The controller counts the CAS latency from the command *it* issued, but the
// device counts from the edge *it* latched that command on, and those two edges
// are half a cycle apart (the device runs on the shifted clock).  Which cycle
// the data lands in is therefore a phase question, and no amount of reasoning
// settles it -- the board has to say.  So this probe does not guess: it reads
// the same stored word with each of the three whole cycle offsets the
// controller offers and reports
//
//   * the raw 32 bit value it got (not a pass or a fail: a pass rules out one
//     assumption per board test, a raw value rules out several)
//   * which offset actually read the word back whole
//   * for the burst port -- the one the picture reader uses -- how far the
//     words come out shifted, measured against a walking nibble pattern, and
//     which offset lines them up
//
// Both chosen offsets are then handed to the controller and held, so the
// picture path inherits the calibration.  A burst cannot be aligned by moving
// the sample point alone (in a burst that changes *which* word is taken), so
// what it needs is exactly this: the offset that makes the first delivered word
// be the first word asked for.
//
// Probe areas are in the last two device rows, outside any frame.
//
// Port protocol: a request is presented for one cycle in a cycle where `ready`
// is high, and must not be left asserted.
//=============================================================================
`timescale 1ns / 100fs

module mm_sdram_selftest #(
    parameter [20:0] PROBE_ADDR = 21'h1F_FFFF,   // last word of the store
    parameter [31:0] PROBE_DATA = 32'hA5A5_5A5A,
    parameter [20:0] BURST_BASE = 21'h1F_FE00,   // four words in one device row
    // ---- the picture check -------------------------------------------------
    // Off by default, so every existing testbench drives this probe exactly as
    // before and sees exactly the same behaviour.
    //
    // When it is on, the probe waits for the top level to say the picture is on
    // screen and then reads two words *out of the picture region* through this
    // same single word port -- a path with nothing in common with the burst
    // port or the reader's walk.  The two words are handed to the display as
    // the first pixel's R,G,B and can be held against the file by eye: a store
    // that holds the picture answers with the picture's own bytes, a store that
    // holds a flat pattern answers with the same value twice, and an address
    // that never decoded answers with the probe's own pattern.
    //
    // Why it is needed: page 3's 0xFF count says the loader did *not* hand over
    // blank data, but it cannot tell "the picture" from "any non-blank
    // pattern", and the store's contents in the picture region have never been
    // read back on the board at all -- the probe only ever proves the word at
    // PROBE_ADDR, which is written through a different port.
    parameter integer PIC_CHECK  = 0,
    parameter [20:0]  PIC_ADDR0  = 21'd0,        // picture 0 starts at word 0
    parameter [20:0]  PIC_ADDR1  = 21'd229_920,  // (ROWS-1)*480: its stored top row
    // Re-run the whole sweep this often (0 = never).  The startup sweep says
    // what the interface could do when the board was cold; if the picture
    // displays for a moment and then turns into one flat colour, the question
    // is whether the *reads* got worse, and the answer has to be readable
    // while it is happening -- so the probe writes its known word and reads it
    // back again and again, and pages 4..8 become a live meter instead of a
    // power-up snapshot.
    parameter integer RESWEEP_CYCLES = 0
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        ready,

    // where the controller should take read data, held once the sweep is over
    output reg  [2:0]  rd_adj,
    output reg  [2:0]  br_adj,

    // single word port
    output reg  [20:0] addr,
    output reg  [31:0] wdata,
    output reg  [3:0]  wdm,
    output reg         wr_req,
    input  wire        wr_done,
    output reg         rd_req,
    input  wire        rd_dv,
    input  wire [31:0] rdata,

    // burst read port
    output reg  [20:0] b_addr,
    output reg  [8:0]  b_len,
    output reg         b_req,
    input  wire        b_dv,
    input  wire [31:0] b_data,

    // status
    output reg         busy,
    output reg         done,
    output reg  [31:0] got_word,    // raw, at the offset that worked (or at 0)
    output reg  [3:0]  lanes_word,  // which of its bytes matched
    output reg  [2:0]  adj_word,    // the code that read it whole
    output reg         ok_word,
    output reg         ok_burst,
    output reg  [3:0]  shift_burst, // how far the words came out shifted, F = ?
    output reg  [2:0]  adj_burst,
    output reg  [7:0]  top_burst,

    // picture check
    input  wire        pic_go,      // level: the picture is on screen
    output reg         pic_busy,    // high between the trigger and the answer
    output reg  [23:0] pic_w0,      // first pixel (R,G,B) at PIC_ADDR0
    output reg  [23:0] pic_w1       // first pixel (R,G,B) at PIC_ADDR1
);

    localparam [3:0] S_WAIT = 4'd0,
                     S_WGO  = 4'd1,   // the one write
                     S_WRUN = 4'd2,
                     S_RGO  = 4'd3,   // a single word read at the current offset
                     S_RRUN = 4'd4,
                     S_BWG  = 4'd5,   // four words, one single word write each
                     S_BWR  = 4'd6,
                     S_BGO  = 4'd7,   // a burst read at the current offset
                     S_BCO  = 4'd8,
                     S_FIN  = 4'd9,
                     S_HOLD = 4'd10,
                     // picture check: address, wait, address, wait, hold
                     S_P0   = 4'd11,
                     S_P0W  = 4'd12,
                     S_P1   = 4'd13,
                     S_P1W  = 4'd14;

    reg [3:0] state;
    reg [2:0] try;              // which code is being tried
    reg [1:0] bi, bri;
    reg       word_found;
    reg       burst_found;
    reg [15:0] pic_wait;        // picture check: cycles since the last request
    reg [27:0] hold_cnt;        // how long the answer has stood

    wire [31:0] pdata = PROBE_DATA;

    wire [31:0] b_word   = 32'hA5A5_0000 | (32'd1 << {bi,  2'b00});

    wire [3:0] lanes_r = { (rdata[31:24] == pdata[31:24]),
                           (rdata[23:16] == pdata[23:16]),
                           (rdata[15:8]  == pdata[15:8]),
                           (rdata[7:0]   == pdata[7:0])  };

    // How far the burst came out shifted: the first word delivered should be the
    // first word asked for, so if it is the pattern written for word k, the run
    // is k words late.
    wire m0 = (b_data == 32'hA5A5_0001);
    wire m1 = (b_data == 32'hA5A5_0010);
    wire m2 = (b_data == 32'hA5A5_0100);
    wire m3 = (b_data == 32'hA5A5_1000);
    wire [3:0] shift_now = m0 ? 4'd0 : m1 ? 4'd1 : m2 ? 4'd2 : m3 ? 4'd3
                             : 4'hF;

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_WAIT;
            try <= 3'd0;
            bi          <= 2'd0;
            bri         <= 2'd0;
            word_found  <= 1'b0;
            burst_found <= 1'b0;
            rd_adj      <= 2'd0;
            br_adj      <= 2'd0;
            addr        <= PROBE_ADDR;
            wdata       <= PROBE_DATA;
            wdm         <= 4'h0;
            wr_req      <= 1'b0;
            rd_req      <= 1'b0;
            b_addr      <= BURST_BASE;
            b_len       <= 9'd4;
            b_req       <= 1'b0;
            busy        <= 1'b1;
            done        <= 1'b0;
            got_word    <= 32'd0;
            lanes_word  <= 4'd0;
            adj_word    <= 2'd0;
            ok_word     <= 1'b0;
            ok_burst    <= 1'b0;
            shift_burst <= 4'hF;
            adj_burst   <= 2'd0;
            top_burst   <= 8'd0;
            pic_busy    <= 1'b0;
            pic_w0      <= 24'd0;
            pic_w1      <= 24'd0;
            pic_wait    <= 16'd0;
            hold_cnt    <= 28'd0;
        end else begin
            done   <= 1'b0;
            wr_req <= 1'b0;
            rd_req <= 1'b0;
            b_req  <= 1'b0;

            case (state)
                S_WAIT: if (ready) state <= S_WGO;

                S_WGO: if (ready) begin
                           wr_req <= 1'b1;
                           state  <= S_WRUN;
                       end

                S_WRUN: if (wr_done) begin
                            try <= 3'd0;
                            state <= S_RGO;
                        end

                // ---- one stored word, read at each offset ------------------
                S_RGO: if (ready) begin
                           rd_adj <= try;
                           rd_req <= 1'b1;
                           state  <= S_RRUN;
                       end

                S_RRUN: if (rd_dv) begin
                            // Offset zero is the design's own default, so keep
                            // its value whatever happens: if nothing works that
                            // is the value worth looking at.
                            if ((try == 3'd0) || ((rdata == pdata) && !word_found)) begin
                                got_word   <= rdata;
                                lanes_word <= lanes_r;
                                adj_word   <= try;
                            end
                            if (rdata == pdata) word_found <= 1'b1;
                            // The burst probe shares the burst port with the
                            // picture reader (top level muxes the two), and a
                            // request the reader presents in a cycle this
                            // probe takes is simply *dropped* -- the reader
                            // then waits for words that never come and stays
                            // there for good.  The reader is only active once
                            // the picture is on screen, so after that this
                            // probe never touches the burst port again: the
                            // burst answer (page 7) keeps the value it measured
                            // at power-up, which is what it is for.
                            if (try == 3'd7) state <= pic_go ? S_FIN : S_BWG;
                            else begin
                                try <= try + 3'd1;
                                state <= S_RGO;
                            end
                        end

                // ---- the four words for the burst probe -------------------
                S_BWG: if (ready) begin
                           addr   <= BURST_BASE + {19'd0, bi};
                           wdata  <= b_word;
                           wr_req <= 1'b1;
                           state  <= S_BWR;
                       end

                S_BWR: if (wr_done) begin
                           if (bi == 2'd3) begin
                               try <= 3'd0;
                               state <= S_BGO;
                           end else begin
                               bi    <= bi + 2'd1;
                               state <= S_BWG;
                           end
                       end

                // ---- one burst, read at each offset ------------------------
                S_BGO: if (ready) begin
                           br_adj <= try;
                           b_addr <= BURST_BASE;
                           b_len  <= 9'd4;
                           b_req  <= 1'b1;
                           bri    <= 2'd0;
                           state  <= S_BCO;
                       end

                S_BCO: if (b_dv) begin
                           if (bri == 2'd0) begin
                               top_burst <= b_data[31:24];
                               // the first offset that lines the run up wins;
                               // otherwise remember the last measurement
                               if ((shift_now == 4'd0) && !burst_found) begin
                                   burst_found <= 1'b1;
                                   adj_burst   <= try;
                                   shift_burst <= 4'd0;
                               end else if (!burst_found) begin
                                   shift_burst <= shift_now;
                               end
                           end
                           if (bri == 2'd3) begin
                               if (try == 3'd7) state <= S_FIN;
                               else begin
                                   try <= try + 3'd1;
                                   state <= S_BGO;
                               end
                           end else begin
                               bri <= bri + 2'd1;
                           end
                       end

                S_FIN: begin
                    // hand the calibration to the controller and hold it
                    ok_word  <= word_found;
                    ok_burst <= burst_found;
                    rd_adj   <= adj_word;
                    br_adj   <= adj_burst;
                    done     <= 1'b1;
                    busy     <= 1'b0;
                    hold_cnt <= 28'd0;
                    state    <= S_HOLD;
                end

                // The answer stands until the top level says the picture has
                // landed (then it also reads the picture back), or until the
                // re-sweep timer expires and the whole sequence starts again --
                // that is what turns pages 4..8 into a meter that keeps saying
                // whether the device still answers, rather than a snapshot of
                // how it behaved when the board was cold.
                S_HOLD: begin
                    hold_cnt <= hold_cnt + 28'd1;
                    if ((RESWEEP_CYCLES != 0) && (hold_cnt == RESWEEP_CYCLES)) begin
                        hold_cnt    <= 28'd0;
                        try         <= 3'd0;
                        bi          <= 2'd0;
                        bri         <= 2'd0;
                        word_found  <= 1'b0;
                        // burst_found / adj_burst / shift_burst / top_burst are
                        // deliberately *not* cleared: the burst phase only runs
                        // before the picture is up, and page 7 should keep
                        // showing what it measured then
                        // the picture read first (it is read only), then the
                        // sweep that rewrites and re-reads the known word
                        if ((PIC_CHECK != 0) && pic_go) begin
                            pic_busy <= 1'b1;
                            addr     <= PIC_ADDR0;
                            state    <= S_P0;
                        end else begin
                            addr  <= PROBE_ADDR;
                            state <= S_WGO;
                        end
                    end
                end

                // ---- the picture, read back through the single word port ---
                //
                // Unlike the startup sweep, this runs while the picture reader
                // is fetching, and the controller prefers the burst port: a
                // request presented in a `ready` cycle that the reader also
                // claims is simply dropped.  So each of the two reads is timed
                // out and asked for again -- without that, one collision would
                // leave pic_busy high and the answer pages blank for good.
                S_P0: if (ready) begin
                          rd_req   <= 1'b1;
                          pic_wait <= 16'd0;
                          state    <= S_P0W;
                      end

                S_P0W: if (rd_dv) begin
                           pic_w0 <= rdata[23:0];
                           addr   <= PIC_ADDR1;
                           state  <= S_P1;
                       end else if (pic_wait == 16'd1023) begin
                           state  <= S_P0;
                       end else begin
                           pic_wait <= pic_wait + 16'd1;
                       end

                S_P1: if (ready) begin
                          rd_req   <= 1'b1;
                          pic_wait <= 16'd0;
                          state    <= S_P1W;
                      end

                S_P1W: if (rd_dv) begin
                           pic_w1   <= rdata[23:0];
                           pic_busy <= 1'b0;
                           // straight into the next sweep, which rewrites and
                           // re-reads the known word: that is what makes pages
                           // 4..8 a live meter
                           addr     <= PROBE_ADDR;
                           wdata    <= PROBE_DATA;
                           wdm      <= 4'h0;
                           state    <= S_WGO;
                       end else if (pic_wait == 16'd1023) begin
                           state  <= S_P1;
                       end else begin
                           pic_wait <= pic_wait + 16'd1;
                       end

                default: ;      // unreachable: every state above is listed
            endcase
        end
    end

endmodule
