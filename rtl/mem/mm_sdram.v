//=============================================================================
// mm_sdram.v -- self written SDRAM controller for the on-package 2M x 32 device
//
// The SDRAM sits inside the EG4S20BG256 package: EG_PHY_SDRAM_2M_32 is a hard
// macro that reaches the die directly, so there are no board pins to assign
// and no flight time to budget for.  The vendor reference design drives it with
//
//   * cs_n tied low, cke tied high, the command on {ras_n, cas_n, we_n}
//   * device clock = 125 MHz shifted 180 degrees, logic clock = 0 degrees
//
// and that is what this controller does too.  Everything else here (init
// sequence, refresh, row tracking, read capture) is written from the JEDEC
// command set.
//
// Geometry, straight from the device: 4 banks x 2048 rows x 256 columns x 32
// bits = 2M words.  The application address is therefore
//
//   [20:19] bank   [18:8] row   [7:0] column
//
// Access is one 32 bit word at a time, and rows are kept open:
//
//   same row as the last access  -> just issue the read or write (about five
//                                   cycles per word, i.e. ~100 MB/s)
//   different row, bank open     -> precharge, tRP, activate, tRCD, access
//
// Row hits matter: displaying 640 x 480 needs 1920 bytes per 32 us line, i.e.
// about 60 MB/s, and activating a row per word would only reach ~45 MB/s.  A
// row of the frame store is 480 words, so the long sequential reads and writes
// of the picture paths are row hits almost all of the time.
//
// Timings are in *cycles* of the memory clock (62.5 MHz, 16 ns each) and are
// deliberately conservative.  The JEDEC minima quoted below are the ones the
// vendor's sdc is written against at 125 MHz, so every parameter here is worth
// at least twice its minimum: tRP 3 cycles = 48 ns (min 20), tRCD 3 = 48 (20),
// tRFC 9 = 144 (70), tRAS 6 = 96 (42), tWR 2 = 32 (16).  Refreshes come every
// 1953 cycles = 31.25 us, the cadence the vendor's global_def.v computes (3906
// cycles of its 125 MHz clock) and 4x the 64 ms / 2048 rows budget.
//
// Application side contract: a request is only accepted while `ready` is high,
// and `wr_done` / `rd_dv` pulse once when it completes.
//=============================================================================
`timescale 1ns / 100fs

module mm_sdram #(
    parameter integer CAS_LAT        = 3,       // read latency (mode register bits 6:4)
    parameter integer T_RP           = 3,       // precharge -> next command
    parameter integer T_RCD          = 3,       // active -> read/write
    parameter integer T_RFC          = 9,       // auto refresh cycle
    parameter integer T_MRD          = 2,       // mode register -> next command
    parameter integer T_RAS          = 6,       // active -> precharge
    parameter integer T_WR           = 2,       // write -> precharge
    parameter integer INIT_WAIT      = 12_500,  // 200 us at 62.5 MHz
    parameter integer REFRESH_PERIOD = 1953     // 31.25 us at 62.5 MHz
)(
    input  wire        clk,        // 62.5 MHz, controller logic (0 degrees)
    // The device's clock: the *shifted* (180 degree) channel of the same PLL,
    // brought out on its own PLL output (C2) so the macro's clock net has a
    // single load and therefore almost no skew against the macro's own pads.
    // This is the arrangement the vendor's sdc describes for this interface --
    // "SDRAM controller internal 0-degree / 180-degree clock paths", with the
    // pair declared a false path there -- and it is the one TD's timing model
    // for the macro wants: with the macro on the 0-degree clock the timer
    // reports half a nanosecond of hold violations *inside* the macro, and on
    // this channel the design is clean.
    //
    // Feeding the macro the 0-degree clock was tried on the board as well (and
    // the shifted one through a BUFG), on the theory that latching commands on
    // the edge that changes them would explain a device that never answers.
    // Both read back the same all-ones bus, so that theory is dead; they are
    // recorded in doc/路线图.md rather than kept here.
    input  wire        clk_dev,
    // The shifted channel again (C4, through a BUFG because it drives fabric
    // flops only): a second sample of the data bus, for the case where the
    // device answers half a cycle away from where the counting below puts it.
    // Nothing selects it at the moment -- see dq_take.
    input  wire        clk_sft,    // 62.5 MHz shifted 180 degrees, capture only
    input  wire        rst,

    output wire        ready,      // a new request may be presented
    // Where in the CAS latency window the data is taken.  The controller counts
    // the latency from the command it issued, but the device counts from the
    // edge *it* latched that command on, and those two edges are half a cycle
    // apart (the device runs on the shifted clock) -- so which cycle the data
    // actually lands in is a question only the board can answer.  The power-up
    // probe tries each code and hands back the one that works, for both the
    // single word path and the burst path.
    //
    // The codes are deliberately not 0,1,2,3 = 0,1,2,3 cycles later: an offset
    // that only ever moves one way cannot find a device that answers *sooner*
    // than the count assumes, and "never in the window" looks exactly the same
    // as "too late" from the outside.  0 = as built, 1 = one later, 2 = one
    // earlier, 3 = two later.
    // rd_adj[1:0] / br_adj[1:0] pick the whole cycle offset (see lat_with_off
    // below); bit 2 picks which of the two bus samples to take.
    input  wire [2:0]  rd_adj,
    input  wire [2:0]  br_adj,
    input  wire        wr_req,
    input  wire [20:0] wr_addr,
    input  wire [31:0] wr_data,
    input  wire [3:0]  wr_dm,      // byte masks, 1 = that byte is NOT written
    output reg         wr_done,

    input  wire        rd_req,
    input  wire [20:0] rd_addr,
    output reg         rd_dv,
    output reg  [31:0] rd_data,

    // burst read: a run of words inside one row, one per cycle at the output.
    // The picture path uses this because a word-at-a-time round trip costs
    // about seven cycles (≈15 M words/s), while 640x480 needs 18.4 M pixels/s
    // and three bytes per pixel.
    input  wire        br_req,
    input  wire [20:0] br_addr,     // first word; the whole run must be in one row
    input  wire [8:0]  br_len,      // words to read, 1..256
    output reg         br_dv,       // one pulse per word
    output reg  [31:0] br_data,
    output wire        br_busy,

    output reg         init_done,
    output reg  [4:0]  state_dbg,

    // "the bus read back what we drove": while the controller is driving DQ for
    // a write, the input side should see that same value.  It is the cheapest
    // way to split "our capture path is blind" from "the device never answers",
    // which look identical from read data alone -- a silent device and a broken
    // input both read as all ones.
    output reg         echo_ok
);

    // command encoding on {ras_n, cas_n, we_n}, with cs_n tied low
    localparam [2:0] CMD_NOP       = 3'b111,
                     CMD_ACTIVE    = 3'b011,
                     CMD_READ      = 3'b101,
                     CMD_WRITE     = 3'b100,
                     CMD_PRECHARGE = 3'b010,
                     CMD_REFRESH   = 3'b001,
                     CMD_MRS       = 3'b000;

    // Seventeen states, so four bits would not do: see the note in
    // mm_sd_probe.v about a state register that is one bit too narrow.
    localparam [4:0] S_INIT      = 5'd0,
                     S_INIT_PRE  = 5'd1,
                     S_INIT_REF1 = 5'd2,
                     S_INIT_REF2 = 5'd3,
                     S_INIT_MRS  = 5'd4,
                     S_IDLE      = 5'd5,
                     // one cycle between "a request arrived" and "which way does
                     // it go", so the row-hit comparison is not in series with
                     // the state register's own input (see the note at S_DEC)
                     S_DEC       = 5'd18,
                     S_HIT       = 5'd6,   // row already open: just the access
                     S_WAIT_OK   = 5'd7,   // wait for tRAS/tWR, or tRP
                     S_PRE_GO    = 5'd8,   // issue the precharge
                     S_PRE       = 5'd9,   // precharge issued, wait tRP
                     S_ACT_GO    = 5'd10,  // issue the activate
                     S_ACT       = 5'd11,  // activate issued, wait tRCD
                     S_RW_GO     = 5'd12,  // issue the access after an activate
                     S_RD_LAT    = 5'd13,  // wait CAS latency, capture
                     S_WR_REC    = 5'd14,  // write recovery
                     S_DONE      = 5'd15,  // pulse wr_done / rd_dv
                     S_REF       = 5'd16,  // refresh issued, wait tRFC
                     S_BR_RUN    = 5'd17;  // burst read: issue and capture, one per cycle

    // Mode register value: A2..A0 = burst length 1 (000), A3 = sequential (0),
    // A6..A4 = CAS latency, everything above A6 zero.
    localparam [2:0]  CL_BITS  = CAS_LAT;
    localparam [10:0] MODE_REG = {4'b0000, CL_BITS, 1'b0, 3'b000};
    localparam [3:0]  CAS4     = CAS_LAT;    // width explicit, for the adder below

    // The sample offset code, applied to a base latency count.
    function [3:0] lat_with_off;
        input [3:0] base;
        input [1:0] code;
        begin
            case (code)
                2'd0:    lat_with_off = base;
                2'd1:    lat_with_off = base + 4'd1;
                2'd2:    lat_with_off = base - 4'd1;
                default: lat_with_off = base + 4'd2;
            endcase
        end
    endfunction

    reg [4:0]  state;
    reg [15:0] wait_cnt;
    reg [15:0] refresh_cnt;
    reg        refresh_due;
    reg        ref_pending;     // the pending precharge is for a refresh
    reg        do_precharge;    // ... or for a row change
    reg        dec_hit;         // the row is already open
    reg        dec_pre;         // ... it is open on a different row
    reg        dec_tRP;         // ... it is closed, but tRP has not passed yet

    // guards
    reg [7:0]  ras_cnt;         // since the last activate
    reg [7:0]  wr_cnt;          // since the last write
    reg [7:0]  pre_cnt;         // since the last precharge
    reg [3:0]  lat_cnt;         // cycles left before a read word is taken

    // The device timing guards, registered.
    //
    // Comparing a counter against a small constant is cheap on its own, but the
    // comparisons sat at the head of a six level cone that ended at the enable
    // pins of the SDRAM command registers, together with the "which row is
    // open" decision and the request: 7.8 ns of an 8 ns period, five of those
    // nanoseconds routing.  Behind a flop they are off that path, and the only
    // difference is that a guard is honoured one cycle later -- which for a
    // device *minimum* (tRP/tRCD/tRAS/tWR/tRFC/tMRD) is always the safe
    // direction.  The read capture counters are unaffected: the single word one
    // is loaded where the command is issued, and the burst one is held until
    // the first READ goes out, so both stay tied to the command, not the guard.
    reg rdy_init, rdy_rp, rdy_rfc, rdy_mrd, rdy_rcd, rdy_wr;
    reg rdy_ras, rdy_wr_c, rdy_rp_c;

    always @(posedge clk) begin
        if (rst) begin
            rdy_init <= 1'b0;
            rdy_rp   <= 1'b0;
            rdy_rfc  <= 1'b0;
            rdy_mrd  <= 1'b0;
            rdy_rcd  <= 1'b0;
            rdy_wr   <= 1'b0;
            rdy_ras  <= 1'b0;
            rdy_wr_c <= 1'b0;
            rdy_rp_c <= 1'b0;
        end else begin
            rdy_init <= (wait_cnt >= INIT_WAIT);
            rdy_rp   <= (wait_cnt >= T_RP);
            rdy_rfc  <= (wait_cnt >= T_RFC);
            rdy_mrd  <= (wait_cnt >= T_MRD);
            rdy_rcd  <= (wait_cnt >= T_RCD);
            rdy_wr   <= (wait_cnt >= T_WR);
            rdy_ras  <= (ras_cnt  >= T_RAS);
            rdy_wr_c <= (wr_cnt   >= T_WR);
            rdy_rp_c <= (pre_cnt  >= T_RP);
        end
    end

    // burst read bookkeeping
    reg        burst_pending;   // this access is a burst, not a single word
    reg [8:0]  br_left;         // words still to issue
    reg [8:0]  br_taken;        // words still to capture
    reg [8:0]  br_wait;         // cycles to the first capture (CAS latency)
    reg [7:0]  br_col;          // column of the next word to issue

    // latched request
    reg        req_is_wr;
    reg [1:0]  req_ba;
    reg [10:0] req_row;
    reg [7:0]  req_col;
    reg [31:0] req_data;
    reg [3:0]  req_dm;

    // which row each bank has open, if any
    reg        bank_open [0:3];
    reg [10:0] open_row  [0:3];

    // device pins
    reg        sd_ras, sd_cas, sd_we;
    reg [10:0] sd_addr;
    reg [1:0]  sd_ba;
    reg [3:0]  sd_dm;
    reg [31:0] sd_dq_out;
    reg        sd_dq_oe;

    wire [31:0] sd_dq_in;
    wire [31:0] sd_dq;

    assign ready = (state == S_IDLE) && !refresh_due;

    assign sd_dq_in = sd_dq;
    assign sd_dq    = sd_dq_oe ? sd_dq_out : 32'hzzzz_zzzz;

    // A second sample of the same bus, taken on the shifted clock: the device
    // launches its read data on an edge, and which edge that is relative to the
    // command is the one thing about this interface that cannot be settled by
    // reasoning -- so both samples are offered and the power-up probe picks.
    // (This register is the only fabric load on that clock; the macro is not on
    // it any more.)
    reg [31:0] sd_dq_sft;

    always @(posedge clk_sft)
        sd_dq_sft <= sd_dq;

    // Only the plain sample is used: taking the data on the shifted clock as
    // well was tried (both as the sole path and as a selectable option) and the
    // tool's SDRAM model wants the device on the shifted clock, not the capture.
    // rd_adj[2] / br_adj[2] are therefore ignored, so codes 4..7 behave exactly
    // like 0..3 -- the probe simply tries each of them and finds the same
    // answer twice.
    wire [31:0] dq_take = sd_dq_in;
    //
    // The echo: while we are driving the bus it is *us* driving it, so the
    // input side must see our own value.  If it does not, no amount of read
    // timing work will help -- the capture path itself is not seeing the bus.
    always @(posedge clk) begin
        if (rst)
            echo_ok <= 1'b0;
        else if (sd_dq_oe && (sd_dq_in == sd_dq_out))
            echo_ok <= 1'b1;
    end

    // the device macro: command encoded by ras/cas/we, always selected
    EG_PHY_SDRAM_2M_32 u_sdram (
        .clk   (clk_dev),
        .ras_n (sd_ras),
        .cas_n (sd_cas),
        .we_n  (sd_we),
        .addr  (sd_addr),
        .ba    (sd_ba),
        .dq    (sd_dq),
        .cs_n  (1'b0),
        .dm0   (sd_dm[0]),
        .dm1   (sd_dm[1]),
        .dm2   (sd_dm[2]),
        .dm3   (sd_dm[3]),
        .cke   (1'b1)
    );

    //------------------------------------------------------------------
    // little helpers used from more than one state
    //------------------------------------------------------------------
    task issue_active;
        begin
            sd_ras  <= 1'b0;
            sd_cas  <= 1'b1;
            sd_we   <= 1'b1;
            sd_addr <= req_row;
            sd_ba   <= req_ba;
        end
    endtask

    task issue_precharge_all;
        begin
            sd_ras  <= 1'b0;
            sd_cas  <= 1'b1;
            sd_we   <= 1'b0;        // PRECHARGE
            sd_addr <= 11'h400;     // A10 = 1: all banks
            sd_ba   <= 2'd0;
            pre_cnt <= 8'd0;
            bank_open[0] <= 1'b0;
            bank_open[1] <= 1'b0;
            bank_open[2] <= 1'b0;
            bank_open[3] <= 1'b0;
        end
    endtask

    // the access itself: A10 = 0, so the row stays open for the next word
    task issue_access;
        begin
            sd_ras  <= 1'b1;
            sd_cas  <= 1'b0;
            sd_we   <= req_is_wr ? 1'b0 : 1'b1;
            sd_addr <= {3'b000, req_col};
            sd_ba   <= req_ba;
            if (req_is_wr) begin
                sd_dq_out <= req_data;
                sd_dm     <= req_dm;
                sd_dq_oe  <= 1'b1;
                wr_cnt    <= 8'd0;
            end else begin
                // Count the CAS latency from the command itself rather than
                // from whatever cycle the FSM happens to reach its capture
                // state: the command is taken by the device on the middle of
                // this cycle, so the word is on the bus from the middle of
                // cycle k+CAS, and the rising edge that ends that cycle is
                // centred in it.
                lat_cnt <= lat_with_off(CAS4, rd_adj[1:0]);
            end
        end
    endtask

    // row hit / miss decision, made from the inputs in the cycle the request
    // arrives (the request itself is latched and acted on one cycle later)
    wire [1:0]  want_ba  = br_req ? br_addr[20:19] :
                           (wr_req ? wr_addr[20:19] : rd_addr[20:19]);
    wire [10:0] want_row = br_req ? br_addr[18:8]  :
                           (wr_req ? wr_addr[18:8]  : rd_addr[18:8]);
    wire        want_hit = bank_open[want_ba] && (open_row[want_ba] == want_row);

    // needs a precharge before it can start?
    wire        need_pre = bank_open[want_ba] && !want_hit;

    assign br_busy = burst_pending;

    // Taken in the cycle the request arrives and acted on in S_DEC, so the row
    // comparison is out of the path into the state register.  Declared after
    // want_hit / need_pre on purpose: a task that reads a wire declared later
    // would see an implicit one instead (this project has been bitten by that).
    task latch_decision;
        begin
            dec_hit <= want_hit;
            dec_pre <= need_pre;
            dec_tRP <= (pre_cnt < T_RP);
            state   <= S_DEC;
        end
    endtask

    //------------------------------------------------------------------
    // sequence
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state        <= S_INIT;
            wait_cnt     <= 16'd0;
            refresh_cnt  <= 16'd0;
            refresh_due  <= 1'b0;
            ref_pending  <= 1'b0;
            do_precharge <= 1'b0;
            ras_cnt      <= 8'hFF;
            wr_cnt       <= 8'hFF;
            pre_cnt      <= 8'hFF;
            dec_hit      <= 1'b0;
            dec_pre      <= 1'b0;
            dec_tRP      <= 1'b0;
            lat_cnt      <= 4'd0;
            burst_pending <= 1'b0;
            br_left      <= 9'd0;
            br_taken     <= 9'd0;
            br_wait      <= 9'd0;
            br_col       <= 8'd0;
            br_dv        <= 1'b0;
            br_data      <= 32'd0;
            sd_ras       <= 1'b1;
            sd_cas       <= 1'b1;
            sd_we        <= 1'b1;
            sd_addr      <= 11'd0;
            sd_ba        <= 2'd0;
            sd_dm        <= 4'h0;
            sd_dq_out    <= 32'd0;
            sd_dq_oe     <= 1'b0;
            wr_done      <= 1'b0;
            rd_dv        <= 1'b0;
            rd_data      <= 32'd0;
            init_done    <= 1'b0;
            state_dbg    <= S_INIT;
            req_is_wr    <= 1'b0;
            req_ba       <= 2'd0;
            req_row      <= 11'd0;
            req_col      <= 8'd0;
            req_data     <= 32'd0;
            req_dm       <= 4'h0;
            bank_open[0] <= 1'b0;
            bank_open[1] <= 1'b0;
            bank_open[2] <= 1'b0;
            bank_open[3] <= 1'b0;
            open_row[0]  <= 11'd0;
            open_row[1]  <= 11'd0;
            open_row[2]  <= 11'd0;
            open_row[3]  <= 11'd0;
        end else begin
            // every cycle starts with "no command, data bus released"
            sd_ras    <= 1'b1;
            sd_cas    <= 1'b1;
            sd_we     <= 1'b1;
            sd_dq_oe  <= 1'b0;
            sd_dm     <= 4'h0;
            wr_done   <= 1'b0;
            rd_dv     <= 1'b0;
            br_dv     <= 1'b0;
            state_dbg <= state;

            // rolling guards, saturating
            if (ras_cnt != 8'hFF) ras_cnt <= ras_cnt + 8'd1;
            if (wr_cnt  != 8'hFF) wr_cnt  <= wr_cnt  + 8'd1;
            if (pre_cnt != 8'hFF) pre_cnt <= pre_cnt + 8'd1;

            // refresh bookkeeping while sitting idle
            if ((state == S_IDLE) && !refresh_due) begin
                refresh_cnt <= refresh_cnt + 16'd1;
                if (refresh_cnt >= REFRESH_PERIOD)
                    refresh_due <= 1'b1;
            end

            case (state)
                // ---- power up: stay quiet, then the JEDEC sequence ------
                S_INIT: begin
                    if (rdy_init) begin
                        wait_cnt <= 16'd0;
                        state    <= S_INIT_PRE;
                    end else begin
                        wait_cnt <= wait_cnt + 16'd1;
                    end
                end

                S_INIT_PRE: begin
                    if (wait_cnt == 16'd0)
                        issue_precharge_all;
                    if (rdy_rp) begin
                        wait_cnt <= 16'd0;
                        state    <= S_INIT_REF1;
                    end else begin
                        wait_cnt <= wait_cnt + 16'd1;
                    end
                end

                S_INIT_REF1, S_INIT_REF2: begin
                    if (wait_cnt == 16'd0) begin
                        sd_ras  <= 1'b0;
                        sd_cas  <= 1'b0;
                        sd_we   <= 1'b1;        // AUTO REFRESH
                        sd_addr <= 11'd0;
                        sd_ba   <= 2'd0;
                    end
                    if (rdy_rfc) begin
                        wait_cnt <= 16'd0;
                        state    <= (state == S_INIT_REF1) ? S_INIT_REF2 : S_INIT_MRS;
                    end else begin
                        wait_cnt <= wait_cnt + 16'd1;
                    end
                end

                S_INIT_MRS: begin
                    if (wait_cnt == 16'd0) begin
                        sd_ras  <= 1'b0;
                        sd_cas  <= 1'b0;
                        sd_we   <= 1'b0;        // LOAD MODE REGISTER
                        sd_addr <= MODE_REG;
                        sd_ba   <= 2'd0;
                    end
                    if (rdy_mrd) begin
                        wait_cnt    <= 16'd0;
                        refresh_cnt <= 16'd0;
                        init_done   <= 1'b1;
                        state       <= S_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 16'd1;
                    end
                end

                // ---- waiting for work -----------------------------------
                S_IDLE: begin
                    if (refresh_due) begin
                        do_precharge <= 1'b1;
                        ref_pending  <= 1'b1;
                        wait_cnt     <= 16'd0;
                        state        <= S_WAIT_OK;
                    end else if (br_req) begin
                        // burst read: latch the run, then take the usual
                        // precharge / activate route if the row is not open
                        req_is_wr     <= 1'b0;
                        req_ba        <= want_ba;
                        req_row       <= want_row;
                        burst_pending <= 1'b1;
                        br_left       <= br_len;
                        br_taken      <= br_len;
                        // the counter is loaded before the first READ goes out
                        // (one cycle earlier than the single-word path, which
                        // loads it on the command edge), so it starts at
                        // CAS_LAT + 1 to put the first take in the same place
                        br_wait       <= {5'b0, lat_with_off(CAS4 + 4'd1, br_adj[1:0])};
                        br_col        <= br_addr[7:0];
                        latch_decision;
                    end else if (wr_req || rd_req) begin
                        req_is_wr <= wr_req;
                        req_ba    <= want_ba;
                        req_row   <= want_row;
                        req_col   <= wr_req ? wr_addr[7:0] : rd_addr[7:0];
                        req_data  <= wr_data;
                        req_dm    <= wr_dm;
                        latch_decision;
                    end
                end

                // The row-hit comparison, the request and the guards used to sit
                // in series with this state register's input, and the request
                // address comes from the reader's own state machine a module
                // away: eight logic levels, 7.8 ns of an 8 ns period, more than
                // half of it routing.  Taking the comparison into a flop first
                // takes it off that path.  It costs one cycle per *new* access;
                // a burst is unaffected, because it issues every one of its
                // reads from S_BR_RUN once it gets there.
                S_DEC: begin
                    if (dec_hit) begin
                        state <= S_HIT;
                    end else if (dec_pre) begin
                        do_precharge <= 1'b1;
                        ref_pending  <= 1'b0;
                        wait_cnt     <= 16'd0;
                        state        <= S_WAIT_OK;
                    end else if (dec_tRP) begin
                        do_precharge <= 1'b0;
                        ref_pending  <= 1'b0;
                        wait_cnt     <= 16'd0;
                        state        <= S_WAIT_OK;
                    end else begin
                        state <= S_ACT_GO;
                    end
                end

                // row hit: the address is stable now, so issue the access
                S_HIT: begin
                    if (burst_pending) begin
                        state <= S_BR_RUN;
                    end else begin
                        issue_access;
                        state <= req_is_wr ? S_WR_REC : S_RD_LAT;
                    end
                end

                // tRAS must have passed before a precharge and tWR after a
                // write; when only tRP is being waited for, that is the test
                S_WAIT_OK: begin
                    if (do_precharge) begin
                        if (rdy_ras && rdy_wr_c)
                            state <= S_PRE_GO;
                    end else begin
                        if (rdy_rp_c)
                            state <= S_ACT_GO;
                    end
                end

                S_PRE_GO: begin
                    issue_precharge_all;
                    wait_cnt <= 16'd0;
                    state    <= S_PRE;
                end

                S_PRE: begin
                    if (rdy_rp) begin
                        wait_cnt <= 16'd0;
                        state    <= ref_pending ? S_REF : S_ACT_GO;
                    end else begin
                        wait_cnt <= wait_cnt + 16'd1;
                    end
                end

                S_ACT_GO: begin
                    issue_active;
                    ras_cnt           <= 8'd0;
                    bank_open[req_ba] <= 1'b1;
                    open_row[req_ba]  <= req_row;
                    wait_cnt          <= 16'd0;
                    state             <= S_ACT;
                end

                S_ACT: begin
                    if (rdy_rcd) begin
                        wait_cnt <= 16'd0;
                        state    <= S_RW_GO;
                    end else begin
                        wait_cnt <= wait_cnt + 16'd1;
                    end
                end

                S_RW_GO: begin
                    if (burst_pending) begin
                        state <= S_BR_RUN;
                    end else begin
                        issue_access;
                        state <= req_is_wr ? S_WR_REC : S_RD_LAT;
                    end
                end

                // burst read: one READ issued per cycle, one word captured per
                // cycle, so a run of n words takes about n + CAS_LAT cycles
                S_BR_RUN: begin
                    if (br_left != 9'd0) begin
                        sd_ras  <= 1'b1;
                        sd_cas  <= 1'b0;        // READ, no auto precharge
                        sd_we   <= 1'b1;
                        sd_addr <= {3'b000, br_col};
                        sd_ba   <= req_ba;
                        br_col  <= br_col + 8'd1;
                        br_left <= br_left - 9'd1;
                    end
                    if (br_wait != 9'd0) begin
                        br_wait <= br_wait - 9'd1;
                    end else if (br_taken != 9'd0) begin
                        br_data  <= dq_take;    // visible on the next cycle
                        br_dv    <= 1'b1;
                        br_taken <= br_taken - 9'd1;
                    end else begin
                        burst_pending <= 1'b0;
                        state         <= S_IDLE;
                    end
                end

                // CAS latency: the device drives the word half a clock after
                // this side's rising edge (it samples on the 180 degree clock),
                // so the edge that ends cycle k+CAS_LAT is centred in the data
                // window and is where it gets taken.
                S_RD_LAT: begin
                    if (lat_cnt == 4'd0) begin
                        rd_data  <= dq_take;
                        state    <= S_DONE;
                    end else begin
                        lat_cnt <= lat_cnt - 4'd1;
                    end
                end

                S_WR_REC: begin
                    if (rdy_wr) begin
                        wait_cnt <= 16'd0;
                        state    <= S_DONE;
                    end else begin
                        wait_cnt <= wait_cnt + 16'd1;
                    end
                end

                S_DONE: begin
                    if (req_is_wr)
                        wr_done <= 1'b1;
                    else
                        rd_dv <= 1'b1;
                    state <= S_IDLE;
                end

                S_REF: begin
                    if (wait_cnt == 16'd0) begin
                        sd_ras  <= 1'b0;
                        sd_cas  <= 1'b0;
                        sd_we   <= 1'b1;
                        sd_addr <= 11'd0;
                        sd_ba   <= 2'd0;
                    end
                    if (rdy_rfc) begin
                        wait_cnt    <= 16'd0;
                        refresh_cnt <= 16'd0;
                        refresh_due <= 1'b0;
                        ref_pending <= 1'b0;
                        state       <= S_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 16'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
