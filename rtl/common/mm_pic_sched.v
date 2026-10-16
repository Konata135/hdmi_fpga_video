//=============================================================================
// mm_pic_sched.v -- which picture is loaded where, and which one is on screen
//
// The scan hands over a table of up to eight pictures; the frame store has room
// for eight (doc/SDRAM内存映射.md, slot i at i * 230400 words).  This module
// walks the table and keeps one thing moving at a time:
//
//   * it loads the pictures into their own slots, in order, starting with slot
//     0, one loader run at a time.  Slot 0 is therefore ready first and the
//     screen comes up on it; the rest are filled in behind while the picture is
//     already on screen (the loader writes through the single word port while
//     the reader fetches through the burst port, which is what the controller's
//     two ports are for).
//   * a key press moves the screen on to the next *loaded* picture, and a
//     switch turns a one-per-second automatic walk over the loaded ones on and
//     off.  Both are the same step here: `disp_sel` walks the loaded set.
//
// A load that fails is retried a second later, up to MAX_TRIES times, and then
// that slot is given up on and the others carry on: a bad picture costs one
// picture, not the carousel.
//
// What leaves the module is `disp_sel` (with `disp_tgl` toggling once per
// change) and `load_slot`; the bases are the top level's business, since that is
// where the arithmetic and the two clock domains live.
//=============================================================================
`timescale 1ns / 100fs

module mm_pic_sched #(
    parameter integer N_SLOTS         = 8,           // must be 8: the masks are 8 bit
    parameter integer RETRY_CYCLES    = 50_000_000,  // 1 s at 50 MHz
    parameter integer MAX_TRIES       = 3,
    parameter integer CAROUSEL_CYCLES = 50_000_000   // 1 s per picture
)(
    input  wire        clk,
    input  wire        rst,

    // the scan's picture table, read one entry at a time
    output reg  [2:0]  tbl_index,
    input  wire [3:0]  images_found,
    input  wire        scan_parked,     // the scan has finished and parked

    // the loader
    output reg         load_start,
    output reg  [2:0]  load_slot,
    input  wire        load_busy,
    input  wire        load_done,       // one cycle: that frame is in
    input  wire [3:0]  load_phase,      // 4'hF = it gave up

    // the controls
    input  wire        key_press,       // one cycle: show the next picture
    input  wire        auto_on,         // the switch, already debounced

    // what the screen should be showing
    output reg  [2:0]  disp_sel,
    output reg         disp_tgl,        // one toggle per change of disp_sel

    // for the on-board display
    output wire [3:0]  loaded_count,
    output wire [3:0]  state_dbg
);

    localparam [2:0] S_IDLE  = 3'd0,    // pick the next slot to fetch
                     S_REQ   = 3'd1,    // pulse the loader
                     S_BEG   = 3'd2,    // wait for the loader to pick it up
                     S_LOAD  = 3'd3,    // wait for it
                     S_RETRY = 3'd4,    // it failed: wait a second
                     S_DONE  = 3'd5;    // nothing left to load

    localparam [20:0] RETRY = RETRY_CYCLES;
    localparam [26:0] CAROU = CAROUSEL_CYCLES;

    reg  [7:0]  loaded;                 // slots with a picture in them
    reg  [7:0]  bad;                    // slots given up on
    reg         announced;              // the top level has been told which slot
    reg  [2:0]  state;
    reg  [2:0]  slot;                   // the slot being fetched
    reg  [1:0]  tries;
    reg  [20:0] retry_cnt;
    reg  [26:0] carou_cnt;

    assign state_dbg = {1'b0, state};

    // the first set bit at or after `from`, wrapping: which picture is next.
    // Written as a scan rather than a priority encoder so the wrap costs
    // nothing: the index is three bits, so it wraps by itself.
    function [2:0] next_set;
        input [7:0] mask;
        input [2:0] from;
        integer i;
        reg [2:0] k;
        reg       found;
        begin
            next_set = 3'd0;
            found    = 1'b0;
            for (i = 0; i < 8; i = i + 1) begin
                k = from + i[2:0];
                if (!found && mask[k]) begin
                    next_set = k;
                    found    = 1'b1;
                end
            end
        end
    endfunction

    // Slots 0..images_found-1 are the ones the scan vouched for; the rest of
    // the mask is not ours to load.  (1 << 8 wraps to 0 in eight bits, so the
    // whole-found mask comes out right for eight pictures as well.)
    wire [7:0] found_mask = (8'd1 << images_found) - 8'd1;
    wire [7:0] want_mask  = found_mask & ~loaded & ~bad;
    wire       any_want   = (want_mask != 8'd0);
    wire [2:0] want_slot  = next_set(want_mask, 3'd0);

    localparam [1:0] TRIES_MAX = MAX_TRIES - 1;

    //------------------------------------------------------------------
    // loading, one loader run at a time, slot 0 first
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            slot       <= 3'd0;
            tries      <= 2'd0;
            retry_cnt  <= 21'd0;
            tbl_index  <= 3'd0;
            load_start <= 1'b0;
            load_slot  <= 3'd0;
            loaded     <= 8'd0;
            bad        <= 8'd0;
        end else begin
            load_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    // the table is only worth walking once the scan has parked
                    if (scan_parked && any_want) begin
                        slot      <= want_slot;
                        tbl_index <= want_slot;
                        tries     <= 2'd0;
                        state     <= S_REQ;
                    end else if (scan_parked) begin
                        state <= S_DONE;
                    end
                end

                // the entry is combinational off tbl_index, so it is already
                // there; ask as soon as the loader has finished with the port
                S_REQ: begin
                    tbl_index <= slot;
                    if (!load_busy) begin
                        load_slot  <= slot;
                        load_start <= 1'b1;
                        retry_cnt  <= RETRY;
                        state      <= S_BEG;
                    end
                end

                // The loader's phase still says "failed" from the last attempt
                // until it has picked this one up, so the failure test below is
                // only meaningful once it is running: wait for it to be busy.
                S_BEG: begin
                    if (load_busy) state <= S_LOAD;
                end

                S_LOAD: begin
                    if (load_done) begin
                        loaded[slot] <= 1'b1;
                        state        <= S_IDLE;
                    end else if (load_phase == 4'hF) begin
                        // it gave up: try again in a second, and if it keeps
                        // giving up, leave it out and carry on with the rest
                        if (tries == TRIES_MAX) begin
                            bad[slot] <= 1'b1;
                            state     <= S_IDLE;
                        end else begin
                            tries <= tries + 2'd1;
                            state <= S_RETRY;
                        end
                    end
                end

                S_RETRY: begin
                    if (retry_cnt == 21'd0) state <= S_REQ;
                    else                    retry_cnt <= retry_cnt - 21'd1;
                end

                // S_DONE: nothing left to load.  A card that is put in after
                // the scan (the probe rescans when one appears) can add pictures
                // to the table, so this is not a one-way door.
                default: begin
                    state <= any_want ? S_IDLE : S_DONE;
                end
            endcase
        end
    end

    //------------------------------------------------------------------
    // the screen: the next loaded picture, on a key press or on the timer
    //
    // One event, two sources.  The switch only decides whether the timer runs;
    // the step is the same walk over the loaded set either way, so a key press
    // while the carousel is running does the obvious thing.
    //------------------------------------------------------------------
    wire [2:0] adv_to    = next_set(loaded, disp_sel + 3'd1);
    wire       can_adv   = (loaded != 8'd0) && (adv_to != disp_sel);
    wire       carou_tick = (carou_cnt == CAROU);
    wire       step_now  = can_adv && (key_press || (auto_on && carou_tick));

    always @(posedge clk) begin
        if (rst) begin
            disp_sel  <= 3'd0;
            disp_tgl  <= 1'b0;
            carou_cnt <= 27'd0;
            announced <= 1'b0;      // driven here and nowhere else
        end else begin
            if (carou_tick)         carou_cnt <= 27'd0;
            else if (auto_on)       carou_cnt <= carou_cnt + 27'd1;

            if ((loaded != 8'd0) && (!announced || !loaded[disp_sel])) begin
                // The first picture to arrive takes the screen, and the top
                // level has to be *told*.  It only lets the reader fetch once it
                // has been told which slot to point at, and "slot 0, the one we
                // were going to show anyway" is still something it has to be
                // told: leaving that one out is how the board ended up showing
                // nothing but the colour bars (the picture loaded into slot 0
                // and nothing ever pointed the reader at it).  The same branch
                // covers the other order, where the first picture to arrive is
                // not the slot we started on.
                disp_sel  <= next_set(loaded, 3'd0);
                disp_tgl  <= ~disp_tgl;
                announced <= 1'b1;
            end else if (step_now) begin
                disp_sel <= adv_to;
                disp_tgl <= ~disp_tgl;
            end
        end
    end

    // how many pictures are in: the on-board display shows this
    reg  [3:0] cnt_r;
    integer    ci;
    always @(*) begin
        cnt_r = 4'd0;
        for (ci = 0; ci < 8; ci = ci + 1)
            cnt_r = cnt_r + loaded[ci];
    end
    assign loaded_count = cnt_r;

endmodule
