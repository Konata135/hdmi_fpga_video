//=============================================================================
// top.v -- HX4S20C HDMI multimedia player, top level
//
//   board 50 MHz --> mm_clk_gen --> 25 MHz pixel clock
//                               \-> 125 MHz TMDS serialiser clock
//                               \-> mm_reset_ctrl / mm_reset_sync
//
//   mm_video_timing -> mm_test_pattern -> mm_hdmi_tx -> HDMI_B connector
//
// The HDMI transmitter is entirely implemented in this project: TMDS
// encoding, control periods, guard bands, preamble, all four serialiser
// lanes and the clock lane.  The only vendor cells used are device
// primitives (EG_PHY_PLL, EG_LOGIC_BUFG, EG_LOGIC_ODDR).
//
// Later milestones replace mm_test_pattern with the TF card image pipeline
// and add the audio path.
//=============================================================================
`timescale 1ns / 100fs

module top (
    input  wire clk,           // 50 MHz on-board oscillator, pin R7
    input  wire rst_n,         // push button, active low, pin A2

    // HDMI type A on the HDMI_B connector
    output wire HDMI_CLK_P,
    output wire HDMI_D0_P,
    output wire HDMI_D1_P,
    output wire HDMI_D2_P,

    // six digit seven segment status display
    output wire [5:0] seg_sel,
    output wire [7:0] seg_data,

    // TF card, SPI mode
    output wire sd_ncs,
    output wire sd_dclk,
    output wire sd_mosi,
    input  wire sd_miso
);

    //------------------------------------------------------------------
    // clocks and reset
    //------------------------------------------------------------------
    wire pix_clk;
    wire pix_5x_clk;
    wire mem_clk;
    wire mem_clk_dev;
    wire mem_clk_sft;
    wire pll_lock;

    mm_clk_gen u_clk_gen (
        .refclk     (clk),
        .pix_clk    (pix_clk),
        .pix_5x_clk (pix_5x_clk),
        .mem_clk    (mem_clk),
        .mem_clk_dev(mem_clk_dev),
        .mem_clk_sft(mem_clk_sft),
        .pll_lock   (pll_lock)
    );

    wire por_n;

    mm_reset_ctrl #(
        .HOLD_CYCLES (500_000),
        .N_LOCK      (1)
    ) u_reset_ctrl (
        .clk_ref   (clk),
        .btn_rst_n (rst_n),
        .pll_lock  ({pll_lock}),
        .por_n     (por_n)
    );

    wire pix_rst;

    mm_reset_sync u_pix_rst_sync (
        .clk   (pix_clk),
        .por_n (por_n),
        .rst   (pix_rst)
    );

    //------------------------------------------------------------------
    // video timing
    //------------------------------------------------------------------
    wire [11:0] h_cnt;
    wire [10:0] v_cnt;
    wire        hs;
    wire        vs;
    wire        de;
    wire        frame_first;
    wire        line_last;

    mm_video_timing #(
        .H_ACTIVE (640),
        .H_FP     (16),
        .H_SYNC   (96),
        .H_BP     (48),
        .V_ACTIVE (480),
        .V_FP     (10),
        .V_SYNC   (2),
        .V_BP     (33)
    ) u_video_timing (
        .pix_clk     (pix_clk),
        .rst         (pix_rst),
        .h_cnt       (h_cnt),
        .v_cnt       (v_cnt),
        .hs          (hs),
        .vs          (vs),
        .de          (de),
        .frame_first (frame_first),
        .line_last   (line_last)
    );

    //------------------------------------------------------------------
    // picture source
    //
    // The bring-up pattern is still built here and is what the screen shows
    // until a picture has been loaded out of the card; from then on the frame
    // store drives the transmitter (see the picture path further down).
    //------------------------------------------------------------------
    wire [23:0] vout_rgb;
    wire [23:0] pattern_rgb;

    mm_test_pattern #(
        .H_ACTIVE (640),
        .V_ACTIVE (480)
    ) u_test_pattern (
        .pix_clk     (pix_clk),
        .rst         (pix_rst),
        .frame_first (frame_first),
        .h_cnt       (h_cnt),
        .v_cnt       (v_cnt),
        .rgb         (pattern_rgb)
    );

    //------------------------------------------------------------------
    // HDMI transmitter
    //------------------------------------------------------------------
    mm_hdmi_tx u_hdmi_tx (
        .pix_clk    (pix_clk),
        .pix_5x_clk (pix_5x_clk),
        .rst        (pix_rst),
        .de         (de),
        .hs         (hs),
        .vs         (vs),
        .rgb        (vout_rgb),
        .tmds_clk_p (HDMI_CLK_P),
        .tmds_d0_p  (HDMI_D0_P),
        .tmds_d1_p  (HDMI_D1_P),
        .tmds_d2_p  (HDMI_D2_P)
    );

    //------------------------------------------------------------------
    // SD card bring-up
    //
    // The sector engine inside the probe is shared with the picture loader
    // further down, so the loader's side of that handshake is declared here.
    wire        aux_sec_busy, aux_sec_done, aux_sec_valid;
    wire [3:0]  aux_sec_error;
    wire [8:0]  aux_sec_index;
    wire [7:0]  aux_sec_byte;
    wire        ld_rd_start;
    wire [31:0] ld_rd_lba;

    // mm_sd_probe owns the whole M3 sequence: initialise the card, read the
    // first sector with CMD17, check the MBR signature and decode the first
    // partition entry.  It also owns the retry policy, so nothing here has to
    // keep time: a card inserted after power up is still picked up, and a
    // read that fails is retried instead of leaving the design stuck.
    //
    // The probe drives the card pins itself, so the top level only has to
    // pass them through and put the two result nibbles on the display.
    //------------------------------------------------------------------
    wire [3:0]  sd_phase;       // sequence step
    wire [3:0]  sd_images;      // pictures the scan has accepted
    wire [3:0]  sd_error;       // last failure code, 0 = none
    wire [7:0]  sd_capacity;    // partition size in GiB
    wire [63:0] sd_table;       // first picture: {start_lba, length_in_bytes}
    wire [2:0]  sd_page_sel = 3'd0;

    mm_sd_probe u_sd_probe (
        .clk            (clk),
        .rst            (~por_n),
        .disp_phase     (sd_phase),
        .sig_ok         (),
        .part_type      (),
        .part_start_lba (),
        .part_sectors   (),
        .capacity_gib   (sd_capacity),
        .images_found   (sd_images),
        .tbl_index      (sd_page_sel),
        .tbl_entry      (sd_table),
        .error_code     (sd_error),
        .error_count    (),
        .read_attempts  (),
        .aux_rd_start   (ld_rd_start),
        .aux_rd_lba     (ld_rd_lba),
        .aux_sec_busy   (aux_sec_busy),
        .aux_sec_done   (aux_sec_done),
        .aux_sec_error  (aux_sec_error),
        .aux_sec_valid  (aux_sec_valid),
        .aux_sec_index  (aux_sec_index),
        .aux_sec_byte   (aux_sec_byte),
        .sd_ncs         (sd_ncs),
        .sd_sclk        (sd_dclk),
        .sd_mosi        (sd_mosi),
        .sd_miso        (sd_miso)
    );

    //------------------------------------------------------------------
    // picture path: TF card -> frame store -> HDMI
    //
    // The loader pulls picture 0 out of the card through the probe's sector
    // engine; mm_frame_wr packs the bytes into 32 bit words and writes them
    // into the SDRAM; mm_frame_rd walks the store backwards (a BMP is bottom
    // up) and hands out one pixel per pixel clock.  Until the frame is in, the
    // screen keeps showing the bring-up pattern -- a missing card costs a
    // picture, not a screen.
    //------------------------------------------------------------------
    wire mem_rst;

    mm_reset_sync u_mem_rst_sync (
        .clk   (mem_clk),
        .por_n (por_n),
        .rst   (mem_rst)
    );

    wire        mem_ready;
    wire        wr_req;
    wire [20:0] wr_addr;
    wire [31:0] wr_data;
    wire        br_dv, br_busy;
    wire        rd_pix_ready;          // reader: its word buffer is primed
    wire [15:0] rd_rows_seen;       // reader diagnostic: stored rows walked
    wire [8:0]  rd_used;            // reader diagnostic: words in its buffer
    wire        rd_app_ok;          // ... room for a whole run
    wire [1:0]  rd_mstate;          // ... M_IDLE / M_BURST / M_FLUSH
    wire [31:0] br_data;

    // Both ports are shared with the power-up probe, which has them to itself
    // until it is done: the single word port with the frame writer, and the
    // burst port with the reader.  The probe is over within microseconds of the
    // controller coming up and the loader cannot start until the scan has
    // finished, but the load is gated on the probe being done as well, so the
    // two can never overlap.  The completion side (br_dv/br_data) goes to both
    // consumers; only one of them is ever asking.
    wire        mem_wr_done, mem_rd_dv;
    wire [31:0] mem_rd_data;
    wire        mem_echo_ok;
    wire        st_busy, st_done;
    wire        st_ok_word, st_ok_burst;
    wire [31:0] st_got_word;
    wire [3:0]  st_lanes_word, st_shift_burst;
    wire [2:0]  st_adj_word, st_adj_burst;
    wire [7:0]  st_top_burst;
    wire        frame_active;
    wire [23:0] frame_rgb;
    wire        frame_valid;
    wire [15:0] frame_underrun;
    wire [20:0] st_addr;
    wire [31:0] st_wdata;
    wire [3:0]  st_wdm;
    wire        st_wr_req, st_rd_req;
    wire [20:0] st_b_addr;
    wire [8:0]  st_b_len;
    wire        st_b_req;
    wire        rd_b_req;
    wire [20:0] rd_b_addr;
    wire [8:0]  rd_b_len;

    // Two users share the memory ports: the startup probe and, later, the
    // picture loader (write port) and the frame reader (burst port).  The mux
    // selects on the probe's *request*, not on its `busy`: a request presented
    // in a cycle this side is not selected is simply gone, and both other users
    // are state machines that then wait for a completion that never arrives --
    // the reader hangs mid-burst, which is what turned the picture into one
    // flat colour while the probe happened to be sweeping.  With the request
    // mux the others are blocked for exactly the cycles the probe is asking,
    // and after power-up the probe no longer asks for the burst port at all.
    wire        mem_wr_req  = st_wr_req ? st_wr_req  : wr_req;
    wire [20:0] mem_wr_addr = st_wr_req ? st_addr    : wr_addr;
    wire [31:0] mem_wr_data = st_wr_req ? st_wdata   : wr_data;
    wire [3:0]  mem_wr_dm   = st_wr_req ? st_wdm     : 4'h0;
    wire        mem_br_req  = st_b_req  ? st_b_req   : rd_b_req;
    wire [20:0] mem_br_addr = st_b_req  ? st_b_addr  : rd_b_addr;
    wire [8:0]  mem_br_len  = st_b_req  ? st_b_len   : rd_b_len;

    // Controller logic on the 0 degree clock, the device on the 180 degree
    // one -- the arrangement the vendor reference design uses (see its .sdc).
    mm_sdram u_sdram (
        .clk       (mem_clk),
        .clk_dev   (mem_clk_dev),
        .clk_sft   (mem_clk_sft),
        .rst       (mem_rst),
        .ready     (mem_ready),
        // the probe measures where the read data actually lands and holds the
        // answer here, so the picture path is calibrated too
        .rd_adj    (st_adj_word),
        .br_adj    (st_adj_burst),
        .wr_req    (mem_wr_req),
        .wr_addr   (mem_wr_addr),
        .wr_data   (mem_wr_data),
        .wr_dm     (mem_wr_dm),
        .wr_done   (mem_wr_done),
        .rd_req    (st_rd_req),
        .rd_addr   (st_addr),
        .rd_dv     (mem_rd_dv),
        .rd_data   (mem_rd_data),
        .br_req    (mem_br_req),
        .br_addr   (mem_br_addr),
        .br_len    (mem_br_len),
        .br_dv     (br_dv),
        .br_data   (br_data),
        .br_busy   (br_busy),
        .init_done (),
        .state_dbg (),
        .echo_ok   (mem_echo_ok)
    );

    // A known word through the single word port, then four more through the
    // burst port: the only way to tell whether the device is really there, and
    // whether the path the reader uses samples it correctly, since the
    // controller reports success whether or not anything answers it.
    //
    // PIC_CHECK adds one more thing once the picture is up: the probe reads two
    // words *out of the picture region* through the same single word port and
    // hands them over for the display.  That is the only reading that says
    // whether the store holds the picture or just something non-blank: page 3's
    // 0xFF count cannot tell those apart, and the word the probe tests at
    // power-up is written through a different port than the loader uses.
    wire        pic_go_w;       // driven from frame_display_m2 further down:
                                // the identifier has to exist before it is used,
                                // and an implicit net here would collide with
                                // that reg declaration (see the note below)
    wire        pic_busy;
    wire [23:0] pic_w0, pic_w1;

    mm_sdram_selftest #(
        .PIC_CHECK (1),
        // Re-probe every two seconds so pages 4..8 keep answering "does the
        // device still respond?" instead of describing the power-up moment.
        .RESWEEP_CYCLES (125_000_000)
    ) u_sdram_probe (
        .clk     (mem_clk),
        .rst     (mem_rst),
        .ready   (mem_ready),
        .addr    (st_addr),
        .wdata   (st_wdata),
        .wdm     (st_wdm),
        .wr_req  (st_wr_req),
        .wr_done (mem_wr_done),
        .rd_req  (st_rd_req),
        .rd_dv   (mem_rd_dv),
        .rdata   (mem_rd_data),
        .b_addr  (st_b_addr),
        .b_len   (st_b_len),
        .b_req   (st_b_req),
        .b_dv    (br_dv),
        .b_data  (br_data),
        .rd_adj     (st_adj_word),
        .br_adj     (st_adj_burst),
        .busy    (st_busy),
        .done    (st_done),
        .ok_word    (st_ok_word),
        .got_word   (st_got_word),
        .lanes_word (st_lanes_word),
        .adj_word   (),
        .ok_burst   (st_ok_burst),
        .shift_burst(st_shift_burst),
        .adj_burst  (),
        .top_burst  (st_top_burst),
        .pic_go     (pic_go_w),
        .pic_busy   (pic_busy),
        .pic_w0     (pic_w0),
        .pic_w1     (pic_w1)
    );

    // The probe's answer lives in the memory clock domain and the display and
    // the load gate live in the 50 MHz one.  It must not come across through a
    // plain per bit synchroniser: while it changes once, each of those flops
    // can latch a *different* moment, and since the value never changes again
    // nothing ever corrects it -- which is exactly how a board reading ended up
    // contradicting itself (the byte mask said all four bytes matched while the
    // value plainly did not).  So the whole answer is copied in one go, on the
    // synchronized falling edge of the probe's busy flag, several cycles after
    // everything has settled; the display then reads plain registers in its own
    // domain.
    reg        st_busy_s1, st_busy_s2, st_busy_s3;
    reg        st_lat;              // one cycle: copy the answer across
    reg [31:0] st_gotw_q;
    reg [3:0]  st_lanw_q, st_shb_q;
    reg [1:0]  st_adjw_q, st_adjb_q;
    reg [7:0]  st_byb_q;
    reg        st_okw_q, st_okb_q, st_ech_q;

    // the reader's underrun counter is in the *pixel* domain and keeps moving,
    // so it cannot be latched this way; off by one on a count does not matter
    reg [15:0] st_un_s1, st_un_s2;

    always @(posedge clk or negedge por_n) begin
        if (!por_n) begin
            st_busy_s1 <= 1'b1;
            st_busy_s2 <= 1'b1;
            st_busy_s3 <= 1'b1;
            st_lat     <= 1'b0;
            st_un_s1   <= 16'd0;
            st_un_s2   <= 16'd0;
            st_gotw_q  <= 32'd0;
            st_lanw_q  <= 4'd0;
            st_shb_q   <= 4'd0;
            st_adjw_q  <= 2'd0;
            st_adjb_q  <= 2'd0;
            st_byb_q   <= 8'd0;
            st_okw_q   <= 1'b0;
            st_okb_q   <= 1'b0;
            st_ech_q   <= 1'b0;
        end else begin
            st_busy_s1 <= st_busy;
            st_busy_s2 <= st_busy_s1;
            st_busy_s3 <= st_busy_s2;
            st_lat     <= st_busy_s3 & ~st_busy_s2;
            st_un_s1   <= frame_underrun;
            st_un_s2   <= st_un_s1;
            if (st_lat) begin
                st_gotw_q <= st_got_word;
                st_lanw_q <= st_lanes_word;
                st_shb_q  <= st_shift_burst;
                st_adjw_q <= st_adj_word;
                st_adjb_q <= st_adj_burst;
                st_byb_q  <= st_top_burst;
                st_okw_q  <= st_ok_word;
                st_okb_q  <= st_ok_burst;
                // the controller's own echo: did the input side ever see the
                // value the controller was itself driving out?
                st_ech_q  <= mem_echo_ok;
            end
        end
    end

    reg         load_start;
    reg  [26:0] load_timer;
    reg         load_started;
    reg         frame_display;

    // The picture check's answer crosses the same way, on the falling edge of
    // its own busy flag -- it happens seconds after the startup sweep, long
    // after st_lat has come and gone.
    reg        pic_busy_s1, pic_busy_s2, pic_busy_s3;
    reg [23:0] pic_w0_q, pic_w1_q;

    always @(posedge clk or negedge por_n) begin
        if (!por_n) begin
            pic_busy_s1 <= 1'b0;
            pic_busy_s2 <= 1'b0;
            pic_busy_s3 <= 1'b0;
            pic_w0_q    <= 24'd0;
            pic_w1_q    <= 24'd0;
        end else begin
            pic_busy_s1 <= pic_busy;
            pic_busy_s2 <= pic_busy_s1;
            pic_busy_s3 <= pic_busy_s2;
            if (pic_busy_s3 & ~pic_busy_s2) begin
                pic_w0_q <= pic_w0;
                pic_w1_q <= pic_w1;
            end
        end
    end

    wire        ld_busy, ld_done;
    wire [3:0]  ld_error, ld_phase;
    wire [31:0] ld_bytes_sent;
    wire [15:0] ld_sectors_read;
    wire        ld_hdr_ok;
    wire [7:0]  ld_ff_first;

    wire        ld_frame_start;
    wire [20:0] ld_base_word;
    wire [17:0] ld_words_total;
    wire        ld_byte_valid;
    wire [7:0]  ld_byte_data;
    wire [7:0]  ld_overflow;
    wire        frame_wr_done_tgl;
    wire        frame_wr_busy;

    wire [31:0] img_start_lba = sd_table[63:32];
    wire [31:0] img_length    = sd_table[31:0];

    mm_img_load u_img_load (
        .clk          (clk),
        .rst          (~por_n),
        .start        (load_start),
        .start_lba    (img_start_lba),
        .file_bytes   (img_length),
        .entry_valid  (sd_images != 4'd0),
        .rd_lba       (ld_rd_lba),
        .rd_start     (ld_rd_start),
        .rd_busy      (aux_sec_busy),
        .rd_done      (aux_sec_done),
        .rd_error     (aux_sec_error),
        .rd_index     (aux_sec_index),
        .rd_byte      (aux_sec_byte),
        .rd_valid     (aux_sec_valid),
        .frame_start  (ld_frame_start),
        .base_word    (ld_base_word),
        .words_total  (ld_words_total),
        .byte_valid   (ld_byte_valid),
        .byte_data    (ld_byte_data),
        .frame_done_tgl (frame_wr_done_tgl),
        .busy         (ld_busy),
        .done         (ld_done),
        .error_code   (ld_error),
        .phase        (ld_phase),
        .bytes_sent   (ld_bytes_sent),
        .sectors_read (ld_sectors_read),
        .hdr_ok       (ld_hdr_ok),
        .ff_first     (ld_ff_first)
    );

    // 128 words of packing buffer, not the module's 1024 word default: the byte
    // side delivers one word every 80 ns and the word side drains far faster
    // than that, so a handful of words covers the SDRAM's own latency; a deep
    // buffer buys nothing and its read multiplexer was 5.2 ns of routing
    // straight into wr_data (fanout 64, 68 % of that path).
    mm_frame_wr #(
        .AFIFO_AW (7)
    ) u_frame_wr (
        .clk_byte       (clk),
        .rst_byte       (~por_n),
        .start          (ld_frame_start),
        .base_word      (ld_base_word),
        .words_total    (ld_words_total),
        .byte_valid     (ld_byte_valid),
        .byte_data      (ld_byte_data),
        .overflow_count (ld_overflow),
        .busy           (frame_wr_busy),
        .clk            (mem_clk),
        .rst            (mem_rst),
        .wr_addr        (wr_addr),
        .wr_data        (wr_data),
        .wr_req         (wr_req),
        .wr_done        (mem_wr_done),
        .ready          (mem_ready),
        .frame_done     (),
        .frame_done_tgl (frame_wr_done_tgl),
        .words_written  ()
    );

    mm_frame_rd u_frame_rd (
        .clk            (mem_clk),
        .rst            (mem_rst),
        .base_word      (21'd0),            // picture 0 lives at word 0
        .active         (frame_active),
        .br_req         (rd_b_req),
        .br_addr        (rd_b_addr),
        .br_len         (rd_b_len),
        .br_dv          (br_dv),
        .br_data        (br_data),
        .br_busy        (br_busy),
        .ready          (mem_ready),
        .flush_done     (),
        .words_fetched  (),
        .rows_seen      (rd_rows_seen),
        .dbg_used       (rd_used),
        .dbg_app_ok     (rd_app_ok),
        .dbg_mstate     (rd_mstate),
        .pix_ready      (rd_pix_ready),
        .pix_clk        (pix_clk),
        .pix_rst        (pix_rst),
        .pix_de         (de),
        .pix_rgb        (frame_rgb),
        .pix_valid      (frame_valid),
        .pix_underrun   (frame_underrun),
        .dbg_flush_words()
    );

    //------------------------------------------------------------------
    // when to load, and when the picture takes the screen
    //
    // Start once the scan has parked (phase C) with at least one picture
    // found.  A load that fails is retried once a second for as long as it
    // keeps failing, so pulling the card and putting it back is enough to
    // recover -- nothing here can wedge, it just keeps showing the pattern.
    //------------------------------------------------------------------
    localparam [26:0] LOAD_RETRY_CYCLES = 27'd49_999_999;   // 1 s at 50 MHz

    always @(posedge clk or negedge por_n) begin
        if (!por_n) begin
            load_start    <= 1'b0;
            load_started  <= 1'b0;
            frame_display <= 1'b0;
            load_timer    <= 27'd0;
        end else begin
            load_start <= 1'b0;

            if (ld_done)
                frame_display <= 1'b1;

            if (!load_started) begin
                // the probe must have finished with the single word port before
                // the loader can ask the writer to start using it
                if (!st_busy_s2 && (sd_phase == 4'hC) && (sd_images != 4'd0)) begin
                    load_start   <= 1'b1;
                    load_started <= 1'b1;
                end
            end else if (!frame_display && (ld_phase == 4'hF)) begin
                if (load_timer == LOAD_RETRY_CYCLES) begin
                    load_timer <= 27'd0;
                    load_start <= 1'b1;
                end else begin
                    load_timer <= load_timer + 27'd1;
                end
            end else begin
                load_timer <= 27'd0;
            end
        end
    end

    // The reader may start fetching as soon as the frame is in (that is what
    // `active` says), but the screen changes over on a frame boundary, not in
    // the middle of one: a source that changed halfway down would show a seam,
    // and by the time the boundary comes the reader's buffer is primed, so the
    // first line of the picture is complete.
    // `frame_display` is set in the 50 MHz domain and read by the reader in the
    // memory one, so it goes through a synchroniser instead of straight into
    // the fetch logic.  It only ever changes once, but two flops cost nothing
    // and stop the timer having to relate the two domains through a
    // combinational cone.
    reg frame_display_m1, frame_display_m2;

    always @(posedge mem_clk) begin
        if (mem_rst) begin
            frame_display_m1 <= 1'b0;
            frame_display_m2 <= 1'b0;
        end else begin
            frame_display_m1 <= frame_display;
            frame_display_m2 <= frame_display_m1;
        end
    end

    assign frame_active = frame_display_m2;

    // The probe's picture check starts when the picture is on screen.  The wire
    // itself is declared up at the probe (it has to be: a name used before its
    // declaration becomes an implicit net here, which is how this project once
    // lost a day to a signal that was silently one bit wide).
    assign pic_go_w = frame_display_m2;

    //------------------------------------------------------------------
    // page 13: do the two streams the picture crosses actually vary?
    //
    // Pages 9..12 prove what the *store* holds.  These two counters ask the
    // next question along the path, in the two clock domains the picture moves
    // through: the words the reader receives from the burst port (memory
    // domain): the words the reader receives from the burst port.  It counts
    // how many times the incoming word *changed* inside one 4096 word window,
    // and it re-arms, so page 13's top two digits are a *live* meter of the
    // stream the reader is being fed right now:
    //
    //   d4 d3  changes / 256 in the last window: 0F..10 = nearly every word
    //          differs (a picture being walked), 00 = the same word over and
    //          over (the memory side is handing out one value)
    //   d2     A = a full 4096 word window completed in the last 0.25 s, so
    //          words *are* still arriving; F = they are not
    //
    // Together with page 17 (the same test on what the transmitter gets) and
    // page 18 (does the walk advance), these three separate "no more words",
    // "words but constant" and "words fine, output flat" -- which are three
    // completely different things to fix.
    //------------------------------------------------------------------
    reg [12:0] brd_cnt, brd_chg;
    reg        brd_flag;
    reg [23:0] brd_to;
    reg [31:0] brd_prev;
    reg [7:0]  brd_disp;

    always @(posedge mem_clk) begin
        if (mem_rst) begin
            brd_cnt  <= 13'd0;
            brd_chg  <= 13'd0;
            brd_flag <= 1'b0;
            brd_to   <= 24'd0;
            brd_prev <= 32'd0;
            brd_disp <= 8'd0;
        end else if (frame_active) begin
            if (brd_to != 24'hFFFFFF) brd_to <= brd_to + 24'd1;
            if (br_dv) begin
                brd_to <= 24'd0;
                if (br_data != brd_prev) brd_chg <= brd_chg + 13'd1;
                brd_prev <= br_data;
                if (brd_cnt == 13'd4095) begin
                    brd_cnt  <= 13'd0;
                    brd_chg  <= 13'd0;
                    brd_disp <= brd_chg[11:4];
                    brd_flag <= 1'b1;
                end else begin
                    brd_cnt <= brd_cnt + 13'd1;
                end
            end
            if (brd_to == 24'hFFFFFF) brd_flag <= 1'b0;   // words stopped
        end
    end

    reg frame_display_s1, frame_display_s2;
    reg frame_show;
    // The reader's word buffer has to be primed before the picture is unmasked,
    // otherwise the first pixels of the frame have nothing to show: they repeat
    // the last one, which shifts the whole picture sideways and, because a line
    // wraps, leaves a second copy at the edge.  Whether the buffer is primed by
    // the time the frame boundary comes round is a race, so it was "fine the
    // first few resets" and then visibly offset.  `pix_ready` existed for this
    // and was never wired up.
    reg rd_ready_s1, rd_ready_s2;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            frame_display_s1 <= 1'b0;
            frame_display_s2 <= 1'b0;
            frame_show       <= 1'b0;
            rd_ready_s1      <= 1'b0;
            rd_ready_s2      <= 1'b0;
        end else begin
            frame_display_s1 <= frame_display;
            frame_display_s2 <= frame_display_s1;
            rd_ready_s1      <= rd_pix_ready;
            rd_ready_s2      <= rd_ready_s1;
            if (frame_first)
                frame_show <= frame_display_s2 && rd_ready_s2;
        end
    end

    assign vout_rgb = frame_show ? frame_rgb : pattern_rgb;

    //------------------------------------------------------------------
    // page 17: is the stream *the transmitter gets* as fresh as the stream
    // the reader claims to deliver?
    //
    // Page 13 counts changes on the reader's own `pix_valid` pixels -- i.e.
    // only the pixels it considers fresh -- so it cannot see a stream that is
    // mostly made of stale repeats.  This one counts changes on `vout_rgb`
    // during **every** active pixel once the picture is selected, which is
    // exactly what mm_hdmi_tx serialises.  Same window (1024 pixels), so the
    // two are directly comparable: the file predicts 282 changes in the
    // picture's first 1024 pixels, and the reader measured 282.  Reading 11A
    // here means every displayed pixel really is a fresh picture pixel;
    // anything much smaller means the screen is being fed repeats.
    //------------------------------------------------------------------
    // Re-armed on every 1024 pixel window, so page 17 is a *live* meter of the
    // stream the transmitter is being fed right now -- not a snapshot of the
    // first line and a half after the switch, which is all the earlier version
    // ever showed and is exactly why "the picture works for a moment and then
    // goes flat" could not be seen in it.
    reg [11:0] voc_cnt, voc_chg, voc_last;
    reg [23:0] voc_prev;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            voc_cnt  <= 12'd0;
            voc_chg  <= 12'd0;
            voc_last <= 12'd0;
            voc_prev <= 24'd0;
        end else if (frame_show && de) begin
            if (vout_rgb != voc_prev) voc_chg <= voc_chg + 12'd1;
            voc_prev <= vout_rgb;
            if (voc_cnt == 12'd1023) begin
                voc_cnt  <= 12'd0;
                voc_last <= voc_chg;
                voc_chg  <= 12'd0;
            end else begin
                voc_cnt <= voc_cnt + 12'd1;
            end
        end
    end

    //------------------------------------------------------------------
    // digits 4, 3 and 2 rotate through seventeen pages, two seconds each
    //
    // Three digits cannot carry everything the storage path has to report,
    // so they take turns.  Digits 5, 1 and 0 never change meaning.
    //
    //   page 0  sequence step | pictures found | error code
    //   page 1  how many sectors the loader really read (0x709 for a 640x480
    //           24 bit BMP)
    //   page 2  A (MBR checked) | partition size in GiB
    //   page 3  loader phase | loader error code | 0xFF bytes it handed over
    //   page 4,
    //   page 5,
    //   page 6  the raw 32 bit word the store gave back, 12 bits at a time,
    //           with the byte match mask on the last digit
    //   page 7  A/F burst probe | which byte lanes matched | top byte it read
    //   page 8  A/F both ends clean | packing FIFO overruns | reader underruns
    //   page 9,
    //   page 10 the first pixel of the picture as the *store* holds it, read
    //           back through the probe's single word port once the picture is
    //           on screen: R,G,B as six nibbles
    //   page 11,
    //   page 12 the same for the first pixel of the picture's stored top row
    //           (word 229,920) -- the word the display's first line comes from
    //   page 13 how much the two streams vary: the reader's burst words / 256,
    //           then how much its output pixels changed, / 256 (see above)
    //   page 14 the address the loader was pointed at (the scan's first table
    //           entry, low 12 bits) -- "which picture did we load"
    //   page 15,
    //   page 16 retired: they sampled vout_rgb at fixed instants that did not
    //           line up with the pixel they meant to capture, so their readings
    //           belonged to nothing.  The rotation steps over them.
    //   page 17 how many times vout_rgb changes in the last 1024 active pixels
    //           (re-armed every window, so it is a live meter, not a snapshot)
    //   page 18 how many stored rows the reader's walk has stepped through:
    //           a live counter flickers, a stuck walk sits still
    //------------------------------------------------------------------
    // what the loader has actually pulled off the card: on a good card this is
    // the length the scan worked out from the header (0x709 for a 640x480 24
    // bit BMP), and a wrong parse would show up here as a wrong count
    wire [11:0] ld_sectors_disp = (ld_sectors_read > 16'h0FFF) ? 12'hFFF
                                                               : ld_sectors_read[11:0];
    // Where the loader was told to read.  Page 1 carries how many sectors it
    // transferred, which says nothing about *which* picture it was: that is
    // this address, and since every picture on the card is the same size, "one
    // picture further along" is a whole picture's worth of difference here.
    //
    // Shown in 4 KiB units (LBA >> 4): raw sectors saturated at the first
    // reading (the LBA was past 4095), and a partition offset of a few MiB eats
    // the whole 12 bit range before any picture difference shows.  One picture
    // is 1801 sectors = 112 (0x70) of these units, so the scale still separates
    // "picture 1" from "picture 2" with room to spare.
    wire [11:0] img_start_disp = (img_start_lba > 32'h0000_FFFF) ? 12'hFFF
                                                                : img_start_lba[15:4];

    reg [26:0] page_cnt;        // 2 s at 50 MHz
    // Five bits, not four: there are eighteen pages now, and a wrap written as
    // `page == 4'd16` truncates the literal to 4'd0 -- which pins the display to
    // page 0 instead of rotating (that happened on 2026-10-16: the board sat on
    // `C80`, page 0's own reading, forever).  Keep the labels wide enough too:
    // `4'd16` as a case label is 4'd0 as well, so page 16 would never show.
    localparam [4:0] PAGE_LAST = 5'd19;
    reg [4:0]  page;

    always @(posedge clk or negedge por_n) begin
        if (!por_n) begin
            page_cnt <= 27'd0;
            page     <= 4'd0;
        end else if (page_cnt == 27'd99_999_999) begin
            page_cnt <= 27'd0;
            // pages 15 and 16 are retired (see the page table): step over them
            page     <= (page == 4'd14)     ? 5'd17 :
                        (page == PAGE_LAST) ? 5'd0  :
                                              (page + 5'd1);
        end else begin
            page_cnt <= page_cnt + 27'd1;
        end
    end

    reg [3:0] pg_hi, pg_mid, pg_lo;

    always @(*) begin
        case (page)
            4'd0: begin
                pg_hi  = sd_phase;
                pg_mid = sd_images;
                pg_lo  = sd_error;
            end
            4'd1: begin
                // How many sectors the loader *really* pulled off the card.
                // It is anchored on the length the scan worked out from the
                // header (1801 = 0x709 for a 640x480 24 bit BMP), so a wrong
                // parse shows up here as a wrong count rather than a wrong
                // length: this is the half of that pair worth watching now.
                pg_hi  = ld_sectors_disp[11:8];
                pg_mid = ld_sectors_disp[7:4];
                pg_lo  = ld_sectors_disp[3:0];
            end
            // (the scan's own length for picture 0 used to sit here; it has
            // been 0x709 in every test and what matters is that the loader
            // transfers that many, which is the page above)
            4'd2: begin
                pg_hi  = 4'hA;
                pg_mid = sd_capacity[7:4];
                pg_lo  = sd_capacity[3:0];
            end
            4'd3: begin
                // how far the load got (3 = finished), why it stopped if it
                // did (4 = the sector it was pointed at does not start with
                // "BM"), and how many of the first 255 pixel bytes it handed
                // over were 0xFF: a real picture has almost none, a blank area
                // is nothing else
                pg_hi  = ld_phase;
                pg_mid = ld_error;
                pg_lo  = ld_ff_first[7:4];
            end
            // The next three pages are one number: the raw 32 bit word the
            // store gave back for the word that was written into it.  A pass or
            // fail only rules out one assumption per board test; this rules out
            // several at once -- A5A55A5A is right, 5A5AA5A5 is the byte lanes
            // reversed, 00000000 never landed, FFFFFFFF is a floating bus, and
            // a different value each power up is too little timing margin.
            4'd4: begin
                pg_hi  = st_gotw_q[31:28];
                pg_mid = st_gotw_q[27:24];
                pg_lo  = st_gotw_q[23:20];
            end
            4'd5: begin
                pg_hi  = st_gotw_q[19:16];
                pg_mid = st_gotw_q[15:12];
                pg_lo  = st_gotw_q[11:8];
            end
            4'd6: begin
                pg_hi  = st_gotw_q[7:4];
                pg_mid = st_gotw_q[3:0];
                // which sample code read the word back whole, F if none did.
                // 0 = the design's own assumption; 1/2/3 = a cycle later / a
                // cycle earlier / two later on the 0 degree sample; 4..7 = the
                // same four offsets on the shifted-clock sample.
                pg_lo  = st_okw_q ? {1'b0, st_adjw_q} : 4'hF;
            end
            4'd7: begin
                // the burst port the picture reader uses: A/F for "the first
                // word delivered is the first word asked for", how far it came
                // out shifted (F = it is not even one of the written patterns),
                // and the offset that lines it up
                pg_hi  = st_okb_q ? 4'hA : 4'hF;
                pg_mid = st_shb_q;
                pg_lo  = st_okb_q ? {1'b0, st_adjb_q} : 4'hF;
            end
            // ---- what the store actually holds in the picture --------------
            // Pages 4..8 prove the store works and the burst port samples it
            // right; they say nothing about the *contents* of the picture
            // region, because the word they test is written through the single
            // word port, not by the loader.  These four pages are that reading:
            // the first pixel (R,G,B) of the picture at word 0 and of the
            // picture's stored top row, fetched by the probe through the single
            // word port, i.e. a path that shares nothing with the reader.
            //
            // A store holding the picture answers with the file's own bytes; a
            // store holding a flat pattern answers with the same value twice; a
            // dead or wrong address answers with the probe's own pattern, all
            // ones, or zeros.
            4'd9: begin
                pg_hi  = pic_w0_q[23:20];
                pg_mid = pic_w0_q[19:16];
                pg_lo  = pic_w0_q[15:12];
            end
            4'd10: begin
                pg_hi  = pic_w0_q[11:8];
                pg_mid = pic_w0_q[7:4];
                pg_lo  = pic_w0_q[3:0];
            end
            4'd11: begin
                pg_hi  = pic_w1_q[23:20];
                pg_mid = pic_w1_q[19:16];
                pg_lo  = pic_w1_q[15:12];
            end
            4'd12: begin
                pg_hi  = pic_w1_q[11:8];
                pg_mid = pic_w1_q[7:4];
                pg_lo  = pic_w1_q[3:0];
            end
            // Is anything *moving* on the way from the store to the screen?
            // d4 d3 = the reader's incoming words (changes among the last 4096,
            // /256): 0F..10 = nearly every word differs, 00 = one value over and
            // over again.  d2 = A when a full window of words arrived in the
            // last quarter second, F when the words stopped coming at all.
            5'd13: begin
                pg_hi  = brd_disp[7:4];
                pg_mid = brd_disp[3:0];
                pg_lo  = brd_flag ? 4'hA : 4'hF;
            end
            // Which picture is the loader actually pointed at?  This is the
            // scan's first table entry -- the address, not the length -- and
            // with every 640x480 picture being exactly 0x709 sectors long, one
            // picture's worth of difference here is exactly a difference of
            // 0x709.  It is the only reading that can tell "we loaded picture
            // 1" from "we loaded picture 2", and pages 9..12 only say what
            // came off whatever address this is.
            4'd14: begin
                pg_hi  = img_start_disp[11:8];
                pg_mid = img_start_disp[7:4];
                pg_lo  = img_start_disp[3:0];
            end
            // Pages 15 and 16 used to hold two samples of the signal on the way
            // to the transmitter.  They were removed: the sampling instants did
            // not line up with the pixel they were supposed to capture, so the
            // readings belonged to nothing (docs/路线图.md, 2026-10-17) -- and a
            // meter that can be wrong is worse than no meter.  The rotation
            // skips them; the numbers below kept their page numbers so nothing
            // else had to be renumbered.
            // Is that same stream as fresh as the reader says it is?  11A (282)
            // = every displayed pixel is a fresh picture pixel; much smaller =
            // the screen is being fed repeats.
            5'd17: begin
                pg_hi  = voc_last[11:8];
                pg_mid = voc_last[7:4];
                pg_lo  = voc_last[3:0];
            end
            // Is the reader's walk still moving?  A row advance about every
            // 35 us makes the low bits flicker; a stuck walk sits on one value.
            5'd18: begin
                pg_hi  = rd_rows_seen[11:8];
                pg_mid = rd_rows_seen[7:4];
                pg_lo  = rd_rows_seen[3:0];
            end
            // Who is waiting on whom.  d4 d3 = words sitting in the reader's
            // buffer, /16 (0x00 empty .. 0x10 full).  d2 = {mstate, app_ok,
            // br_busy}: mstate 0 = idle, 1 = waiting for burst words, 2 =
            // flushing.  Full buffer + idle + not allowed to fetch means the
            // *pixel* side stopped taking words; empty buffer + mstate 1 means
            // the reader is waiting for the memory side.
            5'd19: begin
                pg_hi  = rd_used[8:5];
                pg_mid = rd_used[4:1];
                pg_lo  = {rd_mstate, rd_app_ok, br_busy};
            end
            default: begin
                // A = the data bus read back the value the controller was
                // itself driving (so the capture path is not blind), F = it did
                // not.  This one digit splits "our input path sees nothing"
                // from "the device never answers" -- both look like all ones
                // from the read data alone.
                pg_hi  = st_ech_q ? 4'hA : 4'hF;
                pg_mid = ld_overflow;
                pg_lo  = st_un_s2[15:8];
            end
        endcase
    end

    //------------------------------------------------------------------
    // status display
    //
    // Six hex digits, most significant is digit 5:
    //   digit 5  state code     2 = video path ready (see doc/路线图.md)
    //   digit 4  \
    //   digit 3   > storage, rotating through three pages every two seconds
    //   digit 2  /  (see the page table above)
    //   digit 1  heartbeat, 0..F at 4 Hz
    //   digit 0  frame counter, pixel domain
    //
    // It runs on the 50 MHz oscillator alone and is reset only by the
    // power-on reset, so it keeps reporting even when the video or storage
    // path is broken.
    //------------------------------------------------------------------
    localparam [3:0] STATE_VIDEO_OK   = 4'h2;   // pixel domain is producing frames
    localparam [3:0] STATE_VIDEO_DEAD = 4'hE;   // no frames at all

    //------------------------------------------------------------------
    // video domain liveness
    //
    // The heartbeat on digit 1 runs on the 50 MHz oscillator, so it only
    // proves the main clock is alive.  Digit 0 counts frames from the pixel
    // clock domain, which is what separates the two failure modes: no pixel
    // clock at all, versus a working video path whose HDMI link is broken.
    //------------------------------------------------------------------
    reg [5:0] frame_div;
    reg [3:0] frame_tick;
    reg       frames_seen;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            frame_div   <= 6'd0;
            frame_tick  <= 4'd0;
            frames_seen <= 1'b0;
        end else if (frame_first) begin
            frames_seen <= 1'b1;
            // one step per second, deliberately much slower than the main
            // clock heartbeat on digit 1 so the two are easy to tell apart
            if (frame_div == 6'd59) begin
                frame_div  <= 6'd0;
                frame_tick <= frame_tick + 4'd1;
            end else begin
                frame_div <= frame_div + 6'd1;
            end
        end
    end

    // these change far slower than the display scan, so a two stage
    // synchroniser into the 50 MHz domain is enough
    reg [3:0] frame_tick_s1, frame_tick_s2;
    reg       frames_seen_s1, frames_seen_s2;

    always @(posedge clk or negedge por_n) begin
        if (!por_n) begin
            frame_tick_s1  <= 4'd0;
            frame_tick_s2  <= 4'd0;
            frames_seen_s1 <= 1'b0;
            frames_seen_s2 <= 1'b0;
        end else begin
            frame_tick_s1  <= frame_tick;
            frame_tick_s2  <= frame_tick_s1;
            frames_seen_s1 <= frames_seen;
            frames_seen_s2 <= frames_seen_s1;
        end
    end

    reg [23:0] hb_div;
    reg [3:0]  heartbeat;

    always @(posedge clk or negedge por_n) begin
        if (!por_n) begin
            hb_div    <= 24'd0;
            heartbeat <= 4'd0;
        end else if (hb_div == 24'd12_499_999) begin       // 4 Hz at 50 MHz
            hb_div    <= 24'd0;
            heartbeat <= heartbeat + 4'd1;
        end else begin
            hb_div <= hb_div + 24'd1;
        end
    end

    wire [23:0] seg_value = {
        frames_seen_s2 ? STATE_VIDEO_OK : STATE_VIDEO_DEAD,   // [23:20] state
        pg_hi,                  // [19:16] digit 4, depends on the page
        pg_mid,                 // [15:12] digit 3, depends on the page
        pg_lo,                  // [11:8]  digit 2, depends on the page
        heartbeat,              // [7:4]   50 MHz heartbeat
        frame_tick_s2           // [3:0]   frame counter, pixel domain
    };

    mm_seg_display #(
        .CLK_FREQ_HZ (50_000_000),
        .SCAN_HZ     (200)
    ) u_seg_display (
        .clk      (clk),
        .rst_n    (por_n),
        .value    (seg_value),
        .seg_sel  (seg_sel),
        .seg_data (seg_data)
    );

endmodule

