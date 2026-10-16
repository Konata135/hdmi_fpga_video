//=============================================================================
// tb_hdmi_tx.v -- self-checking testbench for the self-written HDMI link
//
// Checks:
//   A. period structure of every pixel of every line
//      - active pixels carry video characters that decode back to the RGB
//        values that were fed in
//      - every blanking pixel carries a control character: {VSYNC, HSYNC} on
//        channel 0 and 0000 on channels 1 and 2
//      - there is no guard band and no preamble anywhere
//   B. clock lane: the serial stream repeats with a 10 bit period and five
//      ones per period, i.e. a 25 MHz square wave
//   C. serialiser bit order: the character is emitted least significant bit
//      first, which is what a TMDS sink expects
//=============================================================================
`timescale 1ns / 100ps

module tb_hdmi_tx;

    localparam integer H_ACTIVE = 640;
    localparam integer V_ACTIVE = 480;
    localparam integer H_TOTAL  = 800;
    localparam integer V_TOTAL  = 525;
    localparam integer FRAMES   = 1;

    integer errors;
    integer fail_shown;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            if (fail_shown < 20) begin
                fail_shown = fail_shown + 1;
                $display("  FAIL: %0s", msg);
            end
        end
    endtask

    //------------------------------------------------------------------
    // clocks: 25 MHz pixel clock and its phase locked 125 MHz serial clock
    //------------------------------------------------------------------
    reg pix_clk;
    reg pix_5x_clk;
    reg rst;

    // 40 ns pixel period and 8 ns serial period.  The serial clock is offset
    // by 2 ns so that no serial edge ever coincides with a pixel edge, which
    // mirrors the fixed but arbitrary phase relationship of the two PLL
    // outputs on the device.
    initial pix_clk = 1'b0;
    always #20 pix_clk = ~pix_clk;                     // rises at 20, 60, 100 ...

    initial begin
        pix_5x_clk = 1'b0;
        #2;
        forever #4 pix_5x_clk = ~pix_5x_clk;           // rises at 2, 10, 18 ...
    end

    initial begin
        rst = 1'b1;
        repeat (12) @(posedge pix_clk);
        rst = 1'b0;
    end

    //------------------------------------------------------------------
    // video timing and the picture the transmitter has to send
    //------------------------------------------------------------------
    wire [11:0] h_cnt;
    wire [10:0] v_cnt;
    wire        hs, vs, de, frame_first, line_last;

    mm_video_timing #(
        .H_ACTIVE (H_ACTIVE),
        .H_FP     (16),
        .H_SYNC   (96),
        .H_BP     (48),
        .V_ACTIVE (V_ACTIVE),
        .V_FP     (10),
        .V_SYNC   (2),
        .V_BP     (33)
    ) dut_timing (
        .pix_clk     (pix_clk),
        .rst         (rst),
        .h_cnt       (h_cnt),
        .v_cnt       (v_cnt),
        .hs          (hs),
        .vs          (vs),
        .de          (de),
        .frame_first (frame_first),
        .line_last   (line_last)
    );

    // a colour that changes with the pixel position, so a swapped or lost
    // pixel cannot go unnoticed
    wire [23:0] rgb = {h_cnt[7:0], v_cnt[7:0], 8'h5A};

    wire tmds_clk_p, tmds_d0_p, tmds_d1_p, tmds_d2_p;

    mm_hdmi_tx dut_tx (
        .pix_clk    (pix_clk),
        .pix_5x_clk (pix_5x_clk),
        .rst        (rst),
        .de         (de),
        .hs         (hs),
        .vs         (vs),
        .rgb        (rgb),
        .tmds_clk_p (tmds_clk_p),
        .tmds_d0_p  (tmds_d0_p),
        .tmds_d1_p  (tmds_d1_p),
        .tmds_d2_p  (tmds_d2_p)
    );

    //------------------------------------------------------------------
    // reference functions
    //------------------------------------------------------------------
    integer k;

    function [9:0] ctl_char;
        input [1:0] c;
        begin
            case (c)
                2'b00:   ctl_char = 10'b1101010100;
                2'b01:   ctl_char = 10'b0010101011;
                2'b10:   ctl_char = 10'b0101010100;
                default: ctl_char = 10'b1010101011;
            endcase
        end
    endfunction

    function [7:0] tmds_decode;
        input [9:0] code;
        reg [7:0] q_m;
        reg       q_m8;
        reg [7:0] d;
        begin
            q_m8 = code[8];
            q_m  = code[9] ? ~code[7:0] : code[7:0];
            d[0] = q_m[0];
            for (k = 1; k < 8; k = k + 1)
                d[k] = q_m8 ? (q_m[k] ^ q_m[k-1])
                            : (~q_m[k] ^ q_m[k-1]);
            tmds_decode = d;
        end
    endfunction

    // expected control value of channel 0 for a blanking pixel; the blanking
    // interval carries plain control characters and nothing else
    function [9:0] exp_char0;
        input        hs_i;
        input        vs_i;
        begin
            exp_char0 = ctl_char({vs_i, hs_i});
        end
    endfunction

    //------------------------------------------------------------------
    // A. per pixel structural check
    //------------------------------------------------------------------
    reg        chk_de;
    reg        chk_valid;
    reg [23:0] chk_rgb;
    reg [9:0]  chk_e0, chk_ex;
    integer    px_checked;
    integer    blank_seen;

    always @(posedge pix_clk) begin
        if (rst) begin
            // do not evaluate a pixel that was presented while the design was
            // still held in reset
            chk_de  <= 1'b0;
            chk_rgb <= 24'd0;
            chk_e0  <= 10'd0;
            chk_ex  <= 10'd0;
            chk_valid <= 1'b0;
        end else begin
            chk_valid <= 1'b1;
            chk_de  <= de;
            chk_rgb <= rgb;
            chk_e0  <= exp_char0(hs, vs);
            chk_ex  <= ctl_char(2'b00);      // channels 1 and 2 always carry 0000
            // the blanking interval is nothing but control characters, so its
            // size is a fixed number of pixels per frame
            if (!de) blank_seen <= blank_seen + 1;
        end
    end

    always @(posedge pix_clk) begin
        #1;
        if (!rst && chk_valid) begin
            px_checked = px_checked + 1;
            if (chk_de) begin
                if (tmds_decode(dut_tx.u_enc_blue.dout) !== chk_rgb[7:0])
                    fail("blue video character does not decode back");
                if (tmds_decode(dut_tx.u_enc_green.dout) !== chk_rgb[15:8])
                    fail("green video character does not decode back");
                if (tmds_decode(dut_tx.u_enc_red.dout) !== chk_rgb[23:16])
                    fail("red video character does not decode back");
            end else begin
                if (dut_tx.u_enc_blue.dout !== chk_e0)
                    fail("blue control character wrong");
                if (dut_tx.u_enc_green.dout !== chk_ex)
                    fail("green control character wrong");
                if (dut_tx.u_enc_red.dout !== chk_ex)
                    fail("red control character wrong");
            end
        end
    end

    //------------------------------------------------------------------
    // B. clock lane: ten bit period, five ones
    //------------------------------------------------------------------
    reg [639:0] clk_stream;
    integer     clk_bits;
    integer     i;
    integer     ones_in_window;

    always @(pix_5x_clk) begin
        #1;
        clk_stream = {clk_stream[638:0], tmds_clk_p};
        clk_bits   = clk_bits + 1;
    end

    //------------------------------------------------------------------
    // C. serialiser bit order, driven by a one-hot sweep
    //------------------------------------------------------------------
    reg  [3:0]  oh_cnt;
    reg  [9:0]  oh_char;
    wire        oh_serial;

    always @(posedge pix_clk or posedge rst) begin
        if (rst)
            oh_cnt <= 4'd0;
        else if (oh_cnt == 4'd9)
            oh_cnt <= 4'd0;
        else
            oh_cnt <= oh_cnt + 4'd1;
    end

    always @(*) begin
        case (oh_cnt)
            4'd0: oh_char = 10'b0000000001;
            4'd1: oh_char = 10'b0000000010;
            4'd2: oh_char = 10'b0000000100;
            4'd3: oh_char = 10'b0000001000;
            4'd4: oh_char = 10'b0000010000;
            4'd5: oh_char = 10'b0000100000;
            4'd6: oh_char = 10'b0001000000;
            4'd7: oh_char = 10'b0010000000;
            4'd8: oh_char = 10'b0100000000;
            default: oh_char = 10'b1000000000;
        endcase
    end

    mm_hdmi_serializer u_ser_order (
        .serial_clk (pix_5x_clk),
        .rst        (rst),
        .char_in    (oh_char),
        .serial_out (oh_serial)
    );

    reg [1023:0] oh_stream;
    integer      oh_bits;

    always @(pix_5x_clk) begin
        #1;
        oh_stream = {oh_stream[1022:0], oh_serial};
        oh_bits   = oh_bits + 1;
    end

    //------------------------------------------------------------------
    // run control
    //------------------------------------------------------------------
    integer frame_cnt;
    integer prev_p;
    integer p;
    integer bad_order;
    integer dbg_n;

    // frame_first is also asserted while the design is held in reset, so the
    // frame counter has to ignore those clock cycles
    always @(posedge pix_clk) begin
        if (rst)
            frame_cnt <= 0;
        else if (frame_first)
            frame_cnt <= frame_cnt + 1;
    end

    initial begin
        errors      = 0;
        fail_shown  = 0;
        chk_de      = 1'b0;
        chk_rgb     = 24'd0;
        chk_e0      = 10'd0;
        chk_ex      = 10'd0;
        px_checked  = 0;
        blank_seen  = 0;
        clk_stream  = 640'd0;
        clk_bits    = 0;
        oh_stream   = 1024'd0;
        oh_bits     = 0;
        frame_cnt   = 0;
        dbg_n       = 0;

        // frame_cnt reaches 1 at the start of frame 0, so wait for the start
        // of frame FRAMES to have FRAMES complete frames behind us
        wait (frame_cnt >= FRAMES + 1);

        // ---- A ------------------------------------------------------
        $display("  A. pixels checked: %0d, of which blanking: %0d",
                 px_checked, blank_seen);
        if (px_checked < FRAMES * H_TOTAL * V_TOTAL - 10)
            fail("not enough pixels were checked");
        // every non active pixel of every line must have been a control pixel
        if (blank_seen !== FRAMES * H_TOTAL * V_TOTAL - FRAMES * H_ACTIVE * V_ACTIVE)
            fail("blanking pixel count does not match the active area geometry");

        // ---- B ------------------------------------------------------
        $display("  B. clock lane bits captured: %0d", clk_bits);
        if (clk_bits < 640)
            fail("clock lane captured too few bits");
        for (i = 200; i < 630; i = i + 1) begin
            if (clk_stream[i] !== clk_stream[i+10]) begin
                fail("clock lane period is not ten bits");
                i = 630;
            end
        end
        ones_in_window = 0;
        for (i = 200; i < 210; i = i + 1)
            ones_in_window = ones_in_window + clk_stream[i];
        if (ones_in_window != 5)
            fail("clock lane does not carry five ones per period");

        // ---- C ------------------------------------------------------
        $display("  C. serialiser bits captured: %0d", oh_bits);
        prev_p     = -100;
        bad_order  = 0;
        for (i = 100; i < 1000; i = i + 1) begin
            if (oh_stream[i] === 1'b1) begin
                p = i;
                if (prev_p >= 0) begin
                    // one-hot sweep: the position inside the ten bit
                    // character must advance by one each time
                    if ((p % 10) !== ((prev_p % 10) + 1) % 10)
                        bad_order = bad_order + 1;
                end
                prev_p = p;
            end
        end
        if (bad_order != 0) begin
            $display("  C. %0d out-of-order characters", bad_order);
            fail("serialiser is not least-significant-bit first");
        end

        if (errors == 0)
            $display("RESULT: PASS -- self-written HDMI link verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #100_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule

