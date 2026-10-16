//=============================================================================
// tb_tmds_encoder.v -- self-checking testbench for mm_tmds_encoder
//
// The encoder is exercised as a black box and checked with an independent
// inverse implementation of the specification:
//
//   1. round trip   : all 256 video bytes, and a long pseudo-random sequence,
//                     are decoded back from the 10-bit characters
//   2. control codes: all four control values map to the four characters the
//                     specification defines
//   3. disparity    : the running disparity stays bounded, and over a long
//                     run of input bytes the number of transmitted ones and
//                     zeros differ by at most a handful (DC balance)
//=============================================================================
`timescale 1ns / 100ps

module tb_tmds_encoder;

    reg        clk;
    reg        rst;
    reg        de;
    reg [7:0]  din;
    reg [1:0]  ctl;
    wire [9:0] dout;

    initial clk = 1'b0;
    always #5 clk = ~clk;       // 10 ns period

    mm_tmds_encoder dut (
        .clk  (clk),
        .rst  (rst),
        .de   (de),
        .din  (din),
        .ctl  (ctl),
        .dout (dout)
    );

    integer errors;
    integer i;
    integer j;
    integer k;
    integer ones_acc;
    integer zeros_acc;
    integer n1;
    integer n0;
    reg [7:0] dec;
    reg [7:0] expect;
    reg [7:0] lfsr;
    integer   disparity_model;
    integer   min_disp;
    integer   max_disp;
    integer   disp_dbg;

    //------------------------------------------------------------------
    // independent implementation of the inverse of the specification
    //------------------------------------------------------------------
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

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    //------------------------------------------------------------------
    // drive one byte and compare the registered character
    //------------------------------------------------------------------
    task send_video;
        input [7:0] byte_in;
        begin
            @(negedge clk);
            de  = 1'b1;
            din = byte_in;
            @(posedge clk);
            #1;
            dec = tmds_decode(dout);
            if (dec !== byte_in) begin
                errors = errors + 1;
                $display("  FAIL: byte %02x encoded to %010b, decoded to %02x",
                         byte_in, dout, dec);
            end
            n1 = 0;
            for (j = 0; j < 10; j = j + 1) n1 = n1 + dout[j];
            ones_acc  = ones_acc  + n1;
            zeros_acc = zeros_acc + (10 - n1);
            disparity_model = disparity_model + (n1 - (10 - n1));
            if (disparity_model < min_disp) min_disp = disparity_model;
            if (disparity_model > max_disp) max_disp = disparity_model;
            // the encoder keeps its own running disparity; it must agree with
            // the ones and zeros that actually left the encoder
            if ($signed(dut.disparity) !== disparity_model) begin
                if (disp_dbg < 6) begin
                    disp_dbg = disp_dbg + 1;
                    $display("  FAIL: byte %02x char %010b: encoder disparity %0d, transmitted disparity %0d",
                             byte_in, dout, $signed(dut.disparity), disparity_model);
                end
                errors = errors + 1;
            end
        end
    endtask

    task check_control;
        input [1:0]  ctl_in;
        input [9:0]  code_exp;
        begin
            @(negedge clk);
            de  = 1'b0;
            ctl = ctl_in;
            @(posedge clk);
            #1;
            if (dout !== code_exp) begin
                errors = errors + 1;
                $display("  FAIL: ctl %b gave %010b, expected %010b",
                         ctl_in, dout, code_exp);
            end
        end
    endtask

    initial begin
        errors          = 0;
        ones_acc        = 0;
        zeros_acc       = 0;
        disparity_model = 0;
        min_disp        = 0;
        max_disp        = 0;
        disp_dbg        = 0;
        de              = 1'b0;
        din             = 8'h00;
        ctl             = 2'b00;

        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        //---- 1. control characters -----------------------------------
        check_control(2'b00, 10'b1101010100);
        check_control(2'b01, 10'b0010101011);
        check_control(2'b10, 10'b0101010100);
        check_control(2'b11, 10'b1010101011);

        //---- 2. round trip over all 256 byte values -------------------
        for (i = 0; i < 256; i = i + 1)
            send_video(i[7:0]);

        //---- 3. long pseudo-random run for DC balance ----------------
        lfsr = 8'hA7;
        for (i = 0; i < 4096; i = i + 1) begin
            lfsr = {lfsr[6:0], lfsr[7] ^ lfsr[5] ^ lfsr[4] ^ lfsr[3]};
            send_video(lfsr);
        end

        $display("  bytes exercised : %0d", 256 + 4096);
        $display("  ones/zeros      : %0d / %0d", ones_acc, zeros_acc);
        $display("  model disparity : min %0d max %0d", min_disp, max_disp);

        if ((ones_acc - zeros_acc > 10) || (zeros_acc - ones_acc > 10))
            fail("DC balance drifted too far over the long run");
        if ((max_disp - min_disp) > 20)
            fail("running disparity excursion too large");

        if (errors == 0)
            $display("RESULT: PASS -- TMDS encoder verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule

