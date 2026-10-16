//=============================================================================
// tb_sdram.v -- self-checking testbench for mm_sdram
//
// The device is the behavioural model in mm_sdram_device.v, which also asserts
// the protocol rules it sees.  On top of "does the data come back", this
// checks:
//
//   1. the init sequence really is precharge, two refreshes, mode register
//   2. writes and reads at addresses that differ in bank, row and column all
//      land in their own place (i.e. the {bank,row,col} split is right)
//   3. the byte masks only write the bytes they should
//   4. a run of consecutive words walks across a column boundary and still
//      comes back in order
//   5. refreshes keep happening while the controller is working, and the data
//      survives them
//   6. the model saw no protocol violation at all
//=============================================================================
`timescale 1ns / 100ps

module tb_sdram;

    reg         clk;
    reg         clk_sft;
    reg         rst;
    wire        ready;
    reg         wr_req;
    reg  [20:0] wr_addr;
    reg  [31:0] wr_data;
    reg  [3:0]  wr_dm;
    wire        wr_done;
    reg         rd_req;
    reg  [20:0] rd_addr;
    wire        rd_dv;
    wire [31:0] rd_data;
    wire        init_done;
    wire [3:0]  state_dbg;
    wire        echo_ok;
    reg         br_req;
    reg  [20:0] br_addr;
    reg  [8:0]  br_len;
    wire        br_dv;
    wire [31:0] br_data;
    wire        br_busy;

    integer errors;
    integer spin;
    integer k;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    // 125 MHz logic clock, and the same clock shifted 180 degrees for the device
    initial clk     = 1'b0;
    initial clk_sft = 1'b1;
// 62.5 MHz: the memory domain was slowed down on the board to double the
// interface's timing margin (see mm_clk_gen), so the testbenches match it
always #8 clk     = ~clk;
always #8 clk_sft = ~clk_sft;

    mm_sdram #(
        .CAS_LAT        (3),
        .T_RP           (3),
        .T_RCD          (3),
        .T_RFC          (9),
        .T_MRD          (2),
        .T_WR           (2),
        .INIT_WAIT      (50),
        .REFRESH_PERIOD (60)      // same 960 ns as before, now at 16 ns a cycle
    ) dut (
        .clk        (clk),
        .clk_dev   (clk),
        .clk_sft    (clk_sft),
        .rst        (rst),
        .ready      (ready),
        .rd_adj    (/*the sample code the probe finds; the model wants one cycle later*/ 3'd1),
        .br_adj    ( 3'd1),
        .wr_req     (wr_req),
        .wr_addr    (wr_addr),
        .wr_data    (wr_data),
        .wr_dm      (wr_dm),
        .wr_done    (wr_done),
        .rd_req     (rd_req),
        .rd_addr    (rd_addr),
        .rd_dv      (rd_dv),
        .rd_data    (rd_data),
        .init_done  (init_done),
        .state_dbg  (state_dbg),
        .echo_ok   (echo_ok),
        .br_req     (br_req),
        .br_addr    (br_addr),
        .br_len     (br_len),
        .br_dv      (br_dv),
        .br_data    (br_data),
        .br_busy    (br_busy)
    );

    // helpers ------------------------------------------------------------
    task do_write;
        input [20:0] a;
        input [31:0] d;
        input [3:0]  dm;
        begin
            @(negedge clk);
            wr_addr = a;
            wr_data = d;
            wr_dm   = dm;
            // the request may only be presented in a cycle where the
            // controller is ready, and must be withdrawn in the next one
            wr_req  = 1'b0;
            while (!ready) @(negedge clk);
            wr_req = 1'b1;
            @(negedge clk);
            wr_req = 1'b0;
            spin = 0;
            while (!wr_done && (spin < 300)) begin
                @(posedge clk);
                spin = spin + 1;
            end
            if (spin >= 300) fail("a write never completed");
            @(negedge clk);
        end
    endtask

    task do_read;
        input  [20:0] a;
        output [31:0] d;
        begin
            @(negedge clk);
            rd_addr = a;
            rd_req  = 1'b0;
            while (!ready) @(negedge clk);
            rd_req = 1'b1;
            @(negedge clk);
            rd_req = 1'b0;
            spin = 0;
            while (!rd_dv && (spin < 300)) begin
                @(posedge clk);
                spin = spin + 1;
            end
            if (spin >= 300) fail("a read never completed");
            d = rd_data;
            @(negedge clk);
        end
    endtask

    reg [31:0] got;

    task check_word;
        input [20:0] a;
        input [31:0] expected;
        begin
            do_read(a, got);
            if (got !== expected) begin
                errors = errors + 1;
                $display("  FAIL: address %h read %h, expected %h",
                         a, got, expected);
            end
        end
    endtask

    // addresses that differ in bank, row and column only
    localparam [20:0] A_BANK0 = 21'h0_0000;     // bank 0 row 0    col 0
    localparam [20:0] A_COL   = 21'h0_000F;     // bank 0 row 0    col 15
    localparam [20:0] A_ROW   = 21'h0_0500;     // bank 0 row 5    col 0
    localparam [20:0] A_BANK  = 21'h2_0000;     // bank 2 row 0    col 0
    localparam [20:0] A_TOP   = 21'h7_0700;     // bank 3 row 7    col 0

    //------------------------------------------------------------------
    // scenario
    //------------------------------------------------------------------
    integer errors_before;
    integer got_n;
    integer burst_cycles;

    initial begin
        errors  = 0;
        rst     = 1'b1;
        wr_req  = 1'b0;
        wr_addr = 21'd0;
        wr_data = 32'd0;
        wr_dm   = 4'h0;
        rd_req  = 1'b0;
        rd_addr = 21'd0;
        br_req  = 1'b0;
        br_addr = 21'd0;
        br_len  = 9'd0;

        repeat (4) @(posedge clk);
        rst = 1'b0;

        // ---- 1. init ---------------------------------------------------
        errors_before = errors;
        spin = 0;
        while (!init_done && (spin < 1000)) begin
            @(posedge clk);
            spin = spin + 1;
        end
        if (!init_done) fail("init never finished");
        if (dut.u_sdram.n_pre !== 1) fail("init did not precharge exactly once");
        if (dut.u_sdram.n_ref !== 2) fail("init did not issue two auto refreshes");
        if (dut.u_sdram.n_mrs !== 1) fail("init did not load the mode register once");
        if (errors == errors_before)
            $display("  init         : PASS (1 precharge, 2 refreshes, 1 mode register)");

        // ---- 2. scattered addresses ------------------------------------
        errors_before = errors;
        do_write(A_BANK0, 32'h1111_0000, 4'h0);
        do_write(A_COL,   32'h2222_000F, 4'h0);
        do_write(A_ROW,   32'h3333_0500, 4'h0);
        do_write(A_BANK,  32'h4444_2000, 4'h0);
        do_write(A_TOP,   32'h5555_0700, 4'h0);
        check_word(A_BANK0, 32'h1111_0000);
        check_word(A_COL,   32'h2222_000F);
        check_word(A_ROW,   32'h3333_0500);
        check_word(A_BANK,  32'h4444_2000);
        check_word(A_TOP,   32'h5555_0700);
        if (errors == errors_before)
            $display("  bank/row/col : PASS (five addresses, five distinct words)");

        // ---- 3. byte masks ---------------------------------------------
        errors_before = errors;
        do_write(A_COL, 32'hAABB_CCDD, 4'h0);
        do_write(A_COL, 32'h1122_3344, 4'hE);   // only byte 0 is written
        check_word(A_COL, 32'hAABB_CC44);
        do_write(A_COL, 32'h5566_7788, 4'h7);   // only byte 3 is written
        check_word(A_COL, 32'h55BB_CC44);
        if (errors == errors_before)
            $display("  byte masks   : PASS (dm keeps the bytes it should)");

        // ---- 4. a run that crosses a column boundary --------------------
        errors_before = errors;
        for (k = 0; k < 16; k = k + 1)
            do_write(21'h0_00F8 + k[20:0], 32'hA000_0000 + k, 4'h0);
        for (k = 0; k < 16; k = k + 1)
            check_word(21'h0_00F8 + k[20:0], 32'hA000_0000 + k);
        if (errors == errors_before)
            $display("  column edge  : PASS (16 words across col 255 -> row 1)");

        // ---- 5. refreshes happen and the data survives ------------------
        errors_before = errors;
        for (k = 0; k < 400; k = k + 1)
            do_read(A_ROW, got);
        if (dut.u_sdram.n_ref < 5) fail("no refreshes happened during the run");
        check_word(A_BANK0, 32'h1111_0000);
        check_word(A_TOP,   32'h5555_0700);
        if (errors == errors_before)
            $display("  refresh      : PASS (%0d refreshes, data intact)",
                     dut.u_sdram.n_ref);

        // ---- 5b. burst read: back to back, not a round trip per word ----
        errors_before = errors;
        for (k = 0; k < 120; k = k + 1)
            do_write(21'h2_0200 + k, 32'hB000_0000 + k, 4'h0);
        br_addr = 21'h2_0200;
        br_len  = 9'd120;
        br_req  = 1'b1;
        @(negedge clk);
        br_req  = 1'b0;
        got_n        = 0;
        burst_cycles = 0;
        while ((got_n < 120) && (burst_cycles < 4000)) begin
            @(negedge clk);
            burst_cycles = burst_cycles + 1;
            if (br_dv) begin
                if (br_data !== (32'hB000_0000 + got_n)) begin
                    errors = errors + 1;
                    $display("  FAIL: burst word %0d = %h, expected %h",
                             got_n, br_data, 32'hB000_0000 + got_n);
                end
                got_n = got_n + 1;
            end
        end
        if (got_n !== 120) fail("the burst did not deliver 120 words");
        // one word per cycle plus the CAS latency; a per-word round trip would
        // be about 7 cycles each, i.e. some 840
        if (burst_cycles > 136) begin
            $display("  FAIL: 120 burst words took %0d cycles, expected about 124",
                     burst_cycles);
            errors = errors + 1;
        end
        if (errors == errors_before)
            $display("  burst        : PASS (120 words in %0d cycles, back to back)",
                     burst_cycles);

        // ---- 6. the device model saw no protocol violation --------------
        // and the input side saw at least one value the controller was itself
        // driving out: if it never does, no read timing work would help
        if (!echo_ok) begin
            $display("  FAIL: the data bus never read back what the controller drove");
            errors = errors + 1;
        end else begin
            $display("  echo         : PASS (the input side sees our own drive)");
        end

        if (dut.u_sdram.errors !== 0) begin
            $display("  FAIL: the device model reported %0d protocol violations",
                     dut.u_sdram.errors);
            errors = errors + 1;
        end else begin
            $display("  protocol     : PASS (%0d activates, %0d reads, %0d writes)",
                     dut.u_sdram.n_act, dut.u_sdram.n_rd, dut.u_sdram.n_wr);
        end

        if (errors == 0)
            $display("RESULT: PASS -- SDRAM controller verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
