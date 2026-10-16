//=============================================================================
// tb_sdram_selftest.v -- the probe asks the store and reports what came back
//
//   1. the word came back and matches: ok_word, the raw value, the byte mask
//   2. it really is in the model's array -- comparing only against the probe's
//      own copy would pass even if nothing was stored
//   3. the burst port (the one the picture reader uses) reads its four words
//      back whole, and they are in the array too
//   4. the device model saw no protocol violation
//=============================================================================
`timescale 1ns / 100ps

module tb_sdram_selftest;

    localparam [20:0] P_ADDR = 21'h1F_FFFF;
    localparam [31:0] P_DATA = 32'hA5A5_5A5A;

    reg clk, clk_sft, rst;

    initial clk     = 1'b0;
    initial clk_sft = 1'b1;
    always #8 clk     = ~clk;           // 62.5 MHz, memory side
    always #8 clk_sft = ~clk_sft;       // ... 180 degrees, to the device

    wire        ready;
    wire [2:0]  st_rd_adj, st_br_adj;
    wire [20:0] st_addr;
    wire [31:0] st_wdata;
    wire [3:0]  st_wdm;
    wire        st_wr_req, st_wr_done;
    wire        st_rd_req, st_rd_dv;
    wire [31:0] st_rdata;
    wire [20:0] st_b_addr;
    wire [8:0]  st_b_len;
    wire        st_b_req, st_b_dv;
    wire [31:0] st_b_data;
    wire        st_busy, st_done;
    wire        st_ok_word, st_ok_burst;
    wire [31:0] st_got_word;
    wire [3:0]  st_lanes_word, st_shift_burst;
    wire [2:0]  st_adj_word, st_adj_burst;
    wire [7:0]  st_top_burst;

    integer errors, k;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    mm_sdram #(
        .INIT_WAIT      (50),
        .REFRESH_PERIOD (1953)
    ) u_ctrl (
        .clk        (clk),
        .clk_dev   (clk),
        .clk_sft    (clk_sft),
        .rst        (rst),
        .ready      (ready),
        .rd_adj     (st_rd_adj),
        .br_adj     (st_br_adj),
        .wr_req     (st_wr_req),
        .wr_addr    (st_addr),
        .wr_data    (st_wdata),
        .wr_dm      (st_wdm),
        .wr_done    (st_wr_done),
        .rd_req     (st_rd_req),
        .rd_addr    (st_addr),
        .rd_dv      (st_rd_dv),
        .rd_data    (st_rdata),
        .init_done  (),
        .state_dbg  (),
        .echo_ok   (),
        .br_req     (st_b_req),
        .br_addr    (st_b_addr),
        .br_len     (st_b_len),
        .br_dv      (st_b_dv),
        .br_data    (st_b_data),
        .br_busy    ()
    );

    mm_sdram_selftest #(
        .PROBE_ADDR (P_ADDR),
        .PROBE_DATA (P_DATA)
    ) dut (
        .clk         (clk),
        .rst         (rst),
        .ready       (ready),
        .addr        (st_addr),
        .wdata       (st_wdata),
        .wdm         (st_wdm),
        .wr_req      (st_wr_req),
        .wr_done     (st_wr_done),
        .rd_req      (st_rd_req),
        .rd_dv       (st_rd_dv),
        .rdata       (st_rdata),
        .b_addr      (st_b_addr),
        .b_len       (st_b_len),
        .b_req       (st_b_req),
        .b_dv        (st_b_dv),
        .b_data      (st_b_data),
        .rd_adj      (st_rd_adj),
        .br_adj      (st_br_adj),
        .busy        (st_busy),
        .done        (st_done),
        .ok_word     (st_ok_word),
        .got_word    (st_got_word),
        .lanes_word  (st_lanes_word),
        .adj_word    (st_adj_word),
        .ok_burst    (st_ok_burst),
        .shift_burst (st_shift_burst),
        .adj_burst   (st_adj_burst),
        .top_burst   (st_top_burst),
        // The probe asks whether the picture has landed before it settles into
        // holding its answer.  There is no picture in this testbench, and the
        // probe *compares* this input to pick its next state: leaving it
        // unconnected made it a floating z, the comparison came out unknown and
        // the state machine went with it, so the sweep never finished.  (The
        // real top level drives it.)
        .pic_go      (1'b0)
    );

    initial begin
        errors = 0;
        rst    = 1'b1;
        repeat (8) @(posedge clk);
        rst = 1'b0;

        k = 0;
        while (!st_done && (k < 20000)) begin
            @(posedge clk);
            k = k + 1;
        end

        if (k >= 20000) begin
            fail("the probe never finished");
        end else begin
            // the raw value is the point of this probe, so report it either way
            $display("  single word  : wrote %h, read back %h, lanes %b",
                     P_DATA, st_got_word, st_lanes_word);

            if (!st_ok_word) begin
                $display("  FAIL: the single word probe reported failure");
                errors = errors + 1;
            end else if (st_got_word !== P_DATA) begin
                $display("  FAIL: read back %h, expected %h", st_got_word, P_DATA);
                errors = errors + 1;
            end else if (st_lanes_word !== 4'hF) begin
                fail("all four bytes should have matched");
            end

            // not just the probe's own copy: the model's array must hold it
            if (u_ctrl.u_sdram.mem[P_ADDR] !== P_DATA) begin
                $display("  FAIL: the store holds %h, expected %h",
                         u_ctrl.u_sdram.mem[P_ADDR], P_DATA);
                errors = errors + 1;
            end else begin
                $display("  stored       : PASS (the word is in the array)");
            end

            $display("  offsets      : single word %0d, burst %0d (shift %h)",
                     st_adj_word, st_adj_burst, st_shift_burst);

            if (!st_ok_burst) begin
                $display("  FAIL: the burst probe could not line the words up, shift %h, top byte %h",
                         st_shift_burst, st_top_burst);
                errors = errors + 1;
            end else if (st_shift_burst !== 4'd0) begin
                fail("the aligned burst should report a shift of zero");
            end else begin
                $display("  burst probe  : PASS (lined up at offset %0d, top byte %h)",
                         st_adj_burst, st_top_burst);
            end

            // No assertion on *which* code wins: that is exactly the thing the
            // board gets to decide, and in simulation the model decides it by
            // its own launch edge.  What must hold is that a code was found and
            // that the value reported for it is the value that was written.

            for (k = 0; k < 4; k = k + 1) begin
                if (u_ctrl.u_sdram.mem[21'h1F_FE00 + k] !==
                    (32'hA5A5_0000 | (32'd1 << (4 * k)))) begin
                    $display("  FAIL: burst word %0d in the store is %h",
                             k, u_ctrl.u_sdram.mem[21'h1F_FE00 + k]);
                    errors = errors + 1;
                end
            end

            if (u_ctrl.u_sdram.errors !== 0) begin
                $display("  FAIL: the SDRAM model reported %0d violations",
                         u_ctrl.u_sdram.errors);
                errors = errors + 1;
            end else begin
                $display("  protocol     : PASS");
            end
        end

        if (errors == 0)
            $display("RESULT: PASS -- SDRAM probe verified");
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
