//=============================================================================
// tb_afifo.v -- self-checking testbench for mm_afifo
//
//  1. words written with the read side idle come back in order
//  2. the FIFO fills to exactly its depth and no further
//  3. reading it empty does not advance or corrupt anything
//  4. the two sides agree on the number of words in flight when idle
//  5. a long run that wraps the pointers many times loses nothing and keeps
//     the order (the write and read clocks are deliberately unrelated:
//     10 ns against 26 ns)
//=============================================================================
`timescale 1ns / 100ps

module tb_afifo;

    localparam integer DW = 32;
    localparam integer AW = 4;              // 16 words deep
    localparam integer DEPTH = 1 << AW;

    reg           wclk, rclk;
    reg           wrst, rrst;
    reg           winc;
    reg  [DW-1:0] wdata;
    wire          wfull;
    wire [AW:0]   wused;
    reg           rinc;
    wire [DW-1:0] rdata;
    wire          rempty;
    wire [AW:0]   rused;

    integer errors;
    integer i;
    integer accepted;
    reg [DW-1:0] got;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial wclk = 1'b0;
    initial rclk = 1'b0;
    always #5  wclk = ~wclk;        // 10 ns
    always #13 rclk = ~rclk;        // 26 ns

    mm_afifo #(
        .DW (DW),
        .AW (AW)
    ) dut (
        .wclk  (wclk), .wrst (wrst), .winc (winc), .wdata (wdata),
        .wfull (wfull), .wused (wused),
        .rclk  (rclk), .rrst (rrst), .rinc (rinc),
        .rdata (rdata), .rempty (rempty), .rused (rused)
    );

    task wr;
        input [DW-1:0] d;
        begin
            @(negedge wclk);
            wdata = d;
            winc  = 1'b1;
            @(negedge wclk);
            winc  = 1'b0;
        end
    endtask

    task rd;
        output [DW-1:0] d;
        begin
            @(negedge rclk);
            while (rempty)
                @(negedge rclk);
            rinc = 1'b1;
            d    = rdata;
            @(negedge rclk);
            rinc = 1'b0;
        end
    endtask

    integer errors_before;

    initial begin
        errors = 0;
        wrst   = 1'b1;
        rrst   = 1'b1;
        winc   = 1'b0;
        rinc   = 1'b0;
        wdata  = 32'd0;

        repeat (4) @(posedge wclk);
        repeat (4) @(posedge rclk);
        wrst = 1'b0;
        rrst = 1'b0;
        repeat (4) @(posedge wclk);
        repeat (4) @(posedge rclk);

        // ---- 1. fill halfway, drain, check the order -------------------
        errors_before = errors;
        for (i = 0; i < 8; i = i + 1)
            wr(32'h1000_0000 + i);
        for (i = 0; i < 8; i = i + 1) begin
            rd(got);
            if (got !== (32'h1000_0000 + i)) begin
                $display("  FAIL: word %0d read %h, expected %h",
                         i, got, 32'h1000_0000 + i);
                errors = errors + 1;
            end
        end
        if (errors == errors_before)
            $display("  order        : PASS (8 words, write clock idle reader)");

        // ---- 2. fill to the brim --------------------------------------
        errors_before = errors;
        accepted = 0;
        while (!wfull && (accepted < 100)) begin
            @(negedge wclk);
            wdata = 32'h2000_0000 + accepted;
            winc  = 1'b1;
            @(negedge wclk);
            winc  = 1'b0;
            accepted = accepted + 1;
        end
        // let the flags settle
        repeat (6) @(posedge wclk);
        repeat (6) @(posedge rclk);
        if (wfull !== 1'b1) fail("the FIFO never reported full");
        if (accepted !== DEPTH) begin
            $display("  FAIL: accepted %0d words into a %0d deep FIFO",
                     accepted, DEPTH);
            errors = errors + 1;
        end
        if (wused !== DEPTH) begin
            $display("  FAIL: used = %0d, expected %0d", wused, DEPTH);
            errors = errors + 1;
        end
        // and the contents must be exactly what went in
        for (i = 0; i < DEPTH; i = i + 1) begin
            rd(got);
            if (got !== (32'h2000_0000 + i)) begin
                $display("  FAIL: full-run word %0d read %h, expected %h",
                         i, got, 32'h2000_0000 + i);
                errors = errors + 1;
            end
        end
        if (errors == errors_before)
            $display("  full depth   : PASS (%0d words exactly, contents intact)", DEPTH);

        // ---- 3. empty is empty ----------------------------------------
        errors_before = errors;
        repeat (6) @(posedge rclk);
        repeat (6) @(posedge wclk);
        if (rempty !== 1'b1) fail("the FIFO did not report empty");
        if (rused !== 0) fail("rused is not zero when empty");
        if (wused !== 0) fail("wused is not zero when empty");
        // a stray read while empty must not move the pointer
        @(negedge rclk);
        rinc = 1'b1;
        @(negedge rclk);
        rinc = 1'b0;
        wr(32'h3000_0000);
        rd(got);
        if (got !== 32'h3000_0000) fail("a read while empty corrupted the stream");
        if (errors == errors_before)
            $display("  empty        : PASS (flags and counts correct, no corruption)");

        // ---- 4. long run, pointers wrap many times --------------------
        errors_before = errors;
        fork_emulator_write;
        if (errors == errors_before)
            $display("  wraparound   : PASS (200 words through a %0d deep FIFO)", DEPTH);

        if (errors == 0)
            $display("RESULT: PASS -- asynchronous FIFO verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    // 200 words written while a reader drains them one at a time, in a
    // separate process so both sides really overlap.  Verilog-2001 has no
    // fork/join in this toolchain, so the reader is a clocked process that
    // pops whenever it can and checks the value it gets.
    integer wsrc;
    integer rsrc;
    reg     run_long;

    task fork_emulator_write;
        begin
            wsrc = 0;
            rsrc = 0;
            run_long = 1'b1;
            // the writer: keep pushing until 200 words are in
            while (wsrc < 200) begin
                @(negedge wclk);
                if (!wfull) begin
                    wdata = 32'h5000_0000 + wsrc;
                    winc  = 1'b1;
                    wsrc  = wsrc + 1;
                    @(negedge wclk);
                    winc  = 1'b0;
                end
            end
            // wait for the reader to finish the last few
            while (rsrc < 200)
                @(negedge rclk);
            run_long = 1'b0;
        end
    endtask

    always @(negedge rclk) begin
        if (run_long) begin
            if (!rempty) begin
                rinc = 1'b1;
                if (rdata !== (32'h5000_0000 + rsrc)) begin
                    errors = errors + 1;
                    $display("  FAIL: wrap run word %0d read %h, expected %h",
                             rsrc, rdata, 32'h5000_0000 + rsrc);
                end
                rsrc = rsrc + 1;
            end else begin
                rinc = 1'b0;
            end
        end
    end

    initial begin
        #5_000_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
