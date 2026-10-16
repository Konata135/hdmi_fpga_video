//=============================================================================
// tb_spi_master.v -- self-checking testbench for mm_spi_master
//
// A behavioural slave drives MISO and watches MOSI, so both directions are
// checked independently of the master's own shift registers.
//
// Checks:
//   1. SCLK half period equals clk_div clock cycles, and SCLK idles low
//   2. MOSI is most significant bit first and changes while SCLK is low
//   3. the byte the slave sends is received intact
//   4. busy is asserted for the transfer and done pulses exactly once
//   5. back-to-back transfers work
//=============================================================================
`timescale 1ns / 100ps

module tb_spi_master;

    reg         clk;
    reg         rst;
    reg  [15:0] clk_div;
    reg         start;
    reg  [7:0]  tx_byte;
    wire [7:0]  rx_byte;
    wire        busy;
    wire        done;
    wire        sclk;
    wire        mosi;
    wire        miso;

    integer errors;

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    initial clk = 1'b0;
    always #5 clk = ~clk;       // 10 ns period

    mm_spi_master dut (
        .clk     (clk),
        .rst     (rst),
        .clk_div (clk_div),
        .start   (start),
        .tx_byte (tx_byte),
        .rx_byte (rx_byte),
        .busy    (busy),
        .done    (done),
        .sclk    (sclk),
        .mosi    (mosi),
        .miso    (miso)
    );

    //------------------------------------------------------------------
    // behavioural slave: shifts MSB first on the falling edge
    //------------------------------------------------------------------
    reg [7:0] slv_sh;
    reg [7:0] slv_rx;
    integer   slv_bits;

    assign miso = slv_sh[7];

    always @(negedge sclk) begin
        slv_sh <= {slv_sh[6:0], 1'b0};
    end

    //------------------------------------------------------------------
    // observers
    //------------------------------------------------------------------
    reg [7:0] mosi_seen;
    integer   mosi_bits;
    integer   done_cnt;
    integer   clk_tick;
    integer   rise_t [0:15];
    integer   rise_n;

    always @(posedge sclk) begin
        mosi_seen = {mosi_seen[6:0], mosi};
        mosi_bits = mosi_bits + 1;
    end

    always @(posedge clk) clk_tick = clk_tick + 1;

    // timestamp every rising edge of SCLK so the period can be measured
    always @(posedge sclk) begin
        if (rise_n < 16) rise_t[rise_n] = clk_tick;
        rise_n = rise_n + 1;
    end

    always @(posedge clk) if (done) done_cnt = done_cnt + 1;

    //------------------------------------------------------------------
    // one transfer
    //------------------------------------------------------------------
    task transfer;
        input [7:0] to_send;
        input [7:0] slave_sends;
        begin
            @(negedge clk);
            slv_sh      = slave_sends;
            mosi_seen   = 8'd0;
            mosi_bits   = 0;
            rise_n      = 0;
            tx_byte     = to_send;
            start       = 1'b1;
            @(negedge clk);
            start       = 1'b0;

            if (!busy) fail("busy was not asserted after start");
            if (sclk !== 1'b0) fail("sclk did not idle low");

            @(posedge done);
            @(negedge clk);

            if (rx_byte !== slave_sends) begin
                $display("  FAIL: received %02x, slave sent %02x", rx_byte, slave_sends);
                errors = errors + 1;
            end
            if (mosi_seen !== to_send) begin
                $display("  FAIL: slave saw %02x, master sent %02x", mosi_seen, to_send);
                errors = errors + 1;
            end
            if (mosi_bits != 8) begin
                $display("  FAIL: %0d bits clocked instead of 8", mosi_bits);
                errors = errors + 1;
            end
            if (busy) fail("busy still asserted at the end of the transfer");
            if (sclk !== 1'b0) fail("sclk did not return low");
        end
    endtask

    integer i;

    initial begin
        errors    = 0;
        rst       = 1'b1;
        clk_div   = 16'd2;
        start     = 1'b0;
        tx_byte   = 8'd0;
        slv_sh    = 8'd0;
        mosi_seen = 8'd0;
        mosi_bits = 0;
        done_cnt  = 0;
        clk_tick  = 0;
        rise_n    = 0;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        // ---- 1. SCLK period with clk_div = 2 ------------------------
        clk_div = 16'd2;
        transfer(8'hA5, 8'h3C);
        if (rise_n != 8) begin
            $display("  FAIL: %0d SCLK rising edges for one byte", rise_n);
            errors = errors + 1;
        end
        if (rise_t[1] - rise_t[0] !== 2 * clk_div) begin
            $display("  FAIL: SCLK period %0d clocks, expected %0d",
                     rise_t[1] - rise_t[0], 2 * clk_div);
            errors = errors + 1;
        end

        // ---- 2. a spread of byte patterns ---------------------------
        transfer(8'h00, 8'hFF);
        transfer(8'hFF, 8'h00);
        transfer(8'h5A, 8'hA5);
        transfer(8'h01, 8'h80);
        transfer(8'h80, 8'h01);

        // ---- 3. slow divider, as used during card initialisation -----
        clk_div = 16'd8;
        transfer(8'hC3, 8'h7E);

        // ---- 4. done pulses exactly once per byte --------------------
        // the observer samples done on a clock edge, so let the last pulse
        // be counted before reading the counter
        repeat (2) @(posedge clk);
        if (done_cnt != 7) begin
            $display("  FAIL: done pulsed %0d times for 7 transfers", done_cnt);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS -- SPI master verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #200_000;
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule

