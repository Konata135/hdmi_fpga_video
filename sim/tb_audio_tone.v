//=============================================================================
// tb_audio_tone.v -- self-checking testbench for mm_audio_tone
//
// The clock is scaled down (250 kHz instead of 25 MHz, 480 Hz instead of
// 48 kHz) so a second of audio is a quarter of a million clocks, not 25
// million.  Everything that matters is a ratio, so the scaling changes nothing:
//
//   1. one sample every CLK_HZ / SAMPLE_HZ clocks (520.83), i.e. the rate is
//      48 kHz to within the fractional divider's own rounding
//   2. the tone's period in *samples* matches the phase increment
//      (2^32 / inc), which is what the note table is for
//   3. with `enable` low the output is silence, and with it high the amplitude
//      stays inside the headroom it promises
//   4. changing the note changes the period in the expected ratio
//=============================================================================
`timescale 1ns / 100ps

module tb_audio_tone;

    localparam integer CLK_HZ    = 250_000;
    localparam integer SAMPLE_HZ = 480;

    reg  clk = 1'b0;
    reg  rst = 1'b1;
    reg  enable = 1'b0;
    reg  [1:0] note_sel = 2'd0;

    wire        sample_valid;
    wire [23:0] sample;

    always #20 clk = ~clk;                  // 25 MHz scaled by 100

    mm_audio_tone #(
        .CLK_HZ    (CLK_HZ),
        .SAMPLE_HZ (SAMPLE_HZ)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .enable       (enable),
        .note_sel     (note_sel),
        .sample_valid (sample_valid),
        .sample       (sample)
    );

    integer errors = 0;
    integer samples = 0;
    integer crossings = 0;
    integer i;
    reg     prev_pos = 1'b0;
    integer peak = 0;

    always @(posedge clk) begin
        if (!rst && sample_valid) begin
            samples = samples + 1;
            if (($signed(sample) > 0) && !prev_pos) crossings = crossings + 1;
            prev_pos = ($signed(sample) > 0);
            if (sample[23] == 1'b0 && sample > peak) peak = sample;
            if (sample[23] == 1'b1 && (-$signed(sample)) > peak) peak = -$signed(sample);
        end
    end

    task fail;
        input [1023:0] msg;
        begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    // Play a known number of clocks and report how many samples and crossings
    // happened, so the caller can turn that into a rate and a period.
    task run_clocks;
        input integer n;
        begin
            samples = 0; crossings = 0; peak = 0;
            repeat (n) @(posedge clk);
        end
    endtask

    integer expect_samples;
    integer period_samples;
    integer expect_period;
    integer period_c4;

    initial begin
        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        // ---- 1/2. 48 kHz and the C4 note --------------------------------
        enable   = 1'b1;
        note_sel = 2'd0;
        run_clocks(CLK_HZ);                 // one second, scaled
        expect_samples = SAMPLE_HZ;
        if ((samples < expect_samples - 2) || (samples > expect_samples + 2)) begin
            $display("  FAIL: %0d samples in a second, expected %0d", samples, expect_samples);
            errors = errors + 1;
        end else begin
            $display("  rate         : PASS (%0d samples, expected %0d)", samples, expect_samples);
        end

        expect_period = 64'd4294967296 / 64'd23409862;  // 2^32 / inc for C4
        period_samples = samples / crossings;
        period_c4      = period_samples;
        // Only a couple of cycles fit in the window, so the integer crossing
        // count makes this coarse (about +-25 samples); the ratio check below is
        // the sharp one.
        if ((period_samples < expect_period - 25) || (period_samples > expect_period + 25)) begin
            $display("  FAIL: C4 period is %0d samples, expected %0d", period_samples, expect_period);
            errors = errors + 1;
        end else begin
            $display("  note C4      : PASS (%0d samples per cycle, expected %0d)",
                     period_samples, expect_period);
        end
        if (peak > 2200000) begin
            $display("  FAIL: peak %0d is louder than the headroom promise", peak);
            errors = errors + 1;
        end else begin
            $display("  level        : PASS (peak %0d, inside 2.2e6)", peak);
        end

        // ---- 3. silence when disabled ------------------------------------
        enable = 1'b0;
        repeat (2000) @(posedge clk);        // let the last sample drain out
        samples = 0; crossings = 0; peak = 0;
        repeat (CLK_HZ / 10) @(posedge clk);
        if ((peak != 0) || (crossings != 0)) begin
            $display("  FAIL: the tone is still moving with enable low (peak %0d)", peak);
            errors = errors + 1;
        end else begin
            $display("  silence      : PASS");
        end

        // ---- 4. the note really changes ----------------------------------
        enable   = 1'b1;
        note_sel = 2'd3;
        run_clocks(CLK_HZ);
        expect_period  = 64'd4294967296 / 64'd46819716;  // C5
        period_samples = samples / crossings;
        if ((period_samples < expect_period - 12) || (period_samples > expect_period + 12)) begin
            $display("  FAIL: C5 period is %0d samples, expected %0d", period_samples, expect_period);
            errors = errors + 1;
        end else begin
            $display("  note C5      : PASS (%0d samples per cycle, expected %0d)",
                     period_samples, expect_period);
        end

        // The ratio is the sharp check: the two note increments differ by 2.0,
        // and measuring both the same way cancels the coarse crossing count.
        if ((period_samples < period_c4 / 2 - 4) || (period_samples > period_c4 / 2 + 4)) begin
            $display("  FAIL: C4/C5 periods are %0d and %0d samples, expected a 2:1 ratio",
                     period_c4, period_samples);
            errors = errors + 1;
        end else begin
            $display("  note ratio   : PASS (C4 %0d, C5 %0d samples: 2:1)",
                     period_c4, period_samples);
        end

        if (errors == 0)
            $display("RESULT: PASS -- audio tone verified");
        else
            $display("RESULT: FAIL -- %0d error(s)", errors);
        $finish;
    end

    initial begin
        #100_000_000;          // two one-second runs at the scaled clock
        $display("RESULT: FAIL -- watchdog timeout");
        $finish;
    end

endmodule
