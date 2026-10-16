//=============================================================================
// mm_reset_ctrl.v -- power-on reset controller
//
// Keeps the whole design in reset until:
//   * the board push button is released, and
//   * every PLL reports lock, and
//   * the lock state has been stable for HOLD_CYCLES of the 50 MHz clock.
//
// The output por_n is an asynchronous, active-low reset source.  It is
// re-synchronised in each clock domain by mm_reset_sync.
//=============================================================================
`timescale 1ns / 100fs

module mm_reset_ctrl #(
    parameter integer HOLD_CYCLES = 500_000,   // 10 ms @ 50 MHz
    parameter integer N_LOCK      = 1
)(
    input  wire              clk_ref,      // 50 MHz reference domain
    input  wire              btn_rst_n,    // board push button, active low
    input  wire [N_LOCK-1:0] pll_lock,
    output reg               por_n         // global reset release, active low
);

    // two-stage synchroniser for every lock bit
    reg [N_LOCK-1:0] lock_sync_1;
    reg [N_LOCK-1:0] lock_sync_2;

    always @(posedge clk_ref) begin
        lock_sync_1 <= pll_lock;
        lock_sync_2 <= lock_sync_1;
    end

    wire all_locked = &lock_sync_2;

    reg [19:0] hold_cnt;
    reg        released;

    always @(posedge clk_ref) begin
        if (!btn_rst_n) begin
            hold_cnt <= 20'd0;
            released <= 1'b0;
        end else if (!all_locked) begin
            hold_cnt <= 20'd0;
            released <= 1'b0;
        end else if (!released) begin
            if (hold_cnt == HOLD_CYCLES - 1)
                released <= 1'b1;
            else
                hold_cnt <= hold_cnt + 20'd1;
        end
    end

    always @(posedge clk_ref) begin
        por_n <= released & btn_rst_n;
    end

endmodule

