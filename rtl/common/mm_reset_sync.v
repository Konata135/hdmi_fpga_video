//=============================================================================
// mm_reset_sync.v -- asynchronous assert / synchronous de-assert
//
// Distributes the global por_n into one clock domain.  The reset is applied
// as soon as por_n falls, and released only after two clean edges of clk,
// so no flop can come out of reset on a metastable clock edge.
//=============================================================================
`timescale 1ns / 100fs

module mm_reset_sync (
    input  wire clk,
    input  wire por_n,      // asynchronous source, active low
    output wire rst         // synchronous reset, active high
);

    reg [1:0] sync;

    always @(posedge clk or negedge por_n) begin
        if (!por_n)
            sync <= 2'b00;
        else
            sync <= {sync[0], 1'b1};
    end

    assign rst = ~sync[1];

endmodule

