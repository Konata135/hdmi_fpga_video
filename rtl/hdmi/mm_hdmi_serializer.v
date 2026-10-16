//=============================================================================
// mm_hdmi_serializer.v -- 10:1 TMDS serialiser for one differential pair
//
// This device family has no OSERDES primitive (see the EG4 libraries guide),
// so the serialiser is built from a fabric shift register feeding an
// EG_LOGIC_ODDR IO cell.  For a 640x480@60 mode the character rate is the
// pixel rate (25 MHz) and the serial bit rate is 250 Mbit/s, so the
// serialiser runs on the 125 MHz clock and emits two bits per clock period.
//
// EG_LOGIC_ODDR samples both inputs on the rising edge and then drives d0
// through the first half of the period and d1 through the second half, so
// pair[0] is the earlier of the two bits.
//
// char_in[0] is transmitted first, and the shift register is emptied from the
// bottom, so the stream really is least significant bit first.  That was
// confirmed on hardware: the display locks with this order and the 5x serial
// clock and the pixel clock come from the same PLL, so the divide by five
// counter below stays permanently locked to the character rate.
//=============================================================================
`timescale 1ns / 100fs

module mm_hdmi_serializer #(
    parameter integer DIV        = 5,
    parameter integer LOAD_PHASE = 4
)(
    input  wire       serial_clk,
    input  wire       rst,
    input  wire [9:0] char_in,
    output wire       serial_out
);

    //------------------------------------------------------------------
    // character period counter
    //------------------------------------------------------------------
    reg [2:0] div_cnt;

    always @(posedge serial_clk or posedge rst) begin
        if (rst)
            div_cnt <= 3'd0;
        else
            div_cnt <= (div_cnt == DIV - 1) ? 3'd0 : (div_cnt + 3'd1);
    end

    wire load = (div_cnt == LOAD_PHASE);

    //------------------------------------------------------------------
    // 2 bits per serial clock, low bits first
    //------------------------------------------------------------------
    reg [9:0] shift;
    reg [1:0] pair;

    always @(posedge serial_clk or posedge rst) begin
        if (rst) begin
            shift <= 10'd0;
            pair  <= 2'd0;
        end else begin
            if (load)
                shift <= char_in;
            else
                shift <= {2'b00, shift[9:2]};

            pair <= shift[1:0];
        end
    end

    //------------------------------------------------------------------
    // double data rate output cell
    //------------------------------------------------------------------
    EG_LOGIC_ODDR #(
        .ASYNCRST ("ENABLE")
    ) u_oddr (
        .clk (serial_clk),
        .rst (rst),
        .d0  (pair[0]),     // rising edge : first bit of the pair
        .d1  (pair[1]),     // falling edge: second bit of the pair
        .q   (serial_out)
    );

endmodule

