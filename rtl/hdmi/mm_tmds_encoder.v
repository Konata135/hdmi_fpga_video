//=============================================================================
// mm_tmds_encoder.v -- TMDS 8b/10b character encoder, one channel
//
// Implements the encoding defined by the DVI 1.0 / HDMI 1.4b specification:
//
//   * video data period : 8-bit colour component -> 10-bit video character,
//     transition minimised in the first stage and DC balanced in the second
//     stage using the per-channel running disparity
//   * control period    : 2-bit control value -> one of four 10-bit control
//     characters
//
// dout[0] is the bit that is transmitted first on the TMDS pair.
//
// The running disparity is a signed value kept in two's complement; it is
// only advanced by video characters, which is what the specification
// requires (control characters do not disturb the video balance).
//=============================================================================
`timescale 1ns / 100fs

module mm_tmds_encoder (
    input  wire       clk,
    input  wire       rst,
    input  wire       de,        // video data enable
    input  wire [7:0] din,       // video data, used when de = 1
    input  wire [1:0] ctl,       // control value {D1, D0}, used when de = 0
    output reg  [9:0] dout       // bit 0 is transmitted first
);

    //------------------------------------------------------------------
    // control characters
    //------------------------------------------------------------------
    reg [9:0] ctl_code;

    always @(*) begin
        case (ctl)
            2'b00:   ctl_code = 10'b1101010100;
            2'b01:   ctl_code = 10'b0010101011;
            2'b10:   ctl_code = 10'b0101010100;
            default: ctl_code = 10'b1010101011;
        endcase
    end

    //------------------------------------------------------------------
    // stage 1: minimise the number of transitions
    //------------------------------------------------------------------
    integer   i;
    reg [3:0] n1d;          // number of ones in din
    reg [7:0] q_m;
    reg       q_m8;         // ninth bit of the intermediate word
    reg       use_xnor;

    always @(*) begin
        n1d = 4'd0;
        for (i = 0; i < 8; i = i + 1)
            n1d = n1d + {3'b000, din[i]};

        if ((n1d > 4) || ((n1d == 4) && (din[0] == 1'b0))) begin
            use_xnor = 1'b1;
            q_m8     = 1'b0;
        end else begin
            use_xnor = 1'b0;
            q_m8     = 1'b1;
        end

        q_m[0] = din[0];
        for (i = 1; i < 8; i = i + 1)
            q_m[i] = use_xnor ? ~(q_m[i-1] ^ din[i])
                              :  (q_m[i-1] ^ din[i]);
    end

    //------------------------------------------------------------------
    // stage 2: DC balancing
    //------------------------------------------------------------------
    reg [3:0] n1q;
    reg [3:0] n0q;

    always @(*) begin
        n1q = 4'd0;
        for (i = 0; i < 8; i = i + 1)
            n1q = n1q + {3'b000, q_m[i]};
        n0q = 4'd8 - n1q;
    end

    // running disparity: ones minus zeros of what has been transmitted so
    // far, kept in two's complement
    reg [5:0] disparity;

    wire disp_is_zero = (disparity == 6'd0);
    wire disp_is_neg  = disparity[5];

    reg [9:0] q_out;

    always @(*) begin
        if (disp_is_zero || (n1q == n0q)) begin
            // no bias to correct: the ninth bit tells the sink how the byte
            // was shaped, the tenth bit is its complement
            q_out[9]   = ~q_m8;
            q_out[8]   =  q_m8;
            q_out[7:0] =  q_m8 ? q_m : ~q_m;
        end else if ((!disp_is_neg && !disp_is_zero && (n1q > n0q)) ||
                     ( disp_is_neg &&                   (n0q > n1q))) begin
            // invert the byte to pull the running disparity back towards zero
            q_out[9]   = 1'b1;
            q_out[8]   = q_m8;
            q_out[7:0] = ~q_m;
        end else begin
            q_out[9]   = 1'b0;
            q_out[8]   = q_m8;
            q_out[7:0] = q_m;
        end
    end

    reg [3:0] n1_out;
    reg [3:0] n0_out;

    always @(*) begin
        n1_out = 4'd0;
        for (i = 0; i < 10; i = i + 1)
            n1_out = n1_out + {3'b000, q_out[i]};
        n0_out = 4'd10 - n1_out;
    end

    always @(posedge clk) begin
        if (rst)
            disparity <= 6'd0;
        else if (de)
            disparity <= disparity + ({2'b00, n1_out} - {2'b00, n0_out});
    end

    //------------------------------------------------------------------
    // output register
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst)
            dout <= 10'b1101010100;
        else
            dout <= de ? q_out : ctl_code;
    end

endmodule

