//=============================================================================
// mm_spi_master.v -- SPI master, mode 0, one byte per transfer
//
// Used by the TF card protocol layer.  Mode 0 means:
//   * SCLK idles low
//   * the master changes MOSI on the falling edge
//   * both sides sample on the rising edge
//   * most significant bit first
//
// The half period is taken from clk_div at the start of every transfer, so
// the same hardware runs the card at 100 kHz during initialisation and at
// 25 MHz for data reads.
//
// Chip select is deliberately NOT handled here: the card protocol layer owns
// it, because a command sequence needs CS held across several bytes.
//=============================================================================
`timescale 1ns / 100fs

module mm_spi_master (
    input  wire        clk,
    input  wire        rst,

    input  wire [15:0] clk_div,     // half period in clk cycles, >= 1
    input  wire        start,       // pulse: begin one byte transfer
    input  wire [7:0]  tx_byte,

    output reg  [7:0]  rx_byte,
    output reg         busy,
    output reg         done,        // one cycle pulse when rx_byte is valid

    output reg         sclk,
    output reg         mosi,
    input  wire        miso
);

    localparam [1:0] ST_IDLE = 2'd0,
                     ST_LOW  = 2'd1,
                     ST_HIGH = 2'd2,
                     ST_END  = 2'd3;

    reg [1:0]  state;
    reg [15:0] div_cnt;
    reg [3:0]  bit_cnt;
    reg [7:0]  sh_out;
    reg [7:0]  sh_in;

    wire half_time = (div_cnt == clk_div - 16'd1);

    always @(posedge clk) begin
        if (rst) begin
            state   <= ST_IDLE;
            div_cnt <= 16'd0;
            bit_cnt <= 4'd0;
            sh_out  <= 8'd0;
            sh_in   <= 8'd0;
            rx_byte <= 8'd0;
            busy    <= 1'b0;
            done    <= 1'b0;
            sclk    <= 1'b0;
            mosi    <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    sclk  <= 1'b0;
                    busy  <= 1'b0;
                    if (start) begin
                        sh_out  <= tx_byte;
                        sh_in   <= 8'd0;
                        bit_cnt <= 4'd0;
                        div_cnt <= 16'd0;
                        busy    <= 1'b1;
                        state   <= ST_LOW;
                    end
                end

                // SCLK low: drive the next bit, then hold for a half period
                ST_LOW: begin
                    sclk <= 1'b0;
                    mosi <= sh_out[7];
                    if (half_time) begin
                        div_cnt <= 16'd0;
                        state   <= ST_HIGH;
                    end else begin
                        div_cnt <= div_cnt + 16'd1;
                    end
                end

                // SCLK high: the slave's data is stable, take it
                ST_HIGH: begin
                    sclk <= 1'b1;
                    if (half_time) begin
                        div_cnt <= 16'd0;
                        sh_in   <= {sh_in[6:0], miso};
                        sh_out  <= {sh_out[6:0], 1'b0};
                        bit_cnt <= bit_cnt + 4'd1;
                        state   <= (bit_cnt == 4'd7) ? ST_END : ST_LOW;
                    end else begin
                        div_cnt <= div_cnt + 16'd1;
                    end
                end

                default: begin
                    sclk    <= 1'b0;
                    rx_byte <= sh_in;
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    state   <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

