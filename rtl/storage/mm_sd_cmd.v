//=============================================================================
// mm_sd_cmd.v -- SD card command layer (SPI mode)
//
// Sends one command and collects its response:
//
//   byte 0    0b01xxxxxx          command index
//   byte 1..4 32-bit argument
//   byte 5    {CRC7, 1}           CRC7 over the first five bytes
//
// Then it clocks 0xFF until the card answers (the response always starts
// with a byte whose bit 7 is zero), captures the response and releases the
// chip select.  Chip select is held across the whole exchange, which is why
// it lives here rather than in the byte-level SPI master.
//
// Response length depends on the command:
//   R1 : CMD0 / CMD55 / ACMD41 / CMD17 ...          1 byte
//   R3 : CMD8                                       1 + 4 = 5 bytes (voltage, echo)
//   R7 : CMD58                                      1 + 4 = 5 bytes (OCR)
//
// CRC7 is computed over all 40 command bits.  Only CMD0 and CMD8 strictly
// require a valid CRC in SPI mode, but computing it everywhere costs a few
// LUTs and removes a whole class of failure.
//=============================================================================
`timescale 1ns / 100fs

module mm_sd_cmd #(
    // how many 0xFF bytes to clock while waiting for the response before
    // declaring a timeout.  At the 100 kHz of the initialisation sequence one
    // byte is 80 us, so 6000 bytes is 480 ms, comfortably more than the
    // 100 ms the specification allows a card to take while it is still
    // waking up.
    parameter integer MAX_POLL_BYTES = 6000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] clk_div,      // SPI half period in clk cycles

    input  wire        start,        // pulse: run one command
    input  wire [5:0]  cmd_index,
    input  wire [31:0] cmd_arg,

    output reg         busy,
    output reg         done,         // one cycle pulse when the exchange ends
    output reg  [47:0] resp,         // resp[47:40] = first response byte
    output reg  [2:0]  resp_len,     // number of valid bytes in resp
    output reg         resp_timeout,
    output reg  [15:0] poll_bytes,   // 0xFF bytes needed before the answer

    output wire        sd_ncs,
    output wire        sd_sclk,
    output wire        sd_mosi,
    input  wire        sd_miso
);

    localparam [2:0] ST_IDLE = 3'd0,
                     ST_SEND = 3'd1,
                     ST_POLL = 3'd2,
                     ST_RECV = 3'd3,
                     ST_END  = 3'd4;

    reg [2:0]  state;
    reg [2:0]  byte_idx;
    reg [2:0]  want_len;
    reg [2:0]  got_len;
    reg [15:0] poll_cnt;
    reg        cs_n;

    //------------------------------------------------------------------
    // CRC7 over the 40 command bits
    //------------------------------------------------------------------
    integer   i;
    reg [6:0] crc7;
    reg [39:0] cmd_bits;

    always @(*) begin
        cmd_bits = {2'b01, cmd_index, cmd_arg};
        crc7     = 7'd0;
        for (i = 39; i >= 0; i = i - 1) begin
            if ((crc7[6] ^ cmd_bits[i]) == 1'b1)
                crc7 = {crc7[5:0], 1'b0} ^ 7'h09;   // x^7 + x^3 + 1
            else
                crc7 = {crc7[5:0], 1'b0};
        end
    end

    //------------------------------------------------------------------
    // byte level SPI
    //------------------------------------------------------------------
    reg        spi_start;
    reg  [7:0] spi_tx;
    wire [7:0] spi_rx;
    wire       spi_busy;
    wire       spi_done;

    mm_spi_master u_spi (
        .clk     (clk),
        .rst     (rst),
        .clk_div (clk_div),
        .start   (spi_start),
        .tx_byte (spi_tx),
        .rx_byte (spi_rx),
        .busy    (spi_busy),
        .done    (spi_done),
        .sclk    (sd_sclk),
        .mosi    (sd_mosi),
        .miso    (sd_miso)
    );

    // The whole 48-bit command is assembled first and then shifted out one
    // byte at a time.  Selecting the byte combinationally from byte_idx would
    // read a stale index, because byte_idx is updated on the same clock edge
    // that latches the next byte into the SPI master.
    wire [47:0] cmd_frame_next = {{2'b01, cmd_index}, cmd_arg, {crc7, 1'b1}};
    reg  [47:0] cmd_frame;

    assign sd_ncs = cs_n;

    //------------------------------------------------------------------
    // sequence
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state        <= ST_IDLE;
            cs_n         <= 1'b1;
            busy         <= 1'b0;
            done         <= 1'b0;
            resp         <= 48'd0;
            resp_len     <= 3'd0;
            resp_timeout <= 1'b0;
            poll_bytes   <= 16'd0;
            poll_cnt     <= 16'd0;
            byte_idx     <= 3'd0;
            want_len     <= 3'd1;
            got_len      <= 3'd0;
            spi_start    <= 1'b0;
            spi_tx       <= 8'hFF;
            cmd_frame    <= 48'd0;
        end else begin
            done      <= 1'b0;
            spi_start <= 1'b0;

            case (state)
                ST_IDLE: begin
                    cs_n <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        cs_n      <= 1'b0;
                        resp      <= 48'd0;
                        resp_len  <= 3'd0;
                        resp_timeout <= 1'b0;
                        poll_cnt  <= 16'd0;
                        byte_idx  <= 3'd0;
                        got_len   <= 3'd0;
                        // R3 (CMD8) and R7 (CMD58) are R1 plus four extra bytes
                        want_len  <= (cmd_index == 6'd8 || cmd_index == 6'd58) ? 3'd5 : 3'd1;
                        cmd_frame <= cmd_frame_next;
                        spi_tx    <= cmd_frame_next[47:40];
                        spi_start <= 1'b1;
                        state     <= ST_SEND;
                    end
                end

                ST_SEND: begin
                    if (spi_done) begin
                        byte_idx <= byte_idx + 3'd1;
                        if (byte_idx == 3'd5) begin
                            spi_tx <= 8'hFF;        // start polling for the answer
                            state  <= ST_POLL;
                            if (!spi_busy)
                                spi_start <= 1'b1;
                        end else begin
                            // drop the byte just sent, present the next one
                            cmd_frame <= {cmd_frame[39:0], 8'hFF};
                            spi_tx    <= cmd_frame[39:32];
                            spi_start <= 1'b1;
                        end
                    end
                end

                // clock 0xFF until the card pulls bit 7 low, or give up
                ST_POLL: begin
                    if (spi_done) begin
                        if (spi_rx[7] == 1'b0) begin
                            resp[47:40] <= spi_rx;
                            got_len   <= 3'd1;
                            poll_bytes <= poll_cnt;
                            if (want_len == 3'd1) begin
                                resp_len <= 3'd1;
                                state    <= ST_END;
                            end else begin
                                spi_tx    <= 8'hFF;
                                spi_start <= 1'b1;
                                state     <= ST_RECV;
                            end
                        end else if (poll_cnt == MAX_POLL_BYTES - 1) begin
                            resp_timeout <= 1'b1;
                            poll_bytes   <= poll_cnt;
                            state        <= ST_END;
                        end else begin
                            poll_cnt  <= poll_cnt + 16'd1;
                            spi_start <= 1'b1;
                        end
                    end
                end

                // the extra bytes of R3 / R7
                ST_RECV: begin
                    if (spi_done) begin
                        // each byte is written straight into its own field so
                        // the first response byte always ends up at resp[47:40]
                        case (got_len)
                            3'd1:    resp[39:32] <= spi_rx;
                            3'd2:    resp[31:24] <= spi_rx;
                            3'd3:    resp[23:16] <= spi_rx;
                            3'd4:    resp[15:8]  <= spi_rx;
                            default: resp[7:0]   <= spi_rx;
                        endcase
                        got_len <= got_len + 3'd1;
                        if (got_len + 3'd1 == want_len) begin
                            resp_len <= want_len;
                            state    <= ST_END;
                        end else begin
                            spi_start <= 1'b1;
                        end
                    end
                end

                default: begin
                    cs_n  <= 1'b1;
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

