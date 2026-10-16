//=============================================================================
// mm_sd_sec.v -- CMD17 single block read, SPI mode
//
// Reads one 512 byte sector.  A read is not a plain command: the chip select
// has to stay low from the command frame all the way to the last byte of the
// block, so the whole exchange is one transaction owned by this module:
//
//   CMD17 (0x51) + block address + CRC7   48 bits, most significant bit first
//   R1                                    0x00 means "here it comes"
//   0xFE data token
//   512 data bytes
//   CRC16, two bytes
//
// The CRC16 is not verified here: it protects the card against its own flash
// read errors, and in SPI mode an uncorrectable sector is reported by the
// card with a data error token instead of the 0xFE token, which is what this
// module detects.  The two bytes are still clocked out (the bus has to be
// clocked to finish the block) and kept in crc16 for the record.
//
// Every wait is bounded, so a card that stops talking can never hang the
// design: no R1 within MAX_POLL_BYTES, and no data token within
// MAX_TOKEN_BYTES, both end in done with a non-zero error code.
//
// The address handed in is a block address (LBA).  v1 cards address bytes, so
// byte_addressed shifts it left by nine.
//
// The 512 bytes stream out on data_valid / data_index / data_byte, so the
// caller never has to buffer a whole sector: the bring-up check in
// mm_sd_probe only looks at the two signature bytes and the partition entry.
//=============================================================================
`timescale 1ns / 100fs

module mm_sd_sec #(
    // 0xFF bytes clocked while waiting for the response.  The specification
    // allows a card up to 100 ms before it answers; at 25 MHz one byte is
    // 320 ns, so 100000 bytes is 32 ms, which is generous for a card that has
    // already been through the initialisation sequence.
    parameter integer MAX_POLL_BYTES  = 100_000,
    // 0xFF bytes clocked while waiting for the data token.  NAC is up to
    // 100 ms, which is 312500 bytes at 25 MHz; 400000 leaves 28 % head room.
    parameter integer MAX_TOKEN_BYTES = 400_000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] clk_div,        // SPI half period in clk cycles

    input  wire        start,          // pulse: run one sector read
    input  wire [31:0] block_addr,     // block address (LBA)
    input  wire        byte_addressed, // v1 card: the argument is a byte address

    output reg         busy,
    output reg         done,           // one cycle pulse when the exchange ends
    output reg  [3:0]  error_code,     // 0 none, 1..4 as decoded below
    output reg  [7:0]  r1,             // the response byte, for diagnostics
    output reg  [7:0]  err_token,      // a data error token, if one arrived
    output reg  [15:0] crc16,          // the two CRC bytes, clocked and kept
    output reg  [15:0] poll_bytes,     // 0xFF bytes needed before R1
    output reg  [19:0] token_poll,     // 0xFF bytes needed before the token

    output reg  [8:0]  data_index,     // 0..511
    output reg  [7:0]  data_byte,
    output reg         data_valid,     // one cycle pulse per received byte
    output wire        waiting_data,   // waiting for, or reading, the block

    output wire        sd_ncs,
    output wire        sd_sclk,
    output wire        sd_mosi,
    input  wire        sd_miso
);

    // error codes, also shown on the status display
    localparam [3:0] ERR_NONE      = 4'd0,
                     ERR_NO_R1     = 4'd1,   // the card never answered
                     ERR_R1        = 4'd2,   // answered, but with an error bit
                     ERR_NO_TOKEN  = 4'd3,   // never offered a data block
                     ERR_BAD_TOKEN = 4'd4;   // offered a data error token

    localparam [2:0] ST_IDLE  = 3'd0,
                     ST_SEND  = 3'd1,
                     ST_POLL  = 3'd2,
                     ST_TOKEN = 3'd3,
                     ST_DATA  = 3'd4,
                     ST_CRC   = 3'd5,
                     ST_END   = 3'd6;

    reg [2:0]  state;
    reg [2:0]  byte_idx;
    reg [8:0]  byte_cnt;      // data bytes received so far
    reg [15:0] poll_cnt;
    reg [19:0] token_cnt;
    reg        crc_cnt;
    reg        cs_n;

    assign sd_ncs  = cs_n;
    assign waiting_data = (state == ST_TOKEN) || (state == ST_DATA) || (state == ST_CRC);

    //------------------------------------------------------------------
    // command frame: CMD17 plus CRC7 over the 40 command bits
    //------------------------------------------------------------------
    wire [31:0] cmd_arg  = byte_addressed ? {block_addr[22:0], 9'b0} : block_addr;
    wire [39:0] cmd_bits = {2'b01, 6'd17, cmd_arg};

    integer   i;
    reg [6:0] crc7;

    always @(*) begin
        crc7 = 7'd0;
        for (i = 39; i >= 0; i = i - 1) begin
            if ((crc7[6] ^ cmd_bits[i]) == 1'b1)
                crc7 = {crc7[5:0], 1'b0} ^ 7'h09;   // x^7 + x^3 + 1
            else
                crc7 = {crc7[5:0], 1'b0};
        end
    end

    // The whole frame is assembled first and then shifted out one byte at a
    // time.  Selecting the byte combinationally from byte_idx would read a
    // stale index, because byte_idx advances on the same clock edge that
    // latches the next byte into the SPI master.
    wire [47:0] cmd_frame_next = {cmd_bits, crc7, 1'b1};
    reg  [47:0] cmd_frame;

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

    //------------------------------------------------------------------
    // sequence
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state      <= ST_IDLE;
            cs_n       <= 1'b1;
            busy       <= 1'b0;
            done       <= 1'b0;
            error_code <= ERR_NONE;
            r1         <= 8'd0;
            err_token  <= 8'd0;
            crc16      <= 16'd0;
            poll_bytes <= 16'd0;
            token_poll <= 20'd0;
            poll_cnt   <= 16'd0;
            token_cnt  <= 20'd0;
            data_index <= 9'd0;
            byte_cnt   <= 9'd0;
            data_byte  <= 8'd0;
            data_valid <= 1'b0;
            crc_cnt    <= 1'b0;
            spi_start  <= 1'b0;
            spi_tx     <= 8'hFF;
            cmd_frame  <= 48'd0;
            byte_idx   <= 3'd0;
        end else begin
            done       <= 1'b0;
            data_valid <= 1'b0;
            spi_start  <= 1'b0;

            case (state)
                ST_IDLE: begin
                    cs_n <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        busy       <= 1'b1;
                        cs_n       <= 1'b0;
                        error_code <= ERR_NONE;
                        r1         <= 8'd0;
                        err_token  <= 8'd0;
                        crc16      <= 16'd0;
                        poll_cnt   <= 16'd0;
                        token_cnt  <= 20'd0;
                        data_index <= 9'd0;
                        byte_cnt   <= 9'd0;
                        crc_cnt    <= 1'b0;
                        byte_idx   <= 3'd0;
                        cmd_frame  <= cmd_frame_next;
                        spi_tx     <= cmd_frame_next[47:40];
                        spi_start  <= 1'b1;
                        state      <= ST_SEND;
                    end
                end

                ST_SEND: begin
                    if (spi_done) begin
                        byte_idx <= byte_idx + 3'd1;
                        if (byte_idx == 3'd5) begin
                            spi_tx <= 8'hFF;        // start polling for R1
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

                // clock 0xFF until the card answers (bit 7 low), or give up
                ST_POLL: begin
                    if (spi_done) begin
                        if (spi_rx[7] == 1'b0) begin
                            r1         <= spi_rx;
                            poll_bytes <= poll_cnt;
                            if (spi_rx != 8'h00) begin
                                error_code <= ERR_R1;
                                state      <= ST_END;
                            end else begin
                                token_cnt <= 20'd0;
                                spi_tx    <= 8'hFF;
                                spi_start <= 1'b1;
                                state     <= ST_TOKEN;
                            end
                        end else if (poll_cnt == MAX_POLL_BYTES - 1) begin
                            poll_bytes <= poll_cnt;
                            error_code <= ERR_NO_R1;
                            state      <= ST_END;
                        end else begin
                            poll_cnt  <= poll_cnt + 16'd1;
                            spi_start <= 1'b1;
                        end
                    end
                end

                // 0xFE starts the block.  A byte of the form 0000xxxx is a
                // data error token: that is how the card reports a sector it
                // could not read out of its own flash.
                ST_TOKEN: begin
                    if (spi_done) begin
                        if (spi_rx == 8'hFE) begin
                            byte_cnt   <= 9'd0;
                            spi_tx     <= 8'hFF;
                            spi_start  <= 1'b1;
                            state      <= ST_DATA;
                        end else if (spi_rx[7:4] == 4'h0) begin
                            err_token  <= spi_rx;
                            error_code <= ERR_BAD_TOKEN;
                            state      <= ST_END;
                        end else if (token_cnt == MAX_TOKEN_BYTES - 1) begin
                            token_poll <= token_cnt;
                            error_code <= ERR_NO_TOKEN;
                            state      <= ST_END;
                        end else begin
                            token_cnt <= token_cnt + 20'd1;
                            spi_start <= 1'b1;
                        end
                    end
                end

                ST_DATA: begin
                    if (spi_done) begin
                        // data_index and data_byte are presented together on
                        // the next clock, so the index has to be the count of
                        // bytes received *before* this one
                        data_byte  <= spi_rx;
                        data_index <= byte_cnt;
                        data_valid <= 1'b1;
                        byte_cnt   <= byte_cnt + 9'd1;
                        if (byte_cnt == 9'd511) begin
                            crc_cnt   <= 1'b0;
                            crc16     <= 16'd0;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                            state     <= ST_CRC;
                        end else begin
                            spi_start <= 1'b1;
                        end
                    end
                end

                // the two CRC16 bytes: clocked, kept, not checked
                ST_CRC: begin
                    if (spi_done) begin
                        crc16 <= {crc16[7:0], spi_rx};
                        if (crc_cnt) begin
                            state <= ST_END;
                        end else begin
                            crc_cnt   <= 1'b1;
                            spi_start <= 1'b1;
                        end
                    end
                end

                default: begin      // ST_END
                    cs_n  <= 1'b1;
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
