//=============================================================================
// mm_sd_init.v -- SD card initialisation sequence, SPI mode
//
// Runs the sequence the specification prescribes:
//
//   CMD0   go idle
//   CMD8   probe for a v2 card and check the supply range (argument 0x1AA)
//   CMD55 + ACMD41   repeated until the card reports it has left idle
//   CMD58  read the OCR, whose bit 30 selects block or byte addressing
//
// A card that answers CMD8 with the "illegal command" bit set is a v1 card:
// it is accepted, ACMD41 is retried with a zero argument, and byte addressing
// is used.
//
// Every command is issued through mm_sd_cmd as its own chip select
// transaction, so nothing here needs the select to be held across commands.
// A sector read does, which is why that lives in a separate module.
//
// Response byte layout used throughout:
//   resp[47:40]  R1
//   resp[39:8]   the four trailing bytes of R3 / R7 (OCR, or the CMD8 echo)
//=============================================================================
`timescale 1ns / 100fs

module mm_sd_init #(
    parameter integer POWER_CYCLES = 100_000,   // 2 ms at 50 MHz before CMD0
    parameter integer ACMD41_TRIES = 2000       // then give up and report error
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        start,           // pulse to begin; done/error pulse back
    output reg         busy,
    output reg         done,
    output reg         error,
    output wire [3:0]  state_code,      // current phase, for the status display

    output reg         high_capacity,   // OCR bit 30: block addressing
    output reg  [31:0] ocr,
    output reg  [2:0]  card_type,       // 0 unknown, 1 v1, 2 v2

    // card
    output wire        sd_ncs,
    output wire        sd_sclk,
    output wire        sd_mosi,
    input  wire        sd_miso
);

    // phase is also the code shown on the status display
    localparam [3:0] ST_IDLE   = 4'h0,
                     ST_POWER  = 4'h1,
                     ST_CMD0   = 4'h2,
                     ST_CMD8   = 4'h3,
                     ST_CMD55  = 4'h4,
                     ST_ACMD41 = 4'h5,
                     ST_CMD58  = 4'h6,
                     ST_DONE   = 4'h7,
                     ST_ERROR  = 4'hF;

    reg [3:0]  phase;
    reg [31:0] wait_cnt;
    reg [15:0] acmd_cnt;

    // command layer handshake, entirely internal
    reg         cmd_start;
    reg  [5:0]  cmd_index;
    reg  [31:0] cmd_arg;
    wire [47:0] resp;
    wire [2:0]  resp_len;
    wire        cmd_busy;
    wire        cmd_done;
    wire        cmd_timeout;

    wire [7:0] r1 = resp[47:40];

    assign state_code = phase;      // exposed for the display

    //------------------------------------------------------------------
    // command layer
    //------------------------------------------------------------------
    mm_sd_cmd u_cmd (
        .clk          (clk),
        .rst          (rst),
        .clk_div      (16'd250),        // 100 kHz SPI during initialisation
        .start        (cmd_start),
        .cmd_index    (cmd_index),
        .cmd_arg      (cmd_arg),
        .busy         (cmd_busy),
        .done         (cmd_done),
        .resp         (resp),
        .resp_len     (resp_len),
        .resp_timeout (cmd_timeout),
        .poll_bytes   (),
        .sd_ncs       (sd_ncs),
        .sd_sclk      (sd_sclk),
        .sd_mosi      (sd_mosi),
        .sd_miso      (sd_miso)
    );

    //------------------------------------------------------------------
    // sequence
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            phase         <= ST_IDLE;
            busy          <= 1'b0;
            done          <= 1'b0;
            error         <= 1'b0;
            cmd_start     <= 1'b0;
            cmd_index     <= 6'd0;
            cmd_arg       <= 32'd0;
            wait_cnt      <= 32'd0;
            acmd_cnt      <= 16'd0;
            high_capacity <= 1'b0;
            ocr           <= 32'd0;
            card_type     <= 3'd0;
        end else begin
            cmd_start <= 1'b0;

            case (phase)
                ST_IDLE: begin
                    busy  <= 1'b0;
                    done  <= 1'b0;
                    error <= 1'b0;
                    if (start) begin
                        busy       <= 1'b1;
                        wait_cnt   <= 32'd0;
                        acmd_cnt   <= 16'd0;
                        card_type  <= 3'd0;
                        phase      <= ST_POWER;
                    end
                end

                // let the card finish its own power up before the first command
                ST_POWER: begin
                    if (wait_cnt == POWER_CYCLES - 1) begin
                        cmd_index <= 6'd0;
                        cmd_arg   <= 32'd0;
                        cmd_start <= 1'b1;
                        phase     <= ST_CMD0;
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                // CMD0 must leave the card idle: R1 = 0x01
                ST_CMD0: begin
                    if (cmd_done) begin
                        if (cmd_timeout || (r1 != 8'h01)) begin
                            phase <= ST_ERROR;
                        end else begin
                            cmd_index <= 6'd8;
                            cmd_arg   <= 32'h0000_01AA;
                            cmd_start <= 1'b1;
                            phase     <= ST_CMD8;
                        end
                    end
                end

                // CMD8 either echoes the argument (v2) or is rejected (v1)
                ST_CMD8: begin
                    if (cmd_done) begin
                        if (cmd_timeout) begin
                            phase <= ST_ERROR;
                        end else if (r1[2]) begin
                            card_type <= 3'd1;      // illegal command: v1 card
                            acmd_cnt  <= 16'd0;
                            cmd_index <= 6'd55;
                            cmd_arg   <= 32'd0;
                            cmd_start <= 1'b1;
                            phase     <= ST_CMD55;
                        end else if ((r1 == 8'h01) && (resp[23:8] == 16'h01AA)) begin
                            card_type <= 3'd2;
                            acmd_cnt  <= 16'd0;
                            cmd_index <= 6'd55;
                            cmd_arg   <= 32'd0;
                            cmd_start <= 1'b1;
                            phase     <= ST_CMD55;
                        end else begin
                            phase <= ST_ERROR;
                        end
                    end
                end

                // CMD55 announces that the next command is an application one
                ST_CMD55: begin
                    if (cmd_done) begin
                        cmd_index <= 6'd41;
                        cmd_arg   <= (card_type == 3'd2) ? 32'h4000_0000 : 32'h0000_0000;
                        cmd_start <= 1'b1;
                        phase     <= ST_ACMD41;
                    end
                end

                // ACMD41 keeps returning 0x01 while the card is still waking up
                ST_ACMD41: begin
                    if (cmd_done) begin
                        acmd_cnt <= acmd_cnt + 16'd1;
                        if (!cmd_timeout && (r1 == 8'h00)) begin
                            cmd_index <= 6'd58;
                            cmd_arg   <= 32'd0;
                            cmd_start <= 1'b1;
                            phase     <= ST_CMD58;
                        end else if (acmd_cnt >= ACMD41_TRIES) begin
                            phase <= ST_ERROR;
                        end else begin
                            cmd_index <= 6'd55;
                            cmd_arg   <= 32'd0;
                            cmd_start <= 1'b1;
                            phase     <= ST_CMD55;
                        end
                    end
                end

                // CMD58: OCR, whose bit 30 selects block addressing
                ST_CMD58: begin
                    if (cmd_done) begin
                        if (cmd_timeout || (r1 != 8'h00)) begin
                            phase <= ST_ERROR;
                        end else begin
                            ocr           <= resp[39:8];
                            high_capacity <= resp[38];
                            phase         <= ST_DONE;
                        end
                    end
                end

                ST_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    phase <= ST_IDLE;
                end

                default: begin      // ST_ERROR
                    busy  <= 1'b0;
                    error <= 1'b1;
                    phase <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

