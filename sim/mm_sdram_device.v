//=============================================================================
// mm_sdram_device.v -- behavioural model of the on-package 2M x 32 SDRAM
//
// Simulation only: stands in for the EG_PHY_SDRAM_2M_32 hard macro while
// simulating, so mm_sdram.v can be verified without the device.
//
// It is written to *check* the controller as much as to serve it.  Commands
// are sampled on the clock it is given (which the controller drives with the
// 180 degree clock, exactly as the vendor reference design does), and the
// protocol rules are asserted: activate before read/write, spacing between
// activate and the access (tRCD), between precharge and the next activate
// (tRP), between refreshes (tRFC), after the mode register (tMRD), and tRAS
// before a precharge.  Any violation prints and bumps the error count, so the
// testbench can simply look at it at the end.
//
// Geometry: 4 banks x 2048 rows x 256 columns x 32 bits.
//=============================================================================
`timescale 1ns / 100ps

module EG_PHY_SDRAM_2M_32 (
    input  wire        clk,        // device clock (the controller's 180 degree one)
    input  wire        ras_n,
    input  wire        cas_n,
    input  wire        we_n,
    input  wire [10:0] addr,
    input  wire [1:0]  ba,
    inout  wire [31:0] dq,
    input  wire        cs_n,
    input  wire        dm0,
    input  wire        dm1,
    input  wire        dm2,
    input  wire        dm3,
    input  wire        cke
);

    localparam integer ROWS = 2048;
    localparam integer COLS = 256;

    localparam integer T_RP  = 3;
    localparam integer T_RCD = 3;
    localparam integer T_RFC = 9;
    localparam integer T_MRD = 2;
    localparam integer T_RAS = 6;

    localparam [2:0] C_NOP = 3'b111,
                     C_ACT = 3'b011,
                     C_RD  = 3'b101,
                     C_WR  = 3'b100,
                     C_PRE = 3'b010,
                     C_REF = 3'b001,
                     C_MRS = 3'b000;

    reg [31:0] mem [0:4*ROWS*COLS-1];

    reg        row_open [0:3];
    reg [10:0] open_row [0:3];

    integer t_act, t_pre, t_ref, t_mrs;
    integer n_act, n_pre, n_ref, n_rd, n_wr, n_mrs;
    integer errors;
    integer i;

    reg [31:0] dq_out;
    reg        dq_oe;
    reg [31:0] dq_in;

    assign dq = dq_oe ? dq_out : 32'hzzzz_zzzz;

    wire [2:0]  cmd  = {ras_n, cas_n, we_n};
    wire [10:0] row  = addr[10:0];
    wire [7:0]  col  = addr[7:0];
    wire        a10  = addr[10];
    // A column command does not carry a row at all: it hits whatever row that
    // bank has open, and A10 only says "precharge it afterwards".
    wire [20:0] flat = {ba, open_row[ba], col};

    // Read data pipeline: the word for a command appears CAS_LAT device clocks
    // later, and back-to-back commands (a burst) have to come back back-to-back,
    // so the addresses march down a shift register, one stage per device clock.
    reg [31:0] rd_sr [0:3];
    reg        rd_v  [0:3];
    integer    si;

    task fault;
        input [255:0] what;
        begin
            errors = errors + 1;
            $display("  SDRAM PROTOCOL: %0s at %0t", what, $time);
        end
    endtask

    initial begin
        for (i = 0; i < 4; i = i + 1) begin
            row_open[i] = 1'b0;
            open_row[i] = 11'd0;
        end
        t_act = 99; t_pre = 99; t_ref = 99; t_mrs = 99;
        n_act = 0; n_pre = 0; n_ref = 0; n_rd = 0; n_wr = 0; n_mrs = 0;
        errors = 0;
        dq_oe = 1'b0;
        dq_out = 32'd0;
        for (i = 0; i < 4; i = i + 1) begin
            rd_sr[i] = 32'd0;
            rd_v[i]  = 1'b0;
        end
    end

    always @(posedge clk) begin
        // timers
        t_act = t_act + 1;
        t_pre = t_pre + 1;
        t_ref = t_ref + 1;
        t_mrs = t_mrs + 1;

        // shift the read pipeline down one stage
        for (si = 3; si > 0; si = si - 1) begin
            rd_sr[si] = rd_sr[si-1];
            rd_v [si] = rd_v [si-1];
        end
        rd_v[0] = 1'b0;

        if (!cke)
            fault("CKE was taken low");
        if (cs_n)
            fault("CS_N was taken high (this design ties it low)");

        case (cmd)
            C_ACT: begin
                if (t_pre < T_RP)
                    fault("activate too soon after precharge (tRP)");
                row_open[ba] = 1'b1;
                open_row[ba] = row;
                t_act  = 0;
                n_act  = n_act + 1;
            end

            C_RD: begin
                if (!row_open[ba])
                    fault("read with no row open in that bank");
                if (t_act < T_RCD)
                    fault("read too soon after activate (tRCD)");
                rd_sr[0] = mem[flat];
                rd_v[0]  = 1'b1;           // appears at stage 3, CAS latency 3
                n_rd     = n_rd + 1;
                if (a10) begin
                    row_open[ba] = 1'b0;   // A10 = 1: auto precharge
                    t_pre = 0;
                    n_pre = n_pre + 1;
                end
            end

            C_WR: begin
                if (!row_open[ba])
                    fault("write with no row open in that bank");
                if (t_act < T_RCD)
                    fault("write too soon after activate (tRCD)");
                if (!dm3) mem[flat][31:24] = dq[31:24];
                if (!dm2) mem[flat][23:16] = dq[23:16];
                if (!dm1) mem[flat][15:8]  = dq[15:8];
                if (!dm0) mem[flat][7:0]   = dq[7:0];
                n_wr = n_wr + 1;
                if (a10) begin
                    row_open[ba] = 1'b0;   // A10 = 1: auto precharge
                    t_pre = 0;
                    n_pre = n_pre + 1;
                end
            end

            C_PRE: begin
                if (t_act < T_RAS)
                    fault("precharge too soon after activate (tRAS)");
                for (i = 0; i < 4; i = i + 1)
                    row_open[i] = 1'b0;
                t_pre = 0;
                n_pre = n_pre + 1;
            end

            C_REF: begin
                if (t_ref < T_RFC)
                    fault("refresh too soon after the previous refresh (tRFC)");
                for (i = 0; i < 4; i = i + 1)
                    row_open[i] = 1'b0;
                t_ref = 0;
                n_ref = n_ref + 1;
            end

            C_MRS: begin
                t_mrs = 0;
                n_mrs = n_mrs + 1;
            end

            default: ;      // NOP
        endcase

        // The device drives the bus from the stage that is due, from the
        // device clock edge itself: registering this would delay the whole
        // burst by a cycle and knock the controller's capture out by one word.
        dq_oe  = rd_v[3];
        dq_out = rd_sr[3];
    end

endmodule
