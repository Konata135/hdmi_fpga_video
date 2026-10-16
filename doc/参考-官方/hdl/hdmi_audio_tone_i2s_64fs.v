
module hdmi_audio_tone_i2s_64fs (
    input  wire        I_mclk,                  // 12.288MHz audio master clock
    input  wire        I_rst,
    input  wire        I_audio_enable,          // high after the first image is committed
    input  wire [1:0]  I_track_sel,             // image index 0..3
    input  wire        I_track_update_toggle,   // toggles on every image commit

    output reg         O_i2s_BCLK,
    output reg         O_i2s_LRCK,
    output reg         O_i2s_DOUT
);

    // 48 kHz phase increments for C4, E4, G4 and C5.
    // phase_inc = round(frequency / 48000 * 2^32)
    function [31:0] note_inc_lut;
        input [1:0] index;
        begin
            case (index)
                2'd0: note_inc_lut = 32'd23409862; // C4 261.63Hz
                2'd1: note_inc_lut = 32'd29494578; // E4 329.63Hz
                2'd2: note_inc_lut = 32'd35075155; // G4 392.00Hz
                default: note_inc_lut = 32'd46819716; // C5 523.25Hz
            endcase
        end
    endfunction

    // Synchronize the image-domain update event into the audio clock domain.
    reg [2:0] S_update_sync;
    reg [2:0] S_enable_sync;
    wire      S_update_pulse;
    wire      S_audio_enable;

    assign S_update_pulse = S_update_sync[2] ^ S_update_sync[1];
    assign S_audio_enable = S_enable_sync[2];

    reg [1:0]  S_track_sel;
    reg [31:0] S_phase_acc;
    wire [31:0] S_note_inc;
    wire [5:0]  S_sine_addr;
    wire signed [23:0] S_sine_data;
    reg signed [23:0]  S_sample24;

    assign S_note_inc = note_inc_lut(S_track_sel);
    assign S_sine_addr = S_phase_acc[31:26];

    AUDIO_SAMPLE_ROM u_audio_sample_rom (
        .doa  (S_sine_data),
        .addra(S_sine_addr),
        .clka (I_mclk)
    );

    reg [5:0]  S_bit_cnt;
    reg [63:0] S_shift_reg;

    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst) begin
            S_update_sync <= 3'b000;
            S_enable_sync <= 3'b000;
            S_track_sel   <= 2'd0;
            S_phase_acc   <= 32'd0;
            S_sample24    <= 24'sd0;
            S_bit_cnt     <= 6'd0;
            S_shift_reg   <= 64'd0;
            O_i2s_BCLK    <= 1'b0;
            O_i2s_LRCK    <= 1'b0;
            O_i2s_DOUT    <= 1'b0;
        end else begin
            S_update_sync <= {S_update_sync[1:0], I_track_update_toggle};
            S_enable_sync <= {S_enable_sync[1:0], I_audio_enable};

            // The image index is stable when the update toggle is synchronized.
            if (S_update_pulse) begin
                S_track_sel <= I_track_sel;
                S_phase_acc <= 32'd0;
            end

            // Sine ROM data is stable before it is loaded into the I2S frame.
            S_sample24 <= S_sine_data;

            // 12.295MHz MCLK toggled once per cycle gives about 6.147MHz BCLK.
            O_i2s_BCLK <= ~O_i2s_BCLK;
            if (O_i2s_BCLK == 1'b1) begin
                O_i2s_DOUT  <= S_shift_reg[63];
                S_shift_reg <= {S_shift_reg[62:0], 1'b0};

                if (S_bit_cnt == 6'd63) begin
                    S_bit_cnt <= 6'd0;

                    if (O_i2s_LRCK == 1'b0) begin
                        // Left channel finished; send the current sample on the right.
                        O_i2s_LRCK <= 1'b1;
                        S_shift_reg <= {S_audio_enable ? S_sample24 : 24'sd0, 40'd0};
                    end else begin
                        // Right channel finished; calculate the next 48 kHz sample.
                        O_i2s_LRCK <= 1'b0;
                        S_shift_reg <= {S_audio_enable ? S_sample24 : 24'sd0, 40'd0};

                        if (!S_audio_enable || S_update_pulse)
                            S_phase_acc <= 32'd0;
                        else
                            S_phase_acc <= S_phase_acc + S_note_inc;
                    end
                end else begin
                    S_bit_cnt <= S_bit_cnt + 6'd1;
                end
            end
        end
    end

endmodule
