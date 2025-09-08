/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_lcd_controller_Andres078(
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // ------------------------------------------------------------
    // Tie-offs / unused
    // ------------------------------------------------------------
    // Do NOT read outputs in the _unused pack; just consume inputs.
    wire _unused = &{ena, ui_in, uio_in, 1'b0};

    // No bidirectional user IOs used: keep them inputs.
    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    // ------------------------------------------------------------
    // LCD interface signals (drive dedicated outputs)
    // ------------------------------------------------------------
    reg        rs;     // register select
    reg        en;     // enable
    reg  [3:0] data;   // D7..D4

    // Map to uo_out (drive all bits!)
    assign uo_out[0]   = rs;
    assign uo_out[1]   = en;
    assign uo_out[3:2] = 2'b00;  // unused, tie low
    assign uo_out[7:4] = data;

    // ------------------------------------------------------------
    // Timing parameters (HD44780)
    // Using localparams to fold at elaboration time (synth-safe).
    // ------------------------------------------------------------
    localparam [31:0] CLK_FREQ        = 32'd50_000_000;      // Hz
    localparam [31:0] EN_PULSE_NS     = 32'd500;             // >=450 ns
    localparam [31:0] EN_PULSE_CYC    = (CLK_FREQ * EN_PULSE_NS) / 32'd1_000_000_000 + 32'd1;

    localparam [31:0] DELAY_15MS_CYC  = (CLK_FREQ * 32'd15)   / 32'd1000;
    localparam [31:0] DELAY_4MS_CYC   = (CLK_FREQ * 32'd4100) / 32'd1_000_000;
    localparam [31:0] DELAY_100US_CYC = (CLK_FREQ * 32'd100)  / 32'd1_000_000;
    localparam [31:0] DELAY_40US_CYC  = (CLK_FREQ * 32'd40)   / 32'd1_000_000;
    localparam [31:0] DELAY_1_6MS_CYC = (CLK_FREQ * 32'd1600) / 32'd1_000_000;

    // ------------------------------------------------------------
    // Message ROM (avoid initial blocks for ASIC)
    // "THE GAME  " (10 chars)
    // ------------------------------------------------------------
    localparam integer MSG_LEN   = 10;
    localparam [3:0]   MSG_LEN_4 = 4'd10;

    function [7:0] msg_byte;
        input [3:0] idx;
        begin
            case (idx)
                4'd0: msg_byte = "T";
                4'd1: msg_byte = "H";
                4'd2: msg_byte = "E";
                4'd3: msg_byte = " ";
                4'd4: msg_byte = "G";
                4'd5: msg_byte = "A";
                4'd6: msg_byte = "M";
                4'd7: msg_byte = "E";
                4'd8: msg_byte = " ";
                4'd9: msg_byte = " ";
                default: msg_byte = 8'h20; // space
            endcase
        end
    endfunction

    // ------------------------------------------------------------
    // Top-level FSM
    // ------------------------------------------------------------
    localparam [4:0]
        S_IDLE       = 5'd0,
        S_WAIT_15MS  = 5'd1,
        S_INIT_1     = 5'd2,
        S_INIT_2     = 5'd3,
        S_INIT_3     = 5'd4,
        S_SET_4BIT   = 5'd5,
        S_FUNC_SET   = 5'd6,
        S_DISP_OFF   = 5'd7,
        S_CLEAR      = 5'd8,
        S_ENTRY      = 5'd9,
        S_DISP_ON    = 5'd10,
        S_WRITE      = 5'd11,
        S_WAIT_BYTE  = 5'd12,
        S_DONE       = 5'd13;

    reg [4:0] state, next_state;

    // ------------------------------------------------------------
    // Byte sender (4-bit mode) control
    // ------------------------------------------------------------
    reg        byte_go;
    reg        byte_is_data;   // 1=data (RS=1), 0=command (RS=0)
    reg [7:0]  byte_val;
    reg        byte_done;

    localparam [2:0]
        B_IDLE   = 3'd0,
        B_SETUPH = 3'd1,
        B_ENHH   = 3'd2,
        B_ENHL   = 3'd3,
        B_SETUPL = 3'd4,
        B_ENLH   = 3'd5,
        B_ENLL   = 3'd6;

    reg [2:0]  bstate;
    reg [31:0] en_cnt;
    reg [31:0] wait_cnt; // reserved, not used otherwise

    // ------------------------------------------------------------
    // Counters and step control
    // ------------------------------------------------------------
    reg [31:0] delay_cnt;
    reg [3:0]  msg_idx;

    localparam [3:0]
        STEP_NONE  = 4'd0,
        STEP_INIT1 = 4'd1,
        STEP_INIT2 = 4'd2,
        STEP_INIT3 = 4'd3,
        STEP_SET4  = 4'd4,
        STEP_FSET  = 4'd5,
        STEP_DOFF  = 4'd6,
        STEP_CLEAR = 4'd7,
        STEP_ENTRY = 4'd8,
        STEP_DON   = 4'd9,
        STEP_WRITE = 4'd10;

    reg [3:0] step;

    // In WAIT_BYTE: 0=wait for byte_done, 1=count post-delay
    reg wait_phase;

    // ============================================================
    // Single-process FSM with active-low asynchronous reset
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Outputs
            rs         <= 1'b0;
            en         <= 1'b0;
            data       <= 4'd0;

            // FSM
            state      <= S_IDLE;
            next_state <= S_IDLE;

            // Byte sender
            bstate       <= B_IDLE;
            en_cnt       <= 32'd0;
            wait_cnt     <= 32'd0;
            byte_go      <= 1'b0;
            byte_is_data <= 1'b0;
            byte_val     <= 8'h00;
            byte_done    <= 1'b0;

            // Counters/steps
            delay_cnt  <= 32'd0;
            msg_idx    <= 4'd0;
            step       <= STEP_NONE;
            wait_phase <= 1'b0;

        end else begin
            // Advance state
            state <= next_state;

            // Byte sender engine
            byte_done <= 1'b0; // one-cycle pulse when finishing
            case (bstate)
                B_IDLE: begin
                    en <= 1'b0;
                    if (byte_go) begin
                        rs      <= byte_is_data;
                        data    <= byte_val[7:4];
                        en_cnt  <= 32'd0;
                        wait_cnt<= 32'd0;
                        bstate  <= B_SETUPH;
                    end
                end

                B_SETUPH: begin
                    bstate <= B_ENHH;
                    en     <= 1'b1;
                    en_cnt <= 32'd1;
                end

                B_ENHH: begin
                    if (en_cnt < EN_PULSE_CYC) begin
                        en_cnt <= en_cnt + 32'd1;
                    end else begin
                        en     <= 1'b0;
                        en_cnt <= 32'd0;
                        bstate <= B_ENHL;
                    end
                end

                B_ENHL: begin
                    if (en_cnt < EN_PULSE_CYC) begin
                        en_cnt <= en_cnt + 32'd1;
                    end else begin
                        data   <= byte_val[3:0];
                        en_cnt <= 32'd0;
                        bstate <= B_SETUPL;
                    end
                end

                B_SETUPL: begin
                    bstate <= B_ENLH;
                    en     <= 1'b1;
                    en_cnt <= 32'd1;
                end

                B_ENLH: begin
                    if (en_cnt < EN_PULSE_CYC) begin
                        en_cnt <= en_cnt + 32'd1;
                    end else begin
                        en     <= 1'b0;
                        en_cnt <= 32'd0;
                        bstate <= B_ENLL;
                    end
                end

                B_ENLL: begin
                    if (en_cnt < EN_PULSE_CYC) begin
                        en_cnt <= en_cnt + 32'd1;
                    end else begin
                        bstate    <= B_IDLE;
                        byte_done <= 1'b1;
                        byte_go   <= 1'b0; // consume request
                    end
                end

                default: bstate <= B_IDLE;
            endcase

            // Top-level FSM
            case (state)
                S_IDLE: begin
                    delay_cnt  <= 32'd0;
                    msg_idx    <= 4'd0;
                    step       <= STEP_NONE;
                    wait_phase <= 1'b0;
                    next_state <= S_WAIT_15MS;
                end

                // Initial wait >15 ms
                S_WAIT_15MS: begin
                    if (delay_cnt >= DELAY_15MS_CYC) begin
                        delay_cnt    <= 32'd0;
                        // Send first 0x30 (as 4-bit: high nibble 0x3, then low)
                        byte_is_data <= 1'b0;
                        byte_val     <= 8'h30;
                        byte_go      <= 1'b1;
                        step         <= STEP_INIT1;
                        wait_phase   <= 1'b0;   // wait for byte_done first
                        next_state   <= S_WAIT_BYTE;
                    end else begin
                        delay_cnt    <= delay_cnt + 32'd1;
                        next_state   <= S_WAIT_15MS;
                    end
                end

                // Flow goes through S_WAIT_BYTE; these "states" are placeholders
                S_INIT_1: begin
                    next_state <= S_INIT_1;
                end

                S_INIT_2: begin
                    byte_is_data <= 1'b0;
                    byte_val     <= 8'h30;
                    byte_go      <= 1'b1;
                    step         <= STEP_INIT2;
                    delay_cnt    <= 32'd0;
                    wait_phase   <= 1'b0;
                    next_state   <= S_WAIT_BYTE;
                end

                S_INIT_3: begin
                    byte_is_data <= 1'b0;
                    byte_val     <= 8'h30;
                    byte_go      <= 1'b1;
                    step         <= STEP_INIT3;
                    delay_cnt    <= 32'd0;
                    wait_phase   <= 1'b0;
                    next_state   <= S_WAIT_BYTE;
                end

                S_SET_4BIT: begin
                    byte_is_data <= 1'b0;
                    byte_val     <= 8'h20; // 4-bit mode
                    byte_go      <= 1'b1;
                    step         <= STEP_SET4;
                    delay_cnt    <= 32'd0;
                    wait_phase   <= 1'b0;
                    next_state   <= S_WAIT_BYTE;
                end

                S_FUNC_SET: begin
                    byte_is_data <= 1'b0;
                    byte_val     <= 8'h28; // 4-bit, 2 lines, 5x8
                    byte_go      <= 1'b1;
                    step         <= STEP_FSET;
                    delay_cnt    <= 32'd0;
                    wait_phase   <= 1'b0;
                    next_state   <= S_WAIT_BYTE;
                end

                S_DISP_OFF: begin
                    byte_is_data <= 1'b0;
                    byte_val     <= 8'h08; // display off
                    byte_go      <= 1'b1;
                    step         <= STEP_DOFF;
                    delay_cnt    <= 32'd0;
                    wait_phase   <= 1'b0;
                    next_state   <= S_WAIT_BYTE;
                end

                S_CLEAR: begin
                    byte_is_data <= 1'b0;
                    byte_val     <= 8'h01; // clear
                    byte_go      <= 1'b1;
                    step         <= STEP_CLEAR;
                    delay_cnt    <= 32'd0;
                    wait_phase   <= 1'b0;
                    next_state   <= S_WAIT_BYTE;
                end

                S_ENTRY: begin
                    byte_is_data <= 1'b0;
                    byte_val     <= 8'h06; // entry mode I/D=1, S=0
                    byte_go      <= 1'b1;
                    step         <= STEP_ENTRY;
                    delay_cnt    <= 32'd0;
                    wait_phase   <= 1'b0;
                    next_state   <= S_WAIT_BYTE;
                end

                S_DISP_ON: begin
                    byte_is_data <= 1'b0;
                    byte_val     <= 8'h0C; // display on, cursor off, blink off
                    byte_go      <= 1'b1;
                    step         <= STEP_DON;
                    delay_cnt    <= 32'd0;
                    wait_phase   <= 1'b0;
                    next_state   <= S_WAIT_BYTE;
                end

                // Write message (one byte at a time)
                S_WRITE: begin
                    if (msg_idx < MSG_LEN_4) begin
                        byte_is_data <= 1'b1;
                        byte_val     <= msg_byte(msg_idx);
                        byte_go      <= 1'b1;
                        step         <= STEP_WRITE;
                        delay_cnt    <= 32'd0;
                        wait_phase   <= 1'b0;
                        next_state   <= S_WAIT_BYTE;
                    end else begin
                        next_state   <= S_DONE;
                    end
                end

                // Wait for byte_done, then post-delay depending on step
                S_WAIT_BYTE: begin
                    if (wait_phase == 1'b0) begin
                        if (byte_done) begin
                            wait_phase <= 1'b1;     // start counting delay
                            delay_cnt  <= 32'd0;
                        end
                        next_state <= S_WAIT_BYTE;
                    end else begin
                        case (step)
                            STEP_INIT1: begin
                                if (delay_cnt >= DELAY_4MS_CYC) begin
                                    delay_cnt  <= 32'd0;
                                    wait_phase <= 1'b0;
                                    next_state <= S_INIT_2;
                                end else begin
                                    delay_cnt  <= delay_cnt + 32'd1;
                                    next_state <= S_WAIT_BYTE;
                                end
                            end
                            STEP_INIT2: begin
                                if (delay_cnt >= DELAY_100US_CYC) begin
                                    delay_cnt  <= 32'd0;
                                    wait_phase <= 1'b0;
                                    next_state <= S_INIT_3;
                                end else begin
                                    delay_cnt  <= delay_cnt + 32'd1;
                                    next_state <= S_WAIT_BYTE;
                                end
                            end
                            STEP_INIT3: begin
                                if (delay_cnt >= DELAY_100US_CYC) begin
                                    delay_cnt  <= 32'd0;
                                    wait_phase <= 1'b0;
                                    next_state <= S_SET_4BIT;
                                end else begin
                                    delay_cnt  <= delay_cnt + 32'd1;
                                    next_state <= S_WAIT_BYTE;
                                end
                            end
                            STEP_SET4: begin
                                if (delay_cnt >= DELAY_100US_CYC) begin
                                    delay_cnt  <= 32'd0;
                                    wait_phase <= 1'b0;
                                    next_state <= S_FUNC_SET;
                                end else begin
                                    delay_cnt  <= delay_cnt + 32'd1;
                                    next_state <= S_WAIT_BYTE;
                                end
                            end
                            STEP_FSET: begin
                                if (delay_cnt >= DELAY_40US_CYC) begin
                                    delay_cnt  <= 32'd0;
                                    wait_phase <= 1'b0;
                                    next_state <= S_DISP_OFF;
                                end else begin
                                    delay_cnt  <= delay_cnt + 32'd1;
                                    next_state <= S_WAIT_BYTE;
                                end
                            end
                            STEP_DOFF: begin
                                if (delay_cnt >= DELAY_40US_CYC) begin
                                    delay_cnt  <= 32'd0;
                                    wait_phase <= 1'b0;
                                    next_state <= S_CLEAR;
                                end else begin
                                    delay_cnt  <= delay_cnt + 32'd1;
                                    next_state <= S_WAIT_BYTE;
                                end
                            end
                            STEP_CLEAR: begin
                                if (delay_cnt >= DELAY_1_6MS_CYC) begin
                                    delay_cnt  <= 32'd0;
                                    wait_phase <= 1'b0;
                                    next_state <= S_ENTRY;
                                end else begin
                                    delay_cnt  <= delay_cnt + 32'd1;
                                    next_state <= S_WAIT_BYTE;
                                end
                            end
                            STEP_ENTRY: begin
                                if (delay_cnt >= DELAY_40US_CYC) begin
                                    delay_cnt  <= 32'd0;
                                    wait_phase <= 1'b0;
                                    next_state <= S_DISP_ON;
                                end else begin
                                    delay_cnt  <= delay_cnt + 32'd1;
                                    next_state <= S_WAIT_BYTE;
                                end
                            end
                            STEP_DON: begin
                                if (delay_cnt >= DELAY_40US_CYC) begin
                                    delay_cnt  <= 32'd0;
                                    wait_phase <= 1'b0;
                                    msg_idx    <= 4'd0;
                                    next_state <= S_WRITE;
                                end else begin
                                    delay_cnt  <= delay_cnt + 32'd1;
                                    next_state <= S_WAIT_BYTE;
                                end
                            end
                            STEP_WRITE: begin
                                if (delay_cnt >= DELAY_40US_CYC) begin
                                    delay_cnt  <= 32'd0;
                                    wait_phase <= 1'b0;
                                    msg_idx    <= msg_idx + 4'd1;
                                    next_state <= S_WRITE;
                                end else begin
                                    delay_cnt  <= delay_cnt + 32'd1;
                                    next_state <= S_WAIT_BYTE;
                                end
                            end
                            default: begin
                                wait_phase <= 1'b0;
                                next_state <= S_DONE;
                            end
                        endcase
                    end
                end

                S_DONE: begin
                    // Hold lines idle
                    en         <= 1'b0;
                    rs         <= 1'b0;
                    data       <= 4'd0;
                    next_state <= S_DONE;
                end

                default: begin
                    next_state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
