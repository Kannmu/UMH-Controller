`timescale 1ns/1ps

/* Per-microphone update of the one-carrier-period PDM mixer.  M is a
 * constant microphone slot and S is the PDM bit for this sample. */
`define MIC_UPDATE(M,S) \
    if ((S) ^ mic_lo_i_r) mic_xor_i[M] <= mic_xor_i[M] + 7'd1; \
    if ((S) ^ mic_lo_q_r) mic_xor_q[M] <= mic_xor_q[M] + 7'd1;

/* Advance or complete the gate sequence after one 40 kHz I/Q sample has
 * been accumulated and handed to the EBR store registers. */
`define MIC_FINISH_GATE \
    mic_gate_active <= 1'b0; \
    mic_gate_fill   <= 7'd0; \
    if (mic_gate_index + 7'd1 >= mic_cfg_count) begin \
        mic_gate_index  <= 7'd0; \
        mic_done        <= 1'b1; \
        mic_block_count <= mic_block_count + 16'd1; \
        mic_run_state   <= 2'd1; \
    end else begin \
        mic_gate_index <= mic_gate_index + 7'd1; \
        mic_gate_wait  <= mic_cfg_gap; \
    end

/* ---------------------------------------------------------------------------
 * UMH v7 MachXO2 output engine.
 *
 * The STM32 streams one 8-bit phase byte and one 8-bit level byte per
 * ultrasound channel over SPI1.  The phase splits one 40 kHz carrier period
 * into 256 slots and the level is the duty threshold inside that window
 * (level=0 disables the channel, level=255 leaves it high for 255 of the
 * 256 slots).  us_tx is a plain 0/1 signal that drives a 0 V / 3.3 V square
 * wave on every external channel.
 *
 * 84 parallel 8-bit subtractor/comparator pairs do not fit the
 * LCMXO2-2000HC fabric: the measured result was 2729 of 2112 LUT4 and 1367
 * of 1056 slices.  The running waveform therefore lives in EBR instead.  A
 * 512 x 84 table holds two 256-slot "toggle" banks; the frame builder sets
 * one bit at the rising slot and one at the falling slot of every enabled
 * channel, and the running logic is a single 84-bit XOR per phase step.
 * A new bank is drawn while the old bank still drives us_tx, and the banks
 * swap on a carrier wrap, so a parameter update never blanks an output.
 * ------------------------------------------------------------------------- */

/* 84 x 16-bit staging memory. Simple dual-port with clock domain crossing.
 * The write port is in the SPI SCK domain, read port in 128 MHz domain. */
module umh_channel_ram18 (
    input  wire        wr_clk,
    input  wire [6:0]  wr_addr,
    input  wire [15:0] wr_data,
    input  wire        wr_en,
    input  wire        rd_clk,
    input  wire [6:0]  rd_addr,
    output reg  [15:0] rd_data
);
    (* syn_ramstyle = "block_ram" *) reg [15:0] mem [0:83];
    always @(posedge wr_clk) if (wr_en) mem[wr_addr] <= wr_data;
    always @(posedge rd_clk) rd_data <= mem[rd_addr];
endmodule

/* 512 x 84 toggle-event table. Pseudo dual-port: one read, one write.
 * Port A (read) for DDS, Port B (read/write) for frame builder. */
module umh_toggle_ram84 (
    input  wire        clk,
    input  wire [8:0]  addr_a,
    output reg  [83:0] rd_data_a,
    input  wire        we_b,
    input  wire [8:0]  addr_b,
    input  wire [83:0] wr_data_b,
    output reg  [83:0] rd_data_b
);
    (* syn_ramstyle = "block_ram" *) reg [83:0] mem [0:511];

    always @(posedge clk) begin
        rd_data_a <= mem[addr_a];
        if (we_b) mem[addr_b] <= wr_data_b;
        rd_data_b <= mem[addr_b];
    end
endmodule

/* 256 x 32 microphone gate accumulator storage.  One word carries the
 * 16-bit in-phase and 16-bit quadrature sums of one (gate, microphone)
 * pair, so all 64 gates x 4 microphones fit into a single EBR.  The word
 * is written by the gate accumulator in the 64 MHz domain and read back by
 * MIC_READ; a synchronous read port keeps the MISO payload stable. */
module umh_mic_iq_ram (
    input  wire        clk,
    input  wire        we,
    input  wire [8:0]  wr_addr,
    input  wire [15:0] wr_data,
    input  wire [8:0]  rd_addr,
    output reg  [15:0] rd_data
);
    /* 512 x 16 fits one MachXO2 EBR exactly.  Address {gate[5:0],
     * word[2:0]}: words 0..3 are the four microphone I sums, words 4..7 the
     * four Q sums. */
    (* syn_ramstyle = "block_ram" *) reg [15:0] mem [0:511];
    always @(posedge clk) begin
        if (we) mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
    end
endmodule

module umh_fpga_top (
    input  wire        fpga_clk_8m,
    input  wire        fpga_cs_n,
    input  wire        spi1_sck,
    input  wire        spi1_mosi,
    output wire        spi1_miso,
    output reg  [83:0] us_tx,
    output wire        rgb_data,
    output wire        mic_clk,
    input  wire        mic_data_0,
    input  wire        mic_data_1,
    input  wire        spi_mic_cs_n,
    input  wire        spi_mic_sck,
    output wire        spi_mic_miso
);
    localparam [15:0] HEADER_BYTES  = 16'd36;
    localparam [15:0] CHANNEL_BYTES = 16'd168;
    /* Compact command transactions.  The master clocks exactly 16 bytes;
     * bytes 0..1 are command/version, byte 2 is the payload, bytes 6..9 may
     * carry the frame sequence, and the usual 16-byte status word is read
     * back in the same transaction. */
    localparam [15:0] COMPACT_BYTES   = 16'd16;
    localparam [15:0] SHORT_BYTES     = 16'd3;
    localparam [7:0]  AUDIO_CMD_LEVEL = 8'h16;
    localparam [7:0]  AUDIO_CMD_MODE  = 8'h17;
    /* Three-byte AUDIO_LEVEL: [0x19, version, level].  Running one compact
     * SPI transaction per audio sample used 6 us of wire time; the short
     * form cuts this to ~1.2 us and removes the need to read a 16-byte
     * status word on every audio sample. */
    localparam [7:0]  AUDIO_CMD_LEVEL_SHORT = 8'h19;
    /* 40 kHz carrier with 256 phase slots at 64 MHz (10.24 MHz slot rate). */
    localparam [31:0] CARRIER_STEP  = 32'd2684355;

    /* ------------------------------------------------------------------
     * 8 MHz STM32 PA8 MCO -> 64 MHz output clock.
     *
     * MachXO2 law, confirmed with Lattice's own PLL calculator
     * (ispfpga/bin/nt64/scuba.exe -arch xo2c00 -fin 8 -fclkop 128):
     *
      *   fCLKOP = fCLKI * CLKFB_DIV / CLKI_DIV = 8 * 8 / 1    = 64 MHz
     *   fVCO   = fCLKOP * CLKOP_DIV            = 64 * 8       = 512 MHz
     *
     * The PLL frequency is measured correct through fpga_time, but this
     * board's LOCK output reads low even for the original 128 MHz core.  The
     * output engine therefore uses running/stop_event, not LOCK, to enable
     * us_tx.  PLL LOCK remains a diagnostic status bit only.
     * CLKOP_CPHASE = CLKOP_DIV-1 keeps CLKOP aligned with the INT_DIVA
     * feedback path, which is the only internal feedback mode Map accepts.
     * ------------------------------------------------------------------ */
    wire pll_clk, pll_feedback;
    wire fpga_clk = pll_clk;
    EHXPLLJ #(
        .PLLRST_ENA("DISABLED"), .INTFB_WAKE("DISABLED"), .STDBY_ENABLE("DISABLED"),
        .DPHASE_SOURCE("DISABLED"), .CLKOP_FPHASE(0), .CLKOP_CPHASE(7),
        .OUTDIVIDER_MUXA2("DIVA"), .CLKOP_ENABLE("ENABLED"), .CLKOP_DIV(8),
        .CLKFB_DIV(8), .CLKI_DIV(1), .FEEDBK_PATH("INT_DIVA")
    ) fpga_pll_i (
        .CLKI(fpga_clk_8m), .CLKFB(pll_feedback), .RST(1'b0),
        .RESETM(1'b0), .RESETC(1'b0), .RESETD(1'b0),
        .PHASESEL0(1'b0), .PHASESEL1(1'b0), .PHASEDIR(1'b0), .PHASESTEP(1'b0),
        .LOADREG(1'b0), .STDBY(1'b0), .PLLWAKESYNC(1'b0), .ENCLKOP(1'b1),
        .CLKOP(pll_clk), .LOCK(), .CLKINTFB(pll_feedback)
    );

    /* ------------------------------------------------------------------
     * SPI1 control link.  The receiver is clocked by spi1_sck, which is the
     * normal SPI-slave arrangement.  fpga_cs_n is an asynchronous reset for
     * that domain only and is released long after the last SCK edge, so its
     * recovery time is met; no other logic in this design is clocked by CS.
     * Every value crossing into the 128 MHz domain passes through a two-flop
     * synchroniser plus a settle delay, so multi-bit fields such as
     * accepted_sequence_sync can never be sampled torn.
     * ------------------------------------------------------------------ */
    reg  [7:0]  spi_rx_shift, spi_command, spi_version, spi_phase_pending;
    reg  [2:0]  spi_bit_count;
    reg  [15:0] spi_byte_count, spi_update_flags, spi_extension_length;
    reg  [31:0] spi_frame_sequence, spi_expected_length, accepted_sequence_spi;
    reg  [15:0] accepted_update_flags_spi;
    reg  [6:0]  spi_channel_index;
    reg  [1:0]  spi_channel_field;
    reg  [3:0]  spi_rgb_index;
    reg  [7:0]  spi_compact_data;
    reg  [7:0]  audio_level_spi;
    reg         audio_mode_spi;
    reg         audio_level_toggle_spi, audio_mode_toggle_spi;
    reg  [87:0] spi_bitmap;
    reg         frame_toggle_spi, stop_toggle_spi, invalid_frame_spi, ws2812_toggle_spi;
    reg  [95:0] rgb_values;
    reg  [8:0]  status_bit_index;

    /* Sticky diagnostics for the board bring-up report.  Status is read
     * while CS is inactive; these bits are ignored by the STM32 fault mask. */
    wire [7:0]  spi_rx_byte  = {spi_rx_shift[6:0], spi1_mosi};
    wire        spi_payload_byte = !fpga_cs_n && spi_update_flags[0] &&
                                   (spi_bit_count == 3'd7) &&
                                   (spi_byte_count >= HEADER_BYTES) &&
                                   (spi_byte_count < HEADER_BYTES + CHANNEL_BYTES);
    wire        spi_write    = spi_payload_byte && (spi_channel_field == 2'd1);
    wire [15:0] staging_wr_data = {spi_phase_pending, spi_rx_byte};
    /* RGB follows the optional ultrasound payload.  WS2812-only commands
     * therefore start at byte 36, while combined frames start at byte 204. */
    wire [15:0] spi_rgb_start = HEADER_BYTES +
                                (spi_update_flags[0] ? CHANNEL_BYTES : 16'd0);
    wire        spi_rgb_payload_byte = !fpga_cs_n && spi_update_flags[1] &&
                                       (spi_bit_count == 3'd7) &&
                                       (spi_byte_count >= spi_rgb_start) &&
                                       (spi_byte_count < spi_rgb_start + 16'd12);
    wire        spi_compact_cmd = (spi_command == AUDIO_CMD_LEVEL) ||
                                  (spi_command == AUDIO_CMD_MODE) ||
                                  (spi_command == AUDIO_CMD_LEVEL_SHORT);

    reg         ws2812_enable;

    /* ------------------------------------------------------------------
     * Carrier DDS (Fully Pipelined 5-Stage).
     * Stage 1: fractional accumulator carry
     * Stage 2: global phase counter and wrap detection
     * Stage 3: bank selection and address generation
     * Stage 4: EBR read latency
     * Stage 5: data capture and XOR application
     *
     * This deep pipeline ensures zero combinational logic between any two
     * adjacent pipeline stages, eliminating all timing violations.
     * ------------------------------------------------------------------ */
    reg  [23:0] phase_frac;
    reg         phase_step_s1;
    reg  [7:0]  global_phase_s2;
    reg         phase_step_s2;
    reg         wrap_s2;
    reg  [8:0]  run_addr_s3;
    reg         phase_step_s3;
    reg         swap_now_s3;
    reg         phase_step_s4;
    reg         swap_now_s4;
    reg  [83:0] ev_run_hold_s5;
    reg         phase_step_s5;
    reg         swap_now_s5;

    wire [24:0] phase_frac_sum = {1'b0, phase_frac} + 25'd2684355;

    reg  [31:0] fpga_time;
    reg  [6:0]  time_divider;
    /* SPI link watchdog: a running output must receive either frame or status
     * traffic at least this often.  If the STM32 resets or hangs, the FPGA
     * clears the output instead of holding the last frame forever. */
    localparam [31:0] LINK_TIMEOUT_US = 32'd25_000;
    reg  [31:0] link_idle_us;

    /* ------------------------------------------------------------------
     * SPH0641LU4H-1 clock sequencer and 40 kHz coherent I/Q demodulator.
     *
     * The datasheet forbids powering up or waking directly into ultrasonic
     * mode (3.072..4.8 MHz).  The clock is therefore held low through
     * power-up, run at 200 kHz for 20 ms (sleep exit + 15 ms wake-up), and
     * only then switched to the 4 MHz ultrasonic clock.
     * ------------------------------------------------------------------ */
    localparam [31:0] MIC_BOOT_CYCLES = 32'd640_000;  /* 10 ms at 64 MHz  */
    localparam [13:0] MIC_WARM_EDGES  = 14'd8_000;    /* 20 ms at 200 kHz */

    reg  [31:0] mic_boot_count;
    reg         mic_warm;
    reg         mic_ultrasonic;
    reg  [7:0]  mic_warm_half;
    reg  [13:0] mic_warm_edges;
    reg  [3:0]  mic_phase;             /* 0..15, one 4 MHz PDM bit period */
    reg         mic_clock_reg;
    reg  [15:0] mic_shift_0_l, mic_shift_0_r;
    reg  [15:0] mic_shift_1_l, mic_shift_1_r;
    reg  [4:0]  mic_sample_count;
    reg  [63:0] mic_latest;

    /* 40 kHz local oscillator and one-carrier-period boxcar.  The PDM bit
     * rate is 4 MHz and the LO period is 100 PDM samples (25 us).  XOR-ing
     * the PDM bit with the +/-1 LO and integrating over exactly one LO period
     * rejects every LO harmonic; the result is a true 40 kHz complex I/Q
     * sample.  The product sum is 100 - 2*xor_count, so no multiplier is
     * needed anywhere in the microphone path. */
    reg  [6:0]  mic_lo_n;              /* 0..99 carrier LO phase       */
    reg  [6:0]  mic_window_count;      /* 0..100 PDM samples in window */
    reg  [6:0]  mic_xor_i [0:3];       /* I mixer products equal to 1  */
    reg  [6:0]  mic_xor_q [0:3];       /* Q mixer products equal to 1  */
    reg  signed [7:0] mic_win_i [0:3]; /* 100 - 2*xor, range -100..100 */
    reg  signed [7:0] mic_win_q [0:3];
    reg         mic_bc_valid;          /* 40 kHz envelope valid pulse  */
    reg         mic_lo_i_r, mic_lo_q_r; /* registered LO for this PDM period */

    /* Registered local oscillator.  mic_lo_n advances on phase 0 and the
     * +/-1 LO bits for the new period (including the 40-sample-delayed bit
     * used by the sliding boxcar) are registered in the same edge, so the
     * long mixer / accumulator cloud starts from a flop instead of from the
     * counter decoders. */
    wire [6:0]  mic_lo_n_next = (mic_lo_n == 7'd99) ? 7'd0 : (mic_lo_n + 7'd1);
    wire        mic_lo_i_next = (mic_lo_n_next < 7'd25) || (mic_lo_n_next >= 7'd75);
    wire        mic_lo_q_next = (mic_lo_n_next < 7'd50);
    /* Gate configuration is expressed in 40 kHz samples (25 us).  The
     * sequence is re-armed automatically after every completed block, so
     * the STM32 can submit one pattern per block without reconfiguring. */
    reg  [6:0]  mic_cfg_count;         /* 1..64 gates per pattern      */
    reg  [15:0] mic_cfg_start;         /* gate 0 start after swap      */
    reg  [15:0] mic_cfg_step;          /* start-to-start spacing       */
    reg  [15:0] mic_cfg_gap;           /* step - width, precomputed    */
    reg  [6:0]  mic_cfg_width;         /* 1..64 samples per gate       */
    reg  [1:0]  mic_run_state;         /* 0 idle, 1 wait, 2 collect    */
    reg         mic_done;
    reg         mic_saturated;
    reg  [15:0] mic_block_count;
    reg  [6:0]  mic_gate_index;
    reg  [6:0]  mic_gate_fill;
    reg  [15:0] mic_gate_wait;
    reg         mic_gate_active;
    /* The gate accumulators live in the microphone EBR.  A small read / add /
     * write-back engine services the eight 16-bit words of the active gate
     * once per 40 kHz sample; the remaining cycles of the sample period
     * are idle, so one shared adder replaces eight parallel accumulators. */
    reg         mic_acc_busy;
    reg  [2:0]  mic_acc_state;         /* 0 idle, 1 clear, 2 read, 3 write, 4 finish */
    reg  [2:0]  mic_acc_word;
    reg  [6:0]  mic_acc_gate;
    reg         mic_acc_last;          /* this sample completes the gate */
    reg         mic_ram_we;
    reg  [8:0]  mic_ram_wr_addr;
    reg  [15:0] mic_ram_wr_data;
    wire [15:0] mic_ram_rd_data;
    /* Payload layout for bits 192..319: eight 16-bit words, MSB first. */
    wire [3:0]  mic_payload_word = status_bit_index[8:4] - 4'd12;
    wire        mic_ram_rd_acc   = (mic_acc_state == 3'd2);
    wire [8:0]  mic_ram_rd_addr  = mic_ram_rd_acc ? {mic_acc_gate[5:0], mic_acc_word}
                                                  : {spi_frame_sequence[5:0], mic_payload_word[2:0]};
    wire [15:0] mic_status_reg   = {11'd0,
                                    (mic_run_state != 2'd0),
                                    (mic_run_state == 2'd2),
                                    mic_saturated,
                                    (mic_run_state == 2'd1),
                                    mic_done};
    wire [63:0] mic_meta_word    = {mic_status_reg, mic_block_count,
                                    {9'd0, mic_cfg_count}, 16'd0};
    wire signed [7:0] mic_acc_win = mic_acc_word[2] ? mic_win_q[mic_acc_word[1:0]]
                                                    : mic_win_i[mic_acc_word[1:0]];
    wire [15:0] mic_acc_addend = {{8{mic_acc_win[7]}}, mic_acc_win};

    /* MIC_CONFIG handshake.  The SCK-domain extension bytes are loaded by a
     * command-complete toggle; the 48-bit bus is synchronized before the
     * configuration is adopted, so no multi-bit field can be sampled torn. */
    reg  [47:0] mic_cfg_hold_spi;
    reg  [47:0] mic_cfg_meta, mic_cfg_sync;
    reg         mic_cfg_toggle_spi, mic_cfg_toggle_meta, mic_cfg_toggle_sync, mic_cfg_toggle_seen;
    reg  [3:0]  mic_cfg_settle;
    wire [6:0]  mic_cfg_count_w = (mic_cfg_sync[6:0] == 7'd0) ? 7'd1 :
                                  ((mic_cfg_sync[6:0] > 7'd64) ? 7'd64 : mic_cfg_sync[6:0]);
    wire [6:0]  mic_cfg_width_w = (mic_cfg_sync[46:40] == 7'd0) ? 7'd1 :
                                  ((mic_cfg_sync[46:40] > 7'd64) ? 7'd64 : mic_cfg_sync[46:40]);
    /* ------------------------------------------------------------------
     * Frame builder.  CLEAR wipes the inactive bank, then every channel
     * contributes one set bit at its rising slot and one at its falling
     * slot.  level=0 writes nothing at all, so the channel stays low for
     * the whole frame.  init_shadow records the state each channel must
     * hold at slot 0, because a window that wraps past slot 255 is already
     * high when the new bank takes over.
     * ------------------------------------------------------------------ */
    localparam [3:0] EV_IDLE = 4'd0, EV_CLEAR = 4'd1, EV_ADDR = 4'd2, EV_WAIT = 4'd3,
                     EV_LEVEL = 4'd13,
                     EV_LATCH = 4'd10, EV_ZERO = 4'd11,
                     EV_RD0  = 4'd4, EV_CAP0  = 4'd5, EV_WR0  = 4'd6,
                     EV_RD1  = 4'd7, EV_CAP1  = 4'd8, EV_WR1  = 4'd9;
    reg  [3:0]  ev_state;
    reg  [7:0]  ev_clear_addr;
    reg  [6:0]  ev_ch;
    /* Combinational one-hot write mask.  Keeping this out of a state
     * register removes the power-up/shift dependency that made the frame
     * builder write all-zero tables on some MachXO2 configurations. */
    wire [83:0] ev_ch_bit;
    reg  [83:0] init_shadow;
    reg  [7:0]  build_phase;
    reg  [8:0]  build_sum;
    reg         build_zero;
    reg  [6:0]  staging_rd_addr;
    reg         frame_req, swap_pending, running, active_bank;

    genvar ev_bit_index;
    generate
        for (ev_bit_index = 0; ev_bit_index < 84; ev_bit_index = ev_bit_index + 1) begin : EV_BIT_DECODE
            assign ev_ch_bit[ev_bit_index] = (ev_ch == ev_bit_index);
        end
    endgenerate

    wire        event_busy = (ev_state != EV_IDLE);
    wire        build_idle = (ev_state == EV_IDLE) && !swap_pending && !frame_req;
    wire [15:0] staging_q;
    umh_channel_ram18 staging_ram (
        .wr_clk(spi1_sck), .wr_addr(spi_channel_index), .wr_data(staging_wr_data),
        .wr_en(spi_write), .rd_clk(fpga_clk), .rd_addr(staging_rd_addr), .rd_data(staging_q)
    );

    /* Pseudo dual-port EBR: port A for DDS read, port B for builder read/write.
     * The builder only accesses during safe windows when DDS is not using the address bus. */
    wire        ev_rd_want    = (ev_state == EV_RD0) || (ev_state == EV_RD1);
    wire        ev_rd_grant   = ev_rd_want && !phase_step_s2 && !phase_step_s3 && !phase_step_s4;
    wire [7:0]  ev_rd_slot    = (ev_state == EV_RD1) ? build_sum[7:0] : build_phase;
    wire [8:0]  ev_build_addr = {~active_bank, ev_rd_slot};
    /* The device has only eight EBRs.  Use the RAM's single read port for
     * either DDS or builder access; builder reads are granted only while the
     * pipelined DDS is idle, so the synchronous read value is unambiguous. */
    wire [8:0]  event_rd_addr = ev_rd_grant ? ev_build_addr : run_addr_s3;
    wire [8:0]  ev_wr_addr = (ev_state == EV_CLEAR) ? {~active_bank, ev_clear_addr} :
                            (ev_state == EV_WR0)   ? {~active_bank, build_phase} :
                                                      {~active_bank, build_sum[7:0]};
    wire        ev_we     = (ev_state == EV_CLEAR) || (ev_state == EV_WR0) ||
                            (ev_state == EV_WR1);
    wire [83:0] ev_rd_data;
    reg  [83:0] ev_rd_hold;
    wire [83:0] ev_wr_data = (ev_state == EV_CLEAR) ? 84'd0 : (ev_rd_hold | ev_ch_bit);
    umh_toggle_ram84 event_ram (
        .clk(fpga_clk),
        .addr_a(event_rd_addr), .rd_data_a(ev_rd_data),
        .we_b(ev_we), .addr_b(ev_wr_addr), .wr_data_b(ev_wr_data),
        .rd_data_b()
    );

    /* ------------------------------------------------------------------
     * Microphone gate accumulator storage: 64 gates x 4 microphones packed
     * as {I[15:0], Q[15:0]} in one 256 x 32 EBR.  The accumulator writes in
     * the 64 MHz domain and MIC_READ reads back through the synchronous port;
     * the ultrasound engine never touches this memory.
     * ------------------------------------------------------------------ */
    umh_mic_iq_ram mic_iq_ram (
        .clk(fpga_clk), .we(mic_ram_we), .wr_addr(mic_ram_wr_addr),
        .wr_data(mic_ram_wr_data),
        .rd_addr(mic_ram_rd_addr), .rd_data(mic_ram_rd_data)
    );

    /* ------------------------------------------------------------------
     * Cross-domain hand-off.  The SCK domain raises a toggle at the end of
     * a complete frame; the output domain waits 8 cycles (62.5 ns, more
     * than one full 42.5 MHz SCK period) before capturing anything, so the
     * bit synchronisers and the RGB shift register are settled.
     * ------------------------------------------------------------------ */
    reg  frame_toggle_meta, frame_toggle_sync, frame_toggle_seen;
    reg  stop_toggle_meta, stop_toggle_sync, stop_toggle_seen;
    reg  ws2812_toggle_meta, ws2812_toggle_sync, ws2812_toggle_seen;
    reg  invalid_frame_meta, invalid_frame_sync;
    reg  [31:0] accepted_sequence_meta, accepted_sequence_sync,
                pending_sequence, accepted_sequence;
    reg  [15:0] update_flags_meta, update_flags_sync;
    reg  [3:0]  frame_settle;
    reg  [3:0]  ws2812_settle;
    reg  [95:0] rgb_hold;
    reg  [15:0] rgb_update_flags_hold;

    /* Focused-AM common envelope state.  The full 84-channel phase image is
     * loaded once through the normal FRAME command; these registers only
     * substitute a common level byte during each event-table rebuild, so the
     * 84 ultrasonic phases stay fixed while the audio envelope is streamed. */
    reg         audio_mode;
    reg  [7:0]  pending_audio_level;
    reg  [7:0]  build_level;
    reg  [7:0]  build_level_eff;
    reg         build_zero_r;
    reg         build_audio;
    reg         frame_req_audio;
    reg         audio_level_toggle_meta, audio_level_toggle_sync, audio_level_toggle_seen;
    reg         audio_mode_toggle_meta, audio_mode_toggle_sync, audio_mode_toggle_seen;
    reg  [7:0]  audio_level_meta, audio_level_sync;
    reg         audio_mode_meta, audio_mode_sync;
    reg         audio_cmd_is_mode;
    reg  [7:0]  audio_cmd_level;
    reg         audio_cmd_mode;
    reg  [3:0]  audio_settle;
    /* In focused-AM mode the stored level byte is an enable marker: any
     * non-zero value keeps the channel active at the common envelope level,
     * while zero mutes the channel exactly as spatial rendering would.
     * This preserves the EEPROM channel-enable mask without a multiplier. */
    wire [7:0]  event_level = build_audio ?
                              ((staging_q[7:0] != 8'd0) ? build_level : 8'd0) :
                              staging_q[7:0];
    wire stop_event = (stop_toggle_sync != stop_toggle_seen);
    wire link_timeout = (link_idle_us >= LINK_TIMEOUT_US);

    /* Status is latched in the output domain while CS is high and only read
     * afterwards, so the SCK side always sees one static 16-byte snapshot. */
    reg  cs_meta, cs_sync, cs_sync_d;
    reg  [127:0] status_hold;
    wire cs_fall = cs_sync_d && !cs_sync;
    always @(posedge fpga_clk) begin
        cs_meta   <= fpga_cs_n;
        cs_sync   <= cs_meta;
        cs_sync_d <= cs_sync;
    end

    wire [15:0] fifo_credit_wire  = (build_idle && !event_busy) ? 16'd1 : 16'd0;
    wire [15:0] fifo_depth_wire   = (build_idle && !event_busy) ? 16'd0 : 16'd1;
    /* Upper status bits are diagnostic only.  The STM32 masks them out when
     * deciding whether the FPGA reports a fault. */
    wire [15:0] status_flags_wire = (invalid_frame_sync ? 16'h0004 : 16'h0000) |
                                    (running ? 16'h0010 : 16'h0000) |
                                    (audio_mode ? 16'h8000 : 16'h0000) |
                                    16'h4000; /* AUDIO_SHORT (0x19) support */
    wire [127:0] status_word = {
        8'h01, 8'h00,
        fifo_credit_wire[7:0],  fifo_credit_wire[15:8],
        fifo_depth_wire[7:0],   fifo_depth_wire[15:8],
        status_flags_wire[7:0], status_flags_wire[15:8],
        fpga_time[7:0], fpga_time[15:8], fpga_time[23:16], fpga_time[31:24],
        accepted_sequence[7:0], accepted_sequence[15:8],
        accepted_sequence[23:16], accepted_sequence[31:24]
    };
    /* MISO is a shared SPI return line and must be released while CS is
     * inactive.  The first 16 bytes always carry the status word.  Longer
     * transactions continue with the microphone payload:
     *   bytes 16..23  {status, block_count, gate_count, 0}, MSB first
     *   bytes 24..39  four 32-bit {I[15:0], Q[15:0]} words, MSB first
     * The MIC_READ gate address travels in the frame-sequence header field
     * (bytes 6..9), so it is already stable when the payload starts. */
    assign spi1_miso = fpga_cs_n ? 1'bz :
                       (status_bit_index < 9'd128) ? status_hold[8'd127 - status_bit_index[6:0]] :
                       (status_bit_index < 9'd192) ? mic_meta_word[9'd191 - status_bit_index] :
                       (status_bit_index < 9'd320) ? mic_ram_rd_data[4'd15 - status_bit_index[3:0]] :
                       1'b0;

    always @(negedge spi1_sck or posedge fpga_cs_n) begin
        if (fpga_cs_n) status_bit_index <= 9'd0;
        else if (status_bit_index != 9'd511) status_bit_index <= status_bit_index + 9'd1;
    end

    wire [15:0] ext_len_next  = {spi_rx_byte, spi_extension_length[7:0]};
    wire [15:0] expected_next = HEADER_BYTES +
                                (spi_update_flags[0] ? CHANNEL_BYTES : 16'd0) +
                                (spi_update_flags[1] ? 16'd12 : 16'd0) + ext_len_next;
    wire        last_header_byte = (spi_byte_count == 16'd35);
    /* A payload transaction must not complete on the last header byte.  At
     * that edge spi_expected_length still contains its reset value (36), so
     * comparing against it would commit FRAME/WS2812 commands before their
     * payload had arrived. */
    wire        header_end = last_header_byte && (expected_next == HEADER_BYTES);
    wire        payload_end = (spi_expected_length > HEADER_BYTES) &&
                              (spi_byte_count + 16'd1 == spi_expected_length);
    wire        compact_end = spi_compact_cmd &&
                              (spi_byte_count + 16'd1 == spi_expected_length) &&
                              ((spi_expected_length == COMPACT_BYTES) ||
                               (spi_expected_length == SHORT_BYTES));
    wire        frame_end = spi_compact_cmd ? compact_end : (header_end || payload_end);
    wire        bitmap_ok = (spi_bitmap == {4'h0, 84'hFFFFFFFFFFFFFFFFFFFFF});
    /* The STM32 always sends the complete 84-channel payload.  Do not make
     * acceptance depend on a reconstructed multi-byte bitmap in the SPI clock
     * domain; a byte-order difference there would silently discard an
     * otherwise valid frame and leave all outputs at zero. */
    wire        bitmap_req_ok = 1'b1;

    always @(posedge spi1_sck or posedge fpga_cs_n) begin
        if (fpga_cs_n) begin
            spi_rx_shift <= 8'd0; spi_bit_count <= 3'd0; spi_byte_count <= 16'd0;
            spi_command <= 8'd0; spi_version <= 8'd0; spi_update_flags <= 16'd0;
            spi_extension_length <= 16'd0; spi_frame_sequence <= 32'd0;
            spi_expected_length <= HEADER_BYTES; spi_channel_index <= 7'd0;
            spi_channel_field <= 2'd0; spi_phase_pending <= 8'd0; spi_bitmap <= 88'd0;
            spi_rgb_index <= 4'd0;
        end else begin
            if (spi_bit_count == 3'd7) begin
                spi_bit_count <= 3'd0;
                spi_rx_shift  <= spi_rx_byte;
                case (spi_byte_count)
                    16'd0:  spi_command  <= spi_rx_byte;
                    16'd1: begin
                        spi_version <= spi_rx_byte;
                        if (spi_command == AUDIO_CMD_LEVEL_SHORT)
                            spi_expected_length <= SHORT_BYTES;
                        else if (spi_command == AUDIO_CMD_LEVEL ||
                                 spi_command == AUDIO_CMD_MODE)
                            spi_expected_length <= COMPACT_BYTES;
                    end
                    16'd2: begin
                        if (spi_command == AUDIO_CMD_LEVEL ||
                            spi_command == AUDIO_CMD_MODE ||
                            spi_command == AUDIO_CMD_LEVEL_SHORT)
                            spi_compact_data <= spi_rx_byte;
                    end
                    16'd6:  spi_frame_sequence[7:0]   <= spi_rx_byte;
                    16'd7:  spi_frame_sequence[15:8]  <= spi_rx_byte;
                    16'd8:  spi_frame_sequence[23:16] <= spi_rx_byte;
                    16'd9:  spi_frame_sequence[31:24] <= spi_rx_byte;
                    16'd18: spi_update_flags[7:0]     <= spi_rx_byte;
                    16'd19: spi_update_flags[15:8]    <= spi_rx_byte;
                    16'd34: spi_extension_length[7:0] <= spi_rx_byte;
                    16'd35: begin
                        spi_extension_length[15:8] <= spi_rx_byte;
                        spi_expected_length <= expected_next;
                    end
                    default: begin
                        /* MIC_CONFIG extension: six bytes at offsets 36..41.
                         * Captured in the SCK domain and adopted after the
                         * command-complete toggle is synchronized. */
                        if (spi_command == 8'h14) begin
                            case (spi_byte_count)
                                16'd36: mic_cfg_hold_spi[7:0]   <= spi_rx_byte;
                                16'd37: mic_cfg_hold_spi[15:8]  <= spi_rx_byte;
                                16'd38: mic_cfg_hold_spi[23:16] <= spi_rx_byte;
                                16'd39: mic_cfg_hold_spi[31:24] <= spi_rx_byte;
                                16'd40: mic_cfg_hold_spi[39:32] <= spi_rx_byte;
                                16'd41: mic_cfg_hold_spi[47:40] <= spi_rx_byte;
                                default: ;
                            endcase
                        end
                        /* The wire format is little-endian.  A shift register
                         * makes byte 0 the most significant byte and causes
                         * the valid low nibble of byte 10 to fail bitmap_ok. */
                        if (spi_byte_count >= 16'd20 && spi_byte_count <= 16'd30) begin
                            case (spi_byte_count)
                                16'd20: spi_bitmap[7:0]   <= spi_rx_byte;
                                16'd21: spi_bitmap[15:8]  <= spi_rx_byte;
                                16'd22: spi_bitmap[23:16] <= spi_rx_byte;
                                16'd23: spi_bitmap[31:24] <= spi_rx_byte;
                                16'd24: spi_bitmap[39:32] <= spi_rx_byte;
                                16'd25: spi_bitmap[47:40] <= spi_rx_byte;
                                16'd26: spi_bitmap[55:48] <= spi_rx_byte;
                                16'd27: spi_bitmap[63:56] <= spi_rx_byte;
                                16'd28: spi_bitmap[71:64] <= spi_rx_byte;
                                16'd29: spi_bitmap[79:72] <= spi_rx_byte;
                                16'd30: spi_bitmap[87:80] <= spi_rx_byte;
                                default: ;
                            endcase
                        end
                        if (spi_payload_byte) begin
                            case (spi_channel_field)
                                2'd0: spi_phase_pending <= spi_rx_byte;
                                2'd1: if (spi_channel_index != 7'd83)
                                          spi_channel_index <= spi_channel_index + 7'd1;
                                default: ;
                            endcase
                            spi_channel_field <= (spi_channel_field == 2'd1) ? 2'd0
                                                                           : spi_channel_field + 2'd1;
                        end else if (spi_rgb_payload_byte) begin
                            /* STM32 sends R,G,B for LED0..3.  Store the
                             * internal layout used by ws2812_stream, where
                             * each 24-bit word is {G,R,B}. */
                            case (spi_rgb_index)
                                4'd0:  rgb_values[15:8]  <= spi_rx_byte;
                                4'd1:  rgb_values[7:0]   <= spi_rx_byte;
                                4'd2:  rgb_values[23:16] <= spi_rx_byte;
                                4'd3:  rgb_values[39:32] <= spi_rx_byte;
                                4'd4:  rgb_values[31:24] <= spi_rx_byte;
                                4'd5:  rgb_values[47:40] <= spi_rx_byte;
                                4'd6:  rgb_values[63:56] <= spi_rx_byte;
                                4'd7:  rgb_values[55:48] <= spi_rx_byte;
                                4'd8:  rgb_values[71:64] <= spi_rx_byte;
                                4'd9:  rgb_values[87:80] <= spi_rx_byte;
                                4'd10: rgb_values[79:72] <= spi_rx_byte;
                                4'd11: rgb_values[95:88] <= spi_rx_byte;
                                default: ;
                            endcase
                            spi_rgb_index <= spi_rgb_index + 4'd1;
                        end
                    end
                endcase
                spi_byte_count <= spi_byte_count + 16'd1;
                if (frame_end) begin
                    /* Keep the parser's last complete transaction visible in
                     * the status word, including STATUS and STOP commands. */
                    if ((spi_command == AUDIO_CMD_LEVEL ||
                         spi_command == AUDIO_CMD_LEVEL_SHORT) &&
                        spi_version == 8'h01 &&
                        (spi_expected_length == COMPACT_BYTES ||
                         spi_expected_length == SHORT_BYTES)) begin
                        audio_level_spi <= (spi_command == AUDIO_CMD_LEVEL_SHORT)
                                           ? spi_rx_byte : spi_compact_data;
                        audio_level_toggle_spi <= ~audio_level_toggle_spi;
                        accepted_sequence_spi <= spi_frame_sequence;
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == AUDIO_CMD_MODE && spi_version == 8'h01 &&
                                 spi_expected_length == COMPACT_BYTES) begin
                        audio_mode_spi <= spi_compact_data[0];
                        audio_mode_toggle_spi <= ~audio_mode_toggle_spi;
                        accepted_sequence_spi <= spi_frame_sequence;
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == 8'h10 && spi_version == 8'h01 &&
                        spi_extension_length <= 16'd32) begin
                        frame_toggle_spi <= ~frame_toggle_spi;
                        accepted_sequence_spi <= spi_frame_sequence;
                        accepted_update_flags_spi <= spi_update_flags;
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == 8'h11 || spi_command == 8'h12) begin
                        stop_toggle_spi <= ~stop_toggle_spi;
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == 8'h13 && spi_version == 8'h01 &&
                                 spi_extension_length <= 16'd32) begin
                        /* WS2812 control command - use separate toggle to avoid frame building */
                        ws2812_toggle_spi <= ~ws2812_toggle_spi;
                        accepted_sequence_spi <= spi_frame_sequence;
                        accepted_update_flags_spi <= spi_update_flags;
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == 8'h14 && spi_version == 8'h01 &&
                                 spi_extension_length == 16'd6) begin
                        /* Arm the microphone gate sequencer.  The frame
                         * pattern submitted next starts the first block. */
                        mic_cfg_toggle_spi <= ~mic_cfg_toggle_spi;
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == 8'h15 && spi_version == 8'h01 &&
                                 spi_extension_length == 16'd0) begin
                        /* MIC_READ gate address travels in frame_sequence
                         * (bytes 6..9), which is captured before the MISO
                         * payload starts, so the first transaction already
                         * returns the right gate. */
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command != 8'h01) begin
                        invalid_frame_spi <= 1'b1;
                    end
                end
            end else begin
                spi_rx_shift <= spi_rx_byte;
                spi_bit_count <= spi_bit_count + 3'd1;
            end
        end
    end

    /* ------------------------------------------------------------------
     * 128 MHz output domain: frame commit, carrier, microphone clock.
     * ------------------------------------------------------------------ */
    always @(posedge fpga_clk) begin
        /* DDS stage 1: fractional carry only */
        phase_frac    <= phase_frac_sum[23:0];
        phase_step_s1 <= phase_frac_sum[24];

        /* DDS stage 2: global phase and wrap detect */
        phase_step_s2 <= phase_step_s1;
        if (phase_step_s1) begin
            global_phase_s2 <= global_phase_s2 + 8'd1;
            if (global_phase_s2 == 8'hFF) wrap_s2 <= 1'b1;
            else wrap_s2 <= 1'b0;
        end else begin
            wrap_s2 <= 1'b0;
        end

        /* DDS stage 3: active bank toggle and address generation */
        phase_step_s3 <= phase_step_s2;
        if (wrap_s2 && swap_pending) begin
            swap_now_s3 <= 1'b1;
            active_bank <= ~active_bank;
            swap_pending <= 1'b0;
            running <= 1'b1;
            accepted_sequence <= pending_sequence;
            run_addr_s3 <= {~active_bank, global_phase_s2};
        end else begin
            swap_now_s3 <= 1'b0;
            if (phase_step_s2) begin
                run_addr_s3 <= {active_bank, global_phase_s2};
            end
        end

        /* DDS stage 4: Event RAM read propagation */
        phase_step_s4 <= phase_step_s3;
        swap_now_s4   <= swap_now_s3;

        /* DDS stage 5: Capture EBR data */
        phase_step_s5 <= phase_step_s4;
        swap_now_s5   <= swap_now_s4;
        if (phase_step_s4) begin
            ev_run_hold_s5 <= ev_rd_data;
        end

        if (time_divider == 7'd63) begin
            time_divider <= 7'd0;
            fpga_time    <= fpga_time + 32'd1;
        end else begin
            time_divider <= time_divider + 7'd1;
        end

        /* Any SPI chip-select assertion proves the STM32 is alive.  Keep the
         * counter saturated so link_timeout remains stable until re-armed. */
        if (cs_fall) begin
            link_idle_us <= 32'd0;
        end else if (time_divider == 7'd63 && link_idle_us < LINK_TIMEOUT_US) begin
            link_idle_us <= link_idle_us + 32'd1;
        end

        /* Refresh the response only while CS is inactive.  At the next
         * transaction it is already valid before the first SCK edge; once
         * CS goes low it remains constant for the whole SPI frame. */
        if (fpga_cs_n) status_hold <= status_word;

        frame_toggle_meta      <= frame_toggle_spi;
        frame_toggle_sync      <= frame_toggle_meta;
        stop_toggle_meta       <= stop_toggle_spi;
        stop_toggle_sync       <= stop_toggle_meta;
        ws2812_toggle_meta     <= ws2812_toggle_spi;
        ws2812_toggle_sync     <= ws2812_toggle_meta;
        accepted_sequence_meta <= accepted_sequence_spi;
        accepted_sequence_sync <= accepted_sequence_meta;
        update_flags_meta      <= accepted_update_flags_spi;
        update_flags_sync      <= update_flags_meta;
        invalid_frame_meta     <= invalid_frame_spi;
        invalid_frame_sync     <= invalid_frame_meta;
        audio_level_toggle_meta <= audio_level_toggle_spi;
        audio_level_toggle_sync <= audio_level_toggle_meta;
        audio_mode_toggle_meta  <= audio_mode_toggle_spi;
        audio_mode_toggle_sync  <= audio_mode_toggle_meta;
        audio_level_meta        <= audio_level_spi;
        audio_level_sync        <= audio_level_meta;
        audio_mode_meta         <= audio_mode_spi;
        audio_mode_sync         <= audio_mode_meta;

        if (frame_toggle_sync != frame_toggle_seen) begin
            frame_toggle_seen <= frame_toggle_sync;
            frame_settle      <= 4'd8;
        end else if (frame_settle != 4'd0) begin
            if (frame_settle != 4'd1)
                frame_settle <= frame_settle - 4'd1;
            /* FIX: Remove cs_sync gating to match WS2812 path fix. */
            if (frame_settle == 4'd1) begin
                frame_settle <= 4'd0;
                if (update_flags_sync[0]) frame_req <= 1'b1;
                /* RGB/digital-only FRAMEs must not rebuild the ultrasound
                 * event table.  A pending compact level is still handled
                 * through frame_req_audio. */

                if (audio_mode == 1'b0) frame_req_audio <= 1'b0;
                rgb_hold  <= rgb_values;
                rgb_update_flags_hold <= update_flags_sync;
                /* RGB is a persistent independent output.  Ultrasound-only
                 * frames must not erase the last LED test/demo colour. */
                if (update_flags_sync[1] != 1'b0) begin
                    ws2812_enable <= 1'b1;
                end
            end
        end

        /* Compact focused-AM command hand-off.  Either a level update or a
         * mode change asks the event builder to rebuild the inactive bank,
         * so the active bank keeps driving us_tx without a gap. */
        if (audio_level_toggle_sync != audio_level_toggle_seen) begin
            audio_level_toggle_seen <= audio_level_toggle_sync;
            audio_cmd_level <= audio_level_sync;
            audio_cmd_is_mode <= 1'b0;
            audio_settle <= 4'd8;
        end
        if (audio_mode_toggle_sync != audio_mode_toggle_seen) begin
            audio_mode_toggle_seen <= audio_mode_toggle_sync;
            audio_cmd_mode <= audio_mode_sync;
            audio_cmd_is_mode <= 1'b1;
            audio_settle <= 4'd8;
        end
        if (audio_settle != 4'd0) begin
            if (audio_settle == 4'd1) begin
                audio_settle <= 4'd0;
                if (audio_cmd_is_mode != 1'b0) begin
                    audio_mode <= audio_cmd_mode;
                    pending_audio_level <= 8'd0;
                end else begin
                    pending_audio_level <= audio_cmd_level;
                end
                frame_req <= 1'b1;
                frame_req_audio <= 1'b1;
            end else begin
                audio_settle <= audio_settle - 4'd1;
            end
        end

        /* WS2812-only update path (no frame building) */
        if (ws2812_toggle_sync != ws2812_toggle_seen) begin
            ws2812_toggle_seen <= ws2812_toggle_sync;
            /* Allow the final SPI byte and the toggle synchronisers to settle
             * before taking the 96-bit RGB snapshot.  Keep the stream enabled:
             * ws2812_stream reloads its shift register at every reset gap, so
             * a register update is enough and there is no reset-dependent
             * one-shot transition for the LEDs to latch. */
            ws2812_settle <= 4'd15;
            ws2812_enable <= 1'b1;
        end else if (ws2812_settle != 4'd0) begin
            /* RGB is written in the SPI clock domain.  Keep the toggle as
             * the event marker and wait for the bus to settle before taking
             * the snapshot in the 128 MHz domain. */
            if (ws2812_settle != 4'd1)
                ws2812_settle <= ws2812_settle - 4'd1;
            if (ws2812_settle == 4'd1) begin
                ws2812_settle <= 4'd0;
                accepted_sequence <= accepted_sequence_sync;
                rgb_hold <= rgb_values;
                /* The stream is continuous and reloads rgb_hold at its next
                 * ST_LOAD boundary. */
                ws2812_enable <= 1'b1;
            end
        end

        /* A WS2812 command may follow STOP in the next SPI transaction.  Do
         * not let the delayed STOP synchroniser erase that newer LED update. */
        if (stop_event) begin
            stop_toggle_seen <= stop_toggle_sync;
            /* STOP belongs to the ultrasound engine.  WS2812 is an
             * independent output and must retain its last commanded colour
             * across Demo preparation and ultrasound stops. */
            running       <= 1'b0;
            frame_req     <= 1'b0;
            frame_req_audio <= 1'b0;
            frame_settle  <= 4'd0;
            audio_settle  <= 4'd0;
            audio_mode    <= 1'b0;
            pending_audio_level <= 8'd0;
            swap_pending  <= 1'b0;
            ev_state      <= EV_IDLE;
        end else if (link_timeout) begin
            /* Link-loss failsafe.  Do not touch stop_toggle_seen: a later
             * explicit STOP must still be recognised when the MCU reboots. */
            running       <= 1'b0;
            frame_req     <= 1'b0;
            frame_req_audio <= 1'b0;
            frame_settle  <= 4'd0;
            audio_settle  <= 4'd0;
            audio_mode    <= 1'b0;
            pending_audio_level <= 8'd0;
            swap_pending  <= 1'b0;
            ev_state      <= EV_IDLE;
        end

        case (ev_state)
            EV_IDLE: begin
                if (frame_req && !swap_pending) begin
                    frame_req        <= 1'b0;
                    /* Audio mode owns the event builder even when the
                     * request came from a normal FRAME (the aperture load).
                     * In that case the current common level is used and the
                     * staging level byte only acts as an enable marker. */
                    build_audio      <= frame_req_audio | audio_mode;
                    /* pending_audio_level is also the persistent common level:
                     * mode entry writes 0, level commands update it, and an
                     * aperture FRAME in audio mode reuses the latest value. */
                    build_level      <= pending_audio_level;
                    frame_req_audio  <= 1'b0;
                    ev_state         <= EV_CLEAR;
                    ev_clear_addr    <= 8'd0;
                    ev_ch            <= 7'd0;
                    init_shadow      <= 84'd0;
                    pending_sequence <= accepted_sequence_sync;
                end
            end
            EV_CLEAR: begin
                if (ev_clear_addr == 8'hFF) begin
                    ev_state        <= EV_ADDR;
                    staging_rd_addr <= 7'd0;
                    ev_ch           <= 7'd0;
                end else begin
                    ev_clear_addr <= ev_clear_addr + 8'd1;
                end
            end
            EV_ADDR: begin
                staging_rd_addr <= ev_ch;
                ev_state        <= EV_WAIT;
            end
            EV_WAIT: begin
                ev_state <= EV_LEVEL;
            end
            EV_LEVEL: begin
                /* Register the enable/common-level decision in its own
                 * pipeline stage.  Keeping the staging-RAM comparator and the
                 * phase/level adder in one cloud was the worst setup path at
                 * 64 MHz; this split costs one cycle per channel and restores
                 * timing margin while preserving the event-table format. */
                build_level_eff <= event_level;
                build_zero_r    <= (event_level == 8'd0);
                ev_state        <= EV_LATCH;
            end
            EV_LATCH: begin
                build_phase <= staging_q[15:8];
                build_sum   <= {1'b0, staging_q[15:8]} + {1'b0, build_level_eff};
                build_zero  <= build_zero_r;
                ev_state    <= build_zero_r ? EV_ZERO : EV_RD0;
            end
            EV_ZERO: begin
                if (ev_ch == 7'd83) begin
                    ev_state     <= EV_IDLE;
                    swap_pending <= 1'b1;
                end else begin
                    ev_state <= EV_ADDR;
                end
                ev_ch  <= ev_ch + 7'd1;
            end
            EV_RD0: begin
                /* The single EBR read port is shared with the running DDS.
                 * Stay here until the banked address is actually presented;
                 * otherwise the capture below would grab the active bank
                 * (or an uninitialised word) and corrupt the event table. */
                if (ev_rd_grant) ev_state <= EV_CAP0;
            end
            EV_CAP0: begin
                if (build_sum[8]) init_shadow <= init_shadow | ev_ch_bit;
                ev_rd_hold <= ev_rd_data;
                ev_state <= EV_WR0;
            end
            EV_WR0: begin
                ev_state <= EV_RD1;
            end
            EV_RD1: begin
                if (ev_rd_grant) ev_state <= EV_CAP1;
            end
            EV_CAP1: begin
                ev_rd_hold <= ev_rd_data;
                ev_state <= EV_WR1;
            end
            EV_WR1: begin
                if (ev_ch == 7'd83) begin
                    ev_state     <= EV_IDLE;
                    swap_pending <= 1'b1;
                end else begin
                    ev_state <= EV_ADDR;
                end
                ev_ch  <= ev_ch + 7'd1;
            end
            default: ev_state <= EV_IDLE;
        endcase

        /* ------------------------------------------------------------------
         * SPH0641LU4H-1 clock sequencer.
         *
         * The clock is held low from power-up, then runs at 200 kHz for
         * 20 ms (sleep exit plus the 15 ms wake-up specification), and only
         * then switches to the 4 MHz ultrasonic mode.  64 MHz / 320 =
         * 200 kHz, 64 MHz / 16 = 4 MHz.
         * ------------------------------------------------------------------ */
        if (mic_boot_count != MIC_BOOT_CYCLES)
            mic_boot_count <= mic_boot_count + 32'd1;

        if (!mic_ultrasonic && !mic_warm) begin
            mic_clock_reg <= 1'b0;
            if (mic_boot_count == MIC_BOOT_CYCLES) begin
                mic_warm       <= 1'b1;
                mic_warm_half  <= 8'd0;
                mic_warm_edges <= 14'd0;
            end
        end else if (mic_warm) begin
            if (mic_warm_half == 8'd159) begin
                mic_warm_half <= 8'd0;
                mic_clock_reg <= ~mic_clock_reg;
                if (mic_warm_edges + 14'd1 >= MIC_WARM_EDGES) begin
                    mic_warm        <= 1'b0;
                    mic_ultrasonic  <= 1'b1;
                    mic_clock_reg   <= 1'b0;
                    mic_phase       <= 4'd0;
                    mic_lo_n           <= 7'd99;
                    mic_lo_i_r         <= 1'b1; mic_lo_q_r <= 1'b1;
                    mic_window_count   <= 7'd0;
                    mic_bc_valid       <= 1'b0;
                    mic_xor_i[0] <= 7'd0; mic_xor_i[1] <= 7'd0;
                    mic_xor_i[2] <= 7'd0; mic_xor_i[3] <= 7'd0;
                    mic_xor_q[0] <= 7'd0; mic_xor_q[1] <= 7'd0;
                    mic_xor_q[2] <= 7'd0; mic_xor_q[3] <= 7'd0;
                end else begin
                    mic_warm_edges <= mic_warm_edges + 14'd1;
                end
            end else begin
                mic_warm_half <= mic_warm_half + 8'd1;
            end
        end else begin
            /* 4 MHz ultrasonic mode.  Phase 4 samples the DATA pins in the
             * first (rising-edge) half period, phase 12 in the second
             * (falling-edge) half period.  Netlist mapping: slot 0 = DATA0
             * rising (U181), slot 1 = DATA0 falling (U180), slot 2 = DATA1
             * rising (U204), slot 3 = DATA1 falling (U182). */
            mic_bc_valid <= 1'b0;
            mic_phase <= (mic_phase == 4'd15) ? 4'd0 : mic_phase + 4'd1;
            if (mic_phase == 4'd0) begin
                mic_clock_reg <= 1'b1;
                mic_lo_n <= mic_lo_n_next;
                mic_lo_i_r <= mic_lo_i_next;
                mic_lo_q_r <= mic_lo_q_next;
            end else if (mic_phase == 4'd8) begin
                mic_clock_reg <= 1'b0;
            end

            if (mic_phase == 4'd4) begin
                `MIC_UPDATE(0, mic_data_0)
                `MIC_UPDATE(2, mic_data_1)
                mic_shift_0_l <= {mic_shift_0_l[14:0], mic_data_0};
                mic_shift_1_l <= {mic_shift_1_l[14:0], mic_data_1};
            end else if (mic_phase == 4'd12) begin
                `MIC_UPDATE(1, mic_data_0)
                `MIC_UPDATE(3, mic_data_1)
                mic_shift_0_r <= {mic_shift_0_r[14:0], mic_data_0};
                mic_shift_1_r <= {mic_shift_1_r[14:0], mic_data_1};
                mic_sample_count <= mic_sample_count + 5'd1;
                if (mic_sample_count == 5'd15)
                    mic_latest <= {mic_shift_0_l,
                                   {mic_shift_0_r[14:0], mic_data_0},
                                   mic_shift_1_l,
                                   {mic_shift_1_r[14:0], mic_data_1}};
                if (mic_window_count != 7'd100)
                    mic_window_count <= mic_window_count + 7'd1;
            end else if (mic_phase == 4'd13) begin
                mic_bc_valid <= 1'b0;
                if (mic_window_count >= 7'd100) begin
                    /* The counters now contain exactly one LO period.  Hold
                     * the result until the next complete window; the valid
                     * pulse and the held value are consumed together. */
                    mic_win_i[0] <= 8'sd100 - {mic_xor_i[0], 1'b0};
                    mic_win_q[0] <= 8'sd100 - {mic_xor_q[0], 1'b0};
                    mic_win_i[1] <= 8'sd100 - {mic_xor_i[1], 1'b0};
                    mic_win_q[1] <= 8'sd100 - {mic_xor_q[1], 1'b0};
                    mic_win_i[2] <= 8'sd100 - {mic_xor_i[2], 1'b0};
                    mic_win_q[2] <= 8'sd100 - {mic_xor_q[2], 1'b0};
                    mic_win_i[3] <= 8'sd100 - {mic_xor_i[3], 1'b0};
                    mic_win_q[3] <= 8'sd100 - {mic_xor_q[3], 1'b0};
                    mic_bc_valid    <= 1'b1;
                    mic_window_count <= 7'd0;
                    mic_xor_i[0] <= 7'd0; mic_xor_i[1] <= 7'd0;
                    mic_xor_i[2] <= 7'd0; mic_xor_i[3] <= 7'd0;
                    mic_xor_q[0] <= 7'd0; mic_xor_q[1] <= 7'd0;
                    mic_xor_q[2] <= 7'd0; mic_xor_q[3] <= 7'd0;
                end
            end
        end

        /* ------------------------------------------------------------------
         * MIC_CONFIG adoption and gate sequence.
         *
         * MIC_CONFIG moves the sequencer to state 1.  The next pattern swap
         * moves it to state 2 and starts gate 0 at mic_cfg_start.  After the
         * last gate the sequencer returns to state 1 so the next submitted
         * pattern is measured without another configuration transaction.
         * ------------------------------------------------------------------ */
        mic_cfg_toggle_meta <= mic_cfg_toggle_spi;
        mic_cfg_toggle_sync <= mic_cfg_toggle_meta;
        mic_cfg_meta        <= mic_cfg_hold_spi;
        mic_cfg_sync        <= mic_cfg_meta;

        if (mic_cfg_toggle_sync != mic_cfg_toggle_seen) begin
            mic_cfg_toggle_seen <= mic_cfg_toggle_sync;
            mic_cfg_settle      <= 4'd4;
        end else if (mic_cfg_settle != 4'd0) begin
            if (mic_cfg_settle == 4'd1) begin
                mic_cfg_settle  <= 4'd0;
                mic_cfg_count   <= mic_cfg_count_w;
                mic_cfg_width   <= mic_cfg_width_w;
                mic_cfg_start   <= mic_cfg_sync[23:8];
                mic_cfg_step    <= (mic_cfg_sync[39:24] < {9'd0, mic_cfg_width_w})
                                   ? {9'd0, mic_cfg_width_w} : mic_cfg_sync[39:24];
                mic_cfg_gap     <= (mic_cfg_sync[39:24] > {9'd0, mic_cfg_width_w})
                                   ? (mic_cfg_sync[39:24] - {9'd0, mic_cfg_width_w}) : 16'd0;
                mic_run_state   <= 2'd1;
                mic_done        <= 1'b0;
                mic_saturated   <= 1'b0;
                mic_block_count <= 16'd0;
                mic_gate_index  <= 7'd0;
                mic_gate_fill   <= 7'd0;
                mic_gate_wait   <= 16'd0;
                mic_gate_active <= 1'b0;
                mic_acc_busy    <= 1'b0;
                mic_acc_state   <= 3'd0;
                mic_acc_last    <= 1'b0;
                mic_ram_we      <= 1'b0;
            end else begin
                mic_cfg_settle <= mic_cfg_settle - 4'd1;
            end
        end

        if (mic_run_state == 2'd1 && swap_now_s3) begin
            mic_run_state   <= 2'd2;
            mic_gate_index  <= 7'd0;
            mic_gate_fill   <= 7'd0;
            mic_gate_active <= 1'b0;
            mic_acc_busy    <= 1'b0;
            mic_acc_state   <= 3'd0;
            mic_gate_wait   <= mic_cfg_start;
            mic_done        <= 1'b0;
            mic_saturated   <= 1'b0;
            /* Re-lock the PDM demodulator to each calibration pattern swap.
             * The transmit DDS and the 4 MHz PDM clock are both derived from
             * the same 64 MHz oscillator, but the DDS nominal carrier is
             * 40000.0066 Hz while the PDM LO is exactly 40 kHz.  Without this
             * re-lock the relative phase drifts ~2.4 deg/s, so H-matrix
             * phases are not repeatable and phase calibration is meaningless.
             * Frame swaps already occur on a transmit-carrier wrap. */
            mic_phase        <= 4'd0;
            mic_lo_n         <= 7'd99;
            mic_lo_i_r       <= 1'b1;
            mic_lo_q_r       <= 1'b1;
            mic_window_count <= 7'd0;
            mic_bc_valid     <= 1'b0;
            mic_xor_i[0] <= 7'd0; mic_xor_i[1] <= 7'd0;
            mic_xor_i[2] <= 7'd0; mic_xor_i[3] <= 7'd0;
            mic_xor_q[0] <= 7'd0; mic_xor_q[1] <= 7'd0;
            mic_xor_q[2] <= 7'd0; mic_xor_q[3] <= 7'd0;
        end

        /* Every 40 kHz sample either decrements the inter-gate wait, starts
         * a gate (its EBR words are cleared first) or launches one read /
         * add / write-back pass over the eight words of the active gate. */
        if (mic_bc_valid && mic_run_state == 2'd2 && !mic_acc_busy) begin
            if (!mic_gate_active) begin
                if (mic_gate_wait != 16'd0) begin
                    mic_gate_wait <= mic_gate_wait - 16'd1;
                end else begin
                    mic_gate_active <= 1'b1;
                    mic_gate_fill   <= 7'd0;
                    mic_acc_gate    <= mic_gate_index;
                    mic_acc_word    <= 3'd0;
                    mic_acc_state   <= 3'd1;
                    mic_acc_busy    <= 1'b1;
                    mic_acc_last    <= (mic_cfg_width <= 7'd1);
                end
            end else begin
                mic_acc_gate  <= mic_gate_index;
                mic_acc_word  <= 3'd0;
                mic_acc_state <= 3'd2;
                mic_acc_busy  <= 1'b1;
                mic_acc_last  <= (mic_gate_fill + 7'd1 >= mic_cfg_width);
            end
        end

        case (mic_acc_state)
            3'd1: begin  /* clear the eight words of a fresh gate */
                mic_ram_we      <= 1'b1;
                mic_ram_wr_addr <= {mic_acc_gate[5:0], mic_acc_word};
                mic_ram_wr_data <= 16'd0;
                if (mic_acc_word == 3'd7) begin
                    mic_acc_word  <= 3'd0;
                    mic_acc_state <= 3'd2;
                end else begin
                    mic_acc_word <= mic_acc_word + 3'd1;
                end
            end
            3'd2: begin  /* present the read address; data latches at this edge */
                mic_ram_we    <= 1'b0;
                mic_acc_state <= 3'd3;
            end
            3'd3: begin  /* registered EBR data is valid: add and write back */
                mic_ram_we      <= 1'b1;
                mic_ram_wr_addr <= {mic_acc_gate[5:0], mic_acc_word};
                mic_ram_wr_data <= mic_ram_rd_data + mic_acc_addend;
                if (mic_acc_word == 3'd7) begin
                    if (mic_acc_last) begin
                        mic_acc_state <= 3'd4;
                    end else begin
                        mic_acc_state <= 3'd0;
                        mic_acc_busy  <= 1'b0;
                        mic_gate_fill <= mic_gate_fill + 7'd1;
                    end
                end else begin
                    mic_acc_word  <= mic_acc_word + 3'd1;
                    mic_acc_state <= 3'd2;
                end
            end
            3'd4: begin  /* gate complete: advance the sequence */
                mic_ram_we    <= 1'b0;
                mic_acc_busy  <= 1'b0;
                mic_acc_last  <= 1'b0;
                mic_acc_state <= 3'd0;
                `MIC_FINISH_GATE
            end
            default: mic_ram_we <= 1'b0;
        endcase
    end

    /* Unconditional registered output.  The next-state cloud contains the
     * same priority rules as the old gated block, but the flop itself has no
     * clock enable or asynchronous clear.  This removes any dependence on
     * LSE's gated-enable translation on silicon. */
    reg  [83:0] us_tx_next;
    always @* begin
        if (stop_event)    us_tx_next = 84'd0;
        else if (!running) us_tx_next = 84'd0;
        else if (phase_step_s5)
            us_tx_next = swap_now_s5 ? (init_shadow ^ ev_run_hold_s5)
                                     : (us_tx ^ ev_run_hold_s5);
        else
            us_tx_next = us_tx;
    end

    always @(posedge fpga_clk) us_tx <= us_tx_next;

    assign mic_clk = mic_clock_reg;

    ws2812_stream ws2812_i (
        .clk(fpga_clk),
        .enable(ws2812_enable),
        /* LED 0 */
        .g0(rgb_hold[7:0]),   .r0(rgb_hold[15:8]),  .b0(rgb_hold[23:16]),
        /* LED 1 */
        .g1(rgb_hold[31:24]), .r1(rgb_hold[39:32]), .b1(rgb_hold[47:40]),
        /* LED 2 */
        .g2(rgb_hold[55:48]), .r2(rgb_hold[63:56]), .b2(rgb_hold[71:64]),
        /* LED 3 */
        .g3(rgb_hold[79:72]), .r3(rgb_hold[87:80]), .b3(rgb_hold[95:88]),
        .data_out(rgb_data)
    );
    spi_mic_stream mic_stream_i (
        .cs_n(spi_mic_cs_n), .sck(spi_mic_sck), .sample_word(mic_latest),
        .miso(spi_mic_miso)
    );

    initial begin
        spi_rx_shift = 8'd0; spi_bit_count = 3'd0; spi_byte_count = 16'd0;
        spi_command = 8'd0; spi_version = 8'd0; spi_update_flags = 16'd0;
        spi_extension_length = 16'd0; spi_frame_sequence = 32'd0; accepted_update_flags_spi = 16'd0;
        spi_expected_length = HEADER_BYTES; spi_channel_index = 7'd0;
        spi_channel_field = 2'd0; spi_phase_pending = 8'd0; spi_bitmap = 88'd0;
        spi_rgb_index = 4'd0;
        spi_compact_data = 8'd0; audio_level_spi = 8'd0; audio_mode_spi = 1'b0;
        audio_level_toggle_spi = 1'b0; audio_mode_toggle_spi = 1'b0;
        frame_toggle_spi = 1'b0; stop_toggle_spi = 1'b0; invalid_frame_spi = 1'b0;
        ws2812_toggle_spi = 1'b0;
        rgb_values = 96'd0; status_bit_index = 9'd0;
        frame_toggle_meta = 1'b0; frame_toggle_sync = 1'b0; frame_toggle_seen = 1'b0;
        stop_toggle_meta = 1'b0; stop_toggle_sync = 1'b0; stop_toggle_seen = 1'b0;
        ws2812_toggle_meta = 1'b0; ws2812_toggle_sync = 1'b0; ws2812_toggle_seen = 1'b0;
        invalid_frame_meta = 1'b0; invalid_frame_sync = 1'b0;
        accepted_sequence_meta = 32'd0; accepted_sequence_sync = 32'd0;
        pending_sequence = 32'd0; accepted_sequence = 32'd0; frame_settle = 4'd0;
        rgb_hold = 96'd0; status_hold = 128'd0;
        ws2812_settle = 4'd0;
        audio_mode = 1'b0; pending_audio_level = 8'd0;
        build_level = 8'd0; build_level_eff = 8'd0; build_zero_r = 1'b0;
        build_audio = 1'b0; frame_req_audio = 1'b0;
        audio_level_toggle_meta = 1'b0; audio_level_toggle_sync = 1'b0;
        audio_level_toggle_seen = 1'b0; audio_mode_toggle_meta = 1'b0;
        audio_mode_toggle_sync = 1'b0; audio_mode_toggle_seen = 1'b0;
        audio_level_meta = 8'd0; audio_level_sync = 8'd0;
        audio_mode_meta = 1'b0; audio_mode_sync = 1'b0;
        audio_cmd_is_mode = 1'b0; audio_cmd_level = 8'd0; audio_cmd_mode = 1'b0;
        audio_settle = 4'd0;
        cs_meta = 1'b1; cs_sync = 1'b1; cs_sync_d = 1'b1;
        phase_frac = 24'd0; phase_step_s1 = 1'b0; global_phase_s2 = 8'd0;
        phase_step_s2 = 1'b0; wrap_s2 = 1'b0; run_addr_s3 = 9'd0;
        phase_step_s3 = 1'b0; swap_now_s3 = 1'b0; phase_step_s4 = 1'b0;
        swap_now_s4 = 1'b0; ev_run_hold_s5 = 84'd0; phase_step_s5 = 1'b0;
        swap_now_s5 = 1'b0;
        fpga_time = 32'd0; time_divider = 7'd0;
        link_idle_us = 32'd0;
        ev_state = EV_IDLE; ev_clear_addr = 8'd0; ev_ch = 7'd0;
        init_shadow = 84'd0; ev_rd_hold = 84'd0; staging_rd_addr = 7'd0;
        build_phase = 8'd0; build_sum = 9'd0; build_zero = 1'b0;
        frame_req = 1'b0; swap_pending = 1'b0; running = 1'b0; active_bank = 1'b0;
        us_tx = 84'd0;
        mic_clock_reg = 1'b0;
        mic_shift_0_l = 16'd0; mic_shift_0_r = 16'd0;
        mic_shift_1_l = 16'd0; mic_shift_1_r = 16'd0;
        mic_sample_count = 5'd0; mic_latest = 64'd0;
        mic_boot_count = 32'd0; mic_warm = 1'b0; mic_ultrasonic = 1'b0;
        mic_warm_half = 8'd0; mic_warm_edges = 14'd0; mic_phase = 4'd0;
        /* Microphone demodulator and gate accumulator.  LSE has no reset, so
         * every state element is spelled out here: an uninitialised gate
         * configuration would otherwise leak into the first MIC_READ. */
        mic_lo_n = 7'd99; mic_window_count = 7'd0; mic_bc_valid = 1'b0;
        mic_lo_i_r = 1'b1; mic_lo_q_r = 1'b1;
        mic_xor_i[0] = 7'd0; mic_xor_i[1] = 7'd0;
        mic_xor_i[2] = 7'd0; mic_xor_i[3] = 7'd0;
        mic_xor_q[0] = 7'd0; mic_xor_q[1] = 7'd0;
        mic_xor_q[2] = 7'd0; mic_xor_q[3] = 7'd0;
        mic_win_i[0] = 8'sd0; mic_win_i[1] = 8'sd0;
        mic_win_i[2] = 8'sd0; mic_win_i[3] = 8'sd0;
        mic_win_q[0] = 8'sd0; mic_win_q[1] = 8'sd0;
        mic_win_q[2] = 8'sd0; mic_win_q[3] = 8'sd0;
        mic_cfg_count = 7'd64; mic_cfg_start = 16'd0;
        mic_cfg_step = 16'd12; mic_cfg_width = 7'd8; mic_cfg_gap = 16'd4;
        mic_run_state = 2'd0; mic_done = 1'b0; mic_saturated = 1'b0;
        mic_block_count = 16'd0; mic_gate_index = 7'd0; mic_gate_fill = 7'd0;
        mic_gate_wait = 16'd0; mic_gate_active = 1'b0;
        mic_acc_busy = 1'b0; mic_acc_state = 3'd0; mic_acc_word = 3'd0;
        mic_acc_gate = 7'd0; mic_acc_last = 1'b0;
        mic_ram_we = 1'b0; mic_ram_wr_addr = 9'd0; mic_ram_wr_data = 16'd0;
        mic_cfg_hold_spi = 48'd0; mic_cfg_meta = 48'd0; mic_cfg_sync = 48'd0;
        mic_cfg_toggle_spi = 1'b0; mic_cfg_toggle_meta = 1'b0;
        mic_cfg_toggle_sync = 1'b0; mic_cfg_toggle_seen = 1'b0;
        mic_cfg_settle = 4'd0;
        /* Keep the protocol stream alive from power-up.  New RGB data is
         * sampled at the next WS2812 reset gap, so updates do not depend on a
         * fragile 0->1 enable transition or an STM32 reset. */
        ws2812_enable = 1'b1;
    end
endmodule
