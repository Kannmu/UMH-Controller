`timescale 1ns/1ps

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

/* 84 x 16-bit staging memory.  The write port runs in the SPI SCK domain,
 * the read port in the 128 MHz output domain. */
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

/* 512 x 84 toggle-event table.  Address bit 8 selects the bank.  A set bit
 * means "toggle this channel at this phase slot"; because every enabled
 * channel has exactly two events per carrier period, XOR-ing the table
 * entry into the running output reproduces the duty window exactly. */
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
    /* 40 kHz with a 32-bit accumulator at 128 MHz. */
    localparam [31:0] CARRIER_STEP  = 32'd1342177;

    /* ------------------------------------------------------------------
     * 8 MHz STM32 PA8 MCO -> 128 MHz output clock.
     *
     * MachXO2 law, confirmed with Lattice's own PLL calculator
     * (ispfpga/bin/nt64/scuba.exe -arch xo2c00 -fin 8 -fclkop 128):
     *
     *   fCLKOP = fCLKI * CLKFB_DIV / CLKI_DIV = 8 * 16 / 1   = 128 MHz
     *   fVCO   = fCLKOP * CLKOP_DIV            = 128 * 4      = 512 MHz
     *
     * scuba returns "Div: 1, 16, 4, ..." and "VCO/BW: 512.000 0.767" for
     * this request, so CLKFB_DIV must be 16.  With CLKFB_DIV=64 the VCO
     * would be pushed to 2048 MHz and the PLL could never lock.
     * CLKOP_CPHASE = CLKOP_DIV-1 keeps CLKOP aligned with the INT_DIVA
     * feedback path, which is the only internal feedback mode Map accepts.
     * ------------------------------------------------------------------ */
    wire pll_clk, pll_locked, pll_feedback;
    wire fpga_clk = pll_clk;
    EHXPLLJ #(
        .PLLRST_ENA("DISABLED"), .INTFB_WAKE("DISABLED"), .STDBY_ENABLE("DISABLED"),
        .DPHASE_SOURCE("DISABLED"), .CLKOP_FPHASE(0), .CLKOP_CPHASE(3),
        .OUTDIVIDER_MUXA2("DIVA"), .CLKOP_ENABLE("ENABLED"), .CLKOP_DIV(4),
        .CLKFB_DIV(16), .CLKI_DIV(1), .FEEDBK_PATH("INT_DIVA")
    ) fpga_pll_i (
        .CLKI(fpga_clk_8m), .CLKFB(pll_feedback), .RST(1'b0),
        .RESETM(1'b0), .RESETC(1'b0), .RESETD(1'b0),
        .PHASESEL0(1'b0), .PHASESEL1(1'b0), .PHASEDIR(1'b0), .PHASESTEP(1'b0),
        .LOADREG(1'b0), .STDBY(1'b0), .PLLWAKESYNC(1'b0), .ENCLKOP(1'b0),
        .CLKOP(pll_clk), .LOCK(pll_locked), .CLKINTFB(pll_feedback)
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
    reg  [6:0]  spi_channel_index;
    reg  [1:0]  spi_channel_field;
    reg  [87:0] spi_bitmap;
    reg         frame_toggle_spi, stop_toggle_spi, invalid_frame_spi;
    reg  [95:0] rgb_values;
    reg  [6:0]  status_bit_index;

    wire [7:0]  spi_rx_byte  = {spi_rx_shift[6:0], spi1_mosi};
    wire        spi_payload_byte = !fpga_cs_n && spi_update_flags[0] &&
                                   (spi_bit_count == 3'd7) &&
                                   (spi_byte_count >= HEADER_BYTES) &&
                                   (spi_byte_count < HEADER_BYTES + CHANNEL_BYTES);
    wire        spi_write    = spi_payload_byte && (spi_channel_field == 2'd1);
    wire [15:0] staging_wr_data = {spi_phase_pending, spi_rx_byte};

    /* ------------------------------------------------------------------
     * Carrier DDS.  The 24-bit fractional accumulator is isolated from the
     * 84-bit output path.  Its registered carry is the only signal that
     * advances the 8-bit event-table phase, so no wide output control signal
     * depends on a long 32-bit carry chain.
     * ------------------------------------------------------------------ */
    reg  [23:0] phase_frac;
    reg         phase_step_reg;
    reg  [7:0]  global_phase;
    reg  [31:0] fpga_time;
    reg  [6:0]  time_divider;
    reg         phase_step_d1;
    reg         phase_step_d2;
    reg         phase_step_d3;
    reg         swap_now_d1;
    reg         swap_now_d2;
    reg         swap_now_d3;
    reg  [8:0]  run_addr_reg;
    reg  [83:0] ev_run_hold;
    wire [24:0] phase_frac_sum = {1'b0, phase_frac} + 25'd1342177;
    wire        phase_step     = phase_step_reg;
    wire [7:0]  next_global_phase = global_phase + 8'd1;
    wire        carrier_wrap   = phase_step_reg && (global_phase == 8'hFF);

    /* Microphone PDM sampler. */
    reg  [6:0]  mic_divider;
    reg         mic_tick;
    reg         mic_clock_reg;
    reg  [15:0] mic_shift_0_l, mic_shift_0_r;
    reg  [15:0] mic_shift_1_l, mic_shift_1_r;
    reg  [4:0]  mic_sample_count;
    reg  [63:0] mic_latest;

    /* ------------------------------------------------------------------
     * Frame builder.  CLEAR wipes the inactive bank, then every channel
     * contributes one set bit at its rising slot and one at its falling
     * slot.  level=0 writes nothing at all, so the channel stays low for
     * the whole frame.  init_shadow records the state each channel must
     * hold at slot 0, because a window that wraps past slot 255 is already
     * high when the new bank takes over.
     * ------------------------------------------------------------------ */
    localparam [3:0] EV_IDLE = 4'd0, EV_CLEAR = 4'd1, EV_ADDR = 4'd2, EV_WAIT = 4'd3,
                     EV_LATCH = 4'd10, EV_ZERO = 4'd11,
                     EV_RD0  = 4'd4, EV_CAP0  = 4'd5, EV_WR0  = 4'd6,
                     EV_RD1  = 4'd7, EV_CAP1  = 4'd8, EV_WR1  = 4'd9;
    reg  [3:0]  ev_state;
    reg  [7:0]  ev_clear_addr;
    reg  [6:0]  ev_ch;
    reg  [83:0] ev_bit, init_shadow;
    reg  [7:0]  build_phase;
    reg  [8:0]  build_sum;
    reg         build_zero;
    reg  [6:0]  staging_rd_addr;
    reg         frame_req, swap_pending, running, active_bank;

    wire        event_busy = (ev_state != EV_IDLE);
    wire        build_idle = (ev_state == EV_IDLE) && !swap_pending && !frame_req;
    wire [15:0] staging_q;
    umh_channel_ram18 staging_ram (
        .wr_clk(spi1_sck), .wr_addr(spi_channel_index), .wr_data(staging_wr_data),
        .wr_en(spi_write), .rd_clk(fpga_clk), .rd_addr(staging_rd_addr), .rd_data(staging_q)
    );

    /* True dual-port EBR: port A is dedicated to the running waveform and
     * port B is dedicated to frame construction. */
    wire [8:0]  ev_build_addr = (ev_state == EV_CLEAR) ? {~active_bank, ev_clear_addr} :
                                (ev_state == EV_RD0 || ev_state == EV_WR0) ? {~active_bank, build_phase} :
                                (ev_state == EV_RD1 || ev_state == EV_WR1) ? {~active_bank, build_sum[7:0]} :
                                9'd0;
    wire        swap_now      = swap_pending && carrier_wrap;
    wire        run_bank      = active_bank ^ swap_now ^ swap_now_d1;
    wire        ev_we_b = (ev_state == EV_CLEAR) || (ev_state == EV_WR0) ||
                          (ev_state == EV_WR1);
    wire [83:0] ev_rd_data_a;
    wire [83:0] ev_rd_data_b;
    reg  [83:0] ev_rd_hold;
    wire [83:0] ev_wr_data_b = (ev_state == EV_CLEAR) ? 84'd0 : (ev_rd_hold | ev_bit);
    umh_toggle_ram84 event_ram (
        .clk(fpga_clk),
        .addr_a(run_addr_reg), .rd_data_a(ev_rd_data_a),
        .we_b(ev_we_b), .addr_b(ev_build_addr), .wr_data_b(ev_wr_data_b),
        .rd_data_b(ev_rd_data_b)
    );

    /* ------------------------------------------------------------------
     * Cross-domain hand-off.  The SCK domain raises a toggle at the end of
     * a complete frame; the output domain waits 8 cycles (62.5 ns, more
     * than one full 42.5 MHz SCK period) before capturing anything, so the
     * bit synchronisers and the RGB shift register are settled.
     * ------------------------------------------------------------------ */
    reg  frame_toggle_meta, frame_toggle_sync, frame_toggle_seen;
    reg  stop_toggle_meta, stop_toggle_sync, stop_toggle_seen;
    reg  invalid_frame_meta, invalid_frame_sync;
    reg  [31:0] accepted_sequence_meta, accepted_sequence_sync,
                pending_sequence, accepted_sequence;
    reg  [3:0]  frame_settle;
    reg  [95:0] rgb_hold;
    wire stop_event = (stop_toggle_sync != stop_toggle_seen);

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
    wire [15:0] status_flags_wire = (invalid_frame_sync ? 16'h0004 : 16'h0000) |
                                    (running ? 16'h0010 : 16'h0000);
    wire [127:0] status_word = {
        8'h01, 8'h00,
        fifo_credit_wire[7:0],  fifo_credit_wire[15:8],
        fifo_depth_wire[7:0],   fifo_depth_wire[15:8],
        status_flags_wire[7:0], status_flags_wire[15:8],
        fpga_time[7:0], fpga_time[15:8], fpga_time[23:16], fpga_time[31:24],
        accepted_sequence[7:0], accepted_sequence[15:8],
        accepted_sequence[23:16], accepted_sequence[31:24]
    };
    assign spi1_miso = fpga_cs_n ? 1'b0 : status_hold[7'd127 - status_bit_index];

    always @(negedge spi1_sck or posedge fpga_cs_n) begin
        if (fpga_cs_n) status_bit_index <= 7'd0;
        else if (status_bit_index != 7'd127) status_bit_index <= status_bit_index + 7'd1;
    end

    wire [15:0] ext_len_next  = {spi_rx_byte, spi_extension_length[7:0]};
    wire [15:0] expected_next = HEADER_BYTES +
                                (spi_update_flags[0] ? CHANNEL_BYTES : 16'd0) +
                                (spi_update_flags[1] ? 16'd12 : 16'd0) + ext_len_next;
    wire        last_header_byte = (spi_byte_count == 16'd35);
    wire        frame_end = last_header_byte ? (expected_next == HEADER_BYTES)
                                             : (spi_byte_count + 16'd1 == spi_expected_length);
    wire        bitmap_ok = (spi_bitmap == {4'h0, 84'hFFFFFFFFFFFFFFFFFFFFF});
    wire        bitmap_req_ok = !spi_update_flags[0] || bitmap_ok;

    always @(posedge spi1_sck or posedge fpga_cs_n) begin
        if (fpga_cs_n) begin
            spi_rx_shift <= 8'd0; spi_bit_count <= 3'd0; spi_byte_count <= 16'd0;
            spi_command <= 8'd0; spi_version <= 8'd0; spi_update_flags <= 16'd0;
            spi_extension_length <= 16'd0; spi_frame_sequence <= 32'd0;
            spi_expected_length <= HEADER_BYTES; spi_channel_index <= 7'd0;
            spi_channel_field <= 2'd0; spi_phase_pending <= 8'd0; spi_bitmap <= 88'd0;
        end else begin
            if (spi_bit_count == 3'd7) begin
                spi_bit_count <= 3'd0;
                spi_rx_shift  <= spi_rx_byte;
                case (spi_byte_count)
                    16'd0:  spi_command  <= spi_rx_byte;
                    16'd1:  spi_version  <= spi_rx_byte;
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
                        if (spi_byte_count >= 16'd20 && spi_byte_count <= 16'd30)
                            spi_bitmap <= {spi_bitmap[79:0], spi_rx_byte};
                        if (spi_payload_byte) begin
                            case (spi_channel_field)
                                2'd0: spi_phase_pending <= spi_rx_byte;
                                2'd1: if (spi_channel_index != 7'd83)
                                          spi_channel_index <= spi_channel_index + 7'd1;
                                default: ;
                            endcase
                            spi_channel_field <= (spi_channel_field == 2'd1) ? 2'd0
                                                                           : spi_channel_field + 2'd1;
                        end else if (spi_update_flags[1] &&
                                     spi_byte_count >= HEADER_BYTES + CHANNEL_BYTES &&
                                     spi_byte_count <  HEADER_BYTES + CHANNEL_BYTES + 16'd12) begin
                            rgb_values <= {rgb_values[87:0], spi_rx_byte};
                        end
                    end
                endcase
                spi_byte_count <= spi_byte_count + 16'd1;
                if (frame_end) begin
                    if (spi_command == 8'h10 && spi_version == 8'h01 &&
                        ext_len_next <= 16'd32 && bitmap_req_ok) begin
                        frame_toggle_spi <= ~frame_toggle_spi;
                        accepted_sequence_spi <= spi_frame_sequence;
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == 8'h11 || spi_command == 8'h12) begin
                        stop_toggle_spi <= ~stop_toggle_spi;
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
        /* DDS stage 1: fractional carry only. */
        phase_frac     <= phase_frac_sum[23:0];
        phase_step_reg <= phase_frac_sum[24];

        /* DDS stage 2: advance the event-table slot from the registered carry. */
        if (phase_step_reg)
            global_phase <= next_global_phase;
        phase_step_d1 <= phase_step_reg;
        phase_step_d2 <= phase_step_d1;
        phase_step_d3 <= phase_step_d2;
        swap_now_d1   <= swap_now;
        swap_now_d2   <= swap_now_d1;
        swap_now_d3   <= swap_now_d2;
        run_addr_reg  <= {run_bank, phase_step_reg ? next_global_phase : global_phase};
        ev_run_hold   <= ev_rd_data_a;
        if (time_divider == 7'd127) begin
            time_divider <= 7'd0;
            fpga_time    <= fpga_time + 32'd1;
        end else begin
            time_divider <= time_divider + 7'd1;
        end

        if (cs_fall) status_hold <= status_word;

        frame_toggle_meta      <= frame_toggle_spi;
        frame_toggle_sync      <= frame_toggle_meta;
        stop_toggle_meta       <= stop_toggle_spi;
        stop_toggle_sync       <= stop_toggle_meta;
        accepted_sequence_meta <= accepted_sequence_spi;
        accepted_sequence_sync <= accepted_sequence_meta;
        invalid_frame_meta     <= invalid_frame_spi;
        invalid_frame_sync     <= invalid_frame_meta;

        if (frame_toggle_sync != frame_toggle_seen) begin
            frame_toggle_seen <= frame_toggle_sync;
            frame_settle      <= 4'd8;
        end else if (frame_settle != 4'd0) begin
            frame_settle <= frame_settle - 4'd1;
            if (frame_settle == 4'd1) begin
                frame_req <= 1'b1;
                rgb_hold  <= rgb_values;
            end
        end

        if (stop_event) begin
            stop_toggle_seen <= stop_toggle_sync;
            running          <= 1'b0;
            frame_req        <= 1'b0;
            frame_settle     <= 4'd0;
            swap_pending     <= 1'b0;
            ev_state         <= EV_IDLE;
        end else if (swap_now_d1) begin
            running           <= 1'b1;
            swap_pending      <= 1'b0;
            active_bank       <= ~active_bank;
            accepted_sequence <= pending_sequence;
        end

        case (ev_state)
            EV_IDLE: begin
                if (frame_req && !swap_pending) begin
                    frame_req        <= 1'b0;
                    ev_state         <= EV_CLEAR;
                    ev_clear_addr    <= 8'd0;
                    ev_ch            <= 7'd0;
                    init_shadow      <= 84'd0;
                    pending_sequence <= accepted_sequence_sync;
                end
            end
            EV_CLEAR: begin
                ev_clear_addr <= ev_clear_addr + 8'd1;
                if (ev_clear_addr == 8'hFF) begin
                    ev_state        <= EV_ADDR;
                    staging_rd_addr <= 7'd0;
                    ev_ch           <= 7'd0;
                end
            end
            EV_ADDR: begin
                staging_rd_addr <= ev_ch;
                ev_state        <= EV_WAIT;
            end
            EV_WAIT: begin
                ev_state <= EV_LATCH;
            end
            EV_LATCH: begin
                build_phase <= staging_q[15:8];
                build_sum   <= {1'b0, staging_q[15:8]} + {1'b0, staging_q[7:0]};
                build_zero  <= (staging_q[7:0] == 8'd0);
                ev_state    <= (staging_q[7:0] == 8'd0) ? EV_ZERO : EV_RD0;
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
                ev_state <= EV_CAP0;
            end
            EV_CAP0: begin
                if (build_sum[8]) init_shadow <= init_shadow | ev_bit;
                ev_rd_hold <= ev_rd_data_b;
                ev_state <= EV_WR0;
            end
            EV_WR0: begin
                ev_state <= EV_RD1;
            end
            EV_RD1: begin
                ev_state <= EV_CAP1;
            end
            EV_CAP1: begin
                ev_rd_hold <= ev_rd_data_b;
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

        ev_bit <= 84'd1 << ev_ch;

        if (!pll_locked || stop_event) begin
            us_tx <= 84'd0;
        end else if (!running) begin
            us_tx <= 84'd0;
        end else if (phase_step_d3) begin
            /* DDS stage 3: bank activation and event toggle share one edge. */
            if (swap_now_d3)
                us_tx <= init_shadow ^ ev_run_hold;
            else
                us_tx <= us_tx ^ ev_run_hold;
        end

        /* 128 MHz / (2 * 16) = 4 MHz microphone clock. */
        mic_tick <= (mic_divider == 6'd15);
        if (mic_tick) begin
            mic_divider   <= 6'd0;
            mic_clock_reg <= ~mic_clock_reg;
            /* Each SPH0641 pair puts left/right PDM on opposite clock edges.
             * The edge polarity is retained in the source marker below:
             * MIC0/1 are DATA0 rising/falling, MIC2/3 are DATA1 rising/falling. */
            if (!mic_clock_reg) begin
                mic_shift_0_l <= {mic_shift_0_l[14:0], mic_data_0};
                mic_shift_1_l <= {mic_shift_1_l[14:0], mic_data_1};
            end else begin
                mic_shift_0_r <= {mic_shift_0_r[14:0], mic_data_0};
                mic_shift_1_r <= {mic_shift_1_r[14:0], mic_data_1};
                mic_sample_count <= mic_sample_count + 5'd1;
                if (mic_sample_count == 5'd15)
                    mic_latest <= {mic_shift_0_l,
                                   {mic_shift_0_r[14:0], mic_data_0},
                                   mic_shift_1_l,
                                   {mic_shift_1_r[14:0], mic_data_1}};
            end
        end else begin
            mic_divider <= mic_divider + 6'd1;
        end
    end

    assign mic_clk = mic_clock_reg;

    ws2812_stream ws2812_i (
        .clk(fpga_clk),
        .g(rgb_hold[87:80]), .r(rgb_hold[95:88]), .b(rgb_hold[79:72]),
        .data_out(rgb_data)
    );
    spi_mic_stream mic_stream_i (
        .cs_n(spi_mic_cs_n), .sck(spi_mic_sck), .sample_word(mic_latest),
        .miso(spi_mic_miso)
    );

    initial begin
        spi_rx_shift = 8'd0; spi_bit_count = 3'd0; spi_byte_count = 16'd0;
        spi_command = 8'd0; spi_version = 8'd0; spi_update_flags = 16'd0;
        spi_extension_length = 16'd0; spi_frame_sequence = 32'd0;
        spi_expected_length = HEADER_BYTES; spi_channel_index = 7'd0;
        spi_channel_field = 2'd0; spi_phase_pending = 8'd0; spi_bitmap = 88'd0;
        frame_toggle_spi = 1'b0; stop_toggle_spi = 1'b0; invalid_frame_spi = 1'b0;
        rgb_values = 96'd0; status_bit_index = 7'd0;
        frame_toggle_meta = 1'b0; frame_toggle_sync = 1'b0; frame_toggle_seen = 1'b0;
        stop_toggle_meta = 1'b0; stop_toggle_sync = 1'b0; stop_toggle_seen = 1'b0;
        invalid_frame_meta = 1'b0; invalid_frame_sync = 1'b0;
        accepted_sequence_meta = 32'd0; accepted_sequence_sync = 32'd0;
        pending_sequence = 32'd0; accepted_sequence = 32'd0; frame_settle = 4'd0;
        rgb_hold = 96'd0; status_hold = 128'd0;
        cs_meta = 1'b1; cs_sync = 1'b1; cs_sync_d = 1'b1;
        phase_frac = 24'd0; phase_step_reg = 1'b0; global_phase = 8'd0;
        fpga_time = 32'd0; time_divider = 7'd0; phase_step_d1 = 1'b0; phase_step_d2 = 1'b0; phase_step_d3 = 1'b0;
        swap_now_d1 = 1'b0; swap_now_d2 = 1'b0; swap_now_d3 = 1'b0; ev_run_hold = 84'd0;
        run_addr_reg = 9'd0;
        ev_state = EV_IDLE; ev_clear_addr = 8'd0; ev_ch = 7'd0;
        ev_bit = 84'd1; init_shadow = 84'd0; ev_rd_hold = 84'd0; staging_rd_addr = 7'd0;
        build_phase = 8'd0; build_sum = 9'd0; build_zero = 1'b0;
        frame_req = 1'b0; swap_pending = 1'b0; running = 1'b0; active_bank = 1'b0;
        us_tx = 84'd0;
        mic_divider = 6'd0; mic_clock_reg = 1'b0;
        mic_shift_0_l = 16'd0; mic_shift_0_r = 16'd0;
        mic_shift_1_l = 16'd0; mic_shift_1_r = 16'd0;
        mic_tick = 1'b0; mic_sample_count = 5'd0; mic_latest = 64'd0;
    end
endmodule
