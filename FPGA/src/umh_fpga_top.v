`timescale 1ns/1ps

/* M: microphone slot 0..3, S: that slot's PDM bit for this sample.
   Window value is matches-mismatches over 100 samples, accumulated as +-1 so the
   old 100-2*mismatch subtractor disappears. */
`define MIC_UPDATE(M,S) \
    mic_acc_i[M] <= ((S) ^ mic_lo_i_r) ? (mic_acc_i[M] - 8'sd1) : (mic_acc_i[M] + 8'sd1); \
    mic_acc_q[M] <= ((S) ^ mic_lo_q_r) ? (mic_acc_q[M] - 8'sd1) : (mic_acc_q[M] + 8'sd1);

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

module umh_channel_ram18 (
    input  wire        wr_clk,
    input  wire [7:0]  wr_addr,
    input  wire [7:0]  wr_data,
    input  wire        wr_en,
    input  wire        rd_clk,
    input  wire [7:0]  rd_addr,
    output reg  [7:0]  rd_data
);
    (* syn_ramstyle = "block_ram" *) reg [7:0] mem [0:255];
    always @(posedge wr_clk) if (wr_en) mem[wr_addr] <= wr_data;
    always @(posedge rd_clk) rd_data <= mem[rd_addr];
endmodule

module umh_toggle_ram84 (
    input  wire        clk,
    input  wire [8:0]  addr_a,
    output reg  [83:0] rd_data_a,
    input  wire        we_b,
    input  wire [8:0]  addr_b,
    input  wire [83:0] wr_data_b
);
    (* syn_ramstyle = "block_ram" *) reg [83:0] mem [0:511];

    always @(posedge clk) begin
        rd_data_a <= mem[addr_a];
        if (we_b) mem[addr_b] <= wr_data_b;
    end
endmodule

module umh_mic_iq_ram (
    input  wire        clk,
    input  wire        we,
    input  wire [8:0]  wr_addr,
    input  wire [15:0] wr_data,
    input  wire [8:0]  rd_addr,
    output reg  [15:0] rd_data
);

    (* syn_ramstyle = "block_ram" *) reg [15:0] mem [0:511];
    always @(posedge clk) begin
        if (we) mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
    end
endmodule

module umh_level_fifo (
    input  wire       wr_clk,
    input  wire       we,
    input  wire [7:0] wr_addr,
    input  wire [7:0] wr_data,
    input  wire       rd_clk,
    input  wire [7:0] rd_addr,
    output reg  [7:0] rd_data
);
    (* syn_ramstyle = "block_ram" *) reg [7:0] mem [0:255];
    always @(posedge wr_clk) if (we) mem[wr_addr] <= wr_data;
    always @(posedge rd_clk) rd_data <= mem[rd_addr];
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
    localparam [15:0] PHASE_BYTES   = 16'd84;
    localparam [15:0] COMPACT_BYTES   = 16'd16;
    localparam [7:0]  MOD_CMD_FIXED  = 8'h17;
    localparam [7:0]  MOD_CMD_FEED = 8'h1A;
    localparam [7:0]  MOD_CMD_PHASE = 8'h1B;
    localparam [31:0] CARRIER_STEP  = 32'd2684355;

    /* INT_DIVA with these ICP/LPF/FREQUENCY_PIN attributes is the only 8->64 MHz setting Map accepts. */
    wire pll_clk, pll_feedback, pll_lock;
    wire fpga_clk = pll_clk;
    EHXPLLJ #(
        .PLLRST_ENA("DISABLED"), .INTFB_WAKE("DISABLED"), .STDBY_ENABLE("DISABLED"),
        .DPHASE_SOURCE("DISABLED"), .CLKOP_FPHASE(0), .CLKOP_CPHASE(7),
        .OUTDIVIDER_MUXA2("DIVA"), .CLKOP_ENABLE("ENABLED"), .CLKOP_DIV(8),
        .CLKFB_DIV(8), .CLKI_DIV(1), .FEEDBK_PATH("INT_DIVA"),
        .PLL_LOCK_MODE(0)
    ) fpga_pll_i (
        .CLKI(fpga_clk_8m), .CLKFB(pll_feedback), .RST(1'b0),
        .RESETM(1'b0), .RESETC(1'b0), .RESETD(1'b0),
        .PHASESEL0(1'b0), .PHASESEL1(1'b0), .PHASEDIR(1'b0), .PHASESTEP(1'b0),
        .LOADREG(1'b0), .STDBY(1'b0), .PLLWAKESYNC(1'b0), .ENCLKOP(1'b1),
        .CLKOP(pll_clk), .LOCK(pll_lock), .CLKINTFB(pll_feedback)
    )
        /* synthesis FREQUENCY_PIN_CLKOP="64.000000" */
        /* synthesis FREQUENCY_PIN_CLKI="8.000000" */
        /* synthesis ICP_CURRENT="9" */
        /* synthesis LPF_RESISTOR="72" */;

    reg [1:0] pll_lock_sync = 2'b00;
    always @(posedge pll_clk) pll_lock_sync <= {pll_lock_sync[0], pll_lock};

    reg  [7:0]  spi_rx_shift, spi_command, spi_version;
    reg  [2:0]  spi_bit_count;
    reg  [8:0]  spi_byte_count, spi_expected_last;
    reg         expected_valid;
    reg  [15:0] spi_update_flags, spi_extension_length;
    reg  [15:0] spi_frame_sequence, accepted_sequence_spi;
    reg  [1:0]  accepted_update_flags_spi;
    reg  [6:0]  spi_channel_index;
    reg         spi_channel_page;
    reg         spi_compact_data;
    reg         mod_level_spi;
    reg         mod_toggle_spi;
    reg  [7:0]  feed_wr_spi;
    reg         frame_toggle_spi, stop_toggle_spi, invalid_frame_spi, ws2812_toggle_spi;
    reg  [23:0] rgb_values;
    reg  [1:0]  spi_rgb_captured;
    reg         rgb_values_complete_spi;
    reg  [8:0]  status_bit_index;

    wire [7:0]  spi_rx_byte  = {spi_rx_shift[6:0], spi1_mosi};
    wire        spi_phase_only = (spi_command == MOD_CMD_PHASE);
    wire        spi_payload_byte = !fpga_cs_n && spi_update_flags[0] &&
                                   (spi_bit_count == 3'd7) &&
                                   (spi_byte_count >= HEADER_BYTES) &&
                                   (spi_byte_count < HEADER_BYTES + CHANNEL_BYTES);
    /* Staging page bit: 0 = phase page, 1 = level page. */
    wire        staging_wr_page = spi_phase_only ? 1'b0 : spi_channel_page;
    wire [7:0]  staging_wr_addr = {staging_wr_page, spi_channel_index};
    wire [8:0]  spi_rgb_start = HEADER_BYTES[8:0] +
                                (spi_update_flags[0] ? CHANNEL_BYTES[8:0] : 9'd0);
    wire [1:0]  spi_rgb_offset = spi_byte_count[1:0];
    wire        spi_rgb_payload_byte = !fpga_cs_n && spi_update_flags[1] &&
                                       (spi_bit_count == 3'd7) &&
                                       (spi_byte_count[8:2] == spi_rgb_start[8:2]) &&
                                       (spi_rgb_offset != 2'd3);
    wire        spi_compact_cmd = (spi_command == MOD_CMD_FIXED) ||
                                  (spi_command == 8'h16) || (spi_command == 8'h18);
    /* MOD_FEED is a pure level stream: byte 0 is the opcode, byte 1 the
     * version, every byte after that is one level.  It must not run through the
     * 36-byte header parser, otherwise level bytes 18/19 and 34/35 are decoded
     * as update flags and extension length, which both truncates the block at
     * 32 levels and flags the frame invalid. */
    wire        spi_mod_feed = (spi_command == MOD_CMD_FEED);
    wire        spi_audio_read_cmd = (spi_command == 8'h16);
    wire        spi_audio_reset_cmd = (spi_command == 8'h18);
    wire        feed_we = (spi_bit_count == 3'd7) && spi_mod_feed &&
                           (spi_byte_count[8:1] != 8'd0);

    reg         ws2812_enable;
    wire        ws2812_frame_done;
    reg  [1:0]  ws2812_frames_left;
    localparam [1:0] WS2812_FRAMES = 2'd2;

    reg  [23:0] phase_frac;
    reg         phase_step_s1;
    reg  [7:0]  global_phase_s2;
    reg         phase_step_s2;
    reg         wrap_s2, wrap_s3, wrap_s4, wrap_s5;
    reg  [8:0]  run_addr_s3;
    reg         phase_step_s3;
    reg         swap_now_s3;
    reg         phase_step_s4;
    reg  [83:0] ev_run_hold_s5;

    wire [24:0] phase_frac_sum = {1'b0, phase_frac} + 25'd2684355;

    reg  [31:0] fpga_time;
    reg  [5:0]  time_divider;
    wire        us_tick = (time_divider == 6'd63);

    reg  [3:0]  link_idle;
    reg         link_tick_d;

    wire        mic_boot_done = fpga_time[13] && fpga_time[11];
    wire        mic_warm_done = fpga_time[14] && fpga_time[13] &&
                                fpga_time[12] && fpga_time[11];

    reg         mic_warm;
    reg         mic_ultrasonic;
    reg  [7:0]  mic_warm_half;
    reg  [3:0]  mic_phase;
    reg         mic_clock_reg;
    reg  [15:0] mic_shift_0_l, mic_shift_0_r;
    reg  [15:0] mic_shift_1_l, mic_shift_1_r;
    reg  [3:0]  mic_sample_count;
    reg  [63:0] mic_latest;

    reg  [7:0]  pcm_decim_count;
    reg  [9:0]  pcm_acc;
    reg  signed [15:0] pcm_fifo [0:31];
    reg  [5:0]  pcm_wr_ptr;
    reg  [5:0]  pcm_rd_ptr_spi;
    reg  [5:0]  pcm_rd_sync1, pcm_rd_sync2;
    reg  [5:0]  pcm_wr_sync1_spi, pcm_wr_sync2_spi;
    reg  [4:0]  pcm_read_count_latched_spi;
    wire [5:0]  pcm_count_spi = pcm_wr_sync2_spi - pcm_rd_ptr_spi;
    wire [4:0]  pcm_read_count = (pcm_count_spi > 6'd16) ? 5'd16 : pcm_count_spi[4:0];
    localparam [9:0] PCM_BIAS = 11'd1024 - 11'd500;
    wire [9:0] pcm_acc_next = pcm_acc + {8'd0, mic_data_0} + {8'd0, mic_data_1};

    reg  [6:0]  mic_lo_n;
    reg  [6:0]  mic_window_count;
    reg  signed [7:0] mic_acc_i [0:3];
    reg  signed [7:0] mic_acc_q [0:3];
    reg  signed [7:0] mic_win_i [0:3];
    reg  signed [7:0] mic_win_q [0:3];
    reg         mic_bc_valid;
    reg         mic_lo_i_r, mic_lo_q_r;

    wire [6:0]  mic_lo_n_next = (mic_lo_n == 7'd99) ? 7'd0 : (mic_lo_n + 7'd1);
    wire        mic_lo_i_next = (mic_lo_n_next < 7'd25) || (mic_lo_n_next >= 7'd75);
    wire        mic_lo_q_next = (mic_lo_n_next < 7'd50);

    reg  [6:0]  mic_cfg_count;
    reg  [15:0] mic_cfg_start;
    reg  [15:0] mic_cfg_gap;
    reg  [6:0]  mic_cfg_width;
    reg  [1:0]  mic_run_state;
    reg         mic_done;
    reg         mic_saturated;
    reg  [15:0] mic_block_count;
    reg  [6:0]  mic_gate_index;
    reg  [6:0]  mic_gate_fill;
    reg  [15:0] mic_gate_wait;
    reg         mic_gate_active;

    reg         mic_acc_busy;
    reg  [2:0]  mic_acc_state;
    reg  [2:0]  mic_acc_word;
    reg  [6:0]  mic_acc_gate;
    reg         mic_acc_last;
    reg         mic_ram_we;
    reg  [8:0]  mic_ram_wr_addr;
    reg  [15:0] mic_ram_wr_data;
    wire [15:0] mic_ram_rd_data;

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

    reg  [47:0] mic_cfg_hold_spi;
    reg         mic_cfg_toggle_spi, mic_cfg_toggle_meta, mic_cfg_toggle_sync, mic_cfg_toggle_seen;
    reg  [3:0]  mic_cfg_settle;
    wire [6:0]  mic_cfg_count_w = (mic_cfg_hold_spi[6:0] == 7'd0) ? 7'd1 :
                                  ((mic_cfg_hold_spi[6:0] > 7'd64) ? 7'd64 : mic_cfg_hold_spi[6:0]);
    wire [6:0]  mic_cfg_width_w = (mic_cfg_hold_spi[46:40] == 7'd0) ? 7'd1 :
                                  ((mic_cfg_hold_spi[46:40] > 7'd64) ? 7'd64 : mic_cfg_hold_spi[46:40]);
    wire        mic_width_one   = (mic_cfg_width[6:1] == 6'd0);
    /* 17th bit is the borrow of period-width, so the clamp needs no extra comparator. */
    wire [16:0] mic_cfg_gap_w = {1'b0, mic_cfg_hold_spi[39:24]} - {9'd0, mic_cfg_width_w};

    localparam [2:0] EV_IDLE = 3'd0, EV_CLEAR = 3'd1, EV_WAIT = 3'd2,
                     EV_LEVEL = 3'd3, EV_RD = 3'd4, EV_WR = 3'd5;
    reg  [2:0]  ev_state;
    reg  [6:0]  ev_ch;
    reg  [7:0]  rmw_slot;
    reg  [7:0]  build_level_eff;
    reg         ev_fall, ev_wrap;
    reg         frame_req, swap_pending, running, active_bank;
    wire [8:0]  ev_sum = {1'b0, rmw_slot} + {1'b0, build_level_eff};

    wire [10:0] ev_ch_hi /* synthesis syn_keep=1 */;
    wire [7:0]  ev_ch_lo /* synthesis syn_keep=1 */;
    /* Combinational one-hot decode: an 84-bit shift register wrote all-zero tables on silicon. */
    wire [83:0] ev_ch_bit;
    genvar ev_bit_index;
    generate
        for (ev_bit_index = 0; ev_bit_index < 11; ev_bit_index = ev_bit_index + 1) begin : EV_HI_DECODE
            assign ev_ch_hi[ev_bit_index] = (ev_ch[6:3] == ev_bit_index);
        end
        for (ev_bit_index = 0; ev_bit_index < 8; ev_bit_index = ev_bit_index + 1) begin : EV_LO_DECODE
            assign ev_ch_lo[ev_bit_index] = (ev_ch[2:0] == ev_bit_index);
        end
        for (ev_bit_index = 0; ev_bit_index < 84; ev_bit_index = ev_bit_index + 1) begin : EV_BIT_DECODE
            assign ev_ch_bit[ev_bit_index] = ev_ch_hi[ev_bit_index / 8] & ev_ch_lo[ev_bit_index % 8];
        end
    endgenerate

    wire        build_idle = (ev_state == EV_IDLE) && !swap_pending && !frame_req;
    wire        staging_rd_page = (ev_state == EV_LEVEL) || (ev_state == EV_RD) ||
                                  (ev_state == EV_WR);
    wire [7:0]  staging_q;
    umh_channel_ram18 staging_ram (
        .wr_clk(spi1_sck), .wr_addr(staging_wr_addr), .wr_data(spi_rx_byte),
        .wr_en(spi_payload_byte), .rd_clk(fpga_clk),
        .rd_addr({staging_rd_page, ev_ch}), .rd_data(staging_q)
    );

    wire        ev_in_clear   = (ev_state == EV_CLEAR);
    wire        ev_rd_grant   = (ev_state == EV_RD) && !phase_step_s3;
    wire [8:0]  ev_build_addr = {~active_bank, rmw_slot};
    wire [8:0]  event_rd_addr = ev_rd_grant ? ev_build_addr : run_addr_s3;
    wire        ev_we         = ev_in_clear || (ev_state == EV_WR);
    wire [83:0] ev_rd_data;
    wire [83:0] ev_wr_data    = ev_in_clear ? 84'd0 : (ev_rd_data ^ ev_ch_bit);
    umh_toggle_ram84 event_ram (
        .clk(fpga_clk),
        .addr_a(event_rd_addr), .rd_data_a(ev_rd_data),
        .we_b(ev_we), .addr_b(ev_build_addr), .wr_data_b(ev_wr_data)
    );

    umh_mic_iq_ram mic_iq_ram (
        .clk(fpga_clk), .we(mic_ram_we), .wr_addr(mic_ram_wr_addr),
        .wr_data(mic_ram_wr_data),
        .rd_addr(mic_ram_rd_addr), .rd_data(mic_ram_rd_data)
    );

    reg  frame_toggle_meta, frame_toggle_sync, frame_toggle_seen;
    reg  stop_toggle_meta, stop_toggle_sync, stop_toggle_seen;
    reg  ws2812_toggle_meta, ws2812_toggle_sync, ws2812_toggle_seen;
    reg  invalid_frame_meta, invalid_frame_sync;
    reg  [15:0] pending_sequence, accepted_sequence;
    reg  [3:0]  frame_settle;
    reg  [3:0]  ws2812_settle;

    reg  [23:0] rgb_hold;

    reg         mod_active;
    reg  [7:0]  pending_mod_level;
    reg  [7:0]  build_level;
    reg         build_mod;
    reg         frame_req_mod;
    reg         mod_toggle_meta, mod_toggle_sync, mod_toggle_seen;
    reg  [3:0]  mod_settle;

    reg  [7:0]  feed_wr, feed_rd;
    wire [7:0]  feed_q;
    wire [7:0]  feed_fill = feed_wr - feed_rd;

    wire        feed_pop = wrap_s2 && mod_active && build_idle &&
                            (feed_wr != feed_rd);
    umh_level_fifo level_fifo (
        .wr_clk(spi1_sck), .we(feed_we), .wr_addr(feed_wr_spi), .wr_data(spi_rx_byte),
        .rd_clk(fpga_clk), .rd_addr(feed_rd), .rd_data(feed_q)
    );
    /* In common-level mode a non-zero stored level is only an enable marker for the common envelope. */
    wire [7:0]  event_level = build_mod ?
                              ((staging_q != 8'd0) ? build_level : 8'd0) :
                              staging_q;
    wire stop_event = (stop_toggle_sync != stop_toggle_seen);
    wire link_timeout = &link_idle;

    reg  cs_meta, cs_sync, cs_sync_d;
    reg  [127:0] status_hold;
    reg         status_miso_q;
    wire cs_fall = cs_sync_d && !cs_sync;
    wire cs_rise = !cs_sync_d && cs_sync;
    always @(posedge fpga_clk) begin
        cs_meta   <= fpga_cs_n;
        cs_sync   <= cs_meta;
        cs_sync_d <= cs_sync;
    end

    wire [15:0] fifo_credit_wire  = build_idle ? 16'd1 : 16'd0;
    wire [15:0] fifo_depth_wire   = build_idle ? 16'd0 : 16'd1;

    wire [15:0] status_flags_wire = (invalid_frame_sync ? 16'h0004 : 16'h0000) |
                                    (running ? 16'h0010 : 16'h0000) |
                                    (mod_active ? 16'h8000 : 16'h0000) |
                                    (pll_lock_sync[1] ? 16'h2000 : 16'h0000) |
                                    16'h5000;

    wire [127:0] status_word = {
        8'h01, feed_fill,
        fifo_credit_wire[7:0],  fifo_credit_wire[15:8],
        fifo_depth_wire[7:0],   fifo_depth_wire[15:8],
        status_flags_wire[7:0], status_flags_wire[15:8],
        fpga_time[7:0], fpga_time[15:8], fpga_time[23:16], fpga_time[31:24],
        accepted_sequence[7:0], accepted_sequence[15:8], 16'd0
    };

    /* 128:1 status read mux terminates on a flop: full SCK period to settle. */
    wire [8:0]  status_next_index = status_bit_index + 9'd1;
    wire        status_next_bit   = status_hold[7'd127 - status_next_index[6:0]];
    wire        status_miso_bit   = (status_bit_index == 9'd0) ? status_hold[8'd127]
                                                               : status_miso_q;

    always @(negedge spi1_sck or posedge fpga_cs_n) begin
        if (fpga_cs_n) status_miso_q <= 1'b0;
        else           status_miso_q <= status_next_bit;
    end

    wire [8:0] pcm_payload_bit = status_bit_index - 9'd136;
    wire [4:0] pcm_payload_sample = pcm_payload_bit[8:4];
    wire signed [15:0] pcm_spi_sample =
        pcm_fifo[(pcm_rd_ptr_spi + pcm_payload_sample) & 5'h1f];
    wire pcm_audio_miso = (status_bit_index < 9'd136) ?
        ((status_bit_index < 9'd128) ? status_miso_bit :
         ((pcm_read_count_latched_spi >> (7 - status_bit_index[2:0])) & 1'b1)) :
        ((pcm_payload_sample < pcm_read_count_latched_spi) ? pcm_spi_sample[4'd15 - pcm_payload_bit[3:0]] : 1'b0);
    assign spi1_miso = fpga_cs_n ? 1'bz :
                       (spi_audio_read_cmd ? pcm_audio_miso :
                        (status_bit_index < 9'd128) ? status_miso_bit :
                        (status_bit_index < 9'd192) ? mic_meta_word[9'd191 - status_bit_index] :
                        (status_bit_index < 9'd320) ? mic_ram_rd_data[4'd15 - status_bit_index[3:0]] :
                        1'b0);

    always @(negedge spi1_sck or posedge fpga_cs_n) begin
        if (fpga_cs_n) status_bit_index <= 9'd0;
        else if (status_bit_index != 9'd511) status_bit_index <= status_bit_index + 9'd1;
    end

    wire [15:0] ext_len_next  = {spi_rx_byte, spi_extension_length[7:0]};
    wire [15:0] expected_next = HEADER_BYTES +
                                (spi_phase_only ? PHASE_BYTES :
                                 (spi_update_flags[0] ? CHANNEL_BYTES : 16'd0)) +
                                (spi_update_flags[1] ? 16'd12 : 16'd0) + ext_len_next;
    wire        last_header_byte = (spi_byte_count == 16'd35);
    wire        no_payload_bytes = !spi_phase_only && !spi_update_flags[0] &&
                                   !spi_update_flags[1] && (ext_len_next == 16'd0);
    wire        header_end = last_header_byte && no_payload_bytes;
    wire        payload_end = expected_valid && (spi_byte_count == spi_expected_last);
    wire        compact_end = spi_compact_cmd && (spi_bit_count == 3'd7) &&
                              ((spi_command == 8'h16) ? (spi_byte_count == pcm_read_end_byte) : (spi_byte_count == COMPACT_BYTES - 16'd1));
    wire [8:0] pcm_read_end_byte = 9'd48;
    /* MOD_FEED ends at CS, never at a byte count: the host decides the block
     * length, the level FIFO bounds it, and there is no header to complete. */
    wire        frame_end = spi_mod_feed ? 1'b0 :
                            (spi_compact_cmd ? compact_end : (header_end || payload_end));

    always @(posedge spi1_sck or posedge fpga_cs_n) begin
        if (fpga_cs_n) begin
            spi_rx_shift <= 8'd0; spi_bit_count <= 3'd0; spi_byte_count <= 9'd0;
            spi_command <= 8'd0; spi_version <= 8'd0; spi_update_flags <= 16'd0;
            spi_extension_length <= 16'd0; spi_frame_sequence <= 16'd0;
            spi_expected_last <= 9'd35; expected_valid <= 1'b0;
            spi_channel_index <= 7'd0;
            spi_channel_page <= 1'b0; spi_rgb_captured <= 2'd0;
        end else begin
            if (spi_bit_count == 3'd7) begin
                spi_bit_count <= 3'd0;
                spi_rx_shift  <= spi_rx_byte;
                if (spi_byte_count == 16'd0) begin
                    spi_command <= spi_rx_byte;
                end else if (spi_byte_count == 16'd1) begin
                    spi_version <= spi_rx_byte;
                end else if (!spi_mod_feed) begin
                    case (spi_byte_count)
                        16'd2: begin
                            spi_compact_data <= spi_rx_byte[0];
                            if (spi_command == 8'h16) pcm_read_count_latched_spi <= pcm_read_count;
                        end
                        16'd6:  spi_frame_sequence[7:0]   <= spi_rx_byte;
                        16'd7:  spi_frame_sequence[15:8]  <= spi_rx_byte;
                        16'd18: spi_update_flags[7:0]     <= spi_rx_byte;
                        16'd19: spi_update_flags[15:8]    <= spi_rx_byte;
                        16'd34: spi_extension_length[7:0] <= spi_rx_byte;
                        16'd35: begin
                            spi_extension_length[15:8] <= spi_rx_byte;
                            spi_expected_last <= expected_next[8:0] - 9'd1;
                            expected_valid    <= 1'b1;
                        end
                        default: begin

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
                        if (spi_payload_byte) begin
                            if (spi_phase_only) begin
                                if (spi_channel_index != 7'd83)
                                    spi_channel_index <= spi_channel_index + 7'd1;
                            end else begin
                                if (spi_channel_page && spi_channel_index != 7'd83)
                                    spi_channel_index <= spi_channel_index + 7'd1;
                                spi_channel_page <= ~spi_channel_page;
                            end
                        end else if (spi_rgb_payload_byte) begin

                            case (spi_rgb_offset)
                                2'd0: rgb_values[15:8]  <= spi_rx_byte;
                                2'd1: rgb_values[23:16] <= spi_rx_byte;
                                2'd2: rgb_values[7:0]   <= spi_rx_byte;
                                default: ;
                            endcase
                            spi_rgb_captured <= spi_rgb_captured + 2'd1;
                        end
                    end
                endcase
                end
                spi_byte_count <= spi_byte_count + 9'd1;
                if (frame_end) begin

                    if (spi_command == MOD_CMD_FIXED && spi_version == 8'h01) begin
                        mod_level_spi <= spi_compact_data;
                        mod_toggle_spi <= ~mod_toggle_spi;
                        accepted_sequence_spi <= spi_frame_sequence;
                        invalid_frame_spi <= 1'b0;
                    end else if ((spi_command == 8'h10 || spi_command == MOD_CMD_PHASE) &&
                        spi_version == 8'h01 && spi_extension_length <= 16'd32) begin
                        frame_toggle_spi <= ~frame_toggle_spi;
                        accepted_sequence_spi <= spi_frame_sequence;
                        accepted_update_flags_spi <= spi_update_flags[1:0];
                        rgb_values_complete_spi <= (spi_update_flags[1] != 1'b0) && (spi_rgb_captured == 2'd3);
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == 8'h11 || spi_command == 8'h12) begin
                        stop_toggle_spi <= ~stop_toggle_spi;
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == 8'h13 && spi_version == 8'h01 &&
                                 spi_extension_length <= 16'd32) begin

                        ws2812_toggle_spi <= ~ws2812_toggle_spi;
                        accepted_sequence_spi <= spi_frame_sequence;
                        accepted_update_flags_spi <= spi_update_flags[1:0];
                        rgb_values_complete_spi <= (spi_update_flags[1] != 1'b0) && (spi_rgb_captured == 2'd3);
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == 8'h14 && spi_version == 8'h01 &&
                                 spi_extension_length == 16'd6) begin

                        mic_cfg_toggle_spi <= ~mic_cfg_toggle_spi;
                        invalid_frame_spi <= 1'b0;
                    end else if (spi_command == 8'h15 && spi_version == 8'h01 &&
                                 spi_extension_length == 16'd0) begin

                        invalid_frame_spi <= 1'b0;
                    end else if ((spi_command == 8'h16 && spi_version == 8'h01) ||
                                 (spi_command == 8'h18 && spi_version == 8'h01)) begin
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

    always @(posedge spi1_sck) if (feed_we) feed_wr_spi <= feed_wr_spi + 8'd1;

    always @(posedge fpga_clk) begin
        pcm_rd_sync1 <= pcm_rd_ptr_spi;
        pcm_rd_sync2 <= pcm_rd_sync1;
    end

    wire audio_read_end  = compact_end && spi_audio_read_cmd;
    wire audio_reset_end = compact_end && spi_audio_reset_cmd;

    always @(posedge spi1_sck) begin
        pcm_wr_sync1_spi <= pcm_wr_ptr;
        pcm_wr_sync2_spi <= pcm_wr_sync1_spi;
        if (audio_reset_end)
            pcm_rd_ptr_spi <= pcm_wr_sync2_spi;
        else if (audio_read_end)
            pcm_rd_ptr_spi <= pcm_rd_ptr_spi + pcm_read_count_latched_spi;
    end

    always @(posedge fpga_clk) begin

        phase_frac    <= phase_frac_sum[23:0];
        phase_step_s1 <= phase_frac_sum[24];

        phase_step_s2 <= phase_step_s1;
        if (phase_step_s1) begin
            global_phase_s2 <= global_phase_s2 + 8'd1;
            if (global_phase_s2 == 8'hFF) wrap_s2 <= 1'b1;
            else wrap_s2 <= 1'b0;
        end else begin
            wrap_s2 <= 1'b0;
        end

        phase_step_s3 <= phase_step_s2;
        wrap_s3       <= wrap_s2;
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

        phase_step_s4 <= phase_step_s3;
        wrap_s4       <= wrap_s3;

        wrap_s5 <= wrap_s4;
        if (phase_step_s4 && running && !stop_event && !link_timeout)
            ev_run_hold_s5 <= ev_rd_data;
        else
            ev_run_hold_s5 <= 84'd0;

        time_divider <= time_divider + 6'd1;
        if (us_tick) fpga_time <= fpga_time + 32'd1;

        link_tick_d <= fpga_time[10];
        if (cs_fall)
            link_idle <= 4'd0;
        else if (fpga_time[10] && !link_tick_d && !link_timeout)
            link_idle <= link_idle + 4'd1;

        if (fpga_cs_n) status_hold <= status_word;

        frame_toggle_meta      <= frame_toggle_spi;
        frame_toggle_sync      <= frame_toggle_meta;
        stop_toggle_meta       <= stop_toggle_spi;
        stop_toggle_sync       <= stop_toggle_meta;
        ws2812_toggle_meta     <= ws2812_toggle_spi;
        ws2812_toggle_sync     <= ws2812_toggle_meta;
        invalid_frame_meta     <= invalid_frame_spi;
        invalid_frame_sync     <= invalid_frame_meta;
        mod_toggle_meta  <= mod_toggle_spi;
        mod_toggle_sync  <= mod_toggle_meta;

        if (ws2812_frame_done != 1'b0 && ws2812_frames_left != 2'd0) begin ws2812_frames_left <= ws2812_frames_left - 2'd1; if (ws2812_frames_left == 2'd1) ws2812_enable <= 1'b0; end
        if (frame_toggle_sync != frame_toggle_seen) begin
            frame_toggle_seen <= frame_toggle_sync;
            frame_settle      <= 4'd8;
        end else if (frame_settle != 4'd0) begin
            if (frame_settle != 4'd1)
                frame_settle <= frame_settle - 4'd1;

            if (frame_settle == 4'd1) begin
                frame_settle <= 4'd0;

                if (accepted_update_flags_spi[0]) frame_req <= 1'b1;

                if (mod_active == 1'b0) frame_req_mod <= 1'b0;
                if (accepted_update_flags_spi[1] && rgb_values_complete_spi) rgb_hold <= rgb_values;

                if (accepted_update_flags_spi[1]) begin
                    if (rgb_values_complete_spi) begin ws2812_enable <= 1'b1; ws2812_frames_left <= WS2812_FRAMES; end
                end
            end
        end

        if (cs_rise) feed_wr <= feed_wr_spi;
        if (feed_pop) begin
            feed_rd <= feed_rd + 8'd1;
            pending_mod_level <= feed_q;
            frame_req <= 1'b1;
            frame_req_mod <= 1'b1;
        end

        if (mod_toggle_sync != mod_toggle_seen) begin
            mod_toggle_seen <= mod_toggle_sync;
            mod_settle <= 4'd8;
        end else if (mod_settle != 4'd0) begin
            mod_settle <= mod_settle - 4'd1;
            if (mod_settle == 4'd1) begin
                mod_active <= mod_level_spi;
                pending_mod_level <= 8'd0;
                feed_rd <= feed_wr;
                frame_req <= 1'b1;
                frame_req_mod <= 1'b1;
            end
        end

        if (ws2812_toggle_sync != ws2812_toggle_seen) begin
            ws2812_toggle_seen <= ws2812_toggle_sync;

            ws2812_settle <= 4'd15;

        end else if (ws2812_settle != 4'd0) begin

            if (ws2812_settle != 4'd1)
                ws2812_settle <= ws2812_settle - 4'd1;
            if (ws2812_settle == 4'd1) begin
                ws2812_settle <= 4'd0;
                accepted_sequence <= accepted_sequence_spi;
                if (rgb_values_complete_spi) rgb_hold <= rgb_values;

                if (rgb_values_complete_spi) begin ws2812_enable <= 1'b1; ws2812_frames_left <= WS2812_FRAMES; end
            end
        end

        case (ev_state)
            EV_IDLE: begin
                if (frame_req && !swap_pending) begin
                    frame_req        <= 1'b0;
                    build_mod      <= frame_req_mod | mod_active;
                    build_level      <= pending_mod_level;
                    frame_req_mod  <= 1'b0;
                    ev_state         <= EV_CLEAR;
                    ev_ch            <= 7'd0;
                    rmw_slot         <= 8'd0;
                    build_level_eff  <= 8'd1;
                    pending_sequence <= accepted_sequence_spi;
                end
            end
            EV_CLEAR: begin
                rmw_slot <= ev_sum[7:0];
                if (ev_sum[8]) ev_state <= EV_LEVEL;
            end
            EV_WAIT: begin
                ev_state <= EV_LEVEL;
            end
            EV_LEVEL: begin
                rmw_slot <= staging_q;
                ev_fall  <= 1'b0;
                ev_state <= EV_RD;
            end
            EV_RD: begin
                if (ev_rd_grant) begin
                    build_level_eff <= event_level;
                    ev_state        <= EV_WR;
                end
            end
            EV_WR: begin
                if (!ev_fall) begin
                    rmw_slot <= ev_sum[7:0];
                    ev_wrap  <= ev_sum[8];
                    ev_fall  <= 1'b1;
                    ev_state <= EV_RD;
                end else if (ev_wrap) begin
                    rmw_slot <= 8'd0;
                    ev_wrap  <= 1'b0;
                    ev_state <= EV_RD;
                end else begin
                    if (ev_ch == 7'd83) begin
                        ev_state     <= EV_IDLE;
                        swap_pending <= 1'b1;
                    end else begin
                        ev_state <= EV_WAIT;
                    end
                    ev_ch <= ev_ch + 7'd1;
                end
            end
            default: ev_state <= EV_IDLE;
        endcase

        if (stop_event) begin
            stop_toggle_seen <= stop_toggle_sync;
            running       <= 1'b0;
            frame_req     <= 1'b0;
            frame_req_mod <= 1'b0;
            frame_settle  <= 4'd0;
            mod_settle  <= 4'd0;
            mod_active    <= 1'b0;
            pending_mod_level <= 8'd0;
            swap_pending  <= 1'b0;
            ev_state      <= EV_IDLE;
        end else if (link_timeout) begin

            running       <= 1'b0;
            frame_req     <= 1'b0;
            frame_req_mod <= 1'b0;
            frame_settle  <= 4'd0;
            mod_settle  <= 4'd0;
            mod_active    <= 1'b0;
            pending_mod_level <= 8'd0;
            swap_pending  <= 1'b0;
            ev_state      <= EV_IDLE;
            rgb_hold      <= 24'd0;
        end

        if (!mic_ultrasonic && !mic_warm) begin
            mic_clock_reg <= 1'b0;
            if (mic_boot_done) begin
                mic_warm       <= 1'b1;
                mic_warm_half  <= 8'd0;
            end
        end else if (mic_warm) begin
            if (mic_warm_half == 8'd159) begin
                mic_warm_half <= 8'd0;
                mic_clock_reg <= ~mic_clock_reg;
                if (mic_warm_done) begin
                    mic_warm        <= 1'b0;
                    mic_ultrasonic  <= 1'b1;
                    mic_clock_reg   <= 1'b0;
                    mic_phase       <= 4'd0;
                    mic_lo_n           <= 7'd99;
                    mic_lo_i_r         <= 1'b1; mic_lo_q_r <= 1'b1;
                    mic_window_count   <= 7'd0;
                    mic_bc_valid       <= 1'b0;
                    mic_acc_i[0] <= 8'sd0; mic_acc_i[1] <= 8'sd0;
                    mic_acc_i[2] <= 8'sd0; mic_acc_i[3] <= 8'sd0;
                    mic_acc_q[0] <= 8'sd0; mic_acc_q[1] <= 8'sd0;
                    mic_acc_q[2] <= 8'sd0; mic_acc_q[3] <= 8'sd0;
                end
            end else begin
                mic_warm_half <= mic_warm_half + 8'd1;
            end
        end else begin

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
                pcm_acc <= pcm_acc_next;
                mic_shift_0_l <= {mic_shift_0_l[14:0], mic_data_0};
                mic_shift_1_l <= {mic_shift_1_l[14:0], mic_data_1};
            end else if (mic_phase == 4'd12) begin
                `MIC_UPDATE(1, mic_data_0)
                `MIC_UPDATE(3, mic_data_1)
                pcm_acc <= pcm_acc_next;
                if (pcm_decim_count == 8'd249) begin
                    pcm_decim_count <= 8'd0;
                    if ((pcm_wr_ptr - pcm_rd_sync2) < 6'd32) begin
                        pcm_fifo[pcm_wr_ptr[4:0]] <= {pcm_acc_next, 6'b0};
                        pcm_wr_ptr <= pcm_wr_ptr + 6'd1;
                    end

                    pcm_acc <= PCM_BIAS;
                end else begin
                    pcm_decim_count <= pcm_decim_count + 8'd1;
                end
                mic_shift_0_r <= {mic_shift_0_r[14:0], mic_data_0};
                mic_shift_1_r <= {mic_shift_1_r[14:0], mic_data_1};
                mic_sample_count <= mic_sample_count + 4'd1;
                if (mic_sample_count == 4'd15)
                    mic_latest <= {mic_shift_0_l,
                                   {mic_shift_0_r[14:0], mic_data_0},
                                   mic_shift_1_l,
                                   {mic_shift_1_r[14:0], mic_data_1}};
                if (mic_window_count != 7'd100)
                    mic_window_count <= mic_window_count + 7'd1;
            end else if (mic_phase == 4'd13) begin
                mic_bc_valid <= 1'b0;
                if (mic_window_count == 7'd100) begin

                    mic_win_i[0] <= mic_acc_i[0];
                    mic_win_q[0] <= mic_acc_q[0];
                    mic_win_i[1] <= mic_acc_i[1];
                    mic_win_q[1] <= mic_acc_q[1];
                    mic_win_i[2] <= mic_acc_i[2];
                    mic_win_q[2] <= mic_acc_q[2];
                    mic_win_i[3] <= mic_acc_i[3];
                    mic_win_q[3] <= mic_acc_q[3];
                    mic_bc_valid    <= 1'b1;
                    mic_window_count <= 7'd0;
                    mic_acc_i[0] <= 8'sd0; mic_acc_i[1] <= 8'sd0;
                    mic_acc_i[2] <= 8'sd0; mic_acc_i[3] <= 8'sd0;
                    mic_acc_q[0] <= 8'sd0; mic_acc_q[1] <= 8'sd0;
                    mic_acc_q[2] <= 8'sd0; mic_acc_q[3] <= 8'sd0;
                end
            end
        end

        mic_cfg_toggle_meta <= mic_cfg_toggle_spi;
        mic_cfg_toggle_sync <= mic_cfg_toggle_meta;

        if (mic_cfg_toggle_sync != mic_cfg_toggle_seen) begin
            mic_cfg_toggle_seen <= mic_cfg_toggle_sync;
            mic_cfg_settle      <= 4'd4;
        end else if (mic_cfg_settle != 4'd0) begin
            if (mic_cfg_settle == 4'd1) begin
                mic_cfg_settle  <= 4'd0;
                mic_cfg_count   <= mic_cfg_count_w;
                mic_cfg_width   <= mic_cfg_width_w;
                mic_cfg_start   <= mic_cfg_hold_spi[23:8];
                mic_cfg_gap     <= mic_cfg_gap_w[16] ? 16'd0 : mic_cfg_gap_w[15:0];
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

            mic_phase        <= 4'd0;
            mic_lo_n         <= 7'd99;
            mic_lo_i_r       <= 1'b1;
            mic_lo_q_r       <= 1'b1;
            mic_window_count <= 7'd0;
            mic_bc_valid     <= 1'b0;
            mic_acc_i[0] <= 8'sd0; mic_acc_i[1] <= 8'sd0;
            mic_acc_i[2] <= 8'sd0; mic_acc_i[3] <= 8'sd0;
            mic_acc_q[0] <= 8'sd0; mic_acc_q[1] <= 8'sd0;
            mic_acc_q[2] <= 8'sd0; mic_acc_q[3] <= 8'sd0;
        end

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
                    mic_acc_last    <= mic_width_one;
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
            3'd1: begin
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
            3'd2: begin
                mic_ram_we    <= 1'b0;
                mic_acc_state <= 3'd3;
            end
            3'd3: begin
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
            3'd4: begin
                mic_ram_we    <= 1'b0;
                mic_acc_busy  <= 1'b0;
                mic_acc_last  <= 1'b0;
                mic_acc_state <= 3'd0;
                `MIC_FINISH_GATE
            end
            default: mic_ram_we <= 1'b0;
        endcase
    end

    /* Unconditional register, no gated enable or async clear: LSE gated-enable translation once broke us_tx. */
    reg  [83:0] us_tx_next;
    always @* us_tx_next = ev_run_hold_s5 ^ ((wrap_s5 || !running) ? 84'd0 : us_tx);

    always @(posedge fpga_clk) us_tx <= us_tx_next;

    assign mic_clk = mic_clock_reg;

    ws2812_stream ws2812_i (
        .clk(fpga_clk),
        .enable(ws2812_enable),
        .grb(rgb_hold),
        .frame_done(ws2812_frame_done),
        .data_out(rgb_data)
    );
    spi_mic_stream mic_stream_i (
        .cs_n(spi_mic_cs_n), .sck(spi_mic_sck), .sample_word(mic_latest),
        .miso(spi_mic_miso)
    );

    initial begin
        spi_rx_shift = 8'd0; spi_bit_count = 3'd0; spi_byte_count = 9'd0;
        spi_command = 8'd0; spi_version = 8'd0; spi_update_flags = 16'd0;
        spi_extension_length = 16'd0; spi_frame_sequence = 16'd0; accepted_update_flags_spi = 2'd0;
        spi_expected_last = 9'd35; expected_valid = 1'b0;
        spi_channel_index = 7'd0;
        spi_channel_page = 1'b0; spi_rgb_captured = 2'd0;
        spi_compact_data = 1'b0; mod_level_spi = 1'b0;
        mod_toggle_spi = 1'b0; feed_wr_spi = 8'd0;
        frame_toggle_spi = 1'b0; stop_toggle_spi = 1'b0; invalid_frame_spi = 1'b0;
        ws2812_toggle_spi = 1'b0;
        rgb_values = 24'd0; rgb_values_complete_spi = 1'b0; status_bit_index = 9'd0;
        frame_toggle_meta = 1'b0; frame_toggle_sync = 1'b0; frame_toggle_seen = 1'b0;
        stop_toggle_meta = 1'b0; stop_toggle_sync = 1'b0; stop_toggle_seen = 1'b0;
        ws2812_toggle_meta = 1'b0; ws2812_toggle_sync = 1'b0; ws2812_toggle_seen = 1'b0;
        invalid_frame_meta = 1'b0; invalid_frame_sync = 1'b0;
        pending_sequence = 16'd0; accepted_sequence = 16'd0; frame_settle = 4'd0;
        rgb_hold = 24'd0; status_hold = 128'd0; status_miso_q = 1'b0;
        ws2812_settle = 4'd0;
        mod_active = 1'b0; pending_mod_level = 8'd0;
        build_level = 8'd0; build_level_eff = 8'd0;
        build_mod = 1'b0; frame_req_mod = 1'b0;
        mod_toggle_meta = 1'b0;
        mod_toggle_sync = 1'b0; mod_toggle_seen = 1'b0;
        mod_settle = 4'd0;
        feed_wr = 8'd0; feed_rd = 8'd0;
        cs_meta = 1'b1; cs_sync = 1'b1; cs_sync_d = 1'b1;
        phase_frac = 24'd0; phase_step_s1 = 1'b0; global_phase_s2 = 8'd0;
        phase_step_s2 = 1'b0; wrap_s2 = 1'b0; run_addr_s3 = 9'd0;
        phase_step_s3 = 1'b0; swap_now_s3 = 1'b0; phase_step_s4 = 1'b0;
        ev_run_hold_s5 = 84'd0; wrap_s3 = 1'b0; wrap_s4 = 1'b0; wrap_s5 = 1'b0;
        fpga_time = 32'd0; time_divider = 6'd0;
        link_idle = 4'd0; link_tick_d = 1'b0;
        ev_state = EV_IDLE; ev_ch = 7'd0; rmw_slot = 8'd0;
        ev_fall = 1'b0; ev_wrap = 1'b0;
        frame_req = 1'b0; swap_pending = 1'b0; running = 1'b0; active_bank = 1'b0;
        us_tx = 84'd0;
        mic_clock_reg = 1'b0;
        mic_shift_0_l = 16'd0; mic_shift_0_r = 16'd0;
        mic_shift_1_l = 16'd0; mic_shift_1_r = 16'd0;
        mic_sample_count = 4'd0; mic_latest = 64'd0;
        pcm_decim_count = 8'd0;
        pcm_acc = PCM_BIAS;
        pcm_wr_ptr = 6'd0; pcm_rd_ptr_spi = 6'd0;
        pcm_rd_sync1 = 6'd0; pcm_rd_sync2 = 6'd0;
        pcm_wr_sync1_spi = 6'd0; pcm_wr_sync2_spi = 6'd0;
        pcm_read_count_latched_spi = 5'd0;
        mic_warm = 1'b0; mic_ultrasonic = 1'b0;
        mic_warm_half = 8'd0; mic_phase = 4'd0;

        mic_lo_n = 7'd99; mic_window_count = 7'd0; mic_bc_valid = 1'b0;
        mic_lo_i_r = 1'b1; mic_lo_q_r = 1'b1;
        mic_acc_i[0] = 8'sd0; mic_acc_i[1] = 8'sd0;
        mic_acc_i[2] = 8'sd0; mic_acc_i[3] = 8'sd0;
        mic_acc_q[0] = 8'sd0; mic_acc_q[1] = 8'sd0;
        mic_acc_q[2] = 8'sd0; mic_acc_q[3] = 8'sd0;
        mic_win_i[0] = 8'sd0; mic_win_i[1] = 8'sd0;
        mic_win_i[2] = 8'sd0; mic_win_i[3] = 8'sd0;
        mic_win_q[0] = 8'sd0; mic_win_q[1] = 8'sd0;
        mic_win_q[2] = 8'sd0; mic_win_q[3] = 8'sd0;
        mic_cfg_count = 7'd64; mic_cfg_start = 16'd0;
        mic_cfg_width = 7'd8; mic_cfg_gap = 16'd4;
        mic_run_state = 2'd0; mic_done = 1'b0; mic_saturated = 1'b0;
        mic_block_count = 16'd0; mic_gate_index = 7'd0; mic_gate_fill = 7'd0;
        mic_gate_wait = 16'd0; mic_gate_active = 1'b0;
        mic_acc_busy = 1'b0; mic_acc_state = 3'd0; mic_acc_word = 3'd0;
        mic_acc_gate = 7'd0; mic_acc_last = 1'b0;
        mic_ram_we = 1'b0; mic_ram_wr_addr = 9'd0; mic_ram_wr_data = 16'd0;
        mic_cfg_hold_spi = 48'd0;
        mic_cfg_toggle_spi = 1'b0; mic_cfg_toggle_meta = 1'b0;
        mic_cfg_toggle_sync = 1'b0; mic_cfg_toggle_seen = 1'b0;
        mic_cfg_settle = 4'd0;

        ws2812_enable = 1'b0;
        ws2812_frames_left = 2'd0;
    end
endmodule

