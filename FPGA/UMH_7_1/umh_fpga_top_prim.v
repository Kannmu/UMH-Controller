// Verilog netlist produced by program LSE :  version Diamond (64-bit) 3.13.0.56.2
// Netlist written on Tue Oct 27 21:13:32 2026
//
// Verilog Description of module umh_fpga_top
//

module umh_fpga_top (fpga_clk_8m, fpga_cs_n, spi1_sck, spi1_mosi, spi1_miso, 
            us_tx, rgb_data, mic_clk, mic_data_0, mic_data_1, spi_mic_cs_n, 
            spi_mic_sck, spi_mic_miso) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(83[8:20])
    input fpga_clk_8m;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(84[24:35])
    input fpga_cs_n;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(85[24:33])
    input spi1_sck;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(86[24:32])
    input spi1_mosi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(87[24:33])
    output spi1_miso;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(88[24:33])
    output [83:0]us_tx;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    output rgb_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(90[24:32])
    output mic_clk;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(91[24:31])
    input mic_data_0;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(92[24:34])
    input mic_data_1;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(93[24:34])
    input spi_mic_cs_n;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[24:36])
    input spi_mic_sck;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(95[24:35])
    output spi_mic_miso;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(96[24:36])
    
    wire fpga_clk_8m_c /* synthesis is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(84[24:35])
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(86[24:32])
    wire spi_mic_sck_c /* synthesis is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(95[24:35])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(108[10:17])
    wire spi1_sck_N_155 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(148[17:33])
    wire sck_N_3124 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(10[12:16])
    
    wire GND_net, VCC_net, fpga_cs_n_c, spi1_mosi_c_0, us_tx_c_83, 
        us_tx_c_82, us_tx_c_81, us_tx_c_80, us_tx_c_79, us_tx_c_78, 
        us_tx_c_77, us_tx_c_76, us_tx_c_75, us_tx_c_74, us_tx_c_73, 
        us_tx_c_72, us_tx_c_71, us_tx_c_70, us_tx_c_69, us_tx_c_68, 
        us_tx_c_67, us_tx_c_66, us_tx_c_65, us_tx_c_64, us_tx_c_63, 
        us_tx_c_62, us_tx_c_61, us_tx_c_60, us_tx_c_59, us_tx_c_58, 
        us_tx_c_57, us_tx_c_56, us_tx_c_55, us_tx_c_54, us_tx_c_53, 
        us_tx_c_52, us_tx_c_51, us_tx_c_50, us_tx_c_49, us_tx_c_48, 
        us_tx_c_47, us_tx_c_46, us_tx_c_45, us_tx_c_44, us_tx_c_43, 
        us_tx_c_42, us_tx_c_41, us_tx_c_40, us_tx_c_39, us_tx_c_38, 
        us_tx_c_37, us_tx_c_36, us_tx_c_35, us_tx_c_34, us_tx_c_33, 
        us_tx_c_32, us_tx_c_31, us_tx_c_30, us_tx_c_29, us_tx_c_28, 
        us_tx_c_27, us_tx_c_26, us_tx_c_25, us_tx_c_24, us_tx_c_23, 
        us_tx_c_22, us_tx_c_21, us_tx_c_20, us_tx_c_19, us_tx_c_18, 
        us_tx_c_17, us_tx_c_16, us_tx_c_15, us_tx_c_14, us_tx_c_13, 
        us_tx_c_12, us_tx_c_11, us_tx_c_10, us_tx_c_9, us_tx_c_8, 
        us_tx_c_7, us_tx_c_6, us_tx_c_5, us_tx_c_4, us_tx_c_3, us_tx_c_2, 
        us_tx_c_1, us_tx_c_0, rgb_data_c, mic_clk_c, mic_data_0_c, 
        mic_data_1_c, spi_mic_cs_n_c, spi_mic_miso_c, pll_feedback, 
        pll_lock;
    wire [1:0]pll_lock_sync;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(128[15:28])
    wire [7:0]spi_rx_shift;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(131[17:29])
    wire [7:0]spi_command;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(131[31:42])
    wire [7:0]spi_version;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(131[44:55])
    wire [2:0]spi_bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(132[17:30])
    wire [8:0]spi_byte_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(133[17:31])
    wire [8:0]spi_expected_last;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(133[33:50])
    
    wire expected_valid, n14797;
    wire [15:0]spi_extension_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(135[35:55])
    wire [15:0]spi_frame_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(136[17:35])
    wire [15:0]accepted_sequence_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(136[37:58])
    wire [1:0]accepted_update_flags_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(137[17:42])
    wire [6:0]spi_channel_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(138[17:34])
    
    wire spi_channel_page, spi_compact_data, mod_level_spi, mod_toggle_spi;
    wire [7:0]feed_wr_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(143[17:28])
    
    wire frame_toggle_spi, stop_toggle_spi, invalid_frame_spi, ws2812_toggle_spi;
    wire [23:0]rgb_values;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(145[17:27])
    wire [1:0]spi_rgb_captured;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(146[17:33])
    
    wire rgb_values_complete_spi;
    wire [8:0]status_bit_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(148[17:33])
    
    wire n25582, spi_payload_byte, staging_wr_page;
    wire [8:0]spi_rgb_start;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(159[17:30])
    
    wire n33, n25474, spi_audio_read_cmd, feed_we, ws2812_enable, 
        ws2812_frame_done;
    wire [1:0]ws2812_frames_left;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(181[17:35])
    wire [23:0]phase_frac;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(184[17:27])
    
    wire phase_step_s1;
    wire [7:0]global_phase_s2;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(186[17:32])
    
    wire phase_step_s2, wrap_s2, wrap_s3, wrap_s4, wrap_s5;
    wire [8:0]run_addr_s3;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(189[17:28])
    
    wire phase_step_s3, swap_now_s3, phase_step_s4;
    wire [83:0]ev_run_hold_s5;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:31])
    wire [24:0]phase_frac_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[17:31])
    wire [31:0]fpga_time;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(197[17:26])
    wire [5:0]time_divider;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(198[17:29])
    wire [3:0]link_idle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(201[17:26])
    
    wire link_tick_d, mic_boot_done, mic_warm, mic_ultrasonic;
    wire [7:0]mic_warm_half;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[17:30])
    wire [3:0]mic_phase;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[17:26])
    wire [15:0]mic_shift_0_l;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(213[17:30])
    wire [15:0]mic_shift_0_r;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(213[32:45])
    wire [15:0]mic_shift_1_l;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(214[17:30])
    wire [15:0]mic_shift_1_r;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(214[32:45])
    wire [3:0]mic_sample_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(215[17:33])
    wire [63:0]mic_latest;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[17:27])
    wire [7:0]pcm_decim_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(218[17:32])
    wire [9:0]pcm_acc;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[17:24])
    wire [5:0]pcm_wr_ptr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(221[17:27])
    wire [5:0]pcm_rd_ptr_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(222[17:31])
    wire [5:0]pcm_rd_sync1;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(223[17:29])
    wire [5:0]pcm_rd_sync2;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(223[31:43])
    wire [5:0]pcm_wr_sync1_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(224[17:33])
    wire [5:0]pcm_wr_sync2_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(224[35:51])
    wire [4:0]pcm_read_count_latched_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(225[17:43])
    wire [5:0]pcm_count_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[17:30])
    wire [9:0]pcm_acc_next;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[16:28])
    wire [6:0]mic_lo_n;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(231[17:25])
    wire [6:0]mic_window_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(232[17:33])
    wire [7:0]\mic_acc_i[0] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[23:32])
    wire [7:0]\mic_acc_i[1] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[23:32])
    wire [7:0]\mic_acc_i[2] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[23:32])
    wire [7:0]\mic_acc_i[3] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[23:32])
    wire [7:0]\mic_acc_q[0] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(234[23:32])
    wire [7:0]\mic_acc_q[1] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(234[23:32])
    wire [7:0]\mic_acc_q[2] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(234[23:32])
    wire [7:0]\mic_acc_q[3] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(234[23:32])
    wire [7:0]\mic_win_i[0] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(235[23:32])
    wire [7:0]\mic_win_i[1] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(235[23:32])
    wire [7:0]\mic_win_i[2] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(235[23:32])
    wire [7:0]\mic_win_i[3] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(235[23:32])
    wire [7:0]\mic_win_q[0] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[23:32])
    wire [7:0]\mic_win_q[1] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[23:32])
    wire [7:0]\mic_win_q[2] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[23:32])
    wire [7:0]\mic_win_q[3] ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[23:32])
    
    wire mic_bc_valid, mic_lo_i_r, mic_lo_q_r;
    wire [6:0]mic_lo_n_next;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[17:30])
    
    wire mic_lo_q_next;
    wire [6:0]mic_cfg_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(244[17:30])
    wire [15:0]mic_cfg_start;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[17:30])
    wire [15:0]mic_cfg_gap;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(246[17:28])
    wire [6:0]mic_cfg_width;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(247[17:30])
    wire [1:0]mic_run_state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(248[17:30])
    
    wire mic_done;
    wire [15:0]mic_block_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(251[17:32])
    wire [6:0]mic_gate_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[17:31])
    wire [6:0]mic_gate_fill;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(253[17:30])
    wire [15:0]mic_gate_wait;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(254[17:30])
    
    wire mic_gate_active, mic_acc_busy;
    wire [2:0]mic_acc_state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(258[17:30])
    wire [2:0]mic_acc_word;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(259[17:29])
    wire [6:0]mic_acc_gate;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(260[17:29])
    
    wire mic_acc_last, mic_ram_we;
    wire [8:0]mic_ram_wr_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(263[17:32])
    wire [15:0]mic_ram_wr_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(264[17:32])
    wire [15:0]mic_ram_rd_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(265[17:32])
    wire [8:0]mic_ram_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[17:32])
    wire [7:0]mic_acc_win;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(279[23:34])
    wire [47:0]mic_cfg_hold_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(283[17:33])
    
    wire n28017, mic_cfg_toggle_spi, mic_cfg_toggle_meta, mic_cfg_toggle_sync, 
        mic_cfg_toggle_seen;
    wire [3:0]mic_cfg_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(285[17:31])
    wire [6:0]mic_cfg_width_w;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[17:32])
    wire [16:0]mic_cfg_gap_w;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[17:30])
    wire [6:0]ev_ch;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(297[17:22])
    wire [7:0]rmw_slot;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(298[17:25])
    wire [7:0]build_level_eff;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(299[17:32])
    
    wire ev_fall, ev_wrap, frame_req, swap_pending, running, active_bank;
    wire [8:0]ev_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[17:23])
    
    wire ev_ch_hi_9, ev_ch_hi_8, ev_ch_hi_7, ev_ch_hi_6, ev_ch_hi_5, 
        ev_ch_hi_4, ev_ch_hi_3, ev_ch_hi_2, ev_ch_hi_1, ev_ch_hi_0, 
        ev_ch_lo_7, ev_ch_lo_6, ev_ch_lo_5, ev_ch_lo_4, ev_ch_lo_3, 
        ev_ch_lo_2, ev_ch_lo_1, ev_ch_lo_0, staging_rd_page;
    wire [7:0]staging_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[17:26])
    
    wire ev_in_clear, ev_rd_grant, active_bank_N_892;
    wire [8:0]event_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[17:30])
    
    wire ev_we;
    wire [83:0]ev_rd_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(336[17:27])
    wire [83:0]ev_wr_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[17:27])
    
    wire frame_toggle_meta, frame_toggle_sync, frame_toggle_seen, stop_toggle_meta, 
        stop_toggle_sync, stop_toggle_seen, ws2812_toggle_meta, ws2812_toggle_sync, 
        ws2812_toggle_seen, invalid_frame_meta, invalid_frame_sync;
    wire [15:0]pending_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(354[17:33])
    wire [15:0]accepted_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(354[35:52])
    wire [3:0]frame_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(355[17:29])
    wire [3:0]ws2812_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(356[17:30])
    wire [23:0]rgb_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(358[17:25])
    
    wire mod_active;
    wire [7:0]pending_mod_level;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(361[17:34])
    wire [7:0]build_level;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(362[17:28])
    
    wire build_mod, frame_req_mod, mod_toggle_meta, mod_toggle_sync, 
        mod_toggle_seen;
    wire [3:0]mod_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[17:27])
    wire [7:0]feed_wr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[17:24])
    wire [7:0]feed_rd;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[26:33])
    wire [7:0]feed_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[17:23])
    wire [7:0]feed_fill;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[17:26])
    
    wire feed_pop;
    wire [7:0]event_level;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(379[17:28])
    
    wire stop_event, link_timeout, cs_meta, cs_sync, cs_sync_d;
    wire [127:0]status_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(386[18:29])
    
    wire n10061, n10058, n28016, n13980, status_miso_q, cs_fall, 
        cs_rise, n16730, n24461, n10722, n10723, n10724;
    wire [15:0]fifo_credit_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(396[17:33])
    
    wire n61, n119;
    wire [15:0]fifo_depth_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(397[17:32])
    wire [8:0]status_next_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(415[17:34])
    
    wire status_next_bit;
    wire [8:0]pcm_payload_bit;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(425[16:31])
    
    wire pcm_audio_miso, pll_clk_enable_122, payload_end, compact_end, 
        frame_end, audio_reset_end;
    wire [83:0]us_tx_next;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1014[17:27])
    
    wire spi1_sck_c_enable_144, n10737, n10734, n28011;
    wire [6:0]mic_lo_n_next_6__N_642;
    wire [6:0]mic_cfg_count_w_6__N_850;
    
    wire ev_ch_hi_10, build_idle_N_2714, staging_rd_page_N_2718, staging_rd_page_N_2719, 
        feed_pop_N_2749, n10677, n10654, n19096, spi1_sck_c_enable_122, 
        n28010, n10668, pll_clk_enable_2, n24459, n24458;
    wire [15:0]status_flags_wire_15__N_1355;
    
    wire spi1_sck_c_enable_164, spi1_sck_c_enable_161, n28008;
    wire [6:0]status_next_bit_N_2754;
    
    wire n28007, n24457;
    wire [5:0]pcm_spi_sample_15__N_1371;
    
    wire pll_clk_enable_39, n10, n9, n10739, n24456;
    wire [4:0]pcm_audio_miso_N_2767;
    
    wire n982, n25947, n28627, n8, pll_clk_enable_82, spi1_miso_N_2436, 
        spi1_miso_N_2433, spi1_miso_N_2432, fpga_cs_n_N_2452;
    wire [8:0]status_bit_index_8__N_1377;
    
    wire n10738, spi1_sck_c_enable_38, n12938, spi1_sck_c_enable_39, 
        pll_clk_enable_21, spi1_sck_c_enable_76, n24433, n24431, n24430, 
        n24429, n24428, n4;
    wire [15:0]expected_next_8__N_1396;
    
    wire n10736, n16736, spi1_sck_c_enable_66, n34, spi_phase_only_N_2776, 
        spi_bit_count_2__N_1415, spi_mod_feed_N_2470, n45, n44, n43, 
        n42, n41, n40, n39, n38, pll_clk_enable_304, n11, n4_adj_3147, 
        n13362, n13359, n92, n15, n25491, n27406, n25425, spi_channel_page_N_2479, 
        spi1_sck_c_enable_169, n24451, n15324, n15326, n15330, n15332, 
        n16907, n17690, n27993, n25, n24, n23, n22, n127, n28, 
        n27403, n23783, n23781, n23779, n24165, n17616, n27, n27992, 
        n13623, n15334, n26, n24325, invalid_frame_spi_N_2546, n27400, 
        n17636, n6, mic_cfg_toggle_spi_N_2681, ws2812_toggle_spi_N_2555, 
        stop_toggle_spi_N_2519, n25_adj_3148, frame_toggle_spi_N_2508, 
        mod_toggle_spi_N_2503, invalid_frame_spi_N_2526, n4_adj_3149, 
        n27397, n17637, n17628, n15123, n24318, n27990, n27394, 
        n23773, n27391, n24446, n6_adj_3150, n65, n17693, n23769, 
        n23775, n17390;
    wire [5:0]pcm_rd_ptr_spi_5__N_1610;
    
    wire n23790, n23788, n25510, wrap_s2_N_2598, n23771, active_bank_N_894, 
        active_bank_N_893, n17388, n24445, n27388, n26549, ev_run_hold_s5_83__N_1633, 
        n17704, n14052, n14051, n14049, n14048, n14047, n14046, 
        n14045, n14044, n14043, n14042, n14041, n14040, n14039, 
        n14038, n14037, n14036, n14035, n14034, n14033, n14032, 
        n14031, n14030, n14029, n14028, n14027, n14026, n14025, 
        n14024, n14023, n14022, n14021, n14020, n14019, n14018, 
        n14014, n14013, n14012, n14011, n14010, n14009, n14008, 
        n14007, n14006, n14005, n14004, n14003, n14002, n14001, 
        n14000, n13999, n13998, n13997, n13995, n13994, n13993, 
        n13992, n13991, n13990, n13989, n13988, n15019, n17719, 
        n27379, n25548, n23825, n15021, n38_adj_3151, n7, n27989, 
        n24443, n24442, n24441, n11_adj_3152, n27984, n24440, n3182, 
        n3183, n3184, n25494, n13987, n13986, n13985, n13984, 
        n27983, n10024, n27376, n27373, n25540, n4_adj_3153, n4_adj_3154, 
        n31, pll_clk_enable_91, n13982, n27370, n3334, ws2812_enable_N_2592, 
        ws2812_enable_N_2591, n27367, n3389, n3390, n27632, n25806, 
        n25804, n13983, pll_clk_enable_93, feed_rd_7__N_1692, frame_req_N_2700, 
        pll_clk_enable_85, n27364, n3592, n18, pll_clk_enable_94, 
        n15_adj_3155, n27630, frame_req_mod_N_2737, ws2812_settle_3__N_1697, 
        ws2812_enable_N_2596, n28026, n27634, n20290, n11_adj_3156, 
        n8_adj_3157;
    wire [15:0]accepted_sequence_15__N_995;
    
    wire n17618, n17720, n8_adj_3158, n5, n13, ws2812_enable_N_2578, 
        spi1_sck_c_enable_50, n27355, n27937, n44_adj_3159, n45_adj_3160;
    wire [7:0]build_level_eff_7__N_1772;
    
    wire n24638, n10735, n27352, spi1_sck_c_enable_100, n10_adj_3161, 
        pll_clk_enable_296, n27934, ev_fall_N_2683, swap_pending_N_2707, 
        n25802, n24439, n24438, n24437, n24435, n17, n25492, pll_clk_enable_103, 
        n13_adj_3162, n16910, n17217, n24434, n17676, pll_clk_enable_592, 
        n11_adj_3163, n17608, n27961, n17624, n17614, mic_warm_half_7__N_1887, 
        n25550, mic_ultrasonic_N_1894, n20, n19, n18_adj_3164, n25800, 
        n27633, pll_clk_enable_3, n24423, n28078, n25798, n17610, 
        n17612, n17600, n17604, n17606, n27331, n17620, n24422, 
        n23819, spi1_sck_c_enable_25, mic_lo_i_r_N_2629, n35, n17630, 
        n17632, n4_adj_3165, n25614, n24557, pll_clk_enable_584, n24556, 
        n24555, n24553, n17626, n24552, n24551, n27328, n27972, 
        n4489, n13_adj_3166, n4518, n20_adj_3167, n19_adj_3168, n18_adj_3169, 
        n4547, n16085, n28077, n43_adj_3170, n42_adj_3171, n41_adj_3172, 
        n40_adj_3173, n39_adj_3174, n38_adj_3175, n4576, n23820, n24449, 
        n24324, n23821, n24421, n27971, n24550, n30, n31_adj_3176, 
        n32, pcm_fifo_N_2613;
    wire [5:0]pcm_fifo_N_2607;
    
    wire n27969, n8_adj_3177, n28188, n24549, n9_adj_3178, n24548, 
        n27952, n157, n158, n159, n160, n161, n162, n163, n164, 
        n165, n27968, n16394, spi1_sck_c_enable_10, n24547, n24546, 
        n28186, n28076, n24415, n24414, n24413, n60, n24545, n24447, 
        n27313, n23822, n24323, spi1_sck_c_enable_115, n23823, n23777, 
        n17672, n24412, n24450, n15455, n23824, spi1_sck_c_enable_138, 
        n24544, n28185, n32_adj_3179, n150, n151, n152, n153, 
        n154, n155, n156, pll_clk_enable_193, pll_clk_enable_138, 
        n134, n135, n136, n137, n138, n139, n140, n141, n142, 
        n143, n144, n145, n146, n147, n148, n149, n24542, n27310, 
        n21183, n24541, n24328, pll_clk_enable_141, pll_clk_enable_216, 
        pll_clk_enable_318, pll_clk_enable_124, n12, pll_clk_enable_187, 
        n25796, n27955, n28184, n28183, n17760, n24317, n27527, 
        n12_adj_3180, n27967, n17753, n24407, n24540, n28182, n27526, 
        n24538, n24537, n24406, n24319, n24536, n11_adj_3181, n28181, 
        n6_adj_3182, n24535, n24534, n24533, n15335, n15333, n15331, 
        n15327, n15325, n15323, n15322, spi1_sck_c_enable_143, n62, 
        n24532, n24531, n25716, n24530, n24529, n24405, n24170, 
        n40_adj_3183, n39_adj_3184, n38_adj_3185, n37, n36, n35_adj_3186, 
        n34_adj_3187, n17742, n12_adj_3188, n24327, n24528, n28180, 
        n23736, n11_adj_3189, n8_adj_3190, n5381, n5387, n5388, 
        spi1_sck_c_enable_69, n10_adj_3191, n25791, pll_clk_enable_112, 
        n9_adj_3192, pll_clk_enable_604, n24527, n43_adj_3193, n45_adj_3194, 
        n44_adj_3195, n43_adj_3196, n42_adj_3197, n41_adj_3198, n40_adj_3199, 
        n39_adj_3200, n38_adj_3201, spi1_sck_c_enable_130, n53, spi1_sck_c_enable_108, 
        n35_adj_3202, n34_adj_3203, n33_adj_3204, n32_adj_3205, n31_adj_3206, 
        n30_adj_3207, n12_adj_3208, n17726, spi1_sck_c_enable_92, n24526, 
        n40_adj_3209, n39_adj_3210, n38_adj_3211, n37_adj_3212, n36_adj_3213, 
        n35_adj_3214, n34_adj_3215, n38_adj_3216, pll_clk_enable_121, 
        n7_adj_3217, spi1_sck_c_enable_68, pll_clk_enable_159, pll_clk_enable_262, 
        n24525, n25715, n27940, n15_adj_3218, n14, mic_clk_N_2454, 
        mic_warm_N_1898, n5634, n5635, n5636, n24524, n24523, mic_bc_valid_N_2618, 
        n5658, n5659, n5660, n5661, n5662, n5663, n5664, n5665, 
        n5666, n5667, n5668, n5669, n5670, n5671, n5672, n5673, 
        n5674, n5675, n5676, n5677, n5678, n5679, n5680, n5681, 
        n5682, n5683, n5684, n5685, n5686, n5687, n5688, n5689, 
        n5691, n5692, n5693, n5694, n5695, n5696, n5697, n5698, 
        n5699, n5700, n5701, n5702, n5703, n5704, n5705, n5706, 
        n5707, n5708, n5709, n5710, n5711, n5712, n5713, n5714, 
        n5715, n5716, n5717, n5718, n5719, n5720, n5721, n5722, 
        pll_clk_enable_178, pll_clk_enable_180, n25721, n7_adj_3219, 
        n25421, n27271, pll_clk_enable_264, n27943, n50, n49, n48, 
        n47, n46, n45_adj_3220, n44_adj_3221, n43_adj_3222, n42_adj_3223, 
        n25790, n12_adj_3224, n10_adj_3225, n8_adj_3226, n6_adj_3227, 
        n4_adj_3228, n25517, n12_adj_3229, n10_adj_3230, n8_adj_3231, 
        n6_adj_3232, n4_adj_3233, n26954, n21039, n6_adj_3234, n28024, 
        n45_adj_3235, n44_adj_3236, n43_adj_3237, n42_adj_3238, n41_adj_3239, 
        n40_adj_3240, n39_adj_3241, n38_adj_3242, n16096, n24522, 
        n24521, n25788, n24520, n21153, pll_clk_enable_273, n24519, 
        n85, n84, n83, n82, n81, n80, n79, n78, n77, n24518, 
        n27268, n24517, n27265, n13582, n4_adj_3243, n26950, n16919, 
        n14_adj_3244, n4_adj_3245, n3, n25414, pll_clk_enable_280, 
        n27962, n27960, n27262, n27259, pll_clk_enable_603, n24516, 
        n24515, n4_adj_3246, n12_adj_3247, n12_adj_3248, n26946, n27256, 
        n27253, n25519, n27959, n5_adj_3249, n25413, n24514, pll_clk_enable_31, 
        n27250, pll_clk_enable_111, pll_clk_enable_531, pll_clk_enable_258, 
        n21266, pll_clk_enable_18, n8_adj_3250, n12_adj_3251, pll_clk_enable_311, 
        n28592, pll_clk_enable_102, pll_clk_enable_11, n23792, n21127, 
        n21246, n21230, n20546, n6_adj_3252, n8_adj_3253, n14_adj_3254, 
        n25672, n27958, n20521, n20519, n20539, n1, n8_adj_3255, 
        n25784, n7_adj_3256, n21228, mic_cfg_settle_3__N_2126, mic_acc_busy_N_2279, 
        n27235, n9_adj_3257, n25949, n27982, n8_adj_3258, n7_adj_3259, 
        n24835, n27232, n25778, n25777, n14_adj_3260, n27229, n25149, 
        n76, n75, n74, n73, n72, n71, n70, n21096, n12689, 
        n24822, n25502, n17603;
    wire [3:0]mic_cfg_settle_3__N_2122;
    
    wire n26518, n78_adj_3261, n17622, n27226, n27223, n27956, n27954, 
        n25766, n25765, n25468, n13578, n25482, n25636, n27220, 
        n28187, n10_adj_3262, n24404;
    wire [1:0]mic_run_state_1__N_2390;
    
    wire n10_adj_3263, n25761, n10_adj_3264, n13154, n17650, n11835, 
        n13599, n25757, n22119, n24500, n13586, n24499, n25754, 
        n22120, n22122, n27211, n27953, n28601, n28600, n25313, 
        n24498, n28599, n28598, n23794, n28597, n28596, n28595, 
        n20_adj_3265, n28593, n27208, mic_acc_gate_6__N_2263, mic_gate_active_N_2284, 
        mic_acc_busy_N_2282, n6812, n6813, n6814, n6815, n6816, 
        n6817, n6818, n6819, n6820, n6821, n6822, n6823, n6824, 
        n6825, n6826, n6827, n24623, pll_clk_enable_259, n27205, 
        n11907, n24399, n24398, n11899, n11905, n24397, n11897, 
        n24396, n11895, mic_acc_last_N_2672, mic_acc_last_N_2664, n27202, 
        n27199, n24391, spi1_sck_c_enable_145, n24497, n16938, n24492, 
        mic_acc_last_N_2663, n17617, n17619, n17621, n17623, n17625, 
        n17627, n17629, n17631, n11903, n24390, n11901, mic_gate_active_N_2286, 
        n24491, n25742, n11_adj_3266, n10_adj_3267, spi1_sck_c_enable_162, 
        n24490, n24489;
    wire [2:0]mic_acc_state_2__N_2327;
    
    wire mic_acc_busy_N_2273, mic_acc_last_N_2659, n6973, n6974;
    wire [2:0]mic_acc_word_2__N_2312;
    
    wire n27196;
    wire [15:0]mic_ram_wr_data_15__N_2296;
    wire [6:0]mic_acc_last_N_2665;
    
    wire n10733, n27430, n24316, n27190, n27946;
    wire [6:0]mic_done_N_2647;
    
    wire mic_done_N_2646, n24672, n16926, n27970, n25538, n2, n16922, 
        n24389, n28057, n24384, n24383, n24382, n24380, n24379, 
        n24378, n24377, n24376, n24375, n24374, n24373, n24372, 
        mic_acc_busy_N_2272, n24371, mic_ram_we_N_2673, n27948, n27427, 
        n16904, n4_adj_3268;
    wire [2:0]mic_acc_state_2__N_744;
    
    wire n16929, n16895, mic_done_N_2640, n24329, n27985, n35_adj_3269, 
        n16894, n17_adj_3270, n24484, spi1_sck_c_enable_72, n24483, 
        n27947, n28392, n28391, n27945, n4_adj_3271, n16732, n24482, 
        n16973, spi1_sck_c_enable_146, n27988, n13981, spi1_sck_c_enable_67, 
        n27991, n4_adj_3272, n28025, n8_adj_3273, n25736, n27944, 
        n64;
    wire [7:0]rd_data_7__N_2897;
    
    wire pll_clk_enable_363, n28134, n6_adj_3274, n11_adj_3275, n27942, 
        n22369, n24481, n24315, n27157, n22373, n27154;
    wire [14:0]reset_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[16:27])
    
    wire n24370, n24369, n24367, n24366, n24365, n24364, n24360, 
        n24359, n24358, n24357, n24355, n24480, n24354, n22386, 
        n25471, n24353, n27589, n27941, n16854, n15270, n16841, 
        n28133, n27588, n28132, n27424, n28131, pll_clk_enable_99, 
        n24479, spi1_sck_c_enable_84, n24478, n28006, n27587, n28130, 
        n24352, n27823, n25945, n27822, n25682, n24350, n24477, 
        n24476, n28129, n24475, n24349, n24348, n24347, n24343, 
        n24342, n24341, n24338, n24337, n24336, n24335, n24334, 
        n24333, n24331, n24322, n24321, n27939, n28128, n16, n24474, 
        n27938, n24320, n24330, n24332, n24314, n27127, n25678, 
        n17605, n17607, n17609, n17611, n17613, n17615, n25560, 
        n6_adj_3276, n27586, n16966, n25409, n24473, n37_adj_3277, 
        n17_adj_3278, n23796, n27124, n25730, n28009, n27585, n22401, 
        n22402, n25411, n43_adj_3279, n16932, spi1_sck_c_enable_36, 
        n25666, n28015, n27584, n40_adj_3280, n25544, n4_adj_3281, 
        n24753, n27583, n24472, n24471, n27109, n24470, n24469, 
        n21218, n24468, n28590, n21086, n21213, n27106, n27930, 
        n24467, n24466, n24465, n25727, n27929, n20537, n27103, 
        n16855, n27100, pll_clk_enable_281, n27097, n25624, n20658, 
        n20565, n16758, n16_adj_3282, n15_adj_3283, n8_adj_3284, n27094, 
        n6_adj_3285, n27091, n26601, n27418, n27088, spi1_sck_N_155_enable_9, 
        n16190, n27079, n27076, n27073, spi1_sck_c_enable_71, pll_clk_enable_391, 
        n24464, pll_clk_enable_602, pll_clk_enable_454, pll_clk_enable_516, 
        n16750, n16178, n57, n16176, pll_clk_enable_597, n59, n10805, 
        n25602, n27070, n28033, n27574, n10819, n28003, n16170, 
        n27061, n16168, n104, n13_adj_3286, n27058, n27055, n58, 
        spi1_sck_c_enable_70, n24463, n16154, n27052, n27928, n27049, 
        n27046, n13403, n10912, n27765, n63, n27764, n13399, n27763, 
        n27762, n23987, n81_adj_3287, n94, n10943, n24462, n25584, 
        n6_adj_3288, pll_clk_enable_538, n16144, n27435;
    
    VHI i2 (.Z(VCC_net));
    FD1S3AY cs_meta_1203 (.D(fpga_cs_n_c), .CK(pll_clk), .Q(cs_meta)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(390[12] 394[8])
    defparam cs_meta_1203.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_76), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_76), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i1.GSR = "DISABLED";
    FD1S3AX mem_3303 (.D(feed_we), .CK(pll_clk), .Q(n13997));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam mem_3303.GSR = "DISABLED";
    LUT4 i13_4_lut_3_lut (.A(mic_acc_state[1]), .B(mic_acc_state[0]), .C(mic_acc_state[2]), 
         .Z(n12689)) /* synthesis lut_function=(!(A ((C)+!B)+!A (B+!(C)))) */ ;
    defparam i13_4_lut_3_lut.init = 16'h1818;
    FD1P3AX spi_frame_sequence_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_10), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i15.GSR = "ENABLED";
    LUT4 i15131_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_8)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15131_2_lut_3_lut_4_lut.init = 16'h0010;
    LUT4 i2_3_lut_4_lut (.A(mic_acc_state[1]), .B(mic_acc_state[0]), .C(mic_acc_state[2]), 
         .D(mic_done_N_2646), .Z(n11835)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i2_3_lut_4_lut.init = 16'h0010;
    CCU2D time_divider_3048_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(time_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24433), .S1(n35));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(637[25:44])
    defparam time_divider_3048_add_4_1.INIT0 = 16'hF000;
    defparam time_divider_3048_add_4_1.INIT1 = 16'h0555;
    defparam time_divider_3048_add_4_1.INJECT1_0 = "NO";
    defparam time_divider_3048_add_4_1.INJECT1_1 = "NO";
    FD1P3AX spi_frame_sequence_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_10), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_10), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i13.GSR = "ENABLED";
    LUT4 i10025_4_lut (.A(ev_rd_data[28]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[28])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10025_4_lut.init = 16'h1222;
    FD1P3AX spi_frame_sequence_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_10), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_10), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i11.GSR = "ENABLED";
    LUT4 i10024_4_lut (.A(ev_rd_data[29]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[29])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10024_4_lut.init = 16'h1222;
    LUT4 n20565_bdd_4_lut_16795 (.A(spi_command[3]), .B(spi_command[1]), 
         .C(n18), .D(spi_command[2]), .Z(n28592)) /* synthesis lut_function=((((D)+!C)+!B)+!A) */ ;
    defparam n20565_bdd_4_lut_16795.init = 16'hff7f;
    FD1P3AX spi_frame_sequence_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_10), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_10), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_10), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i7.GSR = "ENABLED";
    LUT4 i14973_2_lut (.A(status_bit_index[5]), .B(n27127), .Z(n25791)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(435[25] 438[29])
    defparam i14973_2_lut.init = 16'h4444;
    FD1P3AX spi_frame_sequence_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_25), 
            .CK(spi1_sck_c), .Q(spi_extension_length[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_25), 
            .CK(spi1_sck_c), .Q(spi_extension_length[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_25), 
            .CK(spi1_sck_c), .Q(spi_extension_length[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_25), 
            .CK(spi1_sck_c), .Q(spi_extension_length[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_25), 
            .CK(spi1_sck_c), .Q(spi_extension_length[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_25), 
            .CK(spi1_sck_c), .Q(spi_extension_length[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_25), 
            .CK(spi1_sck_c), .Q(spi_extension_length[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_25), 
            .CK(spi1_sck_c), .Q(spi_extension_length[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i8.GSR = "ENABLED";
    LUT4 n20565_bdd_4_lut (.A(n20565), .B(spi_byte_count[1]), .C(spi_bit_count_2__N_1415), 
         .D(n28593), .Z(n25482)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;
    defparam n20565_bdd_4_lut.init = 16'h4000;
    FD1S3AY cs_sync_1204 (.D(cs_meta), .CK(pll_clk), .Q(cs_sync)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(390[12] 394[8])
    defparam cs_sync_1204.GSR = "DISABLED";
    LUT4 i4037_3_lut (.A(invalid_frame_spi_N_2546), .B(n21266), .C(n20546), 
         .Z(n14797)) /* synthesis lut_function=(A ((C)+!B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[30] 570[24])
    defparam i4037_3_lut.init = 16'hb3b3;
    FD1S3AY cs_sync_d_1205 (.D(cs_sync), .CK(pll_clk), .Q(cs_sync_d)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(390[12] 394[8])
    defparam cs_sync_d_1205.GSR = "DISABLED";
    FD1S3AX status_miso_q_1206 (.D(status_next_bit), .CK(spi1_sck_N_155), 
            .Q(status_miso_q)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(422[24:57])
    defparam status_miso_q_1206.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .Q(spi_extension_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .Q(spi_extension_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i6.GSR = "ENABLED";
    LUT4 ev_run_hold_s5_83__I_0_i29_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_28), .D(ev_run_hold_s5[28]), .Z(us_tx_next[28])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i29_3_lut_4_lut.init = 16'hbf40;
    LUT4 mic_cfg_count_0__bdd_3_lut_16797 (.A(mic_cfg_count[0]), .B(mic_cfg_count[2]), 
         .C(status_bit_index[1]), .Z(n28595)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam mic_cfg_count_0__bdd_3_lut_16797.init = 16'hacac;
    LUT4 i15054_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_5)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15054_2_lut_3_lut_4_lut.init = 16'h0200;
    PFUMX mic_acc_last_I_96 (.BLUT(mic_acc_last_N_2664), .ALUT(mic_acc_last_N_2672), 
          .C0(mic_gate_active_N_2284), .Z(mic_acc_last_N_2663));
    FD1P3AX spi_extension_length_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .Q(spi_extension_length[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .Q(spi_extension_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i4.GSR = "ENABLED";
    LUT4 i812_2_lut (.A(mic_data_0_c), .B(mic_lo_i_r), .Z(n4489)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam i812_2_lut.init = 16'h6666;
    FD1P3AX spi_extension_length_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .Q(spi_extension_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i3.GSR = "ENABLED";
    LUT4 i1_2_lut (.A(stop_toggle_spi), .B(n20546), .Z(stop_toggle_spi_N_2519)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut.init = 16'h9999;
    FD1P3AX spi_extension_length_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .Q(spi_extension_length[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i2.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i1 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(spi_rx_shift[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_rx_shift_i1.GSR = "ENABLED";
    LUT4 n27232_bdd_4_lut (.A(n27232), .B(\mic_win_q[0] [0]), .C(\mic_win_i[0] [0]), 
         .D(mic_acc_word[0]), .Z(n27235)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27232_bdd_4_lut.init = 16'haad8;
    FD1P3AX spi_extension_length_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .Q(spi_extension_length[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i1.GSR = "ENABLED";
    FD1S3IX swap_pending_1251 (.D(swap_pending_N_2707), .CK(pll_clk), .CD(n10668), 
            .Q(swap_pending)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam swap_pending_1251.GSR = "DISABLED";
    FD1P3AX mic_sample_count_3055__i0 (.D(n25), .SP(pll_clk_enable_82), 
            .CK(pll_clk), .Q(mic_sample_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[37:60])
    defparam mic_sample_count_3055__i0.GSR = "DISABLED";
    FD1P3IX running_1252 (.D(n28627), .SP(active_bank_N_894), .CD(n10668), 
            .CK(pll_clk), .Q(running)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam running_1252.GSR = "DISABLED";
    LUT4 n20565_bdd_4_lut_16796 (.A(spi_command[0]), .B(spi_byte_count[2]), 
         .C(n21213), .D(n28592), .Z(n28593)) /* synthesis lut_function=(!(A ((C)+!B)+!A ((C+!(D))+!B))) */ ;
    defparam n20565_bdd_4_lut_16796.init = 16'h0c08;
    LUT4 mic_cfg_count_0__bdd_3_lut (.A(mic_cfg_count[4]), .B(status_bit_index[1]), 
         .C(mic_cfg_count[6]), .Z(n28596)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam mic_cfg_count_0__bdd_3_lut.init = 16'hb8b8;
    FD1P3IX mod_active_1286 (.D(mod_level_spi), .SP(pll_clk_enable_2), .CD(n10668), 
            .CK(pll_clk), .Q(mod_active)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mod_active_1286.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i28_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_27), .D(ev_run_hold_s5[27]), .Z(us_tx_next[27])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i28_3_lut_4_lut.init = 16'hbf40;
    LUT4 i14754_then_3_lut (.A(pcm_read_count_latched_spi[0]), .B(pcm_read_count_latched_spi[1]), 
         .C(status_bit_index[0]), .Z(n28016)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam i14754_then_3_lut.init = 16'hacac;
    LUT4 i11_4_lut (.A(mic_ultrasonic_N_1894), .B(mic_boot_done), .C(mic_warm_half_7__N_1887), 
         .D(mic_warm), .Z(n20290)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(208[17:25])
    defparam i11_4_lut.init = 16'hcac0;
    LUT4 i14754_else_3_lut (.A(pcm_read_count_latched_spi[3]), .B(pcm_read_count_latched_spi[2]), 
         .C(status_bit_index[0]), .Z(n28015)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam i14754_else_3_lut.init = 16'hcaca;
    CCU2D add_2514_9 (.A0(\mic_acc_q[1] [6]), .B0(n20537), .C0(n4518), 
          .D0(GND_net), .A1(\mic_acc_q[1] [7]), .B1(n20537), .C1(n4518), 
          .D1(GND_net), .CIN(n24431), .S0(n5700), .S1(n5699));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam add_2514_9.INIT0 = 16'h7848;
    defparam add_2514_9.INIT1 = 16'h7848;
    defparam add_2514_9.INJECT1_0 = "NO";
    defparam add_2514_9.INJECT1_1 = "NO";
    LUT4 i1_3_lut (.A(spi_extension_length[0]), .B(spi_extension_length[4]), 
         .C(spi_extension_length[3]), .Z(n25510)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_3_lut.init = 16'hfefe;
    FD1S3AX spi_bit_count_3045__i0 (.D(n20_adj_3167), .CK(spi1_sck_c), .Q(spi_bit_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam spi_bit_count_3045__i0.GSR = "ENABLED";
    OB us_tx_pad_70 (.I(us_tx_c_70), .O(us_tx[70]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    FD1P3AX spi_update_flags__i2 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(expected_next_8__N_1396[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_update_flags__i2.GSR = "ENABLED";
    LUT4 i13616_2_lut (.A(mic_ram_rd_data[0]), .B(mic_acc_win[0]), .Z(mic_ram_wr_data_15__N_2296[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13616_2_lut.init = 16'h6666;
    FD1S3AX feed_wr_spi_3046__i0 (.D(n45), .CK(spi1_sck_c), .Q(feed_wr_spi[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046__i0.GSR = "DISABLED";
    LUT4 i2_4_lut (.A(spi_extension_length[2]), .B(spi_command[0]), .C(spi_extension_length[1]), 
         .D(n16736), .Z(n16085)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;
    defparam i2_4_lut.init = 16'hffdf;
    LUT4 mic_cfg_count_1__bdd_3_lut (.A(mic_cfg_count[1]), .B(mic_cfg_count[3]), 
         .C(status_bit_index[1]), .Z(n28598)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam mic_cfg_count_1__bdd_3_lut.init = 16'hacac;
    OB us_tx_pad_71 (.I(us_tx_c_71), .O(us_tx[71]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    FD1P3AX spi_version_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_version[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_version_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_command[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_command_i0_i7.GSR = "ENABLED";
    OB us_tx_pad_72 (.I(us_tx_c_72), .O(us_tx[72]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_73 (.I(us_tx_c_73), .O(us_tx[73]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_74 (.I(us_tx_c_74), .O(us_tx[74]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_75 (.I(us_tx_c_75), .O(us_tx[75]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_76 (.I(us_tx_c_76), .O(us_tx[76]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    FD1P3AX spi_update_flags__i1 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_rgb_start[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_update_flags__i1.GSR = "ENABLED";
    OB us_tx_pad_69 (.I(us_tx_c_69), .O(us_tx[69]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    LUT4 ev_run_hold_s5_83__I_0_i51_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_50), .D(ev_run_hold_s5[50]), .Z(us_tx_next[50])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i51_3_lut_4_lut.init = 16'hbf40;
    CCU2D add_2514_7 (.A0(\mic_acc_q[1] [4]), .B0(n20537), .C0(n4518), 
          .D0(GND_net), .A1(\mic_acc_q[1] [5]), .B1(n20537), .C1(n4518), 
          .D1(GND_net), .CIN(n24430), .COUT(n24431), .S0(n5702), .S1(n5701));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam add_2514_7.INIT0 = 16'h7848;
    defparam add_2514_7.INIT1 = 16'h7848;
    defparam add_2514_7.INJECT1_0 = "NO";
    defparam add_2514_7.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_132 (.A(spi_extension_length[6]), .B(spi_extension_length[7]), 
         .Z(n4_adj_3246)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i1_2_lut_adj_132.init = 16'heeee;
    LUT4 ev_run_hold_s5_83__I_0_i52_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_51), .D(ev_run_hold_s5[51]), .Z(us_tx_next[51])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i52_3_lut_4_lut.init = 16'hbf40;
    LUT4 i14971_4_lut (.A(n53), .B(status_bit_index[3]), .C(n28601), .D(status_bit_index[5]), 
         .Z(n25790)) /* synthesis lut_function=(A (B (C+!(D)))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(435[25] 438[29])
    defparam i14971_4_lut.init = 16'hc088;
    FD1P3AX spi_command_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_command[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_command_i0_i6.GSR = "ENABLED";
    LUT4 ev_run_hold_s5_83__I_0_i27_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_26), .D(ev_run_hold_s5[26]), .Z(us_tx_next[26])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i27_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX spi_extension_length_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .Q(spi_extension_length[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_extension_length_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_frame_sequence_i0_i0.GSR = "ENABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_76), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i0.GSR = "DISABLED";
    FD1S3AX global_phase_s2_3047__i0 (.D(n45_adj_3160), .CK(pll_clk), .Q(global_phase_s2[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047__i0.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(rgb_values[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i0.GSR = "DISABLED";
    FD1P3IX frame_req_1278 (.D(n28627), .SP(pll_clk_enable_3), .CD(n16096), 
            .CK(pll_clk), .Q(frame_req)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam frame_req_1278.GSR = "DISABLED";
    FD1S3IX mod_settle__i0 (.D(n17388), .CK(pll_clk), .CD(n15455), .Q(mod_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mod_settle__i0.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i0 (.D(pcm_rd_ptr_spi[0]), .CK(pll_clk), .Q(pcm_rd_sync1[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync1_i0.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_133 (.A(status_bit_index[7]), .B(status_bit_index[6]), 
         .Z(n38_adj_3216)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam i1_2_lut_adj_133.init = 16'heeee;
    FD1S3AX pcm_rd_sync2_i0 (.D(pcm_rd_sync1[0]), .CK(pll_clk), .Q(pcm_rd_sync2[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync2_i0.GSR = "DISABLED";
    LUT4 i1_4_lut (.A(mic_phase[1]), .B(n119), .C(n127), .D(mic_phase[2]), 
         .Z(pll_clk_enable_82)) /* synthesis lut_function=(A (B)+!A (B (C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_4_lut.init = 16'hc8cc;
    LUT4 i1_3_lut_adj_134 (.A(status_bit_index[8]), .B(n43_adj_3193), .C(status_bit_index[7]), 
         .Z(n21246)) /* synthesis lut_function=(A+(B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam i1_3_lut_adj_134.init = 16'heaea;
    FD1S3AX pcm_wr_sync1_spi_i0 (.D(pcm_wr_ptr[0]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync1_spi_i0.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i0 (.D(pcm_wr_sync1_spi[0]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync2_spi_i0.GSR = "DISABLED";
    PFUMX i61 (.BLUT(n25788), .ALUT(n37_adj_3277), .C0(status_bit_index[7]), 
          .Z(n40_adj_3280));
    FD1P3AX pcm_rd_ptr_spi_i0 (.D(pcm_rd_ptr_spi_5__N_1610[0]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_rd_ptr_spi_i0.GSR = "DISABLED";
    FD1S3AX phase_frac_i0 (.D(phase_frac_sum[0]), .CK(pll_clk), .Q(phase_frac[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i0.GSR = "DISABLED";
    FD1P3AX spi_command_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_command[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_command_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_version[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_version_i0_i7.GSR = "ENABLED";
    LUT4 LessThan_241_i10_3_lut (.A(n8_adj_3284), .B(pcm_read_count_latched_spi[4]), 
         .C(pcm_payload_bit[8]), .Z(n982)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(432[10:59])
    defparam LessThan_241_i10_3_lut.init = 16'h8e8e;
    FD1P3IX mic_phase__i0 (.D(n92), .SP(pll_clk_enable_124), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_phase[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_phase__i0.GSR = "DISABLED";
    FD1P3AX spi_version_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_version[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_version_i0_i6.GSR = "ENABLED";
    LUT4 i15145_2_lut (.A(spi_byte_count[0]), .B(n25482), .Z(spi1_sck_c_enable_39)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i15145_2_lut.init = 16'h4444;
    FD1P3AX spi_version_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_version[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_version_i0_i5.GSR = "ENABLED";
    LUT4 LessThan_241_i8_3_lut (.A(n6_adj_3150), .B(pcm_read_count_latched_spi[3]), 
         .C(pcm_payload_bit[7]), .Z(n8_adj_3284)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(432[10:59])
    defparam LessThan_241_i8_3_lut.init = 16'h8e8e;
    FD1P3AX spi_version_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_version[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_version_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_version[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_version_i0_i3.GSR = "ENABLED";
    FD1S3IX link_idle__i0 (.D(n11_adj_3152), .CK(pll_clk), .CD(cs_fall), 
            .Q(link_idle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam link_idle__i0.GSR = "DISABLED";
    FD1P3AX spi_version_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_version[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_version_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_version[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_version_i0_i1.GSR = "ENABLED";
    FD1S3AX feed_wr_spi_3046__i7 (.D(n38), .CK(spi1_sck_c), .Q(feed_wr_spi[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046__i7.GSR = "DISABLED";
    CCU2D add_2514_5 (.A0(\mic_acc_q[1] [2]), .B0(n20537), .C0(n4518), 
          .D0(GND_net), .A1(\mic_acc_q[1] [3]), .B1(n20537), .C1(n4518), 
          .D1(GND_net), .CIN(n24429), .COUT(n24430), .S0(n5704), .S1(n5703));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam add_2514_5.INIT0 = 16'h7848;
    defparam add_2514_5.INIT1 = 16'h7848;
    defparam add_2514_5.INJECT1_0 = "NO";
    defparam add_2514_5.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i26_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_25), .D(ev_run_hold_s5[25]), .Z(us_tx_next[25])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i26_3_lut_4_lut.init = 16'hbf40;
    CCU2D feed_wr_spi_3046_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(n16854), .B1(spi_bit_count_2__N_1415), .C1(feed_wr_spi[0]), 
          .D1(GND_net), .COUT(n24377), .S1(n45));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046_add_4_1.INIT0 = 16'hF000;
    defparam feed_wr_spi_3046_add_4_1.INIT1 = 16'h7878;
    defparam feed_wr_spi_3046_add_4_1.INJECT1_0 = "NO";
    defparam feed_wr_spi_3046_add_4_1.INJECT1_1 = "NO";
    FD1P3AX mic_cfg_start_i0_i0 (.D(mic_cfg_hold_spi[8]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i0.GSR = "DISABLED";
    LUT4 mic_cfg_count_1__bdd_2_lut (.A(mic_cfg_count[5]), .B(status_bit_index[1]), 
         .Z(n28599)) /* synthesis lut_function=(A (B)) */ ;
    defparam mic_cfg_count_1__bdd_2_lut.init = 16'h8888;
    FD1S3AX feed_wr_spi_3046__i6 (.D(n39), .CK(spi1_sck_c), .Q(feed_wr_spi[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046__i6.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i53_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_52), .D(ev_run_hold_s5[52]), .Z(us_tx_next[52])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i53_3_lut_4_lut.init = 16'hbf40;
    LUT4 i6949_2_lut (.A(spi1_sck_c_enable_164), .B(expected_next_8__N_1396[3]), 
         .Z(n17720)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i6949_2_lut.init = 16'h2222;
    CCU2D add_2514_3 (.A0(\mic_acc_q[1] [0]), .B0(n20537), .C0(n4518), 
          .D0(GND_net), .A1(\mic_acc_q[1] [1]), .B1(n20537), .C1(n4518), 
          .D1(GND_net), .CIN(n24428), .COUT(n24429), .S0(n5706), .S1(n5705));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam add_2514_3.INIT0 = 16'h7484;
    defparam add_2514_3.INIT1 = 16'h7848;
    defparam add_2514_3.INJECT1_0 = "NO";
    defparam add_2514_3.INJECT1_1 = "NO";
    FD1S3AX feed_wr_spi_3046__i5 (.D(n40), .CK(spi1_sck_c), .Q(feed_wr_spi[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046__i5.GSR = "DISABLED";
    L6MUX21 i16548 (.D0(n28181), .D1(n27960), .SD(status_bit_index[2]), 
            .Z(n28182));
    CCU2D add_2514_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20537), .B1(n4518), .C1(GND_net), .D1(GND_net), .COUT(n24428));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam add_2514_1.INIT0 = 16'hF000;
    defparam add_2514_1.INIT1 = 16'h7777;
    defparam add_2514_1.INJECT1_0 = "NO";
    defparam add_2514_1.INJECT1_1 = "NO";
    LUT4 LessThan_241_i6_3_lut (.A(n4_adj_3154), .B(pcm_read_count_latched_spi[2]), 
         .C(pcm_payload_bit[6]), .Z(n6_adj_3150)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(432[10:59])
    defparam LessThan_241_i6_3_lut.init = 16'h8e8e;
    LUT4 i13539_2_lut_3_lut (.A(n22119), .B(n16855), .C(spi_rgb_captured[0]), 
         .Z(n15_adj_3218)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;
    defparam i13539_2_lut_3_lut.init = 16'h7878;
    LUT4 ev_run_hold_s5_83__I_0_i54_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_53), .D(ev_run_hold_s5[53]), .Z(us_tx_next[53])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i54_3_lut_4_lut.init = 16'hbf40;
    LUT4 LessThan_241_i4_4_lut (.A(pcm_payload_bit[4]), .B(pcm_read_count_latched_spi[1]), 
         .C(pcm_payload_bit[5]), .D(pcm_read_count_latched_spi[0]), .Z(n4_adj_3154)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B ((D)+!C)+!B !(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(432[10:59])
    defparam LessThan_241_i4_4_lut.init = 16'h4d0c;
    PFUMX i14775 (.BLUT(n25715), .ALUT(n25716), .C0(status_next_index[6]), 
          .Z(status_next_bit));
    CCU2D add_842_7 (.A0(mic_window_count[5]), .B0(n13362), .C0(GND_net), 
          .D0(GND_net), .A1(mic_window_count[6]), .B1(n13362), .C1(GND_net), 
          .D1(GND_net), .CIN(n24423), .S0(n10738), .S1(n10739));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(871[41:64])
    defparam add_842_7.INIT0 = 16'h7888;
    defparam add_842_7.INIT1 = 16'h7888;
    defparam add_842_7.INJECT1_0 = "NO";
    defparam add_842_7.INJECT1_1 = "NO";
    FD1P3AX spi_command_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_command[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_command_i0_i4.GSR = "ENABLED";
    FD1S3AX feed_wr_spi_3046__i4 (.D(n41), .CK(spi1_sck_c), .Q(feed_wr_spi[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046__i4.GSR = "DISABLED";
    LUT4 i9791_4_lut (.A(compact_end), .B(spi_mod_feed_N_2470), .C(n25727), 
         .D(n21183), .Z(frame_end)) /* synthesis lut_function=(A (B (C+!(D)))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(460[29] 461[90])
    defparam i9791_4_lut.init = 16'hc088;
    FD1P3AX accepted_sequence_spi_i0_i0 (.D(spi_frame_sequence[0]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i0.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i25_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_24), .D(ev_run_hold_s5[24]), .Z(us_tx_next[24])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i25_3_lut_4_lut.init = 16'hbf40;
    FD1S3AX feed_wr_spi_3046__i3 (.D(n42), .CK(spi1_sck_c), .Q(feed_wr_spi[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046__i3.GSR = "DISABLED";
    CCU2D mic_block_count_3057_add_4_17 (.A0(mic_block_count[15]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24376), .S0(n70));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057_add_4_17.INIT0 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_17.INIT1 = 16'h0000;
    defparam mic_block_count_3057_add_4_17.INJECT1_0 = "NO";
    defparam mic_block_count_3057_add_4_17.INJECT1_1 = "NO";
    CCU2D add_842_5 (.A0(mic_window_count[3]), .B0(n13362), .C0(GND_net), 
          .D0(GND_net), .A1(mic_window_count[4]), .B1(n13362), .C1(GND_net), 
          .D1(GND_net), .CIN(n24422), .COUT(n24423), .S0(n10736), .S1(n10737));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(871[41:64])
    defparam add_842_5.INIT0 = 16'h7888;
    defparam add_842_5.INIT1 = 16'h7888;
    defparam add_842_5.INJECT1_0 = "NO";
    defparam add_842_5.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i55_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_54), .D(ev_run_hold_s5[54]), .Z(us_tx_next[54])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i55_3_lut_4_lut.init = 16'hbf40;
    FD1S3AX feed_wr_spi_3046__i2 (.D(n43), .CK(spi1_sck_c), .Q(feed_wr_spi[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046__i2.GSR = "DISABLED";
    OB us_tx_pad_77 (.I(us_tx_c_77), .O(us_tx[77]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_78 (.I(us_tx_c_78), .O(us_tx[78]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    FD1S3AX time_divider_3048__i0 (.D(n35), .CK(pll_clk), .Q(time_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(637[25:44])
    defparam time_divider_3048__i0.GSR = "DISABLED";
    CCU2D mic_block_count_3057_add_4_15 (.A0(mic_block_count[13]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[14]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24375), .COUT(n24376), .S0(n72), 
          .S1(n71));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057_add_4_15.INIT0 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_15.INIT1 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_15.INJECT1_0 = "NO";
    defparam mic_block_count_3057_add_4_15.INJECT1_1 = "NO";
    LUT4 i10340_2_lut (.A(time_divider[2]), .B(time_divider[5]), .Z(n21086)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10340_2_lut.init = 16'h8888;
    LUT4 i3_4_lut (.A(time_divider[0]), .B(time_divider[3]), .C(time_divider[1]), 
         .D(time_divider[4]), .Z(n24623)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_4_lut.init = 16'h8000;
    PFUMX mux_2584_i2 (.BLUT(n25742), .ALUT(mic_acc_state_2__N_2327[1]), 
          .C0(n11_adj_3156), .Z(mic_acc_state_2__N_744[1]));
    OB us_tx_pad_79 (.I(us_tx_c_79), .O(us_tx[79]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    CCU2D mic_block_count_3057_add_4_13 (.A0(mic_block_count[11]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[12]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24374), .COUT(n24375), .S0(n74), 
          .S1(n73));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057_add_4_13.INIT0 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_13.INIT1 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_13.INJECT1_0 = "NO";
    defparam mic_block_count_3057_add_4_13.INJECT1_1 = "NO";
    FD1S3AX feed_wr_spi_3046__i1 (.D(n44), .CK(spi1_sck_c), .Q(feed_wr_spi[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046__i1.GSR = "DISABLED";
    FD1P3AX accepted_update_flags_spi_i0_i0 (.D(spi_rgb_start[7]), .SP(spi1_sck_c_enable_164), 
            .CK(spi1_sck_c), .Q(accepted_update_flags_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_update_flags_spi_i0_i0.GSR = "DISABLED";
    LUT4 i14966_4_lut (.A(n24672), .B(payload_end), .C(n25624), .D(spi_phase_only_N_2776), 
         .Z(n25727)) /* synthesis lut_function=(A (B)+!A (B+!(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(461[29:90])
    defparam i14966_4_lut.init = 16'hcdcc;
    FD1S3IX mic_acc_last_1340 (.D(mic_acc_last_N_2659), .CK(pll_clk), .CD(mic_acc_busy_N_2272), 
            .Q(mic_acc_last)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_last_1340.GSR = "DISABLED";
    LUT4 i15111_2_lut (.A(pll_clk_enable_597), .B(mic_run_state_1__N_2390[1]), 
         .Z(n10805)) /* synthesis lut_function=(!(A+(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(924[9] 945[12])
    defparam i15111_2_lut.init = 16'h1111;
    FD1P3AX status_bit_index_i0_i0 (.D(status_next_index[0]), .SP(spi1_sck_N_155_enable_9), 
            .CK(spi1_sck_N_155), .Q(status_bit_index[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam status_bit_index_i0_i0.GSR = "ENABLED";
    LUT4 i15082_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_1)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15082_2_lut_3_lut_4_lut.init = 16'h0002;
    FD1P3IX mic_window_count__i0 (.D(n10733), .SP(pll_clk_enable_187), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_window_count[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_window_count__i0.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i0 (.D(mic_done_N_2647[0]), .SP(pll_clk_enable_193), 
            .CD(n13586), .CK(pll_clk), .Q(mic_gate_index[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_index__i0.GSR = "DISABLED";
    FD1P3IX rgb_hold__i0 (.D(rgb_values[0]), .SP(pll_clk_enable_216), .CD(n13578), 
            .CK(pll_clk), .Q(rgb_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i0.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i1 (.D(n5722), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i1.GSR = "DISABLED";
    FD1P3IX mic_bc_valid_1308 (.D(mic_bc_valid_N_2618), .SP(pll_clk_enable_11), 
            .CD(mic_run_state_1__N_2390[1]), .CK(pll_clk), .Q(mic_bc_valid)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_bc_valid_1308.GSR = "DISABLED";
    CCU2D mic_block_count_3057_add_4_11 (.A0(mic_block_count[9]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[10]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24373), .COUT(n24374), .S0(n76), 
          .S1(n75));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057_add_4_11.INIT0 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_11.INIT1 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_11.INJECT1_0 = "NO";
    defparam mic_block_count_3057_add_4_11.INJECT1_1 = "NO";
    LUT4 i15096_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_9)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15096_2_lut_3_lut_4_lut.init = 16'h0020;
    LUT4 i6792_1_lut (.A(n10819), .Z(mic_acc_busy_N_2273)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(947[9] 967[12])
    defparam i6792_1_lut.init = 16'h5555;
    LUT4 i1_2_lut_adj_135 (.A(ws2812_enable_N_2596), .B(ws2812_settle_3__N_1697), 
         .Z(pll_clk_enable_178)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_135.init = 16'heeee;
    CCU2D add_13119_7 (.A0(n16168), .B0(n20539), .C0(n16894), .D0(spi_rgb_start[7]), 
          .A1(n12_adj_3247), .B1(n11899), .C1(n11897), .D1(n11895), 
          .CIN(n24557), .S0(n23820), .S1(n23819));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[33] 448[80])
    defparam add_13119_7.INIT0 = 16'ha655;
    defparam add_13119_7.INIT1 = 16'hfe01;
    defparam add_13119_7.INJECT1_0 = "NO";
    defparam add_13119_7.INJECT1_1 = "NO";
    CCU2D add_13119_5 (.A0(n12938), .B0(n11903), .C0(n8_adj_3273), .D0(n11901), 
          .A1(n16170), .B1(n20539), .C1(n16894), .D1(spi_rgb_start[7]), 
          .CIN(n24556), .COUT(n24557), .S0(n23822), .S1(n23821));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[33] 448[80])
    defparam add_13119_5.INIT0 = 16'ha956;
    defparam add_13119_5.INIT1 = 16'haa59;
    defparam add_13119_5.INJECT1_0 = "NO";
    defparam add_13119_5.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i56_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_55), .D(ev_run_hold_s5[55]), .Z(us_tx_next[55])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i56_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX spi_command_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_command[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_command_i0_i3.GSR = "ENABLED";
    LUT4 staging_q_7__I_0_1370_i4_4_lut (.A(staging_q[3]), .B(build_level[3]), 
         .C(build_mod), .D(n15), .Z(event_level[3])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(379[31] 381[40])
    defparam staging_q_7__I_0_1370_i4_4_lut.init = 16'hca0a;
    LUT4 i15138_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_10)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15138_2_lut_3_lut_4_lut.init = 16'h0040;
    CCU2D add_842_3 (.A0(mic_window_count[1]), .B0(n13362), .C0(GND_net), 
          .D0(GND_net), .A1(mic_window_count[2]), .B1(n13362), .C1(GND_net), 
          .D1(GND_net), .CIN(n24421), .COUT(n24422), .S0(n10734), .S1(n10735));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(871[41:64])
    defparam add_842_3.INIT0 = 16'h7888;
    defparam add_842_3.INIT1 = 16'h7888;
    defparam add_842_3.INJECT1_0 = "NO";
    defparam add_842_3.INJECT1_1 = "NO";
    CCU2D add_13119_3 (.A0(n16176), .B0(n20539), .C0(n16894), .D0(spi_rgb_start[7]), 
          .A1(n16894), .B1(n20539), .C1(n11903), .D1(n8_adj_3273), .CIN(n24555), 
          .COUT(n24556), .S0(n23824), .S1(n23823));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[33] 448[80])
    defparam add_13119_3.INIT0 = 16'haa59;
    defparam add_13119_3.INIT1 = 16'hb44b;
    defparam add_13119_3.INJECT1_0 = "NO";
    defparam add_13119_3.INJECT1_1 = "NO";
    FD1S3IX frame_settle__i0 (.D(n17390), .CK(pll_clk), .CD(n15270), .Q(frame_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam frame_settle__i0.GSR = "DISABLED";
    FD1P3AX build_level_eff_i0_i0 (.D(n10943), .SP(pll_clk_enable_102), 
            .CK(pll_clk), .Q(build_level_eff[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_eff_i0_i0.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i0 (.D(n165), .CK(pll_clk), .Q(fpga_time[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i0.GSR = "DISABLED";
    CCU2D add_13119_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n16894), .B1(n20539), .C1(n11907), .D1(n4_adj_3271), .COUT(n24555), 
          .S1(n23825));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[33] 448[80])
    defparam add_13119_1.INIT0 = 16'hF000;
    defparam add_13119_1.INIT1 = 16'h4bb4;
    defparam add_13119_1.INJECT1_0 = "NO";
    defparam add_13119_1.INJECT1_1 = "NO";
    FD1P3IX mic_gate_fill_i0 (.D(mic_acc_last_N_2665[0]), .SP(pll_clk_enable_91), 
            .CD(n17753), .CK(pll_clk), .Q(mic_gate_fill[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_fill_i0.GSR = "DISABLED";
    LUT4 i14961_4_lut (.A(n25777), .B(status_next_index[0]), .C(n25778), 
         .D(status_next_bit_N_2754[2]), .Z(n25766)) /* synthesis lut_function=(A (B (C+!(D)))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam i14961_4_lut.init = 16'hc088;
    CCU2D add_842_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n23987), .B1(n8_adj_3258), .C1(n7_adj_3259), .D1(n13362), 
          .COUT(n24421), .S1(n10733));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(871[41:64])
    defparam add_842_1.INIT0 = 16'hF000;
    defparam add_842_1.INIT1 = 16'h56aa;
    defparam add_842_1.INJECT1_0 = "NO";
    defparam add_842_1.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i24_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_23), .D(ev_run_hold_s5[23]), .Z(us_tx_next[23])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i24_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i42_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_41), .D(ev_run_hold_s5[41]), .Z(us_tx_next[41])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i42_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i57_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_56), .D(ev_run_hold_s5[56]), .Z(us_tx_next[56])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i57_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i41_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_40), .D(ev_run_hold_s5[40]), .Z(us_tx_next[40])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i41_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27220_bdd_4_lut (.A(n27220), .B(n26950), .C(n26946), .D(status_next_bit_N_2754[3]), 
         .Z(n27223)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27220_bdd_4_lut.init = 16'haad8;
    FD1P3AX status_hold__i1 (.D(accepted_sequence[8]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i1.GSR = "DISABLED";
    LUT4 i3_4_lut_adj_136 (.A(n20658), .B(spi_byte_count[4]), .C(n8_adj_3177), 
         .D(n16926), .Z(n24672)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(490[25:31])
    defparam i3_4_lut_adj_136.init = 16'hfffd;
    LUT4 i4570_4_lut (.A(ev_sum[6]), .B(staging_q[6]), .C(staging_rd_page_N_2718), 
         .D(ev_in_clear), .Z(n15332)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam i4570_4_lut.init = 16'hcac0;
    FD1P3IX mic_cfg_gap_i1 (.D(mic_cfg_gap_w[1]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i1.GSR = "DISABLED";
    PFUMX mux_2584_i1 (.BLUT(n25730), .ALUT(mic_acc_state_2__N_2327[0]), 
          .C0(n11_adj_3156), .Z(mic_acc_state_2__N_744[0]));
    CCU2D ev_ch_3051_add_4_7 (.A0(ev_ch[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_ch[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24553), .S0(n35_adj_3186), .S1(n34_adj_3187));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051_add_4_7.INIT0 = 16'hfaaa;
    defparam ev_ch_3051_add_4_7.INIT1 = 16'hfaaa;
    defparam ev_ch_3051_add_4_7.INJECT1_0 = "NO";
    defparam ev_ch_3051_add_4_7.INJECT1_1 = "NO";
    FD1P3AX spi_command_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_command[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_command_i0_i2.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0 (.D(accepted_sequence_15__N_995[0]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i0.GSR = "DISABLED";
    FD1S3AX phase_step_s1_1243 (.D(phase_frac_sum[24]), .CK(pll_clk), .Q(phase_step_s1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_step_s1_1243.GSR = "DISABLED";
    LUT4 i10076_3_lut_4_lut (.A(frame_settle[1]), .B(frame_settle[0]), .C(n7_adj_3256), 
         .D(frame_settle[2]), .Z(n3389)) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B !((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[33:52])
    defparam i10076_3_lut_4_lut.init = 16'he010;
    FD1S3AX phase_step_s2_1244 (.D(phase_step_s1), .CK(pll_clk), .Q(phase_step_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_step_s2_1244.GSR = "DISABLED";
    FD1S3AX phase_step_s3_1247 (.D(phase_step_s2), .CK(pll_clk), .Q(phase_step_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_step_s3_1247.GSR = "DISABLED";
    FD1S3AX wrap_s3_1248 (.D(wrap_s2), .CK(pll_clk), .Q(wrap_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam wrap_s3_1248.GSR = "DISABLED";
    FD1S3AX swap_now_s3_1249 (.D(active_bank_N_894), .CK(pll_clk), .Q(swap_now_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam swap_now_s3_1249.GSR = "DISABLED";
    FD1S3AX active_bank_1250 (.D(active_bank_N_893), .CK(pll_clk), .Q(active_bank)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam active_bank_1250.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i0 (.D(global_phase_s2[0]), .SP(pll_clk_enable_304), 
            .CK(pll_clk), .Q(run_addr_s3[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam run_addr_s3_i0.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i0 (.D(feed_wr_spi[0]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam feed_wr_i0_i0.GSR = "DISABLED";
    FD1S3AX phase_step_s4_1255 (.D(phase_step_s3), .CK(pll_clk), .Q(phase_step_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_step_s4_1255.GSR = "DISABLED";
    FD1S3AX wrap_s4_1256 (.D(wrap_s3), .CK(pll_clk), .Q(wrap_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam wrap_s4_1256.GSR = "DISABLED";
    FD1S3AX wrap_s5_1257 (.D(wrap_s4), .CK(pll_clk), .Q(wrap_s5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam wrap_s5_1257.GSR = "DISABLED";
    FD1S3AX link_tick_d_1261 (.D(fpga_time[10]), .CK(pll_clk), .Q(link_tick_d)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam link_tick_d_1261.GSR = "DISABLED";
    FD1S3IX ws2812_frames_left_i0 (.D(n3334), .CK(pll_clk), .CD(ws2812_enable_N_2578), 
            .Q(ws2812_frames_left[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ws2812_frames_left_i0.GSR = "DISABLED";
    FD1S3AX frame_toggle_meta_1264 (.D(frame_toggle_spi), .CK(pll_clk), 
            .Q(frame_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam frame_toggle_meta_1264.GSR = "DISABLED";
    FD1S3AX frame_toggle_sync_1265 (.D(frame_toggle_meta), .CK(pll_clk), 
            .Q(frame_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam frame_toggle_sync_1265.GSR = "DISABLED";
    FD1S3AX stop_toggle_meta_1266 (.D(stop_toggle_spi), .CK(pll_clk), .Q(stop_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam stop_toggle_meta_1266.GSR = "DISABLED";
    FD1S3AX stop_toggle_sync_1267 (.D(stop_toggle_meta), .CK(pll_clk), .Q(stop_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam stop_toggle_sync_1267.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_meta_1268 (.D(ws2812_toggle_spi), .CK(pll_clk), 
            .Q(ws2812_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ws2812_toggle_meta_1268.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_sync_1269 (.D(ws2812_toggle_meta), .CK(pll_clk), 
            .Q(ws2812_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ws2812_toggle_sync_1269.GSR = "DISABLED";
    FD1S3AX invalid_frame_meta_1270 (.D(invalid_frame_spi), .CK(pll_clk), 
            .Q(invalid_frame_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam invalid_frame_meta_1270.GSR = "DISABLED";
    FD1S3AX invalid_frame_sync_1271 (.D(invalid_frame_meta), .CK(pll_clk), 
            .Q(invalid_frame_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam invalid_frame_sync_1271.GSR = "DISABLED";
    FD1S3AX mod_toggle_meta_1272 (.D(mod_toggle_spi), .CK(pll_clk), .Q(mod_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mod_toggle_meta_1272.GSR = "DISABLED";
    FD1S3AX mod_toggle_sync_1273 (.D(mod_toggle_meta), .CK(pll_clk), .Q(mod_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mod_toggle_sync_1273.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i0 (.D(n25313), .SP(pll_clk_enable_311), .CK(pll_clk), 
            .Q(rmw_slot[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rmw_slot_i0_i0.GSR = "DISABLED";
    FD1P3AX ws2812_enable_1275 (.D(ws2812_enable_N_2578), .SP(pll_clk_enable_18), 
            .CK(pll_clk), .Q(ws2812_enable)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ws2812_enable_1275.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i0 (.D(feed_q[0]), .SP(pll_clk_enable_318), 
            .CD(n13582), .CK(pll_clk), .Q(pending_mod_level[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_mod_level__i0.GSR = "DISABLED";
    LUT4 staging_q_7__I_0_1370_i3_4_lut (.A(staging_q[2]), .B(build_level[2]), 
         .C(build_mod), .D(n15), .Z(event_level[2])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(379[31] 381[40])
    defparam staging_q_7__I_0_1370_i3_4_lut.init = 16'hca0a;
    FD1P3AX pending_sequence_i0_i0 (.D(accepted_sequence_spi[0]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i0.GSR = "DISABLED";
    FD1P3AX ev_fall_1292 (.D(ev_fall_N_2683), .SP(pll_clk_enable_21), .CK(pll_clk), 
            .Q(ev_fall)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_fall_1292.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i0 (.D(mic_data_0_c), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i0.GSR = "DISABLED";
    CCU2D ev_ch_3051_add_4_5 (.A0(ev_ch[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_ch[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24552), .COUT(n24553), .S0(n37), .S1(n36));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051_add_4_5.INIT0 = 16'hfaaa;
    defparam ev_ch_3051_add_4_5.INIT1 = 16'hfaaa;
    defparam ev_ch_3051_add_4_5.INJECT1_0 = "NO";
    defparam ev_ch_3051_add_4_5.INJECT1_1 = "NO";
    FD1P3AX mic_shift_1_l_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i0.GSR = "DISABLED";
    LUT4 i1_4_lut_adj_137 (.A(fpga_cs_n_c), .B(spi_byte_count[8]), .C(n4_adj_3153), 
         .D(n25492), .Z(n25421)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(138[17:34])
    defparam i1_4_lut_adj_137.init = 16'hfffe;
    FD1P3AX mic_shift_0_r__i1 (.D(mic_data_0_c), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i1.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i1 (.D(mic_data_1_c), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i0.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i1 (.D(\mic_acc_i[3] [0]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[3] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i1.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i1 (.D(\mic_acc_q[3] [0]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[3] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i1.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i0 (.D(n17637), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i0.GSR = "DISABLED";
    FD1S3IX mic_cfg_settle_i0 (.D(mic_cfg_settle_3__N_2122[0]), .CK(pll_clk), 
            .CD(mic_cfg_settle_3__N_2126), .Q(mic_cfg_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_settle_i0.GSR = "DISABLED";
    FD1S3AX mic_cfg_toggle_meta_1322 (.D(mic_cfg_toggle_spi), .CK(pll_clk), 
            .Q(mic_cfg_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_toggle_meta_1322.GSR = "DISABLED";
    FD1S3AX mic_cfg_toggle_sync_1323 (.D(mic_cfg_toggle_meta), .CK(pll_clk), 
            .Q(mic_cfg_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_toggle_sync_1323.GSR = "DISABLED";
    FD1P3AX mic_cfg_width_i0 (.D(mic_cfg_width_w[0]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_width[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_width_i0.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i0 (.D(mic_acc_word[0]), .SP(mic_ram_we_N_2673), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_addr_i0_i0.GSR = "DISABLED";
    FD1P3AX mic_done_1331 (.D(mic_done_N_2640), .SP(pll_clk_enable_31), 
            .CK(pll_clk), .Q(mic_done)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_done_1331.GSR = "DISABLED";
    FD1S3AX mic_acc_state_i0 (.D(mic_acc_state_2__N_744[0]), .CK(pll_clk), 
            .Q(mic_acc_state[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_state_i0.GSR = "DISABLED";
    FD1P3AX spi_command_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_command[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_command_i0_i1.GSR = "ENABLED";
    LUT4 i1_4_lut_adj_138 (.A(spi_channel_page), .B(n11_adj_3181), .C(spi_phase_only_N_2776), 
         .D(n12_adj_3180), .Z(n4_adj_3153)) /* synthesis lut_function=(A (B (D))+!A (B (C+(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(138[17:34])
    defparam i1_4_lut_adj_138.init = 16'hdc50;
    FD1S3AX mic_ram_we_1341 (.D(mic_ram_we_N_2673), .CK(pll_clk), .Q(mic_ram_we)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_we_1341.GSR = "DISABLED";
    PFUMX i4573 (.BLUT(n15334), .ALUT(n25806), .C0(ev_fall_N_2683), .Z(n15335));
    FD1P3AX build_level_i0_i0 (.D(pending_mod_level[0]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(build_level[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_i0_i0.GSR = "DISABLED";
    FD1S3AX us_tx_i1 (.D(us_tx_next[0]), .CK(pll_clk), .Q(us_tx_c_0)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i1.GSR = "DISABLED";
    FD1S3AX pll_lock_sync_i0 (.D(pll_lock), .CK(pll_clk), .Q(pll_lock_sync[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(129[12:77])
    defparam pll_lock_sync_i0.GSR = "DISABLED";
    LUT4 n13_bdd_4_lut_16129 (.A(mic_lo_n_next_6__N_642[2]), .B(mic_lo_n_next_6__N_642[4]), 
         .C(mic_lo_n_next_6__N_642[3]), .D(mic_lo_n_next_6__N_642[1]), .Z(n27435)) /* synthesis lut_function=(!(A (B)+!A (B (C+(D))))) */ ;
    defparam n13_bdd_4_lut_16129.init = 16'h3337;
    OB us_tx_pad_80 (.I(us_tx_c_80), .O(us_tx[80]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    FD1P3IX mic_cfg_count_i5 (.D(mic_cfg_hold_spi[5]), .SP(pll_clk_enable_597), 
            .CD(n17672), .CK(pll_clk), .Q(mic_cfg_count[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_count_i5.GSR = "DISABLED";
    FD1P3AY spi_expected_last_i0 (.D(n2), .SP(spi1_sck_c_enable_70), .CK(spi1_sck_c), 
            .Q(spi_expected_last[0])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_expected_last_i0.GSR = "ENABLED";
    EHXPLLJ fpga_pll_i (.CLKI(fpga_clk_8m_c), .CLKFB(pll_feedback), .PHASESEL0(GND_net), 
            .PHASESEL1(GND_net), .PHASEDIR(GND_net), .PHASESTEP(GND_net), 
            .LOADREG(GND_net), .STDBY(GND_net), .PLLWAKESYNC(GND_net), 
            .RST(GND_net), .RESETC(GND_net), .RESETD(GND_net), .RESETM(GND_net), 
            .ENCLKOP(VCC_net), .ENCLKOS(GND_net), .ENCLKOS2(GND_net), 
            .ENCLKOS3(GND_net), .PLLCLK(GND_net), .PLLRST(GND_net), .PLLSTB(GND_net), 
            .PLLWE(GND_net), .PLLDATI0(GND_net), .PLLDATI1(GND_net), .PLLDATI2(GND_net), 
            .PLLDATI3(GND_net), .PLLDATI4(GND_net), .PLLDATI5(GND_net), 
            .PLLDATI6(GND_net), .PLLDATI7(GND_net), .PLLADDR0(GND_net), 
            .PLLADDR1(GND_net), .PLLADDR2(GND_net), .PLLADDR3(GND_net), 
            .PLLADDR4(GND_net), .CLKOP(pll_clk), .LOCK(pll_lock), .CLKINTFB(pll_feedback)) /* synthesis FREQUENCY_PIN_CLKOP="64.000000", FREQUENCY_PIN_CLKI="8.000000", ICP_CURRENT="9", LPF_RESISTOR="72", syn_instantiated=1 */ ;
    defparam fpga_pll_i.CLKI_DIV = 1;
    defparam fpga_pll_i.CLKFB_DIV = 8;
    defparam fpga_pll_i.CLKOP_DIV = 8;
    defparam fpga_pll_i.CLKOS_DIV = 8;
    defparam fpga_pll_i.CLKOS2_DIV = 8;
    defparam fpga_pll_i.CLKOS3_DIV = 8;
    defparam fpga_pll_i.CLKOP_ENABLE = "ENABLED";
    defparam fpga_pll_i.CLKOS_ENABLE = "ENABLED";
    defparam fpga_pll_i.CLKOS2_ENABLE = "ENABLED";
    defparam fpga_pll_i.CLKOS3_ENABLE = "ENABLED";
    defparam fpga_pll_i.VCO_BYPASS_A0 = "DISABLED";
    defparam fpga_pll_i.VCO_BYPASS_B0 = "DISABLED";
    defparam fpga_pll_i.VCO_BYPASS_C0 = "DISABLED";
    defparam fpga_pll_i.VCO_BYPASS_D0 = "DISABLED";
    defparam fpga_pll_i.CLKOP_CPHASE = 7;
    defparam fpga_pll_i.CLKOS_CPHASE = 0;
    defparam fpga_pll_i.CLKOS2_CPHASE = 0;
    defparam fpga_pll_i.CLKOS3_CPHASE = 0;
    defparam fpga_pll_i.CLKOP_FPHASE = 0;
    defparam fpga_pll_i.CLKOS_FPHASE = 0;
    defparam fpga_pll_i.CLKOS2_FPHASE = 0;
    defparam fpga_pll_i.CLKOS3_FPHASE = 0;
    defparam fpga_pll_i.FEEDBK_PATH = "INT_DIVA";
    defparam fpga_pll_i.FRACN_ENABLE = "DISABLED";
    defparam fpga_pll_i.FRACN_DIV = 0;
    defparam fpga_pll_i.CLKOP_TRIM_POL = "RISING";
    defparam fpga_pll_i.CLKOP_TRIM_DELAY = 0;
    defparam fpga_pll_i.CLKOS_TRIM_POL = "RISING";
    defparam fpga_pll_i.CLKOS_TRIM_DELAY = 0;
    defparam fpga_pll_i.PLL_USE_WB = "DISABLED";
    defparam fpga_pll_i.PREDIVIDER_MUXA1 = 0;
    defparam fpga_pll_i.PREDIVIDER_MUXB1 = 0;
    defparam fpga_pll_i.PREDIVIDER_MUXC1 = 0;
    defparam fpga_pll_i.PREDIVIDER_MUXD1 = 0;
    defparam fpga_pll_i.OUTDIVIDER_MUXA2 = "DIVA";
    defparam fpga_pll_i.OUTDIVIDER_MUXB2 = "DIVB";
    defparam fpga_pll_i.OUTDIVIDER_MUXC2 = "DIVC";
    defparam fpga_pll_i.OUTDIVIDER_MUXD2 = "DIVD";
    defparam fpga_pll_i.PLL_LOCK_MODE = 0;
    defparam fpga_pll_i.STDBY_ENABLE = "DISABLED";
    defparam fpga_pll_i.DPHASE_SOURCE = "DISABLED";
    defparam fpga_pll_i.PLLRST_ENA = "DISABLED";
    defparam fpga_pll_i.MRST_ENA = "DISABLED";
    defparam fpga_pll_i.DCRST_ENA = "DISABLED";
    defparam fpga_pll_i.DDRST_ENA = "DISABLED";
    defparam fpga_pll_i.INTFB_WAKE = "DISABLED";
    FD1P3AY spi_expected_last_i1 (.D(n16178), .SP(spi1_sck_c_enable_70), 
            .CK(spi1_sck_c), .Q(spi_expected_last[1])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_expected_last_i1.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i2 (.D(n23825), .SP(spi1_sck_c_enable_70), 
            .CK(spi1_sck_c), .Q(spi_expected_last[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_expected_last_i2.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i3 (.D(n23824), .SP(spi1_sck_c_enable_70), 
            .CK(spi1_sck_c), .Q(spi_expected_last[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_expected_last_i3.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i4 (.D(n23823), .SP(spi1_sck_c_enable_70), 
            .CK(spi1_sck_c), .Q(spi_expected_last[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_expected_last_i4.GSR = "ENABLED";
    FD1P3AY spi_expected_last_i5 (.D(n23822), .SP(spi1_sck_c_enable_70), 
            .CK(spi1_sck_c), .Q(spi_expected_last[5])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_expected_last_i5.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i6 (.D(n23821), .SP(spi1_sck_c_enable_70), 
            .CK(spi1_sck_c), .Q(spi_expected_last[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_expected_last_i6.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i7 (.D(n23820), .SP(spi1_sck_c_enable_70), 
            .CK(spi1_sck_c), .Q(spi_expected_last[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_expected_last_i7.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i8 (.D(n23819), .SP(spi1_sck_c_enable_70), 
            .CK(spi1_sck_c), .Q(spi_expected_last[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_expected_last_i8.GSR = "ENABLED";
    FD1P3AY pcm_acc_i2 (.D(n5388), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(pcm_acc[2])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pcm_acc_i2.GSR = "DISABLED";
    FD1P3AY pcm_acc_i3 (.D(n5387), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(pcm_acc[3])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pcm_acc_i3.GSR = "DISABLED";
    FD1P3AY pcm_acc_i9 (.D(n5381), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(pcm_acc[9])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pcm_acc_i9.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut (.A(mic_acc_state[0]), .B(n21127), .C(n3), .D(n13399), 
         .Z(n11_adj_3156)) /* synthesis lut_function=(A (B (D)+!B ((D)+!C))+!A (D)) */ ;
    defparam i1_3_lut_4_lut.init = 16'hff02;
    FD1P3IX mic_lo_n_i2 (.D(n10722), .SP(pll_clk_enable_39), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_lo_n[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_lo_n_i2.GSR = "DISABLED";
    FD1P3IX mic_lo_n_i3 (.D(n10723), .SP(pll_clk_enable_39), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_lo_n[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_lo_n_i3.GSR = "DISABLED";
    FD1P3IX mic_lo_n_i4 (.D(n10724), .SP(pll_clk_enable_39), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_lo_n[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_lo_n_i4.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_139 (.A(spi_byte_count[5]), .B(n15_adj_3283), .Z(n10_adj_3263)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_adj_139.init = 16'h2222;
    LUT4 mic_acc_word_2__bdd_4_lut_15979 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [0]), 
         .C(\mic_win_q[1] [0]), .D(mic_acc_word[0]), .Z(n27232)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15979.init = 16'he4aa;
    LUT4 i4_3_lut (.A(spi_channel_index[2]), .B(spi_channel_index[6]), .C(spi_channel_index[3]), 
         .Z(n11_adj_3181)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i4_3_lut.init = 16'h0404;
    LUT4 i15153_3_lut_4_lut (.A(mic_acc_state[0]), .B(n21127), .C(n24753), 
         .D(pll_clk_enable_281), .Z(n25666)) /* synthesis lut_function=(A (B (D)+!B (C (D)))+!A (D)) */ ;
    defparam i15153_3_lut_4_lut.init = 16'hfd00;
    FD1P3AY mic_cfg_count_i6 (.D(mic_cfg_hold_spi[6]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_count[6])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_count_i6.GSR = "DISABLED";
    LUT4 n14044_bdd_3_lut (.A(n14044), .B(n14045), .C(status_bit_index[0]), 
         .Z(n28183)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n14044_bdd_3_lut.init = 16'hacac;
    FD1P3AX mic_cfg_width_i1 (.D(mic_cfg_width_w[1]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_width[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_width_i1.GSR = "DISABLED";
    FD1P3AX mic_cfg_width_i2 (.D(mic_cfg_width_w[2]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_width[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_width_i2.GSR = "DISABLED";
    FD1P3AY mic_cfg_width_i3 (.D(mic_cfg_width_w[3]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_width[3])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_width_i3.GSR = "DISABLED";
    FD1P3AX mic_cfg_width_i4 (.D(mic_cfg_width_w[4]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_width[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_width_i4.GSR = "DISABLED";
    FD1P3AX mic_cfg_width_i5 (.D(mic_cfg_width_w[5]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_width[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_width_i5.GSR = "DISABLED";
    FD1P3AX mic_cfg_width_i6 (.D(mic_cfg_hold_spi[46]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_width[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_width_i6.GSR = "DISABLED";
    FD1S3AX mem_3288 (.D(n63), .CK(pll_clk), .Q(n13983));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam mem_3288.GSR = "DISABLED";
    LUT4 i10054_2_lut (.A(mic_lo_n_next_6__N_642[6]), .B(n13), .Z(mic_lo_n_next[6])) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[33:79])
    defparam i10054_2_lut.init = 16'h8888;
    FD1S3AX mem_3290 (.D(n62), .CK(pll_clk), .Q(n13985));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam mem_3290.GSR = "DISABLED";
    umh_level_fifo level_fifo (.n13984(n13984), .spi1_sck_c(spi1_sck_c), 
            .feed_wr_spi({feed_wr_spi}), .n13986(n13986), .n13980(n13980), 
            .n13982(n13982), .feed_q({feed_q}), .pll_clk(pll_clk), .rd_data_7__N_2897({rd_data_7__N_2897}), 
            .n56({n57, n58, n59, n60, n61, n62, n63, n64}), 
            .spi1_mosi_c_0(spi1_mosi_c_0), .\spi_rx_shift[0] (spi_rx_shift[0]), 
            .\spi_rx_shift[1] (spi_rx_shift[1]), .\spi_rx_shift[2] (spi_rx_shift[2]), 
            .\spi_rx_shift[3] (spi_rx_shift[3]), .\spi_rx_shift[4] (spi_rx_shift[4]), 
            .\spi_rx_shift[5] (spi_rx_shift[5]), .\spi_rx_shift[6] (spi_rx_shift[6]), 
            .n13999(n13999), .n14001(n14001), .n14003(n14003), .n14005(n14005), 
            .n14007(n14007), .n14009(n14009), .n14011(n14011), .n14013(n14013), 
            .feed_we(feed_we), .VCC_net(VCC_net), .GND_net(GND_net), .n13988(n13988), 
            .n13990(n13990), .n13992(n13992), .n13994(n13994)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[20] 377[6])
    LUT4 equal_1614_i16_2_lut_3_lut (.A(spi_byte_count[4]), .B(spi_byte_count[5]), 
         .C(n15_adj_3283), .Z(n16_adj_3282)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(456[97:138])
    defparam equal_1614_i16_2_lut_3_lut.init = 16'hfefe;
    FD1S3AX mem_3286 (.D(n64), .CK(pll_clk), .Q(n13981));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam mem_3286.GSR = "DISABLED";
    LUT4 i5_4_lut (.A(spi_channel_index[4]), .B(spi_channel_index[1]), .C(spi_channel_index[0]), 
         .D(spi_channel_index[5]), .Z(n12_adj_3180)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i5_4_lut.init = 16'h0080;
    LUT4 i13629_2_lut (.A(feed_wr[2]), .B(pll_clk_enable_2), .Z(n23779)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam i13629_2_lut.init = 16'h8888;
    DPR16X4C pcm_fifo1 (.DI0(pcm_acc_next[6]), .DI1(pcm_acc_next[7]), .DI2(pcm_acc_next[8]), 
            .DI3(pcm_acc_next[9]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14051), .RAD0(pcm_spi_sample_15__N_1371[0]), .RAD1(pcm_spi_sample_15__N_1371[1]), 
            .RAD2(pcm_spi_sample_15__N_1371[2]), .RAD3(pcm_spi_sample_15__N_1371[3]), 
            .DO0(n14030), .DO1(n14031), .DO2(n14032), .DO3(n14033));
    defparam pcm_fifo1.initval = "0x0000000000000000";
    LUT4 i4_4_lut (.A(n4_adj_3271), .B(spi_extension_length[4]), .C(spi_extension_length[2]), 
         .D(spi_extension_length[3]), .Z(n21230)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut.init = 16'hfffe;
    CCU2D mic_block_count_3057_add_4_9 (.A0(mic_block_count[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[8]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24372), .COUT(n24373), .S0(n78), 
          .S1(n77));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057_add_4_9.INIT0 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_9.INIT1 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_9.INJECT1_0 = "NO";
    defparam mic_block_count_3057_add_4_9.INJECT1_1 = "NO";
    FD1P3AX spi_command_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_command[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_command_i0_i0.GSR = "ENABLED";
    LUT4 i13631_2_lut (.A(feed_wr[3]), .B(pll_clk_enable_2), .Z(n23777)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam i13631_2_lut.init = 16'h8888;
    LUT4 i2_3_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), .C(spi_byte_count[1]), 
         .Z(n16919)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[28:57])
    defparam i2_3_lut.init = 16'hfefe;
    CCU2D ev_ch_3051_add_4_3 (.A0(ev_ch[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_ch[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24551), .COUT(n24552), .S0(n39_adj_3184), .S1(n38_adj_3185));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051_add_4_3.INIT0 = 16'hfaaa;
    defparam ev_ch_3051_add_4_3.INIT1 = 16'hfaaa;
    defparam ev_ch_3051_add_4_3.INJECT1_0 = "NO";
    defparam ev_ch_3051_add_4_3.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i40_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_39), .D(ev_run_hold_s5[39]), .Z(us_tx_next[39])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i40_3_lut_4_lut.init = 16'hbf40;
    OB us_tx_pad_81 (.I(us_tx_c_81), .O(us_tx[81]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    CCU2D mic_block_count_3057_add_4_7 (.A0(mic_block_count[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24371), .COUT(n24372), .S0(n80), 
          .S1(n79));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_7.INJECT1_0 = "NO";
    defparam mic_block_count_3057_add_4_7.INJECT1_1 = "NO";
    CCU2D mic_block_count_3057_add_4_5 (.A0(mic_block_count[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24370), .COUT(n24371), .S0(n82), 
          .S1(n81));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_5.INJECT1_0 = "NO";
    defparam mic_block_count_3057_add_4_5.INJECT1_1 = "NO";
    OB us_tx_pad_82 (.I(us_tx_c_82), .O(us_tx[82]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    LUT4 i9_4_lut (.A(n25544), .B(n25538), .C(n25540), .D(n16), .Z(payload_end)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i9_4_lut.init = 16'h0100;
    LUT4 i28_1_lut (.A(fpga_cs_n_c), .Z(fpga_cs_n_N_2452)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i28_1_lut.init = 16'h5555;
    OB us_tx_pad_83 (.I(us_tx_c_83), .O(us_tx[83]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    CCU2D mic_block_count_3057_add_4_3 (.A0(mic_block_count[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24369), .COUT(n24370), .S0(n84), 
          .S1(n83));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_block_count_3057_add_4_3.INJECT1_0 = "NO";
    defparam mic_block_count_3057_add_4_3.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i39_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_38), .D(ev_run_hold_s5[38]), .Z(us_tx_next[38])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i39_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i23_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_22), .D(ev_run_hold_s5[22]), .Z(us_tx_next[22])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i23_3_lut_4_lut.init = 16'hbf40;
    LUT4 i13600_2_lut (.A(spi_extension_length[2]), .B(expected_next_8__N_1396[3]), 
         .Z(n11907)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13600_2_lut.init = 16'h6666;
    LUT4 ev_run_hold_s5_83__I_0_i38_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_37), .D(ev_run_hold_s5[37]), .Z(us_tx_next[37])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i38_3_lut_4_lut.init = 16'hbf40;
    LUT4 i3608_2_lut (.A(spi_extension_length[1]), .B(spi_extension_length[0]), 
         .Z(n4_adj_3271)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(492[50:75])
    defparam i3608_2_lut.init = 16'heeee;
    CCU2D mic_block_count_3057_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_block_count[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24369), .S1(n85));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057_add_4_1.INIT0 = 16'hF000;
    defparam mic_block_count_3057_add_4_1.INIT1 = 16'h0555;
    defparam mic_block_count_3057_add_4_1.INJECT1_0 = "NO";
    defparam mic_block_count_3057_add_4_1.INJECT1_1 = "NO";
    LUT4 i13102_3_lut (.A(feed_pop), .B(feed_wr[0]), .C(pll_clk_enable_2), 
         .Z(n23783)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam i13102_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_140 (.A(mic_run_state[0]), .B(mic_run_state[1]), 
         .C(mic_acc_busy), .D(mic_bc_valid), .Z(mic_acc_gate_6__N_2263)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(273[37:60])
    defparam i1_3_lut_4_lut_adj_140.init = 16'h0400;
    LUT4 ev_run_hold_s5_83__I_0_i3_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_2), .D(ev_run_hold_s5[2]), .Z(us_tx_next[2])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i3_3_lut_4_lut.init = 16'hbf40;
    LUT4 i13627_2_lut (.A(feed_wr[1]), .B(pll_clk_enable_2), .Z(n23781)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam i13627_2_lut.init = 16'h8888;
    LUT4 n27190_bdd_4_lut (.A(n27190), .B(n27990), .C(n27993), .D(status_bit_index[3]), 
         .Z(spi1_miso_N_2436)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27190_bdd_4_lut.init = 16'haad8;
    LUT4 ev_run_hold_s5_83__I_0_i37_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_36), .D(ev_run_hold_s5[36]), .Z(us_tx_next[36])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i37_3_lut_4_lut.init = 16'hbf40;
    LUT4 n13_bdd_4_lut (.A(n13), .B(mic_lo_n_next_6__N_642[5]), .C(mic_lo_n_next_6__N_642[6]), 
         .D(n27435), .Z(mic_lo_q_next)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C)))) */ ;
    defparam n13_bdd_4_lut.init = 16'h5f57;
    LUT4 i15038_2_lut (.A(ws2812_settle[0]), .B(ws2812_settle[1]), .Z(n25945)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam i15038_2_lut.init = 16'h9999;
    LUT4 i9937_2_lut (.A(spi_rgb_start[7]), .B(spi_phase_only_N_2776), .Z(n12938)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[33] 448[80])
    defparam i9937_2_lut.init = 16'h8888;
    LUT4 ev_run_hold_s5_83__I_0_i58_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_57), .D(ev_run_hold_s5[57]), .Z(us_tx_next[57])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i58_3_lut_4_lut.init = 16'hbf40;
    OBZ spi1_miso_pad (.I(spi1_miso_N_2432), .T(fpga_cs_n_c), .O(spi1_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(433[12:21])
    LUT4 i1_4_lut_4_lut (.A(n10677), .B(mic_warm), .C(mic_ultrasonic), 
         .D(mic_run_state_1__N_2390[1]), .Z(pll_clk_enable_11)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+!(C+(D)))) */ ;
    defparam i1_4_lut_4_lut.init = 16'hbbb8;
    LUT4 mux_673_i6_3_lut_4_lut (.A(pending_sequence[5]), .B(accepted_sequence_spi[5]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[5])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i6_3_lut_4_lut.init = 16'hacaa;
    LUT4 i2_3_lut_4_lut_adj_141 (.A(spi_byte_count[1]), .B(n8_adj_3177), 
         .C(n16_adj_3282), .D(spi_byte_count[0]), .Z(n17_adj_3270)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[30:53])
    defparam i2_3_lut_4_lut_adj_141.init = 16'hfeff;
    LUT4 ev_run_hold_s5_83__I_0_i22_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_21), .D(ev_run_hold_s5[21]), .Z(us_tx_next[21])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i22_3_lut_4_lut.init = 16'hbf40;
    FD1S3JX ev_state_FSM_i1 (.D(n15021), .CK(pll_clk), .PD(n10668), .Q(build_idle_N_2714));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam ev_state_FSM_i1.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i59_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_58), .D(ev_run_hold_s5[58]), .Z(us_tx_next[58])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i59_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_adj_142 (.A(n11899), .B(n12_adj_3247), .Z(n16170)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i1_2_lut_adj_142.init = 16'h6666;
    LUT4 i15242_2_lut (.A(n10819), .B(n24753), .Z(pll_clk_enable_281)) /* synthesis lut_function=(!(A (B))) */ ;
    defparam i15242_2_lut.init = 16'h7777;
    LUT4 i13547_3_lut_4_lut (.A(n22119), .B(n16855), .C(spi_rgb_captured[0]), 
         .D(spi_rgb_captured[1]), .Z(n14)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;
    defparam i13547_3_lut_4_lut.init = 16'h7f80;
    LUT4 i2_3_lut_4_lut_adj_143 (.A(spi_bit_count[1]), .B(n22122), .C(n17_adj_3270), 
         .D(n17_adj_3278), .Z(spi1_sck_c_enable_50)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(572[22] 575[16])
    defparam i2_3_lut_4_lut_adj_143.init = 16'h0800;
    LUT4 ev_run_hold_s5_83__I_0_i60_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_59), .D(ev_run_hold_s5[59]), .Z(us_tx_next[59])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i60_3_lut_4_lut.init = 16'hbf40;
    LUT4 i3004_2_lut_3_lut (.A(ev_wrap), .B(ev_fall), .C(ev_fall_N_2683), 
         .Z(n13403)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam i3004_2_lut_3_lut.init = 16'h4040;
    LUT4 equal_3041_i10_2_lut_3_lut_3_lut (.A(mic_acc_state[0]), .B(mic_acc_state[2]), 
         .C(mic_acc_state[1]), .Z(n5_adj_3249)) /* synthesis lut_function=((B+!(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(985[13:17])
    defparam equal_3041_i10_2_lut_3_lut_3_lut.init = 16'hdfdf;
    LUT4 i3_4_lut_adj_144 (.A(n16929), .B(n15_adj_3155), .C(n16966), .D(fpga_cs_n_c), 
         .Z(n8_adj_3255)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i3_4_lut_adj_144.init = 16'h0004;
    LUT4 i14981_3_lut (.A(n15123), .B(status_next_bit_N_2754[2]), .C(status_next_index[0]), 
         .Z(n25765)) /* synthesis lut_function=(A (B)+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam i14981_3_lut.init = 16'hc8c8;
    LUT4 ev_run_hold_s5_83__I_0_i36_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_35), .D(ev_run_hold_s5[35]), .Z(us_tx_next[35])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i36_3_lut_4_lut.init = 16'hbf40;
    LUT4 i3_4_lut_adj_145 (.A(spi_byte_count[2]), .B(spi_byte_count[1]), 
         .C(spi_bit_count_2__N_1415), .D(n25491), .Z(n16904)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;
    defparam i3_4_lut_adj_145.init = 16'hffbf;
    LUT4 i1_3_lut_4_lut_adj_146 (.A(mic_phase[3]), .B(n22402), .C(n4_adj_3165), 
         .D(n17217), .Z(pll_clk_enable_584)) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_3_lut_4_lut_adj_146.init = 16'hfe00;
    LUT4 ev_run_hold_s5_83__I_0_i43_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_42), .D(ev_run_hold_s5[42]), .Z(us_tx_next[42])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i43_3_lut_4_lut.init = 16'hbf40;
    LUT4 i6930_4_lut (.A(spi1_sck_c_enable_169), .B(n24638), .C(pcm_count_spi[5]), 
         .D(pcm_count_spi[4]), .Z(n17693)) /* synthesis lut_function=(A (B (C+(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i6930_4_lut.init = 16'ha8a0;
    LUT4 equal_2595_i4_2_lut (.A(mic_acc_state[1]), .B(mic_acc_state[2]), 
         .Z(n3)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam equal_2595_i4_2_lut.init = 16'heeee;
    LUT4 ev_run_hold_s5_83__I_0_i44_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_43), .D(ev_run_hold_s5[43]), .Z(us_tx_next[43])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i44_3_lut_4_lut.init = 16'hbf40;
    LUT4 i14683_4_lut (.A(n21230), .B(n4_adj_3246), .C(n19), .D(n20), 
         .Z(n25624)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14683_4_lut.init = 16'hfffe;
    LUT4 i14605_4_lut (.A(spi_byte_count[1]), .B(spi_byte_count[8]), .C(spi_expected_last[1]), 
         .D(spi_expected_last[8]), .Z(n25544)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14605_4_lut.init = 16'h7bde;
    LUT4 ev_run_hold_s5_83__I_0_i35_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_34), .D(ev_run_hold_s5[34]), .Z(us_tx_next[34])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i35_3_lut_4_lut.init = 16'hbf40;
    LUT4 i14599_4_lut (.A(spi_byte_count[0]), .B(spi_byte_count[4]), .C(spi_expected_last[0]), 
         .D(spi_expected_last[4]), .Z(n25538)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14599_4_lut.init = 16'h7bde;
    LUT4 i14601_4_lut (.A(spi_byte_count[7]), .B(spi_byte_count[5]), .C(spi_expected_last[7]), 
         .D(spi_expected_last[5]), .Z(n25540)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14601_4_lut.init = 16'h7bde;
    LUT4 i6_4_lut (.A(expected_valid), .B(n25548), .C(spi_byte_count[2]), 
         .D(spi_expected_last[2]), .Z(n16)) /* synthesis lut_function=(!((B+!(C (D)+!C !(D)))+!A)) */ ;
    defparam i6_4_lut.init = 16'h2002;
    LUT4 ev_run_hold_s5_83__I_0_i61_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_60), .D(ev_run_hold_s5[60]), .Z(us_tx_next[60])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i61_3_lut_4_lut.init = 16'hbf40;
    LUT4 i8_4_lut (.A(spi_rx_shift[6]), .B(spi_rx_shift[5]), .C(spi_rx_shift[2]), 
         .D(expected_next_8__N_1396[3]), .Z(n19)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i8_4_lut.init = 16'hfffe;
    LUT4 i9_4_lut_adj_147 (.A(spi1_mosi_c_0), .B(n18_adj_3164), .C(spi_rgb_start[7]), 
         .D(spi_extension_length[5]), .Z(n20)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i9_4_lut_adj_147.init = 16'hfffe;
    LUT4 i2_2_lut_3_lut (.A(mic_acc_word[1]), .B(mic_acc_word[0]), .C(mic_acc_word[2]), 
         .Z(n21127)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(978[37:56])
    defparam i2_2_lut_3_lut.init = 16'h8080;
    LUT4 ev_run_hold_s5_83__I_0_i62_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_61), .D(ev_run_hold_s5[61]), .Z(us_tx_next[61])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i62_3_lut_4_lut.init = 16'hbf40;
    LUT4 i3453_2_lut (.A(mic_acc_word[1]), .B(mic_acc_word[0]), .Z(n6974)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(978[37:56])
    defparam i3453_2_lut.init = 16'h6666;
    LUT4 i7_4_lut (.A(spi_rx_shift[3]), .B(spi_rx_shift[0]), .C(spi_rx_shift[4]), 
         .D(spi_rx_shift[1]), .Z(n18_adj_3164)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i7_4_lut.init = 16'hfffe;
    LUT4 i3982_2_lut_3_lut (.A(ws2812_frame_done), .B(ws2812_frames_left[0]), 
         .C(ws2812_frames_left[1]), .Z(n3334)) /* synthesis lut_function=(!(A (B+!(C))+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[13:68])
    defparam i3982_2_lut_3_lut.init = 16'h6464;
    LUT4 n27040_bdd_4_lut_16207_else_3_lut (.A(n14028), .B(n14029), .C(status_bit_index[0]), 
         .Z(n28033)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27040_bdd_4_lut_16207_else_3_lut.init = 16'hacac;
    LUT4 i14609_4_lut (.A(spi_byte_count[3]), .B(spi_byte_count[6]), .C(spi_expected_last[3]), 
         .D(spi_expected_last[6]), .Z(n25548)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14609_4_lut.init = 16'h7bde;
    LUT4 ev_run_hold_s5_83__I_0_i63_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_62), .D(ev_run_hold_s5[62]), .Z(us_tx_next[62])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i63_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i21_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_20), .D(ev_run_hold_s5[20]), .Z(us_tx_next[20])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i21_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i64_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_63), .D(ev_run_hold_s5[63]), .Z(us_tx_next[63])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i64_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i65_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_64), .D(ev_run_hold_s5[64]), .Z(us_tx_next[64])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i65_3_lut_4_lut.init = 16'hbf40;
    LUT4 i3_4_lut_adj_148 (.A(pcm_count_spi[0]), .B(pcm_count_spi[3]), .C(pcm_count_spi[2]), 
         .D(pcm_count_spi[1]), .Z(n24638)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut_adj_148.init = 16'hfffe;
    LUT4 i1_2_lut_adj_149 (.A(spi_byte_count[5]), .B(n15_adj_3283), .Z(n16926)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(456[57:94])
    defparam i1_2_lut_adj_149.init = 16'hdddd;
    FD1S3AX spi_bit_count_3045__i1 (.D(n19_adj_3168), .CK(spi1_sck_c), .Q(spi_bit_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam spi_bit_count_3045__i1.GSR = "ENABLED";
    LUT4 i6_4_lut_adj_150 (.A(spi_version[5]), .B(n12_adj_3208), .C(spi_version[7]), 
         .D(spi_version[4]), .Z(n7_adj_3219)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i6_4_lut_adj_150.init = 16'hfffe;
    FD1S3AX spi_bit_count_3045__i2 (.D(n18_adj_3169), .CK(spi1_sck_c), .Q(spi_bit_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam spi_bit_count_3045__i2.GSR = "ENABLED";
    LUT4 i1_2_lut_3_lut (.A(n10677), .B(mic_warm), .C(mic_run_state_1__N_2390[1]), 
         .Z(n22401)) /* synthesis lut_function=(A (B+(C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_3_lut.init = 16'hf8f8;
    LUT4 i7002_2_lut (.A(pll_clk_enable_603), .B(n5_adj_3249), .Z(n17760)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i7002_2_lut.init = 16'h8888;
    DPR16X4C pcm_fifo7 (.DI0(GND_net), .DI1(GND_net), .DI2(GND_net), .DI3(GND_net), 
            .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), .WAD2(pcm_wr_ptr[2]), 
            .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), .WRE(n14052), .RAD0(pcm_spi_sample_15__N_1371[0]), 
            .RAD1(pcm_spi_sample_15__N_1371[1]), .RAD2(pcm_spi_sample_15__N_1371[2]), 
            .RAD3(pcm_spi_sample_15__N_1371[3]), .DO0(n14034), .DO1(n14035), 
            .DO2(n14036), .DO3(n14037));
    defparam pcm_fifo7.initval = "0x0000000000000000";
    LUT4 i2_3_lut_4_lut_adj_151 (.A(n16938), .B(mod_settle[0]), .C(mod_toggle_sync), 
         .D(mod_toggle_seen), .Z(n21096)) /* synthesis lut_function=(A+!(B (C (D)+!C !(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_151.init = 16'hbffb;
    FD1P3IX mic_acc_i_3___i1 (.D(n5689), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i1.GSR = "DISABLED";
    LUT4 i14927_3_lut_3_lut_4_lut (.A(n21127), .B(mic_acc_state[0]), .C(mic_acc_state[1]), 
         .D(mic_acc_state[2]), .Z(n25742)) /* synthesis lut_function=(!(A (B (C+(D))+!B ((D)+!C))+!A ((D)+!C))) */ ;
    defparam i14927_3_lut_3_lut_4_lut.init = 16'h0078;
    LUT4 i10387_4_lut (.A(build_level_eff_7__N_1772[0]), .B(build_idle_N_2714), 
         .C(ev_rd_grant), .D(staging_rd_page_N_2719), .Z(pll_clk_enable_102)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (B (C)+!B (C (D)))) */ ;
    defparam i10387_4_lut.init = 16'hf0c8;
    LUT4 sub_225_inv_0_i4_1_lut (.A(status_next_index[3]), .Z(status_next_bit_N_2754[3])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam sub_225_inv_0_i4_1_lut.init = 16'h5555;
    LUT4 i5_4_lut_adj_152 (.A(spi_version[3]), .B(spi_version[0]), .C(spi_version[1]), 
         .D(spi_version[6]), .Z(n12_adj_3208)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i5_4_lut_adj_152.init = 16'hfffb;
    LUT4 i1_4_lut_adj_153 (.A(n20539), .B(n14_adj_3244), .C(n10_adj_3267), 
         .D(n21218), .Z(invalid_frame_spi_N_2546)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i1_4_lut_adj_153.init = 16'h0002;
    LUT4 n27358_bdd_4_lut_then_3_lut (.A(n14048), .B(n14049), .C(status_bit_index[0]), 
         .Z(n27929)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27358_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 i2_3_lut_3_lut (.A(spi_command[3]), .B(spi_command[2]), .C(n14_adj_3244), 
         .Z(n16894)) /* synthesis lut_function=((B+(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[61:83])
    defparam i2_3_lut_3_lut.init = 16'hfdfd;
    LUT4 n27358_bdd_4_lut_else_3_lut (.A(n14032), .B(n14033), .C(status_bit_index[0]), 
         .Z(n27928)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27358_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 mux_673_i4_3_lut_4_lut (.A(pending_sequence[3]), .B(accepted_sequence_spi[3]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[3])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i4_3_lut_4_lut.init = 16'hacaa;
    LUT4 mux_673_i3_3_lut_4_lut (.A(pending_sequence[2]), .B(accepted_sequence_spi[2]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[2])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i3_3_lut_4_lut.init = 16'hacaa;
    LUT4 i4_2_lut_2_lut (.A(spi_command[3]), .B(spi_command[2]), .Z(n15_adj_3155)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[61:83])
    defparam i4_2_lut_2_lut.init = 16'h4444;
    CCU2D ev_ch_3051_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_ch[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24551), .S1(n40_adj_3183));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051_add_4_1.INIT0 = 16'hF000;
    defparam ev_ch_3051_add_4_1.INIT1 = 16'h0555;
    defparam ev_ch_3051_add_4_1.INJECT1_0 = "NO";
    defparam ev_ch_3051_add_4_1.INJECT1_1 = "NO";
    LUT4 i13461_2_lut (.A(mic_sample_count[1]), .B(n24170), .Z(n24)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[37:60])
    defparam i13461_2_lut.init = 16'h6666;
    FD1P3IX ev_wrap_1291 (.D(ev_sum[8]), .SP(ev_fall_N_2683), .CD(n13599), 
            .CK(pll_clk), .Q(ev_wrap)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_wrap_1291.GSR = "DISABLED";
    FD1S3AX mem (.D(spi_rx_shift[6]), .CK(spi1_sck_c), .Q(n14014));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem.GSR = "DISABLED";
    LUT4 i1_4_lut_adj_154 (.A(n25550), .B(n25494), .C(n12689), .D(mic_acc_state[0]), 
         .Z(pll_clk_enable_91)) /* synthesis lut_function=(A (B+!((D)+!C))+!A (B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(951[26] 959[20])
    defparam i1_4_lut_adj_154.init = 16'hdcfc;
    LUT4 i4568_4_lut (.A(ev_sum[5]), .B(staging_q[5]), .C(staging_rd_page_N_2718), 
         .D(ev_in_clear), .Z(n15330)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam i4568_4_lut.init = 16'hcac0;
    LUT4 i15219_4_lut (.A(pll_clk_enable_91), .B(n25550), .C(mic_acc_state[0]), 
         .D(n12689), .Z(n17753)) /* synthesis lut_function=(A (B+!(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i15219_4_lut.init = 16'h8aaa;
    LUT4 ev_run_hold_s5_83__I_0_i34_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_33), .D(ev_run_hold_s5[33]), .Z(us_tx_next[33])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i34_3_lut_4_lut.init = 16'hbf40;
    LUT4 i9801_4_lut (.A(n11_adj_3266), .B(n9_adj_3178), .C(n14_adj_3244), 
         .D(n16932), .Z(n20546)) /* synthesis lut_function=(A (B+(D))+!A (B (C)+!B (C (D)))) */ ;
    defparam i9801_4_lut.init = 16'hfac8;
    PFUMX i4571 (.BLUT(n15332), .ALUT(n25804), .C0(ev_fall_N_2683), .Z(n15333));
    LUT4 ev_run_hold_s5_83__I_0_i20_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_19), .D(ev_run_hold_s5[19]), .Z(us_tx_next[19])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i20_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27052_bdd_4_lut (.A(n27052), .B(\mic_win_q[0] [5]), .C(\mic_win_i[0] [5]), 
         .D(mic_acc_word[0]), .Z(n27055)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27052_bdd_4_lut.init = 16'haad8;
    DPR16X4C pcm_fifo6 (.DI0(GND_net), .DI1(GND_net), .DI2(pcm_acc_next[0]), 
            .DI3(pcm_acc_next[1]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14052), .RAD0(pcm_spi_sample_15__N_1371[0]), .RAD1(pcm_spi_sample_15__N_1371[1]), 
            .RAD2(pcm_spi_sample_15__N_1371[2]), .RAD3(pcm_spi_sample_15__N_1371[3]), 
            .DO0(n14038), .DO1(n14039), .DO2(n14040), .DO3(n14041));
    defparam pcm_fifo6.initval = "0x0000000000000000";
    LUT4 i1_2_lut_3_lut_adj_155 (.A(mod_settle[0]), .B(n16938), .C(n21096), 
         .Z(pll_clk_enable_2)) /* synthesis lut_function=(!(A (C)+!A ((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(692[18] 701[12])
    defparam i1_2_lut_3_lut_adj_155.init = 16'h0e0e;
    DPR16X4C pcm_fifo5 (.DI0(pcm_acc_next[2]), .DI1(pcm_acc_next[3]), .DI2(pcm_acc_next[4]), 
            .DI3(pcm_acc_next[5]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14052), .RAD0(pcm_spi_sample_15__N_1371[0]), .RAD1(pcm_spi_sample_15__N_1371[1]), 
            .RAD2(pcm_spi_sample_15__N_1371[2]), .RAD3(pcm_spi_sample_15__N_1371[3]), 
            .DO0(n14042), .DO1(n14043), .DO2(n14044), .DO3(n14045));
    defparam pcm_fifo5.initval = "0x0000000000000000";
    DPR16X4C pcm_fifo4 (.DI0(GND_net), .DI1(GND_net), .DI2(GND_net), .DI3(GND_net), 
            .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), .WAD2(pcm_wr_ptr[2]), 
            .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), .WRE(n14051), .RAD0(pcm_spi_sample_15__N_1371[0]), 
            .RAD1(pcm_spi_sample_15__N_1371[1]), .RAD2(pcm_spi_sample_15__N_1371[2]), 
            .RAD3(pcm_spi_sample_15__N_1371[3]), .DO0(n14018), .DO1(n14019), 
            .DO2(n14020), .DO3(n14021));
    defparam pcm_fifo4.initval = "0x0000000000000000";
    DPR16X4C pcm_fifo3 (.DI0(GND_net), .DI1(GND_net), .DI2(pcm_acc_next[0]), 
            .DI3(pcm_acc_next[1]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14051), .RAD0(pcm_spi_sample_15__N_1371[0]), .RAD1(pcm_spi_sample_15__N_1371[1]), 
            .RAD2(pcm_spi_sample_15__N_1371[2]), .RAD3(pcm_spi_sample_15__N_1371[3]), 
            .DO0(n14022), .DO1(n14023), .DO2(n14024), .DO3(n14025));
    defparam pcm_fifo3.initval = "0x0000000000000000";
    DPR16X4C pcm_fifo2 (.DI0(pcm_acc_next[2]), .DI1(pcm_acc_next[3]), .DI2(pcm_acc_next[4]), 
            .DI3(pcm_acc_next[5]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14051), .RAD0(pcm_spi_sample_15__N_1371[0]), .RAD1(pcm_spi_sample_15__N_1371[1]), 
            .RAD2(pcm_spi_sample_15__N_1371[2]), .RAD3(pcm_spi_sample_15__N_1371[3]), 
            .DO0(n14026), .DO1(n14027), .DO2(n14028), .DO3(n14029));
    defparam pcm_fifo2.initval = "0x0000000000000000";
    CCU2D mic_warm_half_3052_add_4_9 (.A0(mic_warm_half[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24550), .S0(n38_adj_3201));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052_add_4_9.INIT0 = 16'hfaaa;
    defparam mic_warm_half_3052_add_4_9.INIT1 = 16'h0000;
    defparam mic_warm_half_3052_add_4_9.INJECT1_0 = "NO";
    defparam mic_warm_half_3052_add_4_9.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i66_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_65), .D(ev_run_hold_s5[65]), .Z(us_tx_next[65])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i66_3_lut_4_lut.init = 16'hbf40;
    LUT4 mic_acc_word_2__bdd_4_lut_15833 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [5]), 
         .C(\mic_win_q[1] [5]), .D(mic_acc_word[0]), .Z(n27052)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15833.init = 16'he4aa;
    LUT4 i14611_2_lut (.A(mic_acc_last), .B(n21127), .Z(n25550)) /* synthesis lut_function=(A+!(B)) */ ;
    defparam i14611_2_lut.init = 16'hbbbb;
    LUT4 spi1_miso_I_28_i53_4_lut_4_lut (.A(status_bit_index[2]), .B(n21039), 
         .C(n25761), .D(n28011), .Z(n53)) /* synthesis lut_function=(A (D)+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[18:44])
    defparam spi1_miso_I_28_i53_4_lut_4_lut.init = 16'hea40;
    LUT4 i1_2_lut_adj_156 (.A(mic_run_state_1__N_2390[1]), .B(pll_clk_enable_262), 
         .Z(pll_clk_enable_111)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_156.init = 16'heeee;
    LUT4 i1_4_lut_adj_157 (.A(n16973), .B(n25519), .C(spi_extension_length[5]), 
         .D(n25510), .Z(n21218)) /* synthesis lut_function=(A (B+(C))+!A (B+(C (D)))) */ ;
    defparam i1_4_lut_adj_157.init = 16'hfcec;
    LUT4 i3869_1_lut (.A(ws2812_settle[0]), .Z(n1)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[17:38])
    defparam i3869_1_lut.init = 16'h5555;
    LUT4 ev_run_hold_s5_83__I_0_i33_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_32), .D(ev_run_hold_s5[32]), .Z(us_tx_next[32])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i33_3_lut_4_lut.init = 16'hbf40;
    LUT4 i2008_2_lut_4_lut (.A(link_idle[3]), .B(n4), .C(stop_toggle_sync), 
         .D(stop_toggle_seen), .Z(n13578)) /* synthesis lut_function=(A (B (C (D)+!C !(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(787[18] 799[12])
    defparam i2008_2_lut_4_lut.init = 16'h8008;
    LUT4 i1_2_lut_adj_158 (.A(spi_bit_count[0]), .B(spi_bit_count[2]), .Z(n22122)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam i1_2_lut_adj_158.init = 16'h8888;
    LUT4 i15036_3_lut (.A(mic_cfg_settle[2]), .B(mic_cfg_settle[1]), .C(mic_cfg_settle[0]), 
         .Z(mic_cfg_settle_3__N_2122[2])) /* synthesis lut_function=(A (B+(C))+!A !(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(285[17:31])
    defparam i15036_3_lut.init = 16'ha9a9;
    LUT4 i23_4_lut (.A(n25602), .B(mic_ultrasonic_N_1894), .C(mic_warm), 
         .D(mic_ultrasonic), .Z(pll_clk_enable_180)) /* synthesis lut_function=(A (B (C+!(D))+!B !(C+(D)))+!A (B+!(C))) */ ;
    defparam i23_4_lut.init = 16'hc5cf;
    LUT4 i1_2_lut_3_lut_4_lut (.A(mic_acc_busy_N_2272), .B(pll_clk_enable_597), 
         .C(mic_run_state_1__N_2390[1]), .D(mic_gate_active_N_2286), .Z(pll_clk_enable_264)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut.init = 16'hfffe;
    CCU2D mic_warm_half_3052_add_4_7 (.A0(mic_warm_half[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_warm_half[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24549), .COUT(n24550), .S0(n40_adj_3199), 
          .S1(n39_adj_3200));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_warm_half_3052_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_warm_half_3052_add_4_7.INJECT1_0 = "NO";
    defparam mic_warm_half_3052_add_4_7.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_159 (.A(mic_cfg_settle[0]), .B(n17), .Z(mic_acc_busy_N_2279)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_159.init = 16'heeee;
    CCU2D mic_warm_half_3052_add_4_5 (.A0(mic_warm_half[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_warm_half[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24548), .COUT(n24549), .S0(n42_adj_3197), 
          .S1(n41_adj_3198));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_warm_half_3052_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_warm_half_3052_add_4_5.INJECT1_0 = "NO";
    defparam mic_warm_half_3052_add_4_5.INJECT1_1 = "NO";
    CCU2D add_2487_9 (.A0(\mic_acc_i[2] [6]), .B0(n20521), .C0(n4547), 
          .D0(GND_net), .A1(\mic_acc_i[2] [7]), .B1(n20521), .C1(n4547), 
          .D1(GND_net), .CIN(n24367), .S0(n5675), .S1(n5674));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(843[17:43])
    defparam add_2487_9.INIT0 = 16'h7848;
    defparam add_2487_9.INIT1 = 16'h7848;
    defparam add_2487_9.INJECT1_0 = "NO";
    defparam add_2487_9.INJECT1_1 = "NO";
    CCU2D mic_warm_half_3052_add_4_3 (.A0(mic_warm_half[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_warm_half[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24547), .COUT(n24548), .S0(n44_adj_3195), 
          .S1(n43_adj_3196));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_warm_half_3052_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_warm_half_3052_add_4_3.INJECT1_0 = "NO";
    defparam mic_warm_half_3052_add_4_3.INJECT1_1 = "NO";
    PFUMX i64 (.BLUT(n25790), .ALUT(n25791), .C0(status_bit_index[4]), 
          .Z(n43_adj_3279));
    CCU2D mic_warm_half_3052_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_warm_half[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24547), .S1(n45_adj_3194));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052_add_4_1.INIT0 = 16'hF000;
    defparam mic_warm_half_3052_add_4_1.INIT1 = 16'h0555;
    defparam mic_warm_half_3052_add_4_1.INJECT1_0 = "NO";
    defparam mic_warm_half_3052_add_4_1.INJECT1_1 = "NO";
    LUT4 n27340_bdd_4_lut_else_3_lut (.A(n14030), .B(n14031), .C(status_bit_index[0]), 
         .Z(n27934)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27340_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 ev_run_hold_s5_83__I_0_i67_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_66), .D(ev_run_hold_s5[66]), .Z(us_tx_next[66])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i67_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i45_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_44), .D(ev_run_hold_s5[44]), .Z(us_tx_next[44])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i45_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_adj_160 (.A(ws2812_frames_left[1]), .B(ws2812_frames_left[0]), 
         .Z(n16154)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_160.init = 16'h9999;
    LUT4 ev_run_hold_s5_83__I_0_i46_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_45), .D(ev_run_hold_s5[45]), .Z(us_tx_next[45])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i46_3_lut_4_lut.init = 16'hbf40;
    LUT4 i816_2_lut (.A(mic_data_0_c), .B(mic_lo_q_r), .Z(n4518)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam i816_2_lut.init = 16'h6666;
    LUT4 i2_4_lut_then_3_lut_4_lut (.A(pll_clk_enable_597), .B(mic_run_state_1__N_2390[1]), 
         .C(n13399), .D(n10819), .Z(n27962)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;
    defparam i2_4_lut_then_3_lut_4_lut.init = 16'hefff;
    DPR16X4C pcm_fifo0 (.DI0(pcm_acc_next[6]), .DI1(pcm_acc_next[7]), .DI2(pcm_acc_next[8]), 
            .DI3(pcm_acc_next[9]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14052), .RAD0(pcm_spi_sample_15__N_1371[0]), .RAD1(pcm_spi_sample_15__N_1371[1]), 
            .RAD2(pcm_spi_sample_15__N_1371[2]), .RAD3(pcm_spi_sample_15__N_1371[3]), 
            .DO0(n14046), .DO1(n14047), .DO2(n14048), .DO3(n14049));
    defparam pcm_fifo0.initval = "0x0000000000000000";
    LUT4 spi_command_7__I_0_1461_i10_2_lut (.A(spi_command[2]), .B(spi_command[3]), 
         .Z(n10_adj_3267)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(568[34:54])
    defparam spi_command_7__I_0_1461_i10_2_lut.init = 16'heeee;
    CCU2D pcm_wr_ptr_3053_add_4_7 (.A0(pcm_wr_ptr[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24546), .S0(n30_adj_3207));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(855[39:56])
    defparam pcm_wr_ptr_3053_add_4_7.INIT0 = 16'hfaaa;
    defparam pcm_wr_ptr_3053_add_4_7.INIT1 = 16'h0000;
    defparam pcm_wr_ptr_3053_add_4_7.INJECT1_0 = "NO";
    defparam pcm_wr_ptr_3053_add_4_7.INJECT1_1 = "NO";
    LUT4 n27334_bdd_4_lut_then_3_lut (.A(mic_block_count[8]), .B(mic_block_count[10]), 
         .C(status_bit_index[1]), .Z(n27938)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27334_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 ev_run_hold_s5_83__I_0_i19_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_18), .D(ev_run_hold_s5[18]), .Z(us_tx_next[18])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i19_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27082_bdd_4_lut_then_3_lut (.A(n14034), .B(n14035), .C(status_bit_index[0]), 
         .Z(n28025)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27082_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 n27334_bdd_4_lut_else_3_lut (.A(mic_block_count[9]), .B(mic_block_count[11]), 
         .C(status_bit_index[1]), .Z(n27937)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27334_bdd_4_lut_else_3_lut.init = 16'hacac;
    FD1S3AX mem_3310 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(n14012));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3310.GSR = "DISABLED";
    FD1S3AX mem_3309 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(n14010));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3309.GSR = "DISABLED";
    FD1S3AX mem_3308 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(n14008));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3308.GSR = "DISABLED";
    FD1S3AX mem_3307 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(n14006));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3307.GSR = "DISABLED";
    FD1S3AX mem_3306 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(n14004));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3306.GSR = "DISABLED";
    FD1S3AX mem_3305 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(n14002));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3305.GSR = "DISABLED";
    FD1S3AX mem_3304 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(n14000));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3304.GSR = "DISABLED";
    LUT4 staging_q_7__I_0_1370_i8_4_lut (.A(staging_q[7]), .B(build_level[7]), 
         .C(build_mod), .D(n15), .Z(event_level[7])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(379[31] 381[40])
    defparam staging_q_7__I_0_1370_i8_4_lut.init = 16'hca0a;
    CCU2D pcm_wr_ptr_3053_add_4_5 (.A0(pcm_wr_ptr[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_wr_ptr[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24545), .COUT(n24546), .S0(n32_adj_3205), 
          .S1(n31_adj_3206));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(855[39:56])
    defparam pcm_wr_ptr_3053_add_4_5.INIT0 = 16'hfaaa;
    defparam pcm_wr_ptr_3053_add_4_5.INIT1 = 16'hfaaa;
    defparam pcm_wr_ptr_3053_add_4_5.INJECT1_0 = "NO";
    defparam pcm_wr_ptr_3053_add_4_5.INJECT1_1 = "NO";
    LUT4 i4_3_lut_4_lut (.A(spi_command[1]), .B(n25474), .C(n35_adj_3269), 
         .D(n8_adj_3255), .Z(spi1_sck_c_enable_169)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i4_3_lut_4_lut.init = 16'h8000;
    LUT4 i1_2_lut_3_lut_adj_161 (.A(link_idle[0]), .B(link_idle[1]), .C(link_idle[2]), 
         .Z(n4)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(383[25:35])
    defparam i1_2_lut_3_lut_adj_161.init = 16'h8080;
    LUT4 ws2812_toggle_sync_I_0_2_lut (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .Z(ws2812_settle_3__N_1697)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(703[13:53])
    defparam ws2812_toggle_sync_I_0_2_lut.init = 16'h6666;
    LUT4 ev_run_hold_s5_83__I_0_i68_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_67), .D(ev_run_hold_s5[67]), .Z(us_tx_next[67])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i68_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_4_lut_adj_162 (.A(n4_adj_3165), .B(n17217), .C(n22402), .D(mic_phase[3]), 
         .Z(pll_clk_enable_592)) /* synthesis lut_function=(A (B)+!A (B (C+!(D)))) */ ;
    defparam i1_4_lut_adj_162.init = 16'hc8cc;
    CCU2D add_2487_7 (.A0(\mic_acc_i[2] [4]), .B0(n20521), .C0(n4547), 
          .D0(GND_net), .A1(\mic_acc_i[2] [5]), .B1(n20521), .C1(n4547), 
          .D1(GND_net), .CIN(n24366), .COUT(n24367), .S0(n5677), .S1(n5676));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(843[17:43])
    defparam add_2487_7.INIT0 = 16'h7848;
    defparam add_2487_7.INIT1 = 16'h7848;
    defparam add_2487_7.INJECT1_0 = "NO";
    defparam add_2487_7.INJECT1_1 = "NO";
    CCU2D pcm_wr_ptr_3053_add_4_3 (.A0(pcm_wr_ptr[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_wr_ptr[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24544), .COUT(n24545), .S0(n34_adj_3203), 
          .S1(n33_adj_3204));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(855[39:56])
    defparam pcm_wr_ptr_3053_add_4_3.INIT0 = 16'hfaaa;
    defparam pcm_wr_ptr_3053_add_4_3.INIT1 = 16'hfaaa;
    defparam pcm_wr_ptr_3053_add_4_3.INJECT1_0 = "NO";
    defparam pcm_wr_ptr_3053_add_4_3.INJECT1_1 = "NO";
    LUT4 i2955_3_lut_4_lut (.A(mic_acc_busy_N_2272), .B(mic_done_N_2646), 
         .C(pll_clk_enable_597), .D(mic_run_state_1__N_2390[1]), .Z(n13586)) /* synthesis lut_function=(A (B)+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i2955_3_lut_4_lut.init = 16'hddd8;
    CCU2D add_2523_9 (.A0(\mic_acc_i[3] [6]), .B0(n20537), .C0(n4547), 
          .D0(GND_net), .A1(\mic_acc_i[3] [7]), .B1(n20537), .C1(n4547), 
          .D1(GND_net), .CIN(n24415), .S0(n5683), .S1(n5682));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam add_2523_9.INIT0 = 16'h7848;
    defparam add_2523_9.INIT1 = 16'h7848;
    defparam add_2523_9.INJECT1_0 = "NO";
    defparam add_2523_9.INJECT1_1 = "NO";
    LUT4 mux_673_i2_3_lut_4_lut (.A(pending_sequence[1]), .B(accepted_sequence_spi[1]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[1])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i2_3_lut_4_lut.init = 16'hacaa;
    LUT4 i1_2_lut_adj_163 (.A(spi_byte_count[0]), .B(n25482), .Z(spi1_sck_c_enable_10)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam i1_2_lut_adj_163.init = 16'h8888;
    LUT4 i1_2_lut_4_lut (.A(ws2812_settle[3]), .B(ws2812_settle[1]), .C(ws2812_settle[2]), 
         .D(ws2812_settle[0]), .Z(ws2812_enable_N_2596)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(708[22:43])
    defparam i1_2_lut_4_lut.init = 16'hfffe;
    LUT4 i15117_4_lut (.A(phase_step_s4), .B(link_timeout), .C(stop_event), 
         .D(running), .Z(ev_run_hold_s5_83__N_1633)) /* synthesis lut_function=((B+(C+!(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(632[13:69])
    defparam i15117_4_lut.init = 16'hfdff;
    LUT4 i4888_2_lut_4_lut (.A(n10_adj_3263), .B(n16904), .C(n20565), 
         .D(spi_byte_count[0]), .Z(spi1_sck_c_enable_25)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;
    defparam i4888_2_lut_4_lut.init = 16'h0200;
    CCU2D pcm_wr_ptr_3053_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_fifo_N_2607[5]), .B1(pcm_wr_ptr[0]), .C1(GND_net), 
          .D1(GND_net), .COUT(n24544), .S1(n35_adj_3202));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(855[39:56])
    defparam pcm_wr_ptr_3053_add_4_1.INIT0 = 16'hF000;
    defparam pcm_wr_ptr_3053_add_4_1.INIT1 = 16'ha999;
    defparam pcm_wr_ptr_3053_add_4_1.INJECT1_0 = "NO";
    defparam pcm_wr_ptr_3053_add_4_1.INJECT1_1 = "NO";
    CCU2D spi_channel_index_3042_add_4_7 (.A0(spi_channel_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24542), .S0(n35_adj_3214), 
          .S1(n34_adj_3215));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042_add_4_7.INIT0 = 16'hfaaa;
    defparam spi_channel_index_3042_add_4_7.INIT1 = 16'hfaaa;
    defparam spi_channel_index_3042_add_4_7.INJECT1_0 = "NO";
    defparam spi_channel_index_3042_add_4_7.INJECT1_1 = "NO";
    FD1S3AX mem_3292 (.D(n61), .CK(pll_clk), .Q(n13987));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam mem_3292.GSR = "DISABLED";
    FD1S3AX mem_3294 (.D(n60), .CK(pll_clk), .Q(n13989));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam mem_3294.GSR = "DISABLED";
    FD1S3AX mem_3296 (.D(n59), .CK(pll_clk), .Q(n13991));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam mem_3296.GSR = "DISABLED";
    FD1S3AX mem_3298 (.D(n58), .CK(pll_clk), .Q(n13993));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam mem_3298.GSR = "DISABLED";
    FD1S3AX feed_rd_3050__i0 (.D(n64), .CK(pll_clk), .Q(feed_rd[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050__i0.GSR = "DISABLED";
    FD1S3AX mem_3301 (.D(n57), .CK(pll_clk), .Q(n13995));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam mem_3301.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_164 (.A(mic_gate_active), .B(mic_acc_busy_N_2282), 
         .Z(n4_adj_3268)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(947[9] 967[12])
    defparam i1_2_lut_adj_164.init = 16'heeee;
    LUT4 i15202_4_lut (.A(n25411), .B(spi_extension_length[5]), .C(n4_adj_3246), 
         .D(n21230), .Z(n25413)) /* synthesis lut_function=(!(A+(B (C+(D))+!B (C)))) */ ;
    defparam i15202_4_lut.init = 16'h0105;
    LUT4 i15093_2_lut_4_lut (.A(n10_adj_3263), .B(n16904), .C(n20565), 
         .D(spi_byte_count[0]), .Z(spi1_sck_c_enable_38)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i15093_2_lut_4_lut.init = 16'h0002;
    FD1P3IX ev_ch_3051__i6 (.D(n34_adj_3187), .SP(pll_clk_enable_121), .CD(n17726), 
            .CK(pll_clk), .Q(ev_ch[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051__i6.GSR = "DISABLED";
    FD1P3IX ev_ch_3051__i5 (.D(n35_adj_3186), .SP(pll_clk_enable_121), .CD(n17726), 
            .CK(pll_clk), .Q(ev_ch[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051__i5.GSR = "DISABLED";
    FD1P3IX ev_ch_3051__i4 (.D(n36), .SP(pll_clk_enable_121), .CD(n17726), 
            .CK(pll_clk), .Q(ev_ch[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051__i4.GSR = "DISABLED";
    FD1P3IX ev_ch_3051__i3 (.D(n37), .SP(pll_clk_enable_121), .CD(n17726), 
            .CK(pll_clk), .Q(ev_ch[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051__i3.GSR = "DISABLED";
    FD1P3IX ev_ch_3051__i2 (.D(n38_adj_3185), .SP(pll_clk_enable_121), .CD(n17726), 
            .CK(pll_clk), .Q(ev_ch[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051__i2.GSR = "DISABLED";
    FD1P3IX ev_ch_3051__i1 (.D(n39_adj_3184), .SP(pll_clk_enable_121), .CD(n17726), 
            .CK(pll_clk), .Q(ev_ch[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051__i1.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3052__i7 (.D(n38_adj_3201), .SP(pll_clk_enable_122), 
            .CD(n20290), .CK(pll_clk), .Q(mic_warm_half[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052__i7.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3052__i6 (.D(n39_adj_3200), .SP(pll_clk_enable_122), 
            .CD(n20290), .CK(pll_clk), .Q(mic_warm_half[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052__i6.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3052__i5 (.D(n40_adj_3199), .SP(pll_clk_enable_122), 
            .CD(n20290), .CK(pll_clk), .Q(mic_warm_half[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052__i5.GSR = "DISABLED";
    PFUMX i16462 (.BLUT(n28024), .ALUT(n28025), .C0(pcm_spi_sample_15__N_1371[4]), 
          .Z(n28026));
    LUT4 ev_run_hold_s5_83__I_0_i47_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_46), .D(ev_run_hold_s5[46]), .Z(us_tx_next[46])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i47_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27352_bdd_4_lut (.A(n27352), .B(\mic_win_q[0] [2]), .C(\mic_win_i[0] [2]), 
         .D(mic_acc_word[0]), .Z(n27355)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27352_bdd_4_lut.init = 16'haad8;
    FD1P3IX mic_warm_half_3052__i4 (.D(n41_adj_3198), .SP(pll_clk_enable_122), 
            .CD(n20290), .CK(pll_clk), .Q(mic_warm_half[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052__i4.GSR = "DISABLED";
    LUT4 i13274_2_lut (.A(mic_window_count[0]), .B(n13362), .Z(n23987)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(871[41:64])
    defparam i13274_2_lut.init = 16'h8888;
    FD1P3AX mic_sample_count_3055__i3 (.D(n22), .SP(pll_clk_enable_82), 
            .CK(pll_clk), .Q(mic_sample_count[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[37:60])
    defparam mic_sample_count_3055__i3.GSR = "DISABLED";
    LUT4 mic_acc_word_2__bdd_4_lut_16073 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [2]), 
         .C(\mic_win_q[1] [2]), .D(mic_acc_word[0]), .Z(n27352)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16073.init = 16'he4aa;
    FD1P3IX mic_warm_half_3052__i3 (.D(n42_adj_3197), .SP(pll_clk_enable_122), 
            .CD(n20290), .CK(pll_clk), .Q(mic_warm_half[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052__i3.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3052__i2 (.D(n43_adj_3196), .SP(pll_clk_enable_122), 
            .CD(n20290), .CK(pll_clk), .Q(mic_warm_half[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052__i2.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3052__i1 (.D(n44_adj_3195), .SP(pll_clk_enable_122), 
            .CD(n20290), .CK(pll_clk), .Q(mic_warm_half[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052__i1.GSR = "DISABLED";
    LUT4 i6919_1_lut (.A(phase_step_s1), .Z(n17690)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i6919_1_lut.init = 16'h5555;
    LUT4 i6918_2_lut (.A(pll_clk_enable_597), .B(mic_cfg_gap_w[16]), .Z(n17676)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i6918_2_lut.init = 16'h8888;
    FD1P3JX ws2812_settle_i0_i3 (.D(n27634), .SP(pll_clk_enable_178), .PD(ws2812_settle_3__N_1697), 
            .CK(pll_clk), .Q(ws2812_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ws2812_settle_i0_i3.GSR = "DISABLED";
    FD1P3JX ws2812_settle_i0_i2 (.D(n27633), .SP(pll_clk_enable_178), .PD(ws2812_settle_3__N_1697), 
            .CK(pll_clk), .Q(ws2812_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ws2812_settle_i0_i2.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_165 (.A(global_phase_s2[0]), .B(n14_adj_3260), .C(n10_adj_3262), 
         .D(global_phase_s2[3]), .Z(wrap_s2_N_2598)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut_adj_165.init = 16'h8000;
    FD1P3JX ws2812_settle_i0_i1 (.D(n25945), .SP(pll_clk_enable_178), .PD(ws2812_settle_3__N_1697), 
            .CK(pll_clk), .Q(ws2812_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ws2812_settle_i0_i1.GSR = "DISABLED";
    FD1P3IX mic_acc_word_i2 (.D(n6973), .SP(pll_clk_enable_281), .CD(n25666), 
            .CK(pll_clk), .Q(mic_acc_word[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_word_i2.GSR = "DISABLED";
    FD1P3IX mic_acc_word_i1 (.D(n6974), .SP(pll_clk_enable_281), .CD(n25666), 
            .CK(pll_clk), .Q(mic_acc_word[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_word_i1.GSR = "DISABLED";
    LUT4 i3156_2_lut_3_lut (.A(mic_acc_busy_N_2272), .B(mic_done_N_2646), 
         .C(n13586), .Z(pll_clk_enable_193)) /* synthesis lut_function=(A ((C)+!B)+!A (C)) */ ;
    defparam i3156_2_lut_3_lut.init = 16'hf2f2;
    LUT4 i6_4_lut_adj_166 (.A(global_phase_s2[1]), .B(global_phase_s2[4]), 
         .C(global_phase_s2[5]), .D(global_phase_s2[7]), .Z(n14_adj_3260)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_166.init = 16'h8000;
    LUT4 i2_2_lut (.A(global_phase_s2[2]), .B(global_phase_s2[6]), .Z(n10_adj_3262)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut.init = 16'h8888;
    LUT4 i10075_3_lut (.A(frame_settle[1]), .B(n7_adj_3256), .C(frame_settle[0]), 
         .Z(n3390)) /* synthesis lut_function=(A (B (C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[13] 678[16])
    defparam i10075_3_lut.init = 16'h8484;
    LUT4 i1_2_lut_3_lut_adj_167 (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .Z(spi_bit_count_2__N_1415)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam i1_2_lut_3_lut_adj_167.init = 16'h8080;
    LUT4 i2004_2_lut_4_lut (.A(link_idle[3]), .B(n4), .C(stop_toggle_sync), 
         .D(stop_toggle_seen), .Z(n10668)) /* synthesis lut_function=(A (B+!(C (D)+!C !(D)))+!A !(C (D)+!C !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(787[18] 799[12])
    defparam i2004_2_lut_4_lut.init = 16'h8ff8;
    LUT4 i1_2_lut_adj_168 (.A(pll_clk_enable_39), .B(mic_run_state_1__N_2390[1]), 
         .Z(pll_clk_enable_273)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_168.init = 16'heeee;
    CCU2D add_2523_7 (.A0(\mic_acc_i[3] [4]), .B0(n20537), .C0(n4547), 
          .D0(GND_net), .A1(\mic_acc_i[3] [5]), .B1(n20537), .C1(n4547), 
          .D1(GND_net), .CIN(n24414), .COUT(n24415), .S0(n5685), .S1(n5684));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam add_2523_7.INIT0 = 16'h7848;
    defparam add_2523_7.INIT1 = 16'h7848;
    defparam add_2523_7.INJECT1_0 = "NO";
    defparam add_2523_7.INJECT1_1 = "NO";
    LUT4 n27070_bdd_4_lut (.A(n27070), .B(\mic_win_q[2] [4]), .C(\mic_win_i[2] [4]), 
         .D(mic_acc_word[0]), .Z(n27073)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27070_bdd_4_lut.init = 16'haad8;
    LUT4 mic_acc_word_2__bdd_4_lut_15838 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [4]), 
         .C(\mic_win_q[3] [4]), .D(mic_acc_word[0]), .Z(n27070)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15838.init = 16'he4aa;
    LUT4 n27322_bdd_4_lut_16349_then_3_lut (.A(mic_block_count[4]), .B(mic_block_count[6]), 
         .C(status_bit_index[1]), .Z(n27941)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27322_bdd_4_lut_16349_then_3_lut.init = 16'hacac;
    FD1P3IX mic_ram_wr_data_i0_i15 (.D(mic_ram_wr_data_15__N_2296[15]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i15.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i14 (.D(mic_ram_wr_data_15__N_2296[14]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i14.GSR = "DISABLED";
    LUT4 n27322_bdd_4_lut_16349_else_3_lut (.A(mic_block_count[5]), .B(mic_block_count[7]), 
         .C(status_bit_index[1]), .Z(n27940)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27322_bdd_4_lut_16349_else_3_lut.init = 16'hacac;
    LUT4 i9771_2_lut (.A(mic_lo_n_next_6__N_642[0]), .B(n13), .Z(mic_lo_n_next[0])) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[33:79])
    defparam i9771_2_lut.init = 16'h8888;
    FD1P3IX mic_ram_wr_data_i0_i13 (.D(mic_ram_wr_data_15__N_2296[13]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i13.GSR = "DISABLED";
    LUT4 mux_673_i1_3_lut_4_lut (.A(pending_sequence[0]), .B(accepted_sequence_spi[0]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[0])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i1_3_lut_4_lut.init = 16'hacaa;
    LUT4 cs_rise_I_0_2_lut (.A(cs_sync_d), .B(cs_sync), .Z(cs_rise)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(389[20:41])
    defparam cs_rise_I_0_2_lut.init = 16'h4444;
    FD1P3IX mic_ram_wr_data_i0_i12 (.D(mic_ram_wr_data_15__N_2296[12]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i12.GSR = "DISABLED";
    LUT4 i4_2_lut_3_lut (.A(mic_window_count[3]), .B(mic_window_count[0]), 
         .C(n8_adj_3258), .Z(n16750)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[21:47])
    defparam i4_2_lut_3_lut.init = 16'hfefe;
    CCU2D add_2523_5 (.A0(\mic_acc_i[3] [2]), .B0(n20537), .C0(n4547), 
          .D0(GND_net), .A1(\mic_acc_i[3] [3]), .B1(n20537), .C1(n4547), 
          .D1(GND_net), .CIN(n24413), .COUT(n24414), .S0(n5687), .S1(n5686));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam add_2523_5.INIT0 = 16'h7848;
    defparam add_2523_5.INIT1 = 16'h7848;
    defparam add_2523_5.INJECT1_0 = "NO";
    defparam add_2523_5.INJECT1_1 = "NO";
    LUT4 i15199_2_lut_3_lut (.A(n20546), .B(n21266), .C(spi1_sck_c_enable_146), 
         .Z(spi1_sck_c_enable_144)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i15199_2_lut_3_lut.init = 16'h8080;
    LUT4 i14985_2_lut (.A(status_hold[76]), .B(status_next_index[1]), .Z(n25778)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam i14985_2_lut.init = 16'h8888;
    LUT4 i15188_4_lut (.A(n13403), .B(staging_rd_page_N_2719), .C(n13154), 
         .D(n10024), .Z(pll_clk_enable_311)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;
    defparam i15188_4_lut.init = 16'h0001;
    LUT4 i2_3_lut_4_lut_adj_169 (.A(n10668), .B(mod_settle[0]), .C(n16938), 
         .D(feed_rd_7__N_1692), .Z(pll_clk_enable_112)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_169.init = 16'hfffe;
    LUT4 i2_3_lut_adj_170 (.A(mic_cfg_hold_spi[6]), .B(pll_clk_enable_597), 
         .C(n16730), .Z(n24822)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i2_3_lut_adj_170.init = 16'h0404;
    LUT4 i1_4_lut_adj_171 (.A(ws2812_frame_done), .B(ws2812_enable_N_2578), 
         .C(ws2812_frames_left[1]), .D(ws2812_frames_left[0]), .Z(pll_clk_enable_18)) /* synthesis lut_function=(A (B+!(C+!(D)))+!A (B)) */ ;
    defparam i1_4_lut_adj_171.init = 16'hcecc;
    LUT4 equal_3028_i31_3_lut_4_lut (.A(spi_version[2]), .B(n7_adj_3219), 
         .C(n16907), .D(n20539), .Z(n31)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam equal_3028_i31_3_lut_4_lut.init = 16'hfeff;
    LUT4 mux_673_i16_3_lut_4_lut (.A(pending_sequence[15]), .B(accepted_sequence_spi[15]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[15])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i16_3_lut_4_lut.init = 16'hacaa;
    LUT4 i1_3_lut_4_lut_adj_172 (.A(spi_version[2]), .B(n7_adj_3219), .C(n4_adj_3246), 
         .D(n25411), .Z(n25519)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i1_3_lut_4_lut_adj_172.init = 16'hfffe;
    LUT4 i13475_3_lut_4_lut (.A(mic_sample_count[1]), .B(n24170), .C(mic_sample_count[2]), 
         .D(mic_sample_count[3]), .Z(n22)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[37:60])
    defparam i13475_3_lut_4_lut.init = 16'h7f80;
    LUT4 n27076_bdd_4_lut (.A(n27076), .B(\mic_win_q[2] [5]), .C(\mic_win_i[2] [5]), 
         .D(mic_acc_word[0]), .Z(n27079)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27076_bdd_4_lut.init = 16'haad8;
    LUT4 i1_2_lut_adj_173 (.A(mic_cfg_hold_spi[0]), .B(mic_cfg_hold_spi[6]), 
         .Z(mic_cfg_count_w_6__N_850[0])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[35:100])
    defparam i1_2_lut_adj_173.init = 16'h2222;
    LUT4 i13468_2_lut_3_lut (.A(mic_sample_count[1]), .B(n24170), .C(mic_sample_count[2]), 
         .Z(n23)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[37:60])
    defparam i13468_2_lut_3_lut.init = 16'h7878;
    LUT4 i1_2_lut_adj_174 (.A(spi_rgb_captured[0]), .B(spi_rgb_captured[1]), 
         .Z(n16394)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_174.init = 16'h8888;
    LUT4 mic_acc_word_2__bdd_4_lut_15848 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [5]), 
         .C(\mic_win_q[3] [5]), .D(mic_acc_word[0]), .Z(n27076)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15848.init = 16'he4aa;
    LUT4 i1_2_lut_adj_175 (.A(n10677), .B(mic_warm), .Z(mic_warm_N_1898)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_175.init = 16'h4444;
    FD1P3IX mic_ram_wr_data_i0_i11 (.D(mic_ram_wr_data_15__N_2296[11]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i11.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i10 (.D(mic_ram_wr_data_15__N_2296[10]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i10.GSR = "DISABLED";
    FD1P3IX mic_cfg_gap_i0 (.D(mic_cfg_gap_w[0]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i0.GSR = "DISABLED";
    LUT4 n27316_bdd_4_lut_then_3_lut (.A(mic_block_count[0]), .B(mic_block_count[2]), 
         .C(status_bit_index[1]), .Z(n27944)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27316_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 n27316_bdd_4_lut_else_3_lut (.A(mic_block_count[1]), .B(mic_block_count[3]), 
         .C(status_bit_index[1]), .Z(n27943)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27316_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 n20658_bdd_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[5]), .D(spi_byte_count[4]), .Z(n27763)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam n20658_bdd_4_lut.init = 16'h0008;
    LUT4 i10463_2_lut (.A(spi_byte_count[5]), .B(n15_adj_3283), .Z(n21213)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10463_2_lut.init = 16'heeee;
    LUT4 i1_2_lut_3_lut_adj_176 (.A(mod_settle[2]), .B(mod_settle[1]), .C(mod_settle[0]), 
         .Z(n16144)) /* synthesis lut_function=(A (B+(C))+!A !(B+(C))) */ ;
    defparam i1_2_lut_3_lut_adj_176.init = 16'ha9a9;
    LUT4 n1026_bdd_2_lut_16318 (.A(n27763), .B(n20658), .Z(n27764)) /* synthesis lut_function=(A (B)) */ ;
    defparam n1026_bdd_2_lut_16318.init = 16'h8888;
    LUT4 status_next_bit_I_0_i125_then_4_lut (.A(status_hold[104]), .B(n25614), 
         .C(status_next_index[3]), .D(status_next_index[2]), .Z(n27947)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam status_next_bit_I_0_i125_then_4_lut.init = 16'h0800;
    LUT4 staging_q_7__I_0_1370_i7_4_lut (.A(staging_q[6]), .B(build_level[6]), 
         .C(build_mod), .D(n15), .Z(event_level[6])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(379[31] 381[40])
    defparam staging_q_7__I_0_1370_i7_4_lut.init = 16'hca0a;
    LUT4 staging_q_7__I_0_1370_i6_4_lut (.A(staging_q[5]), .B(build_level[5]), 
         .C(build_mod), .D(n15), .Z(event_level[5])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(379[31] 381[40])
    defparam staging_q_7__I_0_1370_i6_4_lut.init = 16'hca0a;
    LUT4 ev_run_hold_s5_83__I_0_i69_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_68), .D(ev_run_hold_s5[68]), .Z(us_tx_next[68])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i69_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_adj_177 (.A(pll_clk_enable_2), .B(n10668), .Z(n13582)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(787[18] 799[12])
    defparam i1_2_lut_adj_177.init = 16'heeee;
    FD1P3IX mic_ram_wr_data_i0_i9 (.D(mic_ram_wr_data_15__N_2296[9]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i9.GSR = "DISABLED";
    LUT4 n14047_bdd_3_lut_16776 (.A(n14047), .B(n14046), .C(status_bit_index[0]), 
         .Z(n28180)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n14047_bdd_3_lut_16776.init = 16'hcaca;
    LUT4 status_next_bit_N_2754_2__bdd_4_lut_16098_4_lut (.A(status_next_index[2]), 
         .B(n27205), .C(n27211), .D(status_next_index[3]), .Z(n27226)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;
    defparam status_next_bit_N_2754_2__bdd_4_lut_16098_4_lut.init = 16'h55d8;
    LUT4 i1_2_lut_adj_178 (.A(ev_fall_N_2683), .B(staging_rd_page_N_2718), 
         .Z(pll_clk_enable_21)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(322[35] 323[54])
    defparam i1_2_lut_adj_178.init = 16'heeee;
    FD1P3IX mic_ram_wr_data_i0_i8 (.D(mic_ram_wr_data_15__N_2296[8]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i8.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_179 (.A(mic_phase[3]), .B(pll_clk_enable_280), .Z(pll_clk_enable_363)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_179.init = 16'h4444;
    FD1P3IX mic_ram_wr_data_i0_i7 (.D(mic_ram_wr_data_15__N_2296[7]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i7.GSR = "DISABLED";
    LUT4 i2_3_lut_4_lut_adj_180 (.A(n31), .B(frame_end), .C(n11_adj_3163), 
         .D(n14797), .Z(spi1_sck_c_enable_164)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i2_3_lut_4_lut_adj_180.init = 16'h8000;
    LUT4 i4_4_lut_adj_181 (.A(mic_sample_count[0]), .B(mic_sample_count[2]), 
         .C(mic_sample_count[1]), .D(n6_adj_3182), .Z(pll_clk_enable_454)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i4_4_lut_adj_181.init = 16'h8000;
    LUT4 ws2812_settle_1__bdd_2_lut (.A(ws2812_settle[1]), .B(n28057), .Z(n10654)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam ws2812_settle_1__bdd_2_lut.init = 16'h4444;
    LUT4 i1_2_lut_adj_182 (.A(mic_sample_count[3]), .B(pll_clk_enable_391), 
         .Z(n6_adj_3182)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_182.init = 16'h8888;
    LUT4 i14896_2_lut_2_lut_3_lut (.A(ev_fall), .B(rmw_slot[0]), .C(build_level_eff[0]), 
         .Z(n25736)) /* synthesis lut_function=(!(A+(B (C)+!B !(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(754[21:29])
    defparam i14896_2_lut_2_lut_3_lut.init = 16'h1414;
    LUT4 status_next_bit_N_2754_2__bdd_4_lut_4_lut (.A(status_next_index[2]), 
         .B(n27373), .C(n27391), .D(status_next_index[3]), .Z(n27394)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;
    defparam status_next_bit_N_2754_2__bdd_4_lut_4_lut.init = 16'h55d8;
    LUT4 i10055_2_lut (.A(mic_lo_n_next_6__N_642[5]), .B(n13), .Z(mic_lo_n_next[5])) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[33:79])
    defparam i10055_2_lut.init = 16'h8888;
    FD1P3IX mic_ram_wr_data_i0_i6 (.D(mic_ram_wr_data_15__N_2296[6]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i6.GSR = "DISABLED";
    LUT4 status_next_bit_I_0_i125_else_4_lut (.A(n26601), .B(n25614), .C(status_next_index[3]), 
         .D(status_next_index[2]), .Z(n27946)) /* synthesis lut_function=(A (B (C+(D))+!B (C))+!A !((C+!(D))+!B)) */ ;
    defparam status_next_bit_I_0_i125_else_4_lut.init = 16'haca0;
    LUT4 i15185_4_lut (.A(n17600), .B(n20519), .C(mic_run_state_1__N_2390[1]), 
         .D(pll_clk_enable_597), .Z(pll_clk_enable_531)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i15185_4_lut.init = 16'hfffb;
    LUT4 ev_run_hold_s5_83__I_0_i18_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_17), .D(ev_run_hold_s5[17]), .Z(us_tx_next[17])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i18_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i48_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_47), .D(ev_run_hold_s5[47]), .Z(us_tx_next[47])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i48_3_lut_4_lut.init = 16'hbf40;
    LUT4 i10059_2_lut (.A(mic_lo_n_next_6__N_642[1]), .B(n13), .Z(mic_lo_n_next[1])) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[33:79])
    defparam i10059_2_lut.init = 16'h8888;
    LUT4 n27088_bdd_4_lut (.A(n27088), .B(\mic_win_q[2] [6]), .C(\mic_win_i[2] [6]), 
         .D(mic_acc_word[0]), .Z(n27091)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27088_bdd_4_lut.init = 16'haad8;
    LUT4 i6866_3_lut (.A(n17636), .B(mic_cfg_gap[0]), .C(n11835), .Z(n17637)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6866_3_lut.init = 16'hcaca;
    LUT4 i4_4_lut_adj_183 (.A(spi_extension_length[11]), .B(spi_extension_length[13]), 
         .C(spi_extension_length[9]), .D(n6_adj_3234), .Z(n25411)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut_adj_183.init = 16'hfffe;
    LUT4 i6865_4_lut (.A(mic_cfg_start[0]), .B(n6827), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17636)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6865_4_lut.init = 16'hcac0;
    FD1P3IX mic_ram_wr_data_i0_i5 (.D(mic_ram_wr_data_15__N_2296[5]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i5.GSR = "DISABLED";
    LUT4 i9820_2_lut (.A(spi_byte_count[4]), .B(spi_byte_count[3]), .Z(n20565)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i9820_2_lut.init = 16'heeee;
    FD1P3IX mic_ram_wr_data_i0_i4 (.D(mic_ram_wr_data_15__N_2296[4]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i4.GSR = "DISABLED";
    LUT4 i1_4_lut_adj_184 (.A(spi_extension_length[15]), .B(spi_extension_length[10]), 
         .C(n6_adj_3285), .D(spi_extension_length[14]), .Z(n6_adj_3234)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_4_lut_adj_184.init = 16'hfffe;
    LUT4 i14773_3_lut_3_lut (.A(status_next_index[5]), .B(n27948), .C(n94), 
         .Z(n25715)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam i14773_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_2_lut_adj_185 (.A(mod_toggle_spi), .B(n31), .Z(mod_toggle_spi_N_2503)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_185.init = 16'h9999;
    LUT4 i1_2_lut_adj_186 (.A(mic_cfg_settle[0]), .B(n17), .Z(mic_cfg_settle_3__N_2122[0])) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_186.init = 16'h4444;
    LUT4 n133_bdd_2_lut_16343_4_lut (.A(fpga_time[14]), .B(fpga_time[12]), 
         .C(mic_boot_done), .D(mic_clk_c), .Z(n27822)) /* synthesis lut_function=(!(A (B (C+(D))+!B (D))+!A (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam n133_bdd_2_lut_16343_4_lut.init = 16'h007f;
    LUT4 mux_1611_i1_4_lut (.A(pcm_audio_miso_N_2767[0]), .B(n26518), .C(n21246), 
         .D(n982), .Z(pcm_audio_miso)) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(429[27] 432[114])
    defparam mux_1611_i1_4_lut.init = 16'hca0a;
    LUT4 i9780_3_lut (.A(mic_cfg_hold_spi[40]), .B(n16732), .C(mic_cfg_hold_spi[46]), 
         .Z(mic_cfg_width_w[0])) /* synthesis lut_function=(!(A (B (C))+!A (B+(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[35] 289[104])
    defparam i9780_3_lut.init = 16'h2b2b;
    LUT4 i5_3_lut (.A(mic_cfg_hold_spi[43]), .B(n10_adj_3264), .C(mic_cfg_hold_spi[45]), 
         .Z(n16732)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i5_3_lut.init = 16'hfefe;
    LUT4 i4_4_lut_adj_187 (.A(mic_cfg_hold_spi[44]), .B(mic_cfg_hold_spi[42]), 
         .C(mic_cfg_hold_spi[40]), .D(mic_cfg_hold_spi[41]), .Z(n10_adj_3264)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut_adj_187.init = 16'hfffe;
    LUT4 i15090_2_lut (.A(mic_acc_state[0]), .B(mic_acc_state[2]), .Z(mic_ram_we_N_2673)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i15090_2_lut.init = 16'h2222;
    LUT4 i1_2_lut_4_lut_adj_188 (.A(fpga_time[14]), .B(fpga_time[12]), .C(mic_boot_done), 
         .D(mic_ultrasonic_N_1894), .Z(n10677)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam i1_2_lut_4_lut_adj_188.init = 16'h8000;
    LUT4 i63_4_lut (.A(n40_adj_3280), .B(n38_adj_3216), .C(status_bit_index[8]), 
         .D(spi1_miso_N_2436), .Z(spi1_miso_N_2433)) /* synthesis lut_function=(!(A (B (C)+!B !((D)+!C))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(435[25] 438[29])
    defparam i63_4_lut.init = 16'h3a0a;
    LUT4 ws2812_settle_1__bdd_4_lut (.A(ws2812_settle_3__N_1697), .B(ws2812_settle[3]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(n28057)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam ws2812_settle_1__bdd_4_lut.init = 16'h0100;
    LUT4 i1_2_lut_3_lut_4_lut_adj_189 (.A(ev_ch_lo_3), .B(ev_ch_hi_10), 
         .C(ev_fall), .D(ev_wrap), .Z(n4_adj_3243)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(764[25:39])
    defparam i1_2_lut_3_lut_4_lut_adj_189.init = 16'h0080;
    LUT4 i3451_1_lut (.A(mic_acc_word[0]), .Z(mic_acc_word_2__N_2312[0])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(978[37:56])
    defparam i3451_1_lut.init = 16'h5555;
    LUT4 i1623_1_lut (.A(fifo_credit_wire[0]), .Z(fifo_depth_wire[0])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(397[37:63])
    defparam i1623_1_lut.init = 16'h5555;
    LUT4 mic_acc_word_2__bdd_4_lut_15858 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [6]), 
         .C(\mic_win_q[3] [6]), .D(mic_acc_word[0]), .Z(n27088)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15858.init = 16'he4aa;
    LUT4 status_next_bit_N_2754_2__bdd_4_lut_15959_4_lut (.A(status_next_index[2]), 
         .B(n26954), .C(n27061), .D(status_next_index[3]), .Z(n27220)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;
    defparam status_next_bit_N_2754_2__bdd_4_lut_15959_4_lut.init = 16'h55d8;
    LUT4 i2_3_lut_4_lut_adj_190 (.A(n10668), .B(ws2812_enable_N_2591), .C(frame_toggle_sync), 
         .D(frame_toggle_seen), .Z(pll_clk_enable_103)) /* synthesis lut_function=(A+(B+!(C (D)+!C !(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_190.init = 16'heffe;
    LUT4 i14917_2_lut_2_lut (.A(status_next_index[1]), .B(status_hold[74]), 
         .Z(n25777)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam i14917_2_lut_2_lut.init = 16'h4444;
    FD1P3IX mic_ram_wr_data_i0_i3 (.D(mic_ram_wr_data_15__N_2296[3]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i3.GSR = "DISABLED";
    LUT4 n27046_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[112]), 
         .C(status_hold[113]), .D(n27046), .Z(n27049)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27046_bdd_4_lut_4_lut.init = 16'hf588;
    FD1P3AX mic_sample_count_3055__i2 (.D(n23), .SP(pll_clk_enable_82), 
            .CK(pll_clk), .Q(mic_sample_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[37:60])
    defparam mic_sample_count_3055__i2.GSR = "DISABLED";
    FD1P3AX mic_sample_count_3055__i1 (.D(n24), .SP(pll_clk_enable_82), 
            .CK(pll_clk), .Q(mic_sample_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[37:60])
    defparam mic_sample_count_3055__i1.GSR = "DISABLED";
    FD1S3AX spi_rx_shift_i7 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(spi_rx_shift[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_rx_shift_i7.GSR = "ENABLED";
    LUT4 i1_2_lut_3_lut_adj_191 (.A(fpga_cs_n_c), .B(n22119), .C(n32_adj_3179), 
         .Z(spi1_sck_c_enable_72)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_191.init = 16'h4040;
    LUT4 n28185_bdd_3_lut (.A(n28185), .B(n28182), .C(status_bit_index[1]), 
         .Z(n28186)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n28185_bdd_3_lut.init = 16'hcaca;
    LUT4 spi_command_7__bdd_4_lut_16554 (.A(spi_command[6]), .B(spi_command[3]), 
         .C(spi_command[2]), .D(n25519), .Z(n28187)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;
    defparam spi_command_7__bdd_4_lut_16554.init = 16'hffef;
    LUT4 i2_2_lut_3_lut_3_lut (.A(mic_acc_busy_N_2282), .B(mic_gate_active), 
         .C(mic_acc_gate_6__N_2263), .Z(n17600)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(960[22] 966[16])
    defparam i2_2_lut_3_lut_3_lut.init = 16'h2020;
    LUT4 i1_3_lut_4_lut_adj_192 (.A(spi_command[0]), .B(spi_extension_length[1]), 
         .C(spi_extension_length[2]), .D(n16736), .Z(n25471)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i1_3_lut_4_lut_adj_192.init = 16'h0002;
    LUT4 i2_3_lut_4_lut_adj_193 (.A(spi_command[2]), .B(spi_command[3]), 
         .C(spi_command[1]), .D(spi_command[0]), .Z(n11_adj_3266)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(568[34:54])
    defparam i2_3_lut_4_lut_adj_193.init = 16'hfeff;
    LUT4 i2133_2_lut_3_lut_3_lut (.A(mic_acc_busy_N_2282), .B(mic_gate_active), 
         .C(mic_acc_gate_6__N_2263), .Z(n10819)) /* synthesis lut_function=(!(A (B (C))+!A (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(960[22] 966[16])
    defparam i2133_2_lut_3_lut_3_lut.init = 16'h2f2f;
    LUT4 i15221_2_lut_3_lut (.A(pll_clk_enable_102), .B(staging_rd_page_N_2719), 
         .C(phase_step_s3), .Z(n17742)) /* synthesis lut_function=(A ((C)+!B)) */ ;
    defparam i15221_2_lut_3_lut.init = 16'ha2a2;
    LUT4 mux_673_i15_3_lut_4_lut (.A(pending_sequence[14]), .B(accepted_sequence_spi[14]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[14])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i15_3_lut_4_lut.init = 16'hacaa;
    LUT4 i4259_3_lut_4_lut (.A(ev_ch_lo_3), .B(ev_ch_hi_10), .C(n13403), 
         .D(n13154), .Z(n15021)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(764[25:39])
    defparam i4259_3_lut_4_lut.init = 16'hff80;
    LUT4 i1_2_lut_3_lut_adj_194 (.A(ev_ch_lo_3), .B(ev_ch_hi_10), .C(n13403), 
         .Z(n16841)) /* synthesis lut_function=(!(A (B+!(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(764[25:39])
    defparam i1_2_lut_3_lut_adj_194.init = 16'h7070;
    LUT4 i1_2_lut_4_lut_adj_195 (.A(mic_cfg_settle[0]), .B(n17), .C(mic_cfg_toggle_seen), 
         .D(mic_cfg_toggle_sync), .Z(pll_clk_enable_93)) /* synthesis lut_function=(A+(B+!(C (D)+!C !(D)))) */ ;
    defparam i1_2_lut_4_lut_adj_195.init = 16'heffe;
    LUT4 i1_2_lut_3_lut_adj_196 (.A(n22373), .B(n119), .C(n104), .Z(pll_clk_enable_391)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_3_lut_adj_196.init = 16'h8080;
    LUT4 n27082_bdd_4_lut_else_3_lut (.A(n14018), .B(n14019), .C(status_bit_index[0]), 
         .Z(n28024)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27082_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 ev_run_hold_s5_83__I_0_i70_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_69), .D(ev_run_hold_s5[69]), .Z(us_tx_next[69])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i70_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_3_lut_adj_197 (.A(n22373), .B(n119), .C(mic_phase[0]), 
         .Z(pll_clk_enable_280)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_3_lut_adj_197.init = 16'h0808;
    LUT4 i1_2_lut_3_lut_adj_198 (.A(pll_clk_enable_597), .B(mic_acc_busy_N_2272), 
         .C(mic_done_N_2646), .Z(pll_clk_enable_262)) /* synthesis lut_function=(A+(B (C))) */ ;
    defparam i1_2_lut_3_lut_adj_198.init = 16'heaea;
    LUT4 i1_2_lut_4_lut_adj_199 (.A(n13_adj_3166), .B(n16910), .C(spi_command[2]), 
         .D(spi_command[3]), .Z(n16932)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(538[35:55])
    defparam i1_2_lut_4_lut_adj_199.init = 16'hfffe;
    LUT4 i15215_3_lut_4_lut (.A(n11_adj_3156), .B(pll_clk_enable_597), .C(mic_run_state_1__N_2390[1]), 
         .D(n10819), .Z(pll_clk_enable_85)) /* synthesis lut_function=((B+(C+!(D)))+!A) */ ;
    defparam i15215_3_lut_4_lut.init = 16'hfdff;
    LUT4 i1_2_lut_3_lut_adj_200 (.A(spi_command[0]), .B(spi_command[1]), 
         .C(n16894), .Z(spi_phase_only_N_2776)) /* synthesis lut_function=(((C)+!B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(151[34:64])
    defparam i1_2_lut_3_lut_adj_200.init = 16'hf7f7;
    LUT4 spi_bit_count_2__N_1415_bdd_4_lut_16367 (.A(spi_bit_count_2__N_1415), 
         .B(n21183), .C(n15_adj_3283), .D(n27765), .Z(compact_end)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;
    defparam spi_bit_count_2__N_1415_bdd_4_lut_16367.init = 16'h0200;
    LUT4 ev_run_hold_s5_83__I_0_i71_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_70), .D(ev_run_hold_s5[70]), .Z(us_tx_next[70])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i71_3_lut_4_lut.init = 16'hbf40;
    FD1P3IX mic_ram_wr_data_i0_i2 (.D(mic_ram_wr_data_15__N_2296[2]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i2.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i1 (.D(mic_ram_wr_data_15__N_2296[1]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i1.GSR = "DISABLED";
    LUT4 spi1_miso_N_2443_2__bdd_4_lut_4_lut (.A(status_bit_index[2]), .B(n27984), 
         .C(n27972), .D(status_bit_index[3]), .Z(n27190)) /* synthesis lut_function=(A (B (D))+!A (C+!(D))) */ ;
    defparam spi1_miso_N_2443_2__bdd_4_lut_4_lut.init = 16'hd855;
    LUT4 i1_2_lut_3_lut_adj_201 (.A(frame_end), .B(spi_bit_count_2__N_1415), 
         .C(fpga_cs_n_c), .Z(spi1_sck_c_enable_67)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i1_2_lut_3_lut_adj_201.init = 16'h0808;
    LUT4 i4361_3_lut_3_lut (.A(status_next_index[1]), .B(status_hold[71]), 
         .C(status_hold[69]), .Z(n15123)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam i4361_3_lut_3_lut.init = 16'he4e4;
    FD1P3IX mic_acc_state_i2 (.D(mic_acc_last), .SP(pll_clk_enable_85), 
            .CD(n27954), .CK(pll_clk), .Q(mic_acc_state[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_state_i2.GSR = "DISABLED";
    LUT4 ev_in_clear_I_0_2_lut (.A(ev_in_clear), .B(ev_fall_N_2683), .Z(ev_we)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(335[33:67])
    defparam ev_in_clear_I_0_2_lut.init = 16'heeee;
    LUT4 i1_2_lut_4_lut_adj_202 (.A(mic_warm), .B(mic_ultrasonic), .C(fpga_time[13]), 
         .D(fpga_time[11]), .Z(n17650)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i1_2_lut_4_lut_adj_202.init = 16'h1000;
    LUT4 i1_2_lut_4_lut_adj_203 (.A(mic_warm), .B(mic_ultrasonic), .C(fpga_time[13]), 
         .D(fpga_time[11]), .Z(pll_clk_enable_122)) /* synthesis lut_function=(A+!(B+!(C (D)))) */ ;
    defparam i1_2_lut_4_lut_adj_203.init = 16'hbaaa;
    LUT4 i6987_4_lut_then_1_lut (.A(pll_clk_enable_85), .Z(n27953)) /* synthesis lut_function=(A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i6987_4_lut_then_1_lut.init = 16'haaaa;
    PFUMX i4569 (.BLUT(n15330), .ALUT(n25802), .C0(ev_fall_N_2683), .Z(n15331));
    LUT4 n27196_bdd_4_lut_16155_4_lut (.A(status_next_index[1]), .B(status_hold[52]), 
         .C(status_hold[53]), .D(n27196), .Z(n27199)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27196_bdd_4_lut_16155_4_lut.init = 16'hf588;
    FD1P3IX mic_gate_fill_i6 (.D(mic_acc_last_N_2665[6]), .SP(pll_clk_enable_91), 
            .CD(n17753), .CK(pll_clk), .Q(mic_gate_fill[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_fill_i6.GSR = "DISABLED";
    LUT4 n27100_bdd_4_lut (.A(n27100), .B(\mic_win_q[0] [7]), .C(\mic_win_i[0] [7]), 
         .D(mic_acc_word[0]), .Z(n27103)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27100_bdd_4_lut.init = 16'haad8;
    LUT4 ev_run_hold_s5_83__I_0_i32_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_31), .D(ev_run_hold_s5[31]), .Z(us_tx_next[31])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i32_3_lut_4_lut.init = 16'hbf40;
    FD1S3AX spi_rx_shift_i6 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(spi_rx_shift[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_rx_shift_i6.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i5 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(spi_rx_shift[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_rx_shift_i5.GSR = "ENABLED";
    FD1P3IX mic_gate_fill_i5 (.D(mic_acc_last_N_2665[5]), .SP(pll_clk_enable_91), 
            .CD(n17753), .CK(pll_clk), .Q(mic_gate_fill[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_fill_i5.GSR = "DISABLED";
    FD1S3AX spi_rx_shift_i4 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(spi_rx_shift[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_rx_shift_i4.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i3 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(spi_rx_shift[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_rx_shift_i3.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i2 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(spi_rx_shift[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_rx_shift_i2.GSR = "ENABLED";
    LUT4 mic_acc_word_2__bdd_4_lut_15863 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [7]), 
         .C(\mic_win_q[1] [7]), .D(mic_acc_word[0]), .Z(n27100)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15863.init = 16'he4aa;
    LUT4 i6987_4_lut_else_1_lut (.A(pll_clk_enable_85), .B(mic_acc_state[1]), 
         .C(mic_acc_state[0]), .D(n21127), .Z(n27952)) /* synthesis lut_function=(!((B (C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i6987_4_lut_else_1_lut.init = 16'h2aaa;
    LUT4 ev_run_hold_s5_83__I_0_i17_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_16), .D(ev_run_hold_s5[16]), .Z(us_tx_next[16])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i17_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i72_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_71), .D(ev_run_hold_s5[71]), .Z(us_tx_next[71])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i72_3_lut_4_lut.init = 16'hbf40;
    LUT4 spi_command_7__bdd_4_lut_16732 (.A(spi_command[1]), .B(n16910), 
         .C(spi_extension_length[5]), .D(n28187), .Z(n28188)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam spi_command_7__bdd_4_lut_16732.init = 16'hfffe;
    LUT4 i6833_3_lut (.A(n17603), .B(mic_cfg_gap[1]), .C(n11835), .Z(n17604)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6833_3_lut.init = 16'hcaca;
    LUT4 i3000_2_lut_3_lut_3_lut (.A(mic_acc_state[0]), .B(mic_acc_state[1]), 
         .C(mic_acc_state[2]), .Z(n13399)) /* synthesis lut_function=(A (C)+!A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i3000_2_lut_3_lut_3_lut.init = 16'he1e1;
    LUT4 i13113_3_lut_4_lut (.A(pcm_rd_ptr_spi[1]), .B(pcm_wr_sync2_spi[1]), 
         .C(compact_end), .D(n16895), .Z(n23794)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[31:74])
    defparam i13113_3_lut_4_lut.init = 16'haaca;
    LUT4 i10515_4_lut_then_3_lut (.A(n21218), .B(spi_command[1]), .C(n16894), 
         .Z(n27956)) /* synthesis lut_function=(A+((C)+!B)) */ ;
    defparam i10515_4_lut_then_3_lut.init = 16'hfbfb;
    LUT4 ev_run_hold_s5_83__I_0_i49_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_48), .D(ev_run_hold_s5[48]), .Z(us_tx_next[48])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i49_3_lut_4_lut.init = 16'hbf40;
    FD1P3IX mic_gate_fill_i4 (.D(mic_acc_last_N_2665[4]), .SP(pll_clk_enable_91), 
            .CD(n17753), .CK(pll_clk), .Q(mic_gate_fill[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_fill_i4.GSR = "DISABLED";
    FD1P3IX mic_gate_fill_i3 (.D(mic_acc_last_N_2665[3]), .SP(pll_clk_enable_91), 
            .CD(n17753), .CK(pll_clk), .Q(mic_gate_fill[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_fill_i3.GSR = "DISABLED";
    CCU2D spi_channel_index_3042_add_4_5 (.A0(spi_channel_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24541), .COUT(n24542), .S0(n37_adj_3212), 
          .S1(n36_adj_3213));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042_add_4_5.INIT0 = 16'hfaaa;
    defparam spi_channel_index_3042_add_4_5.INIT1 = 16'hfaaa;
    defparam spi_channel_index_3042_add_4_5.INJECT1_0 = "NO";
    defparam spi_channel_index_3042_add_4_5.INJECT1_1 = "NO";
    LUT4 i6832_4_lut (.A(mic_cfg_start[1]), .B(n6826), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17603)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6832_4_lut.init = 16'hcac0;
    FD1P3IX mic_gate_fill_i2 (.D(mic_acc_last_N_2665[2]), .SP(pll_clk_enable_91), 
            .CD(n17753), .CK(pll_clk), .Q(mic_gate_fill[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_fill_i2.GSR = "DISABLED";
    FD1P3IX mic_gate_fill_i1 (.D(mic_acc_last_N_2665[1]), .SP(pll_clk_enable_91), 
            .CD(n17753), .CK(pll_clk), .Q(mic_gate_fill[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_fill_i1.GSR = "DISABLED";
    LUT4 n27364_bdd_4_lut (.A(n27364), .B(\mic_win_q[2] [1]), .C(\mic_win_i[2] [1]), 
         .D(mic_acc_word[0]), .Z(n27367)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27364_bdd_4_lut.init = 16'haad8;
    LUT4 i6835_3_lut (.A(n17605), .B(mic_cfg_gap[2]), .C(n11835), .Z(n17606)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6835_3_lut.init = 16'hcaca;
    LUT4 n27106_bdd_4_lut (.A(n27106), .B(\mic_win_q[2] [7]), .C(\mic_win_i[2] [7]), 
         .D(mic_acc_word[0]), .Z(n27109)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27106_bdd_4_lut.init = 16'haad8;
    CCU2D spi_channel_index_3042_add_4_3 (.A0(spi_channel_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24540), .COUT(n24541), .S0(n39_adj_3210), 
          .S1(n38_adj_3211));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042_add_4_3.INIT0 = 16'hfaaa;
    defparam spi_channel_index_3042_add_4_3.INIT1 = 16'hfaaa;
    defparam spi_channel_index_3042_add_4_3.INJECT1_0 = "NO";
    defparam spi_channel_index_3042_add_4_3.INJECT1_1 = "NO";
    FD1P3IX mic_run_state_i1 (.D(mic_run_state_1__N_2390[1]), .SP(pll_clk_enable_111), 
            .CD(mic_done_N_2640), .CK(pll_clk), .Q(mic_run_state[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_run_state_i1.GSR = "DISABLED";
    CCU2D add_2523_3 (.A0(\mic_acc_i[3] [0]), .B0(n20537), .C0(n4547), 
          .D0(GND_net), .A1(\mic_acc_i[3] [1]), .B1(n20537), .C1(n4547), 
          .D1(GND_net), .CIN(n24412), .COUT(n24413), .S0(n5689), .S1(n5688));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam add_2523_3.INIT0 = 16'h7484;
    defparam add_2523_3.INIT1 = 16'h7848;
    defparam add_2523_3.INJECT1_0 = "NO";
    defparam add_2523_3.INJECT1_1 = "NO";
    FD1P3JX mic_cfg_settle_i2 (.D(mic_cfg_settle_3__N_2122[2]), .SP(pll_clk_enable_93), 
            .PD(mic_cfg_settle_3__N_2126), .CK(pll_clk), .Q(mic_cfg_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_settle_i2.GSR = "DISABLED";
    LUT4 mic_acc_word_2__bdd_4_lut_15901 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [7]), 
         .C(\mic_win_q[3] [7]), .D(mic_acc_word[0]), .Z(n27106)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15901.init = 16'he4aa;
    LUT4 i2_2_lut_adj_204 (.A(spi_extension_length[8]), .B(spi_extension_length[12]), 
         .Z(n6_adj_3285)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i2_2_lut_adj_204.init = 16'heeee;
    LUT4 i6834_4_lut (.A(mic_cfg_start[2]), .B(n6825), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17605)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6834_4_lut.init = 16'hcac0;
    LUT4 i2_3_lut_4_lut_adj_205 (.A(mic_phase[3]), .B(n22373), .C(n16750), 
         .D(mic_ultrasonic), .Z(n22369)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i2_3_lut_4_lut_adj_205.init = 16'h0800;
    LUT4 ev_run_hold_s5_83__I_0_i50_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_49), .D(ev_run_hold_s5[49]), .Z(us_tx_next[49])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i50_3_lut_4_lut.init = 16'hbf40;
    LUT4 i6837_3_lut (.A(n17607), .B(mic_cfg_gap[3]), .C(n11835), .Z(n17608)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6837_3_lut.init = 16'hcaca;
    CCU2D add_2523_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20537), .B1(n4547), .C1(GND_net), .D1(GND_net), .COUT(n24412));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam add_2523_1.INIT0 = 16'hF000;
    defparam add_2523_1.INIT1 = 16'h7777;
    defparam add_2523_1.INJECT1_0 = "NO";
    defparam add_2523_1.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_206 (.A(n13_adj_3166), .B(spi1_sck_c_enable_67), .C(n8_adj_3157), 
         .D(spi_command[4]), .Z(spi1_sck_c_enable_68)) /* synthesis lut_function=(A (B)+!A (B (C+(D)))) */ ;
    defparam i1_4_lut_adj_206.init = 16'hccc8;
    LUT4 i3_3_lut (.A(spi_command[5]), .B(n11_adj_3266), .C(invalid_frame_spi_N_2526), 
         .Z(n8_adj_3157)) /* synthesis lut_function=(A+(B+!(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(568[34:54])
    defparam i3_3_lut.init = 16'hefef;
    LUT4 i15067_4_lut (.A(n16085), .B(n25471), .C(n27589), .D(n18), 
         .Z(invalid_frame_spi_N_2526)) /* synthesis lut_function=(!((B+(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(538[30] 570[24])
    defparam i15067_4_lut.init = 16'h0222;
    FD1P3JX ws2812_frames_left_i1 (.D(n16154), .SP(pll_clk_enable_94), .PD(ws2812_enable_N_2578), 
            .CK(pll_clk), .Q(ws2812_frames_left[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ws2812_frames_left_i1.GSR = "DISABLED";
    LUT4 i13115_3_lut_4_lut (.A(pcm_rd_ptr_spi[0]), .B(pcm_wr_sync2_spi[0]), 
         .C(compact_end), .D(n16895), .Z(n23796)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[31:74])
    defparam i13115_3_lut_4_lut.init = 16'haaca;
    FD1P3IX build_level_eff_i0_i7 (.D(event_level[7]), .SP(pll_clk_enable_102), 
            .CD(n17742), .CK(pll_clk), .Q(build_level_eff[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_eff_i0_i7.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_207 (.A(mic_warm), .B(mic_ultrasonic), .Z(n119)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_207.init = 16'h4444;
    PFUMX status_next_bit_I_0_i78 (.BLUT(n25765), .ALUT(n25766), .C0(status_next_bit_N_2754[3]), 
          .Z(n78_adj_3261));
    FD1P3IX build_level_eff_i0_i6 (.D(event_level[6]), .SP(pll_clk_enable_102), 
            .CD(n17742), .CK(pll_clk), .Q(build_level_eff[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_eff_i0_i6.GSR = "DISABLED";
    FD1P3IX build_level_eff_i0_i5 (.D(event_level[5]), .SP(pll_clk_enable_102), 
            .CD(n17742), .CK(pll_clk), .Q(build_level_eff[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_eff_i0_i5.GSR = "DISABLED";
    LUT4 i6836_4_lut (.A(mic_cfg_start[3]), .B(n6824), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17607)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6836_4_lut.init = 16'hcac0;
    LUT4 i13109_3_lut_4_lut (.A(pcm_rd_ptr_spi[3]), .B(pcm_wr_sync2_spi[3]), 
         .C(compact_end), .D(n16895), .Z(n23790)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[31:74])
    defparam i13109_3_lut_4_lut.init = 16'haaca;
    LUT4 ev_run_hold_s5_83__I_0_i30_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_29), .D(ev_run_hold_s5[29]), .Z(us_tx_next[29])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i30_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX mic_cfg_toggle_seen_1324 (.D(mic_cfg_toggle_sync), .SP(mic_cfg_settle_3__N_2126), 
            .CK(pll_clk), .Q(mic_cfg_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_toggle_seen_1324.GSR = "DISABLED";
    LUT4 i13111_3_lut_4_lut (.A(pcm_rd_ptr_spi[2]), .B(pcm_wr_sync2_spi[2]), 
         .C(compact_end), .D(n16895), .Z(n23792)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[31:74])
    defparam i13111_3_lut_4_lut.init = 16'haaca;
    LUT4 ev_run_hold_s5_83__I_0_i31_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_30), .D(ev_run_hold_s5[30]), .Z(us_tx_next[30])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i31_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX stop_toggle_seen_1298 (.D(stop_toggle_sync), .SP(stop_event), 
            .CK(pll_clk), .Q(stop_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam stop_toggle_seen_1298.GSR = "DISABLED";
    LUT4 i6903_3_lut (.A(pll_clk_enable_597), .B(n16730), .C(mic_cfg_hold_spi[6]), 
         .Z(n17672)) /* synthesis lut_function=(A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i6903_3_lut.init = 16'h8282;
    FD1P3AX mod_toggle_seen_1284 (.D(mod_toggle_sync), .SP(feed_rd_7__N_1692), 
            .CK(pll_clk), .Q(mod_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mod_toggle_seen_1284.GSR = "DISABLED";
    LUT4 mic_acc_word_2__bdd_4_lut_16083 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [1]), 
         .C(\mic_win_q[3] [1]), .D(mic_acc_word[0]), .Z(n27364)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16083.init = 16'he4aa;
    LUT4 i1_2_lut_adj_208 (.A(spi_extension_length[1]), .B(spi_extension_length[2]), 
         .Z(n16973)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(561[34:54])
    defparam i1_2_lut_adj_208.init = 16'heeee;
    FD1P3AX frame_toggle_seen_1276 (.D(frame_toggle_sync), .SP(ws2812_enable_N_2592), 
            .CK(pll_clk), .Q(frame_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam frame_toggle_seen_1276.GSR = "DISABLED";
    LUT4 n27202_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[56]), 
         .C(status_hold[57]), .D(n27202), .Z(n27205)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27202_bdd_4_lut_4_lut.init = 16'hf588;
    PFUMX i16456 (.BLUT(n28015), .ALUT(n28016), .C0(status_bit_index[1]), 
          .Z(n28017));
    FD1P3AX mod_toggle_spi_1227 (.D(mod_toggle_spi_N_2503), .SP(spi1_sck_c_enable_67), 
            .CK(spi1_sck_c), .Q(mod_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mod_toggle_spi_1227.GSR = "DISABLED";
    FD1P3IX build_level_eff_i0_i4 (.D(event_level[4]), .SP(pll_clk_enable_102), 
            .CD(n17742), .CK(pll_clk), .Q(build_level_eff[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_eff_i0_i4.GSR = "DISABLED";
    LUT4 i15229_3_lut_4_lut (.A(status_next_index[0]), .B(status_next_index[1]), 
         .C(status_next_index[3]), .D(status_next_index[2]), .Z(n24835)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam i15229_3_lut_4_lut.init = 16'h0800;
    LUT4 i6839_3_lut (.A(n17609), .B(mic_cfg_gap[4]), .C(n11835), .Z(n17610)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6839_3_lut.init = 16'hcaca;
    FD1P3AX mic_ultrasonic_1302 (.D(n28627), .SP(pll_clk_enable_99), .CK(pll_clk), 
            .Q(mic_ultrasonic)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ultrasonic_1302.GSR = "DISABLED";
    LUT4 i10515_4_lut_else_3_lut (.A(n21218), .B(n16932), .C(spi_command[1]), 
         .Z(n27955)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i10515_4_lut_else_3_lut.init = 16'hfefe;
    FD1P3AX ws2812_toggle_seen_1287 (.D(ws2812_toggle_sync), .SP(ws2812_settle_3__N_1697), 
            .CK(pll_clk), .Q(ws2812_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ws2812_toggle_seen_1287.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i73_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_72), .D(ev_run_hold_s5[72]), .Z(us_tx_next[72])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i73_3_lut_4_lut.init = 16'hbf40;
    LUT4 spi_command_7__bdd_3_lut_16734 (.A(spi_command[7]), .B(n25510), 
         .C(n28188), .Z(n16736)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam spi_command_7__bdd_3_lut_16734.init = 16'hfefe;
    FD1P3AX invalid_frame_spi_1229 (.D(invalid_frame_spi_N_2526), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(invalid_frame_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam invalid_frame_spi_1229.GSR = "DISABLED";
    FD1P3AX mod_level_spi_1226 (.D(spi_compact_data), .SP(spi1_sck_c_enable_69), 
            .CK(spi1_sck_c), .Q(mod_level_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mod_level_spi_1226.GSR = "DISABLED";
    FD1P3AX expected_valid_1218 (.D(n28627), .SP(spi1_sck_c_enable_70), 
            .CK(spi1_sck_c), .Q(expected_valid)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam expected_valid_1218.GSR = "ENABLED";
    LUT4 i6838_4_lut (.A(mic_cfg_start[4]), .B(n6823), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17609)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6838_4_lut.init = 16'hcac0;
    LUT4 ev_run_hold_s5_83__I_0_i16_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_15), .D(ev_run_hold_s5[15]), .Z(us_tx_next[15])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i16_3_lut_4_lut.init = 16'hbf40;
    LUT4 i5_3_lut_adj_209 (.A(mic_cfg_hold_spi[3]), .B(n10_adj_3161), .C(mic_cfg_hold_spi[5]), 
         .Z(n16730)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[35:66])
    defparam i5_3_lut_adj_209.init = 16'hfefe;
    L6MUX21 i16802 (.D0(n28600), .D1(n28597), .SD(status_bit_index[0]), 
            .Z(n28601));
    FD1P3AX spi_compact_data_1222 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_71), 
            .CK(spi1_sck_c), .Q(spi_compact_data)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_compact_data_1222.GSR = "DISABLED";
    FD1P3AX spi_channel_page_1220 (.D(spi_channel_page_N_2479), .SP(spi1_sck_c_enable_72), 
            .CK(spi1_sck_c), .Q(spi_channel_page)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam spi_channel_page_1220.GSR = "ENABLED";
    FD1P3AX mic_acc_gate__i1 (.D(mic_gate_index[0]), .SP(mic_acc_busy_N_2273), 
            .CK(pll_clk), .Q(mic_acc_gate[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_gate__i1.GSR = "DISABLED";
    FD1S3AX spi_byte_count_3044__i0 (.D(n50), .CK(spi1_sck_c), .Q(spi_byte_count[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044__i0.GSR = "ENABLED";
    LUT4 i6841_3_lut (.A(n17611), .B(mic_cfg_gap[5]), .C(n11835), .Z(n17612)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6841_3_lut.init = 16'hcaca;
    FD1S3AX spi_rgb_captured_3043__i0 (.D(n15_adj_3218), .CK(spi1_sck_c), 
            .Q(spi_rgb_captured[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(525[49:72])
    defparam spi_rgb_captured_3043__i0.GSR = "ENABLED";
    LUT4 i4_4_lut_adj_210 (.A(mic_cfg_hold_spi[4]), .B(mic_cfg_hold_spi[2]), 
         .C(mic_cfg_hold_spi[0]), .D(mic_cfg_hold_spi[1]), .Z(n10_adj_3161)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[35:66])
    defparam i4_4_lut_adj_210.init = 16'hfffe;
    LUT4 i3_4_lut_adj_211 (.A(spi_byte_count[4]), .B(n6_adj_3274), .C(spi_byte_count[5]), 
         .D(n15_adj_3283), .Z(spi1_sck_c_enable_70)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam i3_4_lut_adj_211.init = 16'h0040;
    LUT4 i6840_4_lut (.A(mic_cfg_start[5]), .B(n6822), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17611)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6840_4_lut.init = 16'hcac0;
    LUT4 i6843_3_lut (.A(n17613), .B(mic_cfg_gap[6]), .C(n11835), .Z(n17614)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6843_3_lut.init = 16'hcaca;
    LUT4 i2_2_lut_adj_212 (.A(n25414), .B(spi_byte_count[0]), .Z(n6_adj_3274)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam i2_2_lut_adj_212.init = 16'h8888;
    LUT4 i6842_4_lut (.A(mic_cfg_start[6]), .B(n6821), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17613)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6842_4_lut.init = 16'hcac0;
    LUT4 i3227_1_lut (.A(spi_extension_length[0]), .Z(n2)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(549[34:64])
    defparam i3227_1_lut.init = 16'h5555;
    LUT4 i4257_3_lut_4_lut (.A(ev_in_clear), .B(build_level_eff_7__N_1772[0]), 
         .C(build_idle_N_2714), .D(ev_sum[8]), .Z(n15019)) /* synthesis lut_function=(A (B (C+!(D))+!B !(D))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam i4257_3_lut_4_lut.init = 16'hc0ea;
    FD1P3IX build_level_eff_i0_i3 (.D(event_level[3]), .SP(pll_clk_enable_102), 
            .CD(n17742), .CK(pll_clk), .Q(build_level_eff[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_eff_i0_i3.GSR = "DISABLED";
    LUT4 i6845_3_lut (.A(n17615), .B(mic_cfg_gap[7]), .C(n11835), .Z(n17616)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6845_3_lut.init = 16'hcaca;
    LUT4 i6844_4_lut (.A(mic_cfg_start[7]), .B(n6820), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17615)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6844_4_lut.init = 16'hcac0;
    FD1P3IX build_level_eff_i0_i2 (.D(event_level[2]), .SP(pll_clk_enable_102), 
            .CD(n17742), .CK(pll_clk), .Q(build_level_eff[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_eff_i0_i2.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i74_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_73), .D(ev_run_hold_s5[73]), .Z(us_tx_next[73])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i74_3_lut_4_lut.init = 16'hbf40;
    PFUMX i16800 (.BLUT(n28599), .ALUT(n28598), .C0(status_bit_index[2]), 
          .Z(n28600));
    LUT4 ev_run_hold_s5_83__I_0_i75_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_74), .D(ev_run_hold_s5[74]), .Z(us_tx_next[74])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i75_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_adj_213 (.A(spi_extension_length[1]), .B(spi_extension_length[0]), 
         .Z(n16178)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_213.init = 16'h9999;
    LUT4 i6847_3_lut (.A(n17617), .B(mic_cfg_gap[8]), .C(n11835), .Z(n17618)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6847_3_lut.init = 16'hcaca;
    LUT4 i6846_4_lut (.A(mic_cfg_start[8]), .B(n6819), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17617)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6846_4_lut.init = 16'hcac0;
    LUT4 ev_run_hold_s5_83__I_0_i15_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_14), .D(ev_run_hold_s5[14]), .Z(us_tx_next[14])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i15_3_lut_4_lut.init = 16'hbf40;
    CCU2D spi_channel_index_3042_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(n22119), .B1(n25421), .C1(spi_channel_index[0]), 
          .D1(GND_net), .COUT(n24540), .S1(n40_adj_3209));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042_add_4_1.INIT0 = 16'hF000;
    defparam spi_channel_index_3042_add_4_1.INIT1 = 16'hd2d2;
    defparam spi_channel_index_3042_add_4_1.INJECT1_0 = "NO";
    defparam spi_channel_index_3042_add_4_1.INJECT1_1 = "NO";
    LUT4 i6849_3_lut (.A(n17619), .B(mic_cfg_gap[9]), .C(n11835), .Z(n17620)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6849_3_lut.init = 16'hcaca;
    LUT4 i6848_4_lut (.A(mic_cfg_start[9]), .B(n6818), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17619)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6848_4_lut.init = 16'hcac0;
    LUT4 i1_4_lut_adj_214 (.A(mic_phase[0]), .B(pcm_acc_next[2]), .C(n22386), 
         .D(n22373), .Z(n5388)) /* synthesis lut_function=(!(A+!(B (C+(D))+!B (C)))) */ ;
    defparam i1_4_lut_adj_214.init = 16'h5450;
    LUT4 i1_3_lut_4_lut_adj_215 (.A(mic_cfg_settle[0]), .B(n17), .C(mic_cfg_toggle_seen), 
         .D(mic_cfg_toggle_sync), .Z(pll_clk_enable_597)) /* synthesis lut_function=(!((B+!(C (D)+!C !(D)))+!A)) */ ;
    defparam i1_3_lut_4_lut_adj_215.init = 16'h2002;
    LUT4 i6_4_lut_adj_216 (.A(pcm_decim_count[2]), .B(n12_adj_3248), .C(n8_adj_3250), 
         .D(pcm_decim_count[1]), .Z(pcm_fifo_N_2613)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam i6_4_lut_adj_216.init = 16'h0040;
    PFUMX i16798 (.BLUT(n28596), .ALUT(n28595), .C0(status_bit_index[2]), 
          .Z(n28597));
    LUT4 ev_run_hold_s5_83__I_0_i76_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_75), .D(ev_run_hold_s5[75]), .Z(us_tx_next[75])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i76_3_lut_4_lut.init = 16'hbf40;
    LUT4 i6851_3_lut (.A(n17621), .B(mic_cfg_gap[10]), .C(n11835), .Z(n17622)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6851_3_lut.init = 16'hcaca;
    LUT4 n27376_bdd_4_lut (.A(n27376), .B(\mic_win_q[0] [1]), .C(\mic_win_i[0] [1]), 
         .D(mic_acc_word[0]), .Z(n27379)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27376_bdd_4_lut.init = 16'haad8;
    FD1S3AX spi_channel_index_3042__i0 (.D(n40_adj_3209), .CK(spi1_sck_c), 
            .Q(spi_channel_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042__i0.GSR = "ENABLED";
    FD1P3IX build_level_eff_i0_i1 (.D(event_level[1]), .SP(pll_clk_enable_102), 
            .CD(n17742), .CK(pll_clk), .Q(build_level_eff[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_eff_i0_i1.GSR = "DISABLED";
    LUT4 mic_acc_word_2__bdd_4_lut_16123 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [1]), 
         .C(\mic_win_q[1] [1]), .D(mic_acc_word[0]), .Z(n27376)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16123.init = 16'he4aa;
    FD1P3IX frame_settle__i3 (.D(n27527), .SP(pll_clk_enable_103), .CD(n10668), 
            .CK(pll_clk), .Q(frame_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam frame_settle__i3.GSR = "DISABLED";
    LUT4 i5_4_lut_adj_217 (.A(pcm_decim_count[3]), .B(pcm_decim_count[0]), 
         .C(pcm_decim_count[7]), .D(pcm_decim_count[5]), .Z(n12_adj_3248)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam i5_4_lut_adj_217.init = 16'h8000;
    LUT4 i1_2_lut_adj_218 (.A(pcm_decim_count[4]), .B(pcm_decim_count[6]), 
         .Z(n8_adj_3250)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam i1_2_lut_adj_218.init = 16'h8888;
    LUT4 n27058_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[44]), 
         .C(status_hold[45]), .D(n27058), .Z(n27061)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27058_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 i6850_4_lut (.A(mic_cfg_start[10]), .B(n6817), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17621)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6850_4_lut.init = 16'hcac0;
    LUT4 i6853_3_lut (.A(n17623), .B(mic_cfg_gap[11]), .C(n11835), .Z(n17624)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6853_3_lut.init = 16'hcaca;
    CCU2D spi_byte_count_3044_add_4_9 (.A0(spi_byte_count[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[8]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24538), .S0(n43_adj_3222), 
          .S1(n42_adj_3223));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044_add_4_9.INIT0 = 16'hfaaa;
    defparam spi_byte_count_3044_add_4_9.INIT1 = 16'hfaaa;
    defparam spi_byte_count_3044_add_4_9.INJECT1_0 = "NO";
    defparam spi_byte_count_3044_add_4_9.INJECT1_1 = "NO";
    LUT4 n133_bdd_2_lut_3_lut_4_lut (.A(mic_phase[3]), .B(mic_phase[0]), 
         .C(mic_phase[1]), .D(mic_phase[2]), .Z(n27823)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam n133_bdd_2_lut_3_lut_4_lut.init = 16'h0001;
    LUT4 i6852_4_lut (.A(mic_cfg_start[11]), .B(n6816), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17623)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6852_4_lut.init = 16'hcac0;
    LUT4 n27304_bdd_4_lut_then_3_lut (.A(n14042), .B(n14043), .C(status_bit_index[0]), 
         .Z(n27959)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27304_bdd_4_lut_then_3_lut.init = 16'hacac;
    FD1P3AX pcm_wr_ptr_3053__i0 (.D(n35_adj_3202), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(pcm_wr_ptr[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(855[39:56])
    defparam pcm_wr_ptr_3053__i0.GSR = "DISABLED";
    LUT4 n27094_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[48]), 
         .C(status_hold[49]), .D(n27094), .Z(n27097)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27094_bdd_4_lut_4_lut.init = 16'hf588;
    PFUMX mux_1619_i1 (.BLUT(n25721), .ALUT(n11_adj_3275), .C0(n21153), 
          .Z(pcm_audio_miso_N_2767[0]));
    LUT4 n27304_bdd_4_lut_else_3_lut (.A(n14026), .B(n14027), .C(status_bit_index[0]), 
         .Z(n27958)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27304_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 i6855_3_lut (.A(n17625), .B(mic_cfg_gap[12]), .C(n11835), .Z(n17626)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6855_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_adj_219 (.A(mic_phase[0]), .B(pcm_acc_next[3]), .C(n22386), 
         .D(n22373), .Z(n5387)) /* synthesis lut_function=(!(A+!(B (C+(D))+!B (C)))) */ ;
    defparam i1_4_lut_adj_219.init = 16'h5450;
    LUT4 i6854_4_lut (.A(mic_cfg_start[12]), .B(n6815), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17625)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6854_4_lut.init = 16'hcac0;
    LUT4 i1_4_lut_adj_220 (.A(mic_phase[0]), .B(pcm_acc_next[9]), .C(n22386), 
         .D(n22373), .Z(n5381)) /* synthesis lut_function=(!(A+!(B (C+(D))+!B (C)))) */ ;
    defparam i1_4_lut_adj_220.init = 16'h5450;
    LUT4 spi_command_5__bdd_4_lut_16756 (.A(spi_command[2]), .B(n9_adj_3178), 
         .C(spi_command[3]), .D(n28077), .Z(n28078)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam spi_command_5__bdd_4_lut_16756.init = 16'h1000;
    PFUMX i16452 (.BLUT(n28009), .ALUT(n28010), .C0(mic_done), .Z(n28011));
    LUT4 spi_command_7__I_0_1480_i9_2_lut (.A(spi_command[0]), .B(spi_command[1]), 
         .Z(n9_adj_3178)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(174[38:60])
    defparam spi_command_7__I_0_1480_i9_2_lut.init = 16'hbbbb;
    LUT4 i12_4_lut (.A(n13_adj_3286), .B(n10677), .C(mic_warm), .D(mic_ultrasonic), 
         .Z(pll_clk_enable_39)) /* synthesis lut_function=(A (B (C))+!A (B (C+(D))+!B !(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(807[18] 892[12])
    defparam i12_4_lut.init = 16'hc5c0;
    LUT4 i6857_3_lut (.A(n17627), .B(mic_cfg_gap[13]), .C(n11835), .Z(n17628)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6857_3_lut.init = 16'hcaca;
    LUT4 i6856_4_lut (.A(mic_cfg_start[13]), .B(n6814), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17627)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6856_4_lut.init = 16'hcac0;
    LUT4 i6859_3_lut (.A(n17629), .B(mic_cfg_gap[14]), .C(n11835), .Z(n17630)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6859_3_lut.init = 16'hcaca;
    LUT4 active_bank_I_0_1_lut (.A(active_bank), .Z(active_bank_N_892)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(333[34:46])
    defparam active_bank_I_0_1_lut.init = 16'h5555;
    LUT4 spi_command_5__bdd_4_lut_16490 (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .C(spi_command[4]), .D(n28076), .Z(n28077)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C (D)))) */ ;
    defparam spi_command_5__bdd_4_lut_16490.init = 16'hf0e0;
    LUT4 n27208_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[60]), 
         .C(status_hold[61]), .D(n27208), .Z(n27211)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27208_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 i1_2_lut_adj_221 (.A(mic_phase[3]), .B(mic_phase[0]), .Z(n127)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[17:26])
    defparam i1_2_lut_adj_221.init = 16'heeee;
    LUT4 i6858_4_lut (.A(mic_cfg_start[14]), .B(n6813), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17629)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6858_4_lut.init = 16'hcac0;
    PFUMX i16450 (.BLUT(n28006), .ALUT(n28007), .C0(status_bit_index[0]), 
          .Z(n28008));
    LUT4 i6861_3_lut (.A(n17631), .B(mic_cfg_gap[15]), .C(n11835), .Z(n17632)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6861_3_lut.init = 16'hcaca;
    LUT4 i6860_4_lut (.A(mic_cfg_start[15]), .B(n6812), .C(n17600), .D(mic_run_state_1__N_2390[1]), 
         .Z(n17631)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i6860_4_lut.init = 16'hcac0;
    LUT4 i15040_2_lut (.A(mic_cfg_settle[1]), .B(mic_cfg_settle[0]), .Z(n25947)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(285[17:31])
    defparam i15040_2_lut.init = 16'h9999;
    LUT4 ev_run_hold_s5_83__I_0_i77_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_76), .D(ev_run_hold_s5[76]), .Z(us_tx_next[76])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i77_3_lut_4_lut.init = 16'hbf40;
    LUT4 i3_4_lut_adj_222 (.A(mic_lo_n[3]), .B(mic_lo_n[4]), .C(mic_lo_n[2]), 
         .D(n25682), .Z(n13)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i3_4_lut_adj_222.init = 16'hfeff;
    LUT4 i15042_4_lut (.A(mic_cfg_settle[3]), .B(mic_cfg_settle[2]), .C(mic_cfg_settle[1]), 
         .D(mic_cfg_settle[0]), .Z(n25949)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(285[17:31])
    defparam i15042_4_lut.init = 16'haaa9;
    LUT4 i3460_2_lut_3_lut (.A(mic_acc_word[1]), .B(mic_acc_word[0]), .C(mic_acc_word[2]), 
         .Z(n6973)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(978[37:56])
    defparam i3460_2_lut_3_lut.init = 16'h7878;
    LUT4 i1_2_lut_adj_223 (.A(pll_clk_enable_280), .B(n25502), .Z(n17217)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(208[17:25])
    defparam i1_2_lut_adj_223.init = 16'heeee;
    LUT4 ev_run_hold_s5_83__I_0_i78_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_77), .D(ev_run_hold_s5[77]), .Z(us_tx_next[77])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i78_3_lut_4_lut.init = 16'hbf40;
    LUT4 i14741_4_lut (.A(mic_lo_n[0]), .B(mic_lo_n[1]), .C(mic_lo_n[5]), 
         .D(mic_lo_n[6]), .Z(n25682)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14741_4_lut.init = 16'h8000;
    LUT4 i10005_4_lut (.A(ev_rd_data[48]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[48])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10005_4_lut.init = 16'h1222;
    LUT4 ev_run_hold_s5_83__I_0_i79_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_78), .D(ev_run_hold_s5[78]), .Z(us_tx_next[78])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i79_3_lut_4_lut.init = 16'hbf40;
    LUT4 i4560_4_lut (.A(ev_sum[1]), .B(staging_q[1]), .C(staging_rd_page_N_2718), 
         .D(ev_in_clear), .Z(n15322)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam i4560_4_lut.init = 16'hcac0;
    LUT4 i2_4_lut_else_3_lut (.A(n10805), .B(n13399), .C(n10819), .D(n27630), 
         .Z(n27961)) /* synthesis lut_function=(!(A (B (C)+!B (C (D))))) */ ;
    defparam i2_4_lut_else_3_lut.init = 16'h5f7f;
    LUT4 n27310_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[16]), 
         .C(status_hold[17]), .D(n27310), .Z(n27313)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27310_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 ev_run_hold_s5_83__I_0_i14_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_13), .D(ev_run_hold_s5[13]), .Z(us_tx_next[13])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i14_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i80_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_79), .D(ev_run_hold_s5[79]), .Z(us_tx_next[79])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i80_3_lut_4_lut.init = 16'hbf40;
    LUT4 i10004_4_lut (.A(ev_rd_data[49]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[49])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10004_4_lut.init = 16'h1222;
    FD1P3JX mic_run_state_i0 (.D(mic_run_state_1__N_2390[0]), .SP(pll_clk_enable_111), 
            .PD(mic_done_N_2640), .CK(pll_clk), .Q(mic_run_state[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_run_state_i0.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i81_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_80), .D(ev_run_hold_s5[80]), .Z(us_tx_next[80])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i81_3_lut_4_lut.init = 16'hbf40;
    LUT4 i10003_4_lut (.A(ev_rd_data[50]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[50])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10003_4_lut.init = 16'h1222;
    LUT4 ev_run_hold_s5_83__I_0_i13_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_12), .D(ev_run_hold_s5[12]), .Z(us_tx_next[12])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i13_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX mic_cfg_hold_spi_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_76), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i3.GSR = "DISABLED";
    CCU2D add_2487_5 (.A0(\mic_acc_i[2] [2]), .B0(n20521), .C0(n4547), 
          .D0(GND_net), .A1(\mic_acc_i[2] [3]), .B1(n20521), .C1(n4547), 
          .D1(GND_net), .CIN(n24365), .COUT(n24366), .S0(n5679), .S1(n5678));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(843[17:43])
    defparam add_2487_5.INIT0 = 16'h7848;
    defparam add_2487_5.INIT1 = 16'h7848;
    defparam add_2487_5.INJECT1_0 = "NO";
    defparam add_2487_5.INJECT1_1 = "NO";
    PFUMX i16440 (.BLUT(n27991), .ALUT(n27992), .C0(status_bit_index[0]), 
          .Z(n27993));
    LUT4 i1_2_lut_adj_224 (.A(spi_byte_count[0]), .B(spi_byte_count[1]), 
         .Z(n16922)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(456[57:94])
    defparam i1_2_lut_adj_224.init = 16'heeee;
    LUT4 ev_run_hold_s5_83__I_0_i82_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_81), .D(ev_run_hold_s5[81]), .Z(us_tx_next[81])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i82_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27328_bdd_4_lut_16161_4_lut (.A(status_next_index[1]), .B(status_hold[20]), 
         .C(status_hold[21]), .D(n27328), .Z(n27331)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27328_bdd_4_lut_16161_4_lut.init = 16'hf588;
    LUT4 n27394_bdd_4_lut (.A(n27394), .B(n27331), .C(n27313), .D(status_next_bit_N_2754[3]), 
         .Z(n27397)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27394_bdd_4_lut.init = 16'haad8;
    OB us_tx_pad_68 (.I(us_tx_c_68), .O(us_tx[68]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_67 (.I(us_tx_c_67), .O(us_tx[67]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_66 (.I(us_tx_c_66), .O(us_tx[66]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_65 (.I(us_tx_c_65), .O(us_tx[65]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_64 (.I(us_tx_c_64), .O(us_tx[64]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_63 (.I(us_tx_c_63), .O(us_tx[63]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_62 (.I(us_tx_c_62), .O(us_tx[62]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_61 (.I(us_tx_c_61), .O(us_tx[61]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_60 (.I(us_tx_c_60), .O(us_tx[60]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_59 (.I(us_tx_c_59), .O(us_tx[59]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_58 (.I(us_tx_c_58), .O(us_tx[58]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_57 (.I(us_tx_c_57), .O(us_tx[57]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_56 (.I(us_tx_c_56), .O(us_tx[56]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_55 (.I(us_tx_c_55), .O(us_tx[55]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_54 (.I(us_tx_c_54), .O(us_tx[54]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_53 (.I(us_tx_c_53), .O(us_tx[53]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_52 (.I(us_tx_c_52), .O(us_tx[52]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_51 (.I(us_tx_c_51), .O(us_tx[51]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_50 (.I(us_tx_c_50), .O(us_tx[50]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_49 (.I(us_tx_c_49), .O(us_tx[49]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_48 (.I(us_tx_c_48), .O(us_tx[48]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_47 (.I(us_tx_c_47), .O(us_tx[47]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_46 (.I(us_tx_c_46), .O(us_tx[46]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_45 (.I(us_tx_c_45), .O(us_tx[45]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_44 (.I(us_tx_c_44), .O(us_tx[44]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_43 (.I(us_tx_c_43), .O(us_tx[43]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_42 (.I(us_tx_c_42), .O(us_tx[42]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_41 (.I(us_tx_c_41), .O(us_tx[41]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_40 (.I(us_tx_c_40), .O(us_tx[40]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_39 (.I(us_tx_c_39), .O(us_tx[39]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_38 (.I(us_tx_c_38), .O(us_tx[38]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_37 (.I(us_tx_c_37), .O(us_tx[37]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_36 (.I(us_tx_c_36), .O(us_tx[36]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_35 (.I(us_tx_c_35), .O(us_tx[35]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_34 (.I(us_tx_c_34), .O(us_tx[34]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_33 (.I(us_tx_c_33), .O(us_tx[33]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_32 (.I(us_tx_c_32), .O(us_tx[32]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_31 (.I(us_tx_c_31), .O(us_tx[31]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_30 (.I(us_tx_c_30), .O(us_tx[30]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_29 (.I(us_tx_c_29), .O(us_tx[29]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_28 (.I(us_tx_c_28), .O(us_tx[28]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_27 (.I(us_tx_c_27), .O(us_tx[27]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_26 (.I(us_tx_c_26), .O(us_tx[26]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_25 (.I(us_tx_c_25), .O(us_tx[25]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_24 (.I(us_tx_c_24), .O(us_tx[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_23 (.I(us_tx_c_23), .O(us_tx[23]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_22 (.I(us_tx_c_22), .O(us_tx[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_21 (.I(us_tx_c_21), .O(us_tx[21]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_20 (.I(us_tx_c_20), .O(us_tx[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_19 (.I(us_tx_c_19), .O(us_tx[19]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_18 (.I(us_tx_c_18), .O(us_tx[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_17 (.I(us_tx_c_17), .O(us_tx[17]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_16 (.I(us_tx_c_16), .O(us_tx[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_15 (.I(us_tx_c_15), .O(us_tx[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_14 (.I(us_tx_c_14), .O(us_tx[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_13 (.I(us_tx_c_13), .O(us_tx[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_12 (.I(us_tx_c_12), .O(us_tx[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_11 (.I(us_tx_c_11), .O(us_tx[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_10 (.I(us_tx_c_10), .O(us_tx[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_9 (.I(us_tx_c_9), .O(us_tx[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_8 (.I(us_tx_c_8), .O(us_tx[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_7 (.I(us_tx_c_7), .O(us_tx[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_6 (.I(us_tx_c_6), .O(us_tx[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_5 (.I(us_tx_c_5), .O(us_tx[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_4 (.I(us_tx_c_4), .O(us_tx[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_3 (.I(us_tx_c_3), .O(us_tx[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_2 (.I(us_tx_c_2), .O(us_tx[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_1 (.I(us_tx_c_1), .O(us_tx[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_0 (.I(us_tx_c_0), .O(us_tx[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB rgb_data_pad (.I(rgb_data_c), .O(rgb_data));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(90[24:32])
    OB mic_clk_pad (.I(mic_clk_c), .O(mic_clk));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(91[24:31])
    OB spi_mic_miso_pad (.I(spi_mic_miso_c), .O(spi_mic_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(96[24:36])
    IB fpga_clk_8m_pad (.I(fpga_clk_8m), .O(fpga_clk_8m_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(84[24:35])
    IB fpga_cs_n_pad (.I(fpga_cs_n), .O(fpga_cs_n_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(85[24:33])
    IB spi1_sck_pad (.I(spi1_sck), .O(spi1_sck_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(86[24:32])
    IB spi1_mosi_pad (.I(spi1_mosi), .O(spi1_mosi_c_0));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(87[24:33])
    IB mic_data_0_pad (.I(mic_data_0), .O(mic_data_0_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(92[24:34])
    IB mic_data_1_pad (.I(mic_data_1), .O(mic_data_1_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(93[24:34])
    IB spi_mic_cs_n_pad (.I(spi_mic_cs_n), .O(spi_mic_cs_n_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[24:36])
    IB spi_mic_sck_pad (.I(spi_mic_sck), .O(spi_mic_sck_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(95[24:35])
    LUT4 spi_frame_sequence_5__I_0_i1_3_lut (.A(mic_acc_word[0]), .B(status_bit_index[4]), 
         .C(n5), .Z(mic_ram_rd_addr[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i1_3_lut.init = 16'hcaca;
    LUT4 ev_run_hold_s5_83__I_0_i83_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_82), .D(ev_run_hold_s5[82]), .Z(us_tx_next[82])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i83_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX mic_cfg_hold_spi_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_76), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_76), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_76), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_84), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_84), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_84), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_84), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_84), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_84), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_84), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_84), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_92), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i16.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_92), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i17.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_92), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i18.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_92), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i19.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_92), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i20.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_92), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i21.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_92), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i22.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_92), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i23.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i24 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_100), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i24.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i25 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_100), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i26 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_100), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i26.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i27 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_100), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i27.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i28 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_100), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i28.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i29 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_100), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i29.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i30 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_100), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i30.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i31 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_100), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i31.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i32 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_108), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i32.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i33 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_108), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i33.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i34 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_108), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i34.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i35 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_108), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i35.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i36 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_108), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i36.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i37 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_108), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i37.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i38 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_108), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i38.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i39 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_108), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i39.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i40 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_115), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i40.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i41 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_115), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i41.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i42 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_115), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i42.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i43 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_115), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i43.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i44 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_115), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i44.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i45 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_115), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i45.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i46 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_115), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_hold_spi_i0_i46.GSR = "DISABLED";
    FD1S3AX global_phase_s2_3047__i1 (.D(n44_adj_3159), .CK(pll_clk), .Q(global_phase_s2[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047__i1.GSR = "DISABLED";
    LUT4 mux_673_i5_3_lut_4_lut (.A(pending_sequence[4]), .B(accepted_sequence_spi[4]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[4])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i5_3_lut_4_lut.init = 16'hacaa;
    FD1S3AX global_phase_s2_3047__i2 (.D(n43_adj_3170), .CK(pll_clk), .Q(global_phase_s2[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047__i2.GSR = "DISABLED";
    FD1S3AX global_phase_s2_3047__i3 (.D(n42_adj_3171), .CK(pll_clk), .Q(global_phase_s2[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047__i3.GSR = "DISABLED";
    FD1S3AX global_phase_s2_3047__i4 (.D(n41_adj_3172), .CK(pll_clk), .Q(global_phase_s2[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047__i4.GSR = "DISABLED";
    FD1S3AX global_phase_s2_3047__i5 (.D(n40_adj_3173), .CK(pll_clk), .Q(global_phase_s2[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047__i5.GSR = "DISABLED";
    FD1S3AX global_phase_s2_3047__i6 (.D(n39_adj_3174), .CK(pll_clk), .Q(global_phase_s2[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047__i6.GSR = "DISABLED";
    FD1S3AX global_phase_s2_3047__i7 (.D(n38_adj_3175), .CK(pll_clk), .Q(global_phase_s2[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047__i7.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(rgb_values[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i1.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(rgb_values[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i2.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(rgb_values[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i3.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(rgb_values[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i4.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(rgb_values[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i5.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(rgb_values[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i6.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(rgb_values[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i7.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_130), 
            .CK(spi1_sck_c), .Q(rgb_values[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i8.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_130), 
            .CK(spi1_sck_c), .Q(rgb_values[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i9.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_130), 
            .CK(spi1_sck_c), .Q(rgb_values[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i10.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_130), 
            .CK(spi1_sck_c), .Q(rgb_values[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i11.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_130), 
            .CK(spi1_sck_c), .Q(rgb_values[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i12.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_130), 
            .CK(spi1_sck_c), .Q(rgb_values[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i13.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_130), 
            .CK(spi1_sck_c), .Q(rgb_values[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i14.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_130), 
            .CK(spi1_sck_c), .Q(rgb_values[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i15.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(rgb_values[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i16.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(rgb_values[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i17.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(rgb_values[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i18.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(rgb_values[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i19.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(rgb_values[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i20.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(rgb_values[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i21.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(rgb_values[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i22.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(rgb_values[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_i0_i23.GSR = "DISABLED";
    FD1P3IX mod_settle__i1 (.D(n16190), .SP(frame_req_N_2700), .CD(n15455), 
            .CK(pll_clk), .Q(mod_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mod_settle__i1.GSR = "DISABLED";
    FD1P3IX mod_settle__i2 (.D(n16144), .SP(frame_req_N_2700), .CD(n15455), 
            .CK(pll_clk), .Q(mod_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mod_settle__i2.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i1 (.D(pcm_rd_ptr_spi[1]), .CK(pll_clk), .Q(pcm_rd_sync1[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync1_i1.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i2 (.D(pcm_rd_ptr_spi[2]), .CK(pll_clk), .Q(pcm_rd_sync1[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync1_i2.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i3 (.D(pcm_rd_ptr_spi[3]), .CK(pll_clk), .Q(pcm_rd_sync1[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync1_i3.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i4 (.D(pcm_rd_ptr_spi[4]), .CK(pll_clk), .Q(pcm_rd_sync1[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync1_i4.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i5 (.D(pcm_rd_ptr_spi[5]), .CK(pll_clk), .Q(pcm_rd_sync1[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync1_i5.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i1 (.D(pcm_rd_sync1[1]), .CK(pll_clk), .Q(pcm_rd_sync2[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync2_i1.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i2 (.D(pcm_rd_sync1[2]), .CK(pll_clk), .Q(pcm_rd_sync2[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync2_i2.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i3 (.D(pcm_rd_sync1[3]), .CK(pll_clk), .Q(pcm_rd_sync2[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync2_i3.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i4 (.D(pcm_rd_sync1[4]), .CK(pll_clk), .Q(pcm_rd_sync2[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync2_i4.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i5 (.D(pcm_rd_sync1[5]), .CK(pll_clk), .Q(pcm_rd_sync2[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(581[12] 584[8])
    defparam pcm_rd_sync2_i5.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i1 (.D(pcm_wr_ptr[1]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync1_spi_i1.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i2 (.D(pcm_wr_ptr[2]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync1_spi_i2.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i3 (.D(pcm_wr_ptr[3]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync1_spi_i3.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i4 (.D(pcm_wr_ptr[4]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync1_spi_i4.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i5 (.D(pcm_wr_ptr[5]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync1_spi_i5.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i1 (.D(pcm_wr_sync1_spi[1]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync2_spi_i1.GSR = "DISABLED";
    FD1P3IX mod_settle__i3 (.D(n3592), .SP(pll_clk_enable_112), .CD(n10668), 
            .CK(pll_clk), .Q(mod_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mod_settle__i3.GSR = "DISABLED";
    CCU2D add_2487_3 (.A0(\mic_acc_i[2] [0]), .B0(n20521), .C0(n4547), 
          .D0(GND_net), .A1(\mic_acc_i[2] [1]), .B1(n20521), .C1(n4547), 
          .D1(GND_net), .CIN(n24364), .COUT(n24365), .S0(n5681), .S1(n5680));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(843[17:43])
    defparam add_2487_3.INIT0 = 16'h7484;
    defparam add_2487_3.INIT1 = 16'h7848;
    defparam add_2487_3.INJECT1_0 = "NO";
    defparam add_2487_3.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3054__i1 (.D(n44_adj_3236), .SP(pll_clk_enable_391), 
            .CD(n23736), .CK(pll_clk), .Q(pcm_decim_count[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054__i1.GSR = "DISABLED";
    FD1P3IX pcm_decim_count_3054__i2 (.D(n43_adj_3237), .SP(pll_clk_enable_391), 
            .CD(n23736), .CK(pll_clk), .Q(pcm_decim_count[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054__i2.GSR = "DISABLED";
    LUT4 spi_frame_sequence_5__I_0_i2_3_lut (.A(mic_acc_word[1]), .B(status_bit_index[5]), 
         .C(n5), .Z(mic_ram_rd_addr[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i2_3_lut.init = 16'hcaca;
    CCU2D add_2487_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20521), .B1(n4547), .C1(GND_net), .D1(GND_net), .COUT(n24364));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(843[17:43])
    defparam add_2487_1.INIT0 = 16'hF000;
    defparam add_2487_1.INIT1 = 16'h7777;
    defparam add_2487_1.INJECT1_0 = "NO";
    defparam add_2487_1.INJECT1_1 = "NO";
    CCU2D add_1585_25 (.A0(phase_frac[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24325), .S0(phase_frac_sum[23]), .S1(phase_frac_sum[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_25.INIT0 = 16'h5aaa;
    defparam add_1585_25.INIT1 = 16'h0000;
    defparam add_1585_25.INJECT1_0 = "NO";
    defparam add_1585_25.INJECT1_1 = "NO";
    FD1S3AX pcm_wr_sync2_spi_i2 (.D(pcm_wr_sync1_spi[2]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync2_spi_i2.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i3 (.D(pcm_wr_sync1_spi[3]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync2_spi_i3.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i4 (.D(pcm_wr_sync1_spi[4]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync2_spi_i4.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i5 (.D(pcm_wr_sync1_spi[5]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_wr_sync2_spi_i5.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i1 (.D(pcm_rd_ptr_spi_5__N_1610[1]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_rd_ptr_spi_i1.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i2 (.D(pcm_rd_ptr_spi_5__N_1610[2]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_rd_ptr_spi_i2.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i3 (.D(pcm_rd_ptr_spi_5__N_1610[3]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_rd_ptr_spi_i3.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i4 (.D(pcm_rd_ptr_spi_5__N_1610[4]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_rd_ptr_spi_i4.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i5 (.D(pcm_rd_ptr_spi_5__N_1610[5]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(589[12] 596[8])
    defparam pcm_rd_ptr_spi_i5.GSR = "DISABLED";
    FD1S3AX phase_frac_i1 (.D(phase_frac_sum[1]), .CK(pll_clk), .Q(phase_frac[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i1.GSR = "DISABLED";
    FD1S3AX phase_frac_i2 (.D(phase_frac_sum[2]), .CK(pll_clk), .Q(phase_frac[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i2.GSR = "DISABLED";
    FD1S3AX phase_frac_i3 (.D(phase_frac_sum[3]), .CK(pll_clk), .Q(phase_frac[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i3.GSR = "DISABLED";
    FD1S3AX phase_frac_i4 (.D(phase_frac_sum[4]), .CK(pll_clk), .Q(phase_frac[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i4.GSR = "DISABLED";
    FD1S3AX phase_frac_i5 (.D(phase_frac_sum[5]), .CK(pll_clk), .Q(phase_frac[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i5.GSR = "DISABLED";
    FD1S3AX phase_frac_i6 (.D(phase_frac_sum[6]), .CK(pll_clk), .Q(phase_frac[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i6.GSR = "DISABLED";
    FD1S3AX phase_frac_i7 (.D(phase_frac_sum[7]), .CK(pll_clk), .Q(phase_frac[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i7.GSR = "DISABLED";
    FD1S3AX phase_frac_i8 (.D(phase_frac_sum[8]), .CK(pll_clk), .Q(phase_frac[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i8.GSR = "DISABLED";
    FD1S3AX phase_frac_i9 (.D(phase_frac_sum[9]), .CK(pll_clk), .Q(phase_frac[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i9.GSR = "DISABLED";
    FD1S3AX phase_frac_i10 (.D(phase_frac_sum[10]), .CK(pll_clk), .Q(phase_frac[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i10.GSR = "DISABLED";
    FD1S3AX phase_frac_i11 (.D(phase_frac_sum[11]), .CK(pll_clk), .Q(phase_frac[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i11.GSR = "DISABLED";
    FD1S3AX phase_frac_i12 (.D(phase_frac_sum[12]), .CK(pll_clk), .Q(phase_frac[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i12.GSR = "DISABLED";
    FD1S3AX phase_frac_i13 (.D(phase_frac_sum[13]), .CK(pll_clk), .Q(phase_frac[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i13.GSR = "DISABLED";
    FD1S3AX phase_frac_i14 (.D(phase_frac_sum[14]), .CK(pll_clk), .Q(phase_frac[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i14.GSR = "DISABLED";
    FD1S3AX phase_frac_i15 (.D(phase_frac_sum[15]), .CK(pll_clk), .Q(phase_frac[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i15.GSR = "DISABLED";
    FD1S3AX phase_frac_i16 (.D(phase_frac_sum[16]), .CK(pll_clk), .Q(phase_frac[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i16.GSR = "DISABLED";
    FD1S3AX phase_frac_i17 (.D(phase_frac_sum[17]), .CK(pll_clk), .Q(phase_frac[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i17.GSR = "DISABLED";
    FD1S3AX phase_frac_i18 (.D(phase_frac_sum[18]), .CK(pll_clk), .Q(phase_frac[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i18.GSR = "DISABLED";
    FD1S3AX phase_frac_i19 (.D(phase_frac_sum[19]), .CK(pll_clk), .Q(phase_frac[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i19.GSR = "DISABLED";
    FD1S3AX phase_frac_i20 (.D(phase_frac_sum[20]), .CK(pll_clk), .Q(phase_frac[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i20.GSR = "DISABLED";
    FD1S3AX phase_frac_i21 (.D(phase_frac_sum[21]), .CK(pll_clk), .Q(phase_frac[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i21.GSR = "DISABLED";
    FD1S3AX phase_frac_i22 (.D(phase_frac_sum[22]), .CK(pll_clk), .Q(phase_frac[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i22.GSR = "DISABLED";
    FD1S3AX phase_frac_i23 (.D(phase_frac_sum[23]), .CK(pll_clk), .Q(phase_frac[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam phase_frac_i23.GSR = "DISABLED";
    FD1P3IX mic_phase__i1 (.D(n5636), .SP(pll_clk_enable_124), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_phase[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_phase__i1.GSR = "DISABLED";
    CCU2D pcm_decim_count_3054_add_4_9 (.A0(pcm_decim_count[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24360), .S0(n38_adj_3242));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054_add_4_9.INIT0 = 16'hfaaa;
    defparam pcm_decim_count_3054_add_4_9.INIT1 = 16'h0000;
    defparam pcm_decim_count_3054_add_4_9.INJECT1_0 = "NO";
    defparam pcm_decim_count_3054_add_4_9.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3054__i3 (.D(n42_adj_3238), .SP(pll_clk_enable_391), 
            .CD(n23736), .CK(pll_clk), .Q(pcm_decim_count[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054__i3.GSR = "DISABLED";
    CCU2D pcm_decim_count_3054_add_4_7 (.A0(pcm_decim_count[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(pcm_decim_count[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24359), .COUT(n24360), .S0(n40_adj_3240), 
          .S1(n39_adj_3241));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054_add_4_7.INIT0 = 16'hfaaa;
    defparam pcm_decim_count_3054_add_4_7.INIT1 = 16'hfaaa;
    defparam pcm_decim_count_3054_add_4_7.INJECT1_0 = "NO";
    defparam pcm_decim_count_3054_add_4_7.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3054__i4 (.D(n41_adj_3239), .SP(pll_clk_enable_391), 
            .CD(n23736), .CK(pll_clk), .Q(pcm_decim_count[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054__i4.GSR = "DISABLED";
    CCU2D pcm_decim_count_3054_add_4_5 (.A0(pcm_decim_count[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(pcm_decim_count[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24358), .COUT(n24359), .S0(n42_adj_3238), 
          .S1(n41_adj_3239));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054_add_4_5.INIT0 = 16'hfaaa;
    defparam pcm_decim_count_3054_add_4_5.INIT1 = 16'hfaaa;
    defparam pcm_decim_count_3054_add_4_5.INJECT1_0 = "NO";
    defparam pcm_decim_count_3054_add_4_5.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3054__i5 (.D(n40_adj_3240), .SP(pll_clk_enable_391), 
            .CD(n23736), .CK(pll_clk), .Q(pcm_decim_count[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054__i5.GSR = "DISABLED";
    CCU2D pcm_decim_count_3054_add_4_3 (.A0(pcm_decim_count[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(pcm_decim_count[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24357), .COUT(n24358), .S0(n44_adj_3236), 
          .S1(n43_adj_3237));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054_add_4_3.INIT0 = 16'hfaaa;
    defparam pcm_decim_count_3054_add_4_3.INIT1 = 16'hfaaa;
    defparam pcm_decim_count_3054_add_4_3.INJECT1_0 = "NO";
    defparam pcm_decim_count_3054_add_4_3.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3054__i6 (.D(n39_adj_3241), .SP(pll_clk_enable_391), 
            .CD(n23736), .CK(pll_clk), .Q(pcm_decim_count[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054__i6.GSR = "DISABLED";
    CCU2D pcm_decim_count_3054_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_decim_count[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24357), .S1(n45_adj_3235));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054_add_4_1.INIT0 = 16'hF000;
    defparam pcm_decim_count_3054_add_4_1.INIT1 = 16'h0555;
    defparam pcm_decim_count_3054_add_4_1.INJECT1_0 = "NO";
    defparam pcm_decim_count_3054_add_4_1.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3054__i7 (.D(n38_adj_3242), .SP(pll_clk_enable_391), 
            .CD(n23736), .CK(pll_clk), .Q(pcm_decim_count[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054__i7.GSR = "DISABLED";
    CCU2D status_bit_index_8__I_0_1363_9 (.A0(status_bit_index[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[8]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24355), .S0(status_bit_index_8__N_1377[7]), 
          .S1(status_bit_index_8__N_1377[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(415[37:60])
    defparam status_bit_index_8__I_0_1363_9.INIT0 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1363_9.INIT1 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1363_9.INJECT1_0 = "NO";
    defparam status_bit_index_8__I_0_1363_9.INJECT1_1 = "NO";
    FD1P3IX ev_ch_3051__i0 (.D(n40_adj_3183), .SP(pll_clk_enable_121), .CD(n17726), 
            .CK(pll_clk), .Q(ev_ch[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(770[30:42])
    defparam ev_ch_3051__i0.GSR = "DISABLED";
    CCU2D status_bit_index_8__I_0_1363_7 (.A0(status_bit_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24354), .COUT(n24355), .S0(status_next_index[5]), 
          .S1(status_next_index[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(415[37:60])
    defparam status_bit_index_8__I_0_1363_7.INIT0 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1363_7.INIT1 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1363_7.INJECT1_0 = "NO";
    defparam status_bit_index_8__I_0_1363_7.INJECT1_1 = "NO";
    FD1P3IX mic_warm_half_3052__i0 (.D(n45_adj_3194), .SP(pll_clk_enable_122), 
            .CD(n20290), .CK(pll_clk), .Q(mic_warm_half[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam mic_warm_half_3052__i0.GSR = "DISABLED";
    CCU2D status_bit_index_8__I_0_1363_5 (.A0(status_bit_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24353), .COUT(n24354), .S0(status_next_index[3]), 
          .S1(status_next_index[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(415[37:60])
    defparam status_bit_index_8__I_0_1363_5.INIT0 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1363_5.INIT1 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1363_5.INJECT1_0 = "NO";
    defparam status_bit_index_8__I_0_1363_5.INJECT1_1 = "NO";
    FD1P3AX ws2812_toggle_spi_1234 (.D(ws2812_toggle_spi_N_2555), .SP(spi1_sck_c_enable_144), 
            .CK(spi1_sck_c), .Q(ws2812_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam ws2812_toggle_spi_1234.GSR = "DISABLED";
    FD1P3AX mic_cfg_toggle_spi_1235 (.D(mic_cfg_toggle_spi_N_2681), .SP(spi1_sck_c_enable_145), 
            .CK(spi1_sck_c), .Q(mic_cfg_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam mic_cfg_toggle_spi_1235.GSR = "DISABLED";
    FD1P3IX mic_phase__i2 (.D(n5635), .SP(pll_clk_enable_124), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_phase[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_phase__i2.GSR = "DISABLED";
    FD1P3IX mic_phase__i3 (.D(n5634), .SP(pll_clk_enable_124), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_phase[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_phase__i3.GSR = "DISABLED";
    FD1P3IX link_idle__i1 (.D(n3184), .SP(pll_clk_enable_141), .CD(cs_fall), 
            .CK(pll_clk), .Q(link_idle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam link_idle__i1.GSR = "DISABLED";
    PFUMX i16438 (.BLUT(n27988), .ALUT(n27989), .C0(status_bit_index[0]), 
          .Z(n27990));
    LUT4 spi_frame_sequence_5__I_0_i3_3_lut (.A(mic_acc_word[2]), .B(status_bit_index[6]), 
         .C(n5), .Z(mic_ram_rd_addr[2])) /* synthesis lut_function=(!(A (B (C))+!A (B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i3_3_lut.init = 16'h3a3a;
    FD1P3IX link_idle__i2 (.D(n3183), .SP(pll_clk_enable_141), .CD(cs_fall), 
            .CK(pll_clk), .Q(link_idle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam link_idle__i2.GSR = "DISABLED";
    FD1P3IX link_idle__i3 (.D(n3182), .SP(pll_clk_enable_141), .CD(cs_fall), 
            .CK(pll_clk), .Q(link_idle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam link_idle__i3.GSR = "DISABLED";
    LUT4 spi_frame_sequence_5__I_0_i4_3_lut (.A(mic_acc_gate[0]), .B(spi_frame_sequence[0]), 
         .C(n5), .Z(mic_ram_rd_addr[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i4_3_lut.init = 16'hcaca;
    LUT4 ev_run_hold_s5_83__I_0_i84_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_83), .D(ev_run_hold_s5[83]), .Z(us_tx_next[83])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i84_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX frame_toggle_spi_1230 (.D(frame_toggle_spi_N_2508), .SP(spi1_sck_c_enable_146), 
            .CK(spi1_sck_c), .Q(frame_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam frame_toggle_spi_1230.GSR = "DISABLED";
    CCU2D status_bit_index_8__I_0_1363_3 (.A0(status_bit_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24352), .COUT(n24353), .S0(status_next_index[1]), 
          .S1(status_next_index[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(415[37:60])
    defparam status_bit_index_8__I_0_1363_3.INIT0 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1363_3.INIT1 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1363_3.INJECT1_0 = "NO";
    defparam status_bit_index_8__I_0_1363_3.INJECT1_1 = "NO";
    LUT4 n27370_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[24]), 
         .C(status_hold[25]), .D(n27370), .Z(n27373)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27370_bdd_4_lut_4_lut.init = 16'hf588;
    FD1P3AX mic_cfg_start_i0_i1 (.D(mic_cfg_hold_spi[9]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i2 (.D(mic_cfg_hold_spi[10]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i3 (.D(mic_cfg_hold_spi[11]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i4 (.D(mic_cfg_hold_spi[12]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i5 (.D(mic_cfg_hold_spi[13]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i6 (.D(mic_cfg_hold_spi[14]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i7 (.D(mic_cfg_hold_spi[15]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i8 (.D(mic_cfg_hold_spi[16]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i9 (.D(mic_cfg_hold_spi[17]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i10 (.D(mic_cfg_hold_spi[18]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i11 (.D(mic_cfg_hold_spi[19]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i12 (.D(mic_cfg_hold_spi[20]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i13 (.D(mic_cfg_hold_spi[21]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i14 (.D(mic_cfg_hold_spi[22]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i15 (.D(mic_cfg_hold_spi[23]), .SP(pll_clk_enable_597), 
            .CK(pll_clk), .Q(mic_cfg_start[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_start_i0_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i1 (.D(spi_frame_sequence[1]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i1.GSR = "DISABLED";
    LUT4 spi_frame_sequence_5__I_0_i5_3_lut (.A(mic_acc_gate[1]), .B(spi_frame_sequence[1]), 
         .C(n5), .Z(mic_ram_rd_addr[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i5_3_lut.init = 16'hcaca;
    LUT4 spi_frame_sequence_5__I_0_i6_3_lut (.A(mic_acc_gate[2]), .B(spi_frame_sequence[2]), 
         .C(n5), .Z(mic_ram_rd_addr[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i6_3_lut.init = 16'hcaca;
    LUT4 spi_frame_sequence_5__I_0_i7_3_lut (.A(mic_acc_gate[3]), .B(spi_frame_sequence[3]), 
         .C(n5), .Z(mic_ram_rd_addr[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i7_3_lut.init = 16'hcaca;
    LUT4 spi_frame_sequence_5__I_0_i8_3_lut (.A(mic_acc_gate[4]), .B(spi_frame_sequence[4]), 
         .C(n5), .Z(mic_ram_rd_addr[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i8_3_lut.init = 16'hcaca;
    CCU2D spi_byte_count_3044_add_4_7 (.A0(spi_byte_count[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24537), .COUT(n24538), .S0(n45_adj_3220), 
          .S1(n44_adj_3221));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044_add_4_7.INIT0 = 16'hfaaa;
    defparam spi_byte_count_3044_add_4_7.INIT1 = 16'hfaaa;
    defparam spi_byte_count_3044_add_4_7.INJECT1_0 = "NO";
    defparam spi_byte_count_3044_add_4_7.INJECT1_1 = "NO";
    CCU2D add_2532_9 (.A0(\mic_acc_q[3] [6]), .B0(n20537), .C0(n4576), 
          .D0(GND_net), .A1(\mic_acc_q[3] [7]), .B1(n20537), .C1(n4576), 
          .D1(GND_net), .CIN(n24407), .S0(n5716), .S1(n5715));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam add_2532_9.INIT0 = 16'h7848;
    defparam add_2532_9.INIT1 = 16'h7848;
    defparam add_2532_9.INJECT1_0 = "NO";
    defparam add_2532_9.INJECT1_1 = "NO";
    LUT4 spi_frame_sequence_5__I_0_i9_3_lut (.A(mic_acc_gate[5]), .B(spi_frame_sequence[5]), 
         .C(n5), .Z(mic_ram_rd_addr[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i9_3_lut.init = 16'hcaca;
    CCU2D spi_byte_count_3044_add_4_5 (.A0(spi_byte_count[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24536), .COUT(n24537), .S0(n47), 
          .S1(n46));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044_add_4_5.INIT0 = 16'hfaaa;
    defparam spi_byte_count_3044_add_4_5.INIT1 = 16'hfaaa;
    defparam spi_byte_count_3044_add_4_5.INJECT1_0 = "NO";
    defparam spi_byte_count_3044_add_4_5.INJECT1_1 = "NO";
    LUT4 i10002_4_lut (.A(ev_rd_data[51]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[51])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10002_4_lut.init = 16'h1222;
    LUT4 i1679_2_lut_3_lut_3_lut (.A(ev_fall), .B(ev_fall_N_2683), .C(ev_wrap), 
         .Z(n10061)) /* synthesis lut_function=(A (B (C))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(754[21:29])
    defparam i1679_2_lut_3_lut_3_lut.init = 16'hc4c4;
    LUT4 i10001_4_lut (.A(ev_rd_data[52]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[52])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10001_4_lut.init = 16'h1222;
    LUT4 i15_3_lut_4_lut_4_lut (.A(ev_fall), .B(pll_clk_enable_538), .C(ev_fall_N_2683), 
         .D(ev_wrap), .Z(pll_clk_enable_121)) /* synthesis lut_function=(!(A (B (C (D))+!B ((D)+!C))+!A ((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(754[21:29])
    defparam i15_3_lut_4_lut_4_lut.init = 16'h0cac;
    LUT4 ev_run_hold_s5_83__I_0_i12_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_11), .D(ev_run_hold_s5[11]), .Z(us_tx_next[11])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i12_3_lut_4_lut.init = 16'hbf40;
    LUT4 i2_2_lut_3_lut_4_lut (.A(mic_phase[3]), .B(mic_phase[0]), .C(mic_phase[1]), 
         .D(n23736), .Z(pll_clk_enable_602)) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i2_2_lut_3_lut_4_lut.init = 16'hfe00;
    LUT4 i14943_2_lut_2_lut (.A(ev_fall), .B(ev_sum[7]), .Z(n25806)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(754[21:29])
    defparam i14943_2_lut_2_lut.init = 16'h4444;
    LUT4 i14939_2_lut_2_lut (.A(ev_fall), .B(ev_sum[2]), .Z(n25798)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(754[21:29])
    defparam i14939_2_lut_2_lut.init = 16'h4444;
    LUT4 mux_3310_i1_3_lut (.A(n13999), .B(n14000), .C(n13998), .Z(rd_data_7__N_2897[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3310_i1_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_adj_225 (.A(mic_phase[2]), .B(mic_phase[1]), .Z(n22373)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_225.init = 16'h2222;
    LUT4 ev_run_hold_s5_83__I_0_i11_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_10), .D(ev_run_hold_s5[10]), .Z(us_tx_next[10])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i11_3_lut_4_lut.init = 16'hbf40;
    LUT4 i8_4_lut_adj_226 (.A(n8_adj_3158), .B(n25672), .C(n25584), .D(n13997), 
         .Z(n13998)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i8_4_lut_adj_226.init = 16'h0100;
    LUT4 i15155_2_lut_3_lut_4_lut (.A(spi_byte_count[0]), .B(n7), .C(spi_byte_count[2]), 
         .D(spi_byte_count[1]), .Z(spi1_sck_c_enable_92)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i15155_2_lut_3_lut_4_lut.init = 16'h4000;
    LUT4 i1_2_lut_adj_227 (.A(mic_phase[3]), .B(mic_phase[0]), .Z(n104)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_227.init = 16'h2222;
    LUT4 i14940_2_lut_2_lut (.A(ev_fall), .B(ev_sum[3]), .Z(n25800)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(754[21:29])
    defparam i14940_2_lut_2_lut.init = 16'h4444;
    LUT4 i14941_2_lut_2_lut (.A(ev_fall), .B(ev_sum[5]), .Z(n25802)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(754[21:29])
    defparam i14941_2_lut_2_lut.init = 16'h4444;
    LUT4 ev_run_hold_s5_83__I_0_i10_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_9), .D(ev_run_hold_s5[9]), .Z(us_tx_next[9])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i10_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX accepted_sequence_spi_i0_i2 (.D(spi_frame_sequence[2]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i3 (.D(spi_frame_sequence[3]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i4 (.D(spi_frame_sequence[4]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i5 (.D(spi_frame_sequence[5]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i6 (.D(spi_frame_sequence[6]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i7 (.D(spi_frame_sequence[7]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i8 (.D(spi_frame_sequence[8]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i9 (.D(spi_frame_sequence[9]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i10 (.D(spi_frame_sequence[10]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i11 (.D(spi_frame_sequence[11]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i12 (.D(spi_frame_sequence[12]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i13 (.D(spi_frame_sequence[13]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i14 (.D(spi_frame_sequence[14]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i15 (.D(spi_frame_sequence[15]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_sequence_spi_i0_i15.GSR = "DISABLED";
    FD1S3AX time_divider_3048__i1 (.D(n34), .CK(pll_clk), .Q(time_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(637[25:44])
    defparam time_divider_3048__i1.GSR = "DISABLED";
    LUT4 i10330_2_lut_3_lut_4_lut (.A(n13), .B(n10677), .C(mic_warm), 
         .D(mic_lo_n_next_6__N_642[4]), .Z(n10724)) /* synthesis lut_function=(!((B (C+!(D))+!B !(D))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(828[18] 892[12])
    defparam i10330_2_lut_3_lut_4_lut.init = 16'h2a00;
    LUT4 n27388_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[28]), 
         .C(status_hold[29]), .D(n27388), .Z(n27391)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27388_bdd_4_lut_4_lut.init = 16'hf588;
    CCU2D spi_byte_count_3044_add_4_3 (.A0(spi_byte_count[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24535), .COUT(n24536), .S0(n49), 
          .S1(n48));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044_add_4_3.INIT0 = 16'hfaaa;
    defparam spi_byte_count_3044_add_4_3.INIT1 = 16'hfaaa;
    defparam spi_byte_count_3044_add_4_3.INJECT1_0 = "NO";
    defparam spi_byte_count_3044_add_4_3.INJECT1_1 = "NO";
    LUT4 spi_byte_count_8__I_0_i11_2_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .Z(n8_adj_3177)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(456[57:94])
    defparam spi_byte_count_8__I_0_i11_2_lut.init = 16'heeee;
    FD1P3AX stop_toggle_spi_1233 (.D(stop_toggle_spi_N_2519), .SP(spi1_sck_c_enable_162), 
            .CK(spi1_sck_c), .Q(stop_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam stop_toggle_spi_1233.GSR = "DISABLED";
    CCU2D status_bit_index_8__I_0_1363_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24352), .S1(status_next_index[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(415[37:60])
    defparam status_bit_index_8__I_0_1363_1.INIT0 = 16'hF000;
    defparam status_bit_index_8__I_0_1363_1.INIT1 = 16'h5555;
    defparam status_bit_index_8__I_0_1363_1.INJECT1_0 = "NO";
    defparam status_bit_index_8__I_0_1363_1.INJECT1_1 = "NO";
    FD1P3IX rgb_values_complete_spi_1232 (.D(n16394), .SP(spi1_sck_c_enable_164), 
            .CD(n17720), .CK(spi1_sck_c), .Q(rgb_values_complete_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam rgb_values_complete_spi_1232.GSR = "DISABLED";
    FD1P3JX build_mod_1295 (.D(mod_active), .SP(pll_clk_enable_538), .PD(n17719), 
            .CK(pll_clk), .Q(build_mod)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_mod_1295.GSR = "DISABLED";
    CCU2D feed_rd_3050_add_4_9 (.A0(n23771), .B0(frame_req_N_2700), .C0(feed_rd[6]), 
          .D0(n21096), .A1(n23769), .B1(frame_req_N_2700), .C1(feed_rd[7]), 
          .D1(n21096), .CIN(n24350), .S0(n58), .S1(n57));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050_add_4_9.INIT0 = 16'h5a9a;
    defparam feed_rd_3050_add_4_9.INIT1 = 16'h5a9a;
    defparam feed_rd_3050_add_4_9.INJECT1_0 = "NO";
    defparam feed_rd_3050_add_4_9.INJECT1_1 = "NO";
    FD1P3JX mic_lo_i_r_1305 (.D(mic_lo_i_r_N_2629), .SP(pll_clk_enable_159), 
            .PD(n22401), .CK(pll_clk), .Q(mic_lo_i_r)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_lo_i_r_1305.GSR = "DISABLED";
    CCU2D feed_rd_3050_add_4_7 (.A0(n23775), .B0(frame_req_N_2700), .C0(feed_rd[4]), 
          .D0(n21096), .A1(n23773), .B1(frame_req_N_2700), .C1(feed_rd[5]), 
          .D1(n21096), .CIN(n24349), .COUT(n24350), .S0(n60), .S1(n59));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050_add_4_7.INIT0 = 16'h5a9a;
    defparam feed_rd_3050_add_4_7.INIT1 = 16'h5a9a;
    defparam feed_rd_3050_add_4_7.INJECT1_0 = "NO";
    defparam feed_rd_3050_add_4_7.INJECT1_1 = "NO";
    LUT4 i14942_2_lut_2_lut (.A(ev_fall), .B(ev_sum[6]), .Z(n25804)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(754[21:29])
    defparam i14942_2_lut_2_lut.init = 16'h4444;
    FD1P3JX mic_lo_q_r_1306 (.D(mic_lo_q_next), .SP(pll_clk_enable_159), 
            .PD(n22401), .CK(pll_clk), .Q(mic_lo_q_r)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_lo_q_r_1306.GSR = "DISABLED";
    CCU2D feed_rd_3050_add_4_5 (.A0(n23779), .B0(frame_req_N_2700), .C0(feed_rd[2]), 
          .D0(n21096), .A1(n23777), .B1(frame_req_N_2700), .C1(feed_rd[3]), 
          .D1(n21096), .CIN(n24348), .COUT(n24349), .S0(n62), .S1(n61));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050_add_4_5.INIT0 = 16'h5a9a;
    defparam feed_rd_3050_add_4_5.INIT1 = 16'h5a9a;
    defparam feed_rd_3050_add_4_5.INJECT1_0 = "NO";
    defparam feed_rd_3050_add_4_5.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i1 (.D(n84), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i1.GSR = "DISABLED";
    CCU2D feed_rd_3050_add_4_3 (.A0(n23783), .B0(frame_req_N_2700), .C0(feed_rd[0]), 
          .D0(n21096), .A1(n23781), .B1(frame_req_N_2700), .C1(feed_rd[1]), 
          .D1(n21096), .CIN(n24347), .COUT(n24348), .S0(n64), .S1(n63));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050_add_4_3.INIT0 = 16'h5a9a;
    defparam feed_rd_3050_add_4_3.INIT1 = 16'h5a9a;
    defparam feed_rd_3050_add_4_3.INJECT1_0 = "NO";
    defparam feed_rd_3050_add_4_3.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i2 (.D(n83), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i2.GSR = "DISABLED";
    CCU2D feed_rd_3050_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(frame_req_N_2700), .B1(n21096), .C1(GND_net), 
          .D1(GND_net), .COUT(n24347));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050_add_4_1.INIT0 = 16'hF000;
    defparam feed_rd_3050_add_4_1.INIT1 = 16'hffff;
    defparam feed_rd_3050_add_4_1.INJECT1_0 = "NO";
    defparam feed_rd_3050_add_4_1.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i3 (.D(n82), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i3.GSR = "DISABLED";
    LUT4 n27424_bdd_4_lut (.A(n27424), .B(\mic_win_q[2] [2]), .C(\mic_win_i[2] [2]), 
         .D(mic_acc_word[0]), .Z(n27427)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27424_bdd_4_lut.init = 16'haad8;
    LUT4 i2_3_lut_4_lut_adj_228 (.A(n10677), .B(mic_warm), .C(mic_ultrasonic), 
         .D(mic_run_state_1__N_2390[1]), .Z(pll_clk_enable_124)) /* synthesis lut_function=(A (B+(C+(D)))+!A (B (D)+!B (C+(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_228.init = 16'hffb8;
    LUT4 mic_acc_word_2__bdd_4_lut (.A(mic_acc_word[2]), .B(\mic_win_i[3] [2]), 
         .C(\mic_win_q[3] [2]), .D(mic_acc_word[0]), .Z(n27424)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut.init = 16'he4aa;
    LUT4 i14938_2_lut_2_lut (.A(ev_fall), .B(ev_sum[1]), .Z(n25796)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(754[21:29])
    defparam i14938_2_lut_2_lut.init = 16'h4444;
    LUT4 n27400_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[116]), 
         .C(status_hold[117]), .D(n27400), .Z(n27403)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27400_bdd_4_lut_4_lut.init = 16'hf588;
    FD1P3IX mic_block_count_3057__i4 (.D(n81), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i4.GSR = "DISABLED";
    CCU2D add_2456_8 (.A0(spi1_mosi_c_0), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24343), .S0(n11895));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[33] 449[86])
    defparam add_2456_8.INIT0 = 16'h5aaa;
    defparam add_2456_8.INIT1 = 16'h0000;
    defparam add_2456_8.INJECT1_0 = "NO";
    defparam add_2456_8.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i5 (.D(n80), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i5.GSR = "DISABLED";
    CCU2D add_2456_6 (.A0(spi_extension_length[6]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_extension_length[7]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24342), .COUT(n24343), .S0(n11899), .S1(n11897));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[33] 449[86])
    defparam add_2456_6.INIT0 = 16'h5aaa;
    defparam add_2456_6.INIT1 = 16'h5aaa;
    defparam add_2456_6.INJECT1_0 = "NO";
    defparam add_2456_6.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i6 (.D(n79), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i6.GSR = "DISABLED";
    CCU2D add_2456_4 (.A0(spi_extension_length[4]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_extension_length[5]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24341), .COUT(n24342), .S0(n11903), .S1(n11901));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[33] 449[86])
    defparam add_2456_4.INIT0 = 16'h5aaa;
    defparam add_2456_4.INIT1 = 16'h5aaa;
    defparam add_2456_4.INJECT1_0 = "NO";
    defparam add_2456_4.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i7 (.D(n78), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i7.GSR = "DISABLED";
    CCU2D add_2456_2 (.A0(spi_extension_length[2]), .B0(expected_next_8__N_1396[3]), 
          .C0(GND_net), .D0(GND_net), .A1(spi_extension_length[3]), .B1(expected_next_8__N_1396[3]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24341), .S1(n11905));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[33] 449[86])
    defparam add_2456_2.INIT0 = 16'h7000;
    defparam add_2456_2.INIT1 = 16'h5666;
    defparam add_2456_2.INJECT1_0 = "NO";
    defparam add_2456_2.INJECT1_1 = "NO";
    LUT4 i10329_2_lut_3_lut_4_lut (.A(n13), .B(n10677), .C(mic_warm), 
         .D(mic_lo_n_next_6__N_642[3]), .Z(n10723)) /* synthesis lut_function=(!((B (C+!(D))+!B !(D))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(828[18] 892[12])
    defparam i10329_2_lut_3_lut_4_lut.init = 16'h2a00;
    LUT4 equal_3299_i8_2_lut (.A(n13994), .B(n13995), .Z(n8_adj_3158)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam equal_3299_i8_2_lut.init = 16'h6666;
    LUT4 i14731_4_lut (.A(n25582), .B(n13986), .C(n25560), .D(n13987), 
         .Z(n25672)) /* synthesis lut_function=(A+(B (C+!(D))+!B (C+(D)))) */ ;
    defparam i14731_4_lut.init = 16'hfbfe;
    FD1P3IX mic_block_count_3057__i8 (.D(n77), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i8.GSR = "DISABLED";
    FD1P3IX mic_block_count_3057__i9 (.D(n76), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i9.GSR = "DISABLED";
    CCU2D mic_lo_n_6__I_0_7 (.A0(mic_lo_n[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_lo_n[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24338), .S0(mic_lo_n_next_6__N_642[5]), 
          .S1(mic_lo_n_next_6__N_642[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[62:79])
    defparam mic_lo_n_6__I_0_7.INIT0 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_7.INIT1 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_7.INJECT1_0 = "NO";
    defparam mic_lo_n_6__I_0_7.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i10 (.D(n75), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[10])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i10.GSR = "DISABLED";
    CCU2D mic_lo_n_6__I_0_5 (.A0(mic_lo_n[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_lo_n[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24337), .COUT(n24338), .S0(mic_lo_n_next_6__N_642[3]), 
          .S1(mic_lo_n_next_6__N_642[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[62:79])
    defparam mic_lo_n_6__I_0_5.INIT0 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_5.INIT1 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_5.INJECT1_0 = "NO";
    defparam mic_lo_n_6__I_0_5.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i11 (.D(n74), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[11])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i11.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i9_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_8), .D(ev_run_hold_s5[8]), .Z(us_tx_next[8])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i9_3_lut_4_lut.init = 16'hbf40;
    FD1S3AX time_divider_3048__i2 (.D(n33), .CK(pll_clk), .Q(time_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(637[25:44])
    defparam time_divider_3048__i2.GSR = "DISABLED";
    FD1S3AX time_divider_3048__i3 (.D(n32), .CK(pll_clk), .Q(time_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(637[25:44])
    defparam time_divider_3048__i3.GSR = "DISABLED";
    FD1S3AX time_divider_3048__i4 (.D(n31_adj_3176), .CK(pll_clk), .Q(time_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(637[25:44])
    defparam time_divider_3048__i4.GSR = "DISABLED";
    FD1S3AX time_divider_3048__i5 (.D(n30), .CK(pll_clk), .Q(time_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(637[25:44])
    defparam time_divider_3048__i5.GSR = "DISABLED";
    FD1P3AX accepted_update_flags_spi_i0_i1 (.D(expected_next_8__N_1396[3]), 
            .SP(spi1_sck_c_enable_164), .CK(spi1_sck_c), .Q(accepted_update_flags_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam accepted_update_flags_spi_i0_i1.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_229 (.A(mic_phase[2]), .B(mic_warm), .Z(n4_adj_3165)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_229.init = 16'hdddd;
    LUT4 i1_2_lut_adj_230 (.A(pcm_fifo_N_2613), .B(pll_clk_enable_391), 
         .Z(n23736)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_230.init = 16'h8888;
    CCU2D mic_lo_n_6__I_0_3 (.A0(mic_lo_n[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_lo_n[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24336), .COUT(n24337), .S0(mic_lo_n_next_6__N_642[1]), 
          .S1(mic_lo_n_next_6__N_642[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[62:79])
    defparam mic_lo_n_6__I_0_3.INIT0 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_3.INIT1 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_3.INJECT1_0 = "NO";
    defparam mic_lo_n_6__I_0_3.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i12 (.D(n73), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[12])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i12.GSR = "DISABLED";
    CCU2D mic_lo_n_6__I_0_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_lo_n[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24336), .S1(mic_lo_n_next_6__N_642[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[62:79])
    defparam mic_lo_n_6__I_0_1.INIT0 = 16'hF000;
    defparam mic_lo_n_6__I_0_1.INIT1 = 16'h5555;
    defparam mic_lo_n_6__I_0_1.INJECT1_0 = "NO";
    defparam mic_lo_n_6__I_0_1.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i13 (.D(n72), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[13])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i13.GSR = "DISABLED";
    CCU2D pcm_acc_9__I_0_11 (.A0(pcm_acc[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24335), .S0(pcm_acc_next[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_11.INIT0 = 16'h5aaa;
    defparam pcm_acc_9__I_0_11.INIT1 = 16'h0000;
    defparam pcm_acc_9__I_0_11.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_11.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3057__i14 (.D(n71), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[14])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i14.GSR = "DISABLED";
    FD1P3IX mic_block_count_3057__i15 (.D(n70), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[15])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i15.GSR = "DISABLED";
    LUT4 i10328_2_lut_3_lut_4_lut (.A(n13), .B(n10677), .C(mic_warm), 
         .D(mic_lo_n_next_6__N_642[2]), .Z(n10722)) /* synthesis lut_function=(!((B (C+!(D))+!B !(D))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(828[18] 892[12])
    defparam i10328_2_lut_3_lut_4_lut.init = 16'h2a00;
    LUT4 spi_byte_count_0__bdd_4_lut (.A(n25414), .B(fpga_cs_n_c), .C(n16_adj_3282), 
         .D(spi_byte_count[1]), .Z(n27574)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;
    defparam spi_byte_count_0__bdd_4_lut.init = 16'h0200;
    LUT4 n27406_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[32]), 
         .C(status_hold[33]), .D(n27406), .Z(n26946)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27406_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 i2_3_lut_4_lut_adj_231 (.A(n22373), .B(n104), .C(pcm_fifo_N_2613), 
         .D(n119), .Z(n19096)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i2_3_lut_4_lut_adj_231.init = 16'h8000;
    LUT4 i10406_2_lut (.A(status_bit_index[7]), .B(status_bit_index[8]), 
         .Z(n21153)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10406_2_lut.init = 16'heeee;
    CCU2D pcm_acc_9__I_0_9 (.A0(pcm_acc[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_acc[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24334), .COUT(n24335), .S0(pcm_acc_next[7]), 
          .S1(pcm_acc_next[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_9.INIT0 = 16'h5aaa;
    defparam pcm_acc_9__I_0_9.INIT1 = 16'h5aaa;
    defparam pcm_acc_9__I_0_9.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_9.INJECT1_1 = "NO";
    FD1P3IX pcm_read_count_latched_spi_i0_i1 (.D(pcm_count_spi[1]), .SP(spi1_sck_c_enable_169), 
            .CD(n17693), .CK(spi1_sck_c), .Q(pcm_read_count_latched_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam pcm_read_count_latched_spi_i0_i1.GSR = "DISABLED";
    CCU2D pcm_acc_9__I_0_7 (.A0(pcm_acc[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_acc[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24333), .COUT(n24334), .S0(pcm_acc_next[5]), 
          .S1(pcm_acc_next[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_7.INIT0 = 16'h5aaa;
    defparam pcm_acc_9__I_0_7.INIT1 = 16'h5aaa;
    defparam pcm_acc_9__I_0_7.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_7.INJECT1_1 = "NO";
    CCU2D pcm_acc_9__I_0_5 (.A0(pcm_acc[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_acc[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24332), .COUT(n24333), .S0(pcm_acc_next[3]), 
          .S1(pcm_acc_next[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_5.INIT0 = 16'h5aaa;
    defparam pcm_acc_9__I_0_5.INIT1 = 16'h5aaa;
    defparam pcm_acc_9__I_0_5.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_5.INJECT1_1 = "NO";
    CCU2D pcm_acc_9__I_0_3 (.A0(pcm_acc[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_acc[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24331), .COUT(n24332), .S0(pcm_acc_next[1]), 
          .S1(pcm_acc_next[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_3.INIT0 = 16'h5aaa;
    defparam pcm_acc_9__I_0_3.INIT1 = 16'h5aaa;
    defparam pcm_acc_9__I_0_3.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_3.INJECT1_1 = "NO";
    LUT4 i1_2_lut_4_lut_adj_232 (.A(n13_adj_3166), .B(n16910), .C(spi_command[2]), 
         .D(spi_command[3]), .Z(n16907)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(174[38:60])
    defparam i1_2_lut_4_lut_adj_232.init = 16'hffef;
    CCU2D pcm_acc_9__I_0_1 (.A0(mic_data_1_c), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_acc[0]), .B1(mic_data_0_c), .C1(GND_net), 
          .D1(GND_net), .COUT(n24331), .S1(pcm_acc_next[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_1.INIT0 = 16'h5000;
    defparam pcm_acc_9__I_0_1.INIT1 = 16'h5666;
    defparam pcm_acc_9__I_0_1.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_1.INJECT1_1 = "NO";
    FD1P3JX ws2812_settle_i0_i0 (.D(n1), .SP(pll_clk_enable_178), .PD(ws2812_settle_3__N_1697), 
            .CK(pll_clk), .Q(ws2812_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ws2812_settle_i0_i0.GSR = "DISABLED";
    CCU2D add_1586_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24330), 
          .S0(ev_sum[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[26:68])
    defparam add_1586_cout.INIT0 = 16'h0000;
    defparam add_1586_cout.INIT1 = 16'h0000;
    defparam add_1586_cout.INJECT1_0 = "NO";
    defparam add_1586_cout.INJECT1_1 = "NO";
    CCU2D add_1586_8 (.A0(rmw_slot[6]), .B0(build_level_eff[6]), .C0(GND_net), 
          .D0(GND_net), .A1(rmw_slot[7]), .B1(build_level_eff[7]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24329), .COUT(n24330), .S0(ev_sum[6]), 
          .S1(ev_sum[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[26:68])
    defparam add_1586_8.INIT0 = 16'h5666;
    defparam add_1586_8.INIT1 = 16'h5666;
    defparam add_1586_8.INJECT1_0 = "NO";
    defparam add_1586_8.INJECT1_1 = "NO";
    CCU2D add_1586_6 (.A0(rmw_slot[4]), .B0(build_level_eff[4]), .C0(GND_net), 
          .D0(GND_net), .A1(rmw_slot[5]), .B1(build_level_eff[5]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24328), .COUT(n24329), .S0(ev_sum[4]), 
          .S1(ev_sum[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[26:68])
    defparam add_1586_6.INIT0 = 16'h5666;
    defparam add_1586_6.INIT1 = 16'h5666;
    defparam add_1586_6.INJECT1_0 = "NO";
    defparam add_1586_6.INJECT1_1 = "NO";
    LUT4 i14643_4_lut (.A(n13980), .B(n13982), .C(n13981), .D(n13983), 
         .Z(n25584)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14643_4_lut.init = 16'h7bde;
    LUT4 i13455_2_lut_3_lut (.A(n22373), .B(n104), .C(mic_sample_count[0]), 
         .Z(n24170)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i13455_2_lut_3_lut.init = 16'h8080;
    FD1P3IX mic_clock_reg_1299 (.D(mic_clk_N_2454), .SP(pll_clk_enable_180), 
            .CD(mic_warm_half_7__N_1887), .CK(pll_clk), .Q(mic_clk_c)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_clock_reg_1299.GSR = "DISABLED";
    CCU2D add_1586_4 (.A0(rmw_slot[2]), .B0(build_level_eff[2]), .C0(GND_net), 
          .D0(GND_net), .A1(rmw_slot[3]), .B1(build_level_eff[3]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24327), .COUT(n24328), .S0(ev_sum[2]), 
          .S1(ev_sum[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[26:68])
    defparam add_1586_4.INIT0 = 16'h5666;
    defparam add_1586_4.INIT1 = 16'h5666;
    defparam add_1586_4.INJECT1_0 = "NO";
    defparam add_1586_4.INJECT1_1 = "NO";
    FD1P3IX pcm_read_count_latched_spi_i0_i2 (.D(pcm_count_spi[2]), .SP(spi1_sck_c_enable_169), 
            .CD(n17693), .CK(spi1_sck_c), .Q(pcm_read_count_latched_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam pcm_read_count_latched_spi_i0_i2.GSR = "DISABLED";
    CCU2D add_1586_2 (.A0(rmw_slot[0]), .B0(build_level_eff[0]), .C0(GND_net), 
          .D0(GND_net), .A1(rmw_slot[1]), .B1(build_level_eff[1]), .C1(GND_net), 
          .D1(GND_net), .COUT(n24327), .S1(ev_sum[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[26:68])
    defparam add_1586_2.INIT0 = 16'h7000;
    defparam add_1586_2.INIT1 = 16'h5666;
    defparam add_1586_2.INJECT1_0 = "NO";
    defparam add_1586_2.INJECT1_1 = "NO";
    FD1P3IX pcm_read_count_latched_spi_i0_i3 (.D(pcm_count_spi[3]), .SP(spi1_sck_c_enable_169), 
            .CD(n17693), .CK(spi1_sck_c), .Q(pcm_read_count_latched_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam pcm_read_count_latched_spi_i0_i3.GSR = "DISABLED";
    CCU2D add_1585_21 (.A0(phase_frac[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24323), .COUT(n24324), .S0(phase_frac_sum[19]), 
          .S1(phase_frac_sum[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_21.INIT0 = 16'h5555;
    defparam add_1585_21.INIT1 = 16'h5aaa;
    defparam add_1585_21.INJECT1_0 = "NO";
    defparam add_1585_21.INJECT1_1 = "NO";
    LUT4 wrap_s2_I_0_1567_2_lut (.A(wrap_s2), .B(swap_pending), .Z(active_bank_N_894)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(614[13:36])
    defparam wrap_s2_I_0_1567_2_lut.init = 16'h8888;
    LUT4 i14949_2_lut_2_lut (.A(ev_fall), .B(ev_sum[4]), .Z(n25784)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(754[21:29])
    defparam i14949_2_lut_2_lut.init = 16'h4444;
    CCU2D add_2532_7 (.A0(\mic_acc_q[3] [4]), .B0(n20537), .C0(n4576), 
          .D0(GND_net), .A1(\mic_acc_q[3] [5]), .B1(n20537), .C1(n4576), 
          .D1(GND_net), .CIN(n24406), .COUT(n24407), .S0(n5718), .S1(n5717));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam add_2532_7.INIT0 = 16'h7848;
    defparam add_2532_7.INIT1 = 16'h7848;
    defparam add_2532_7.INJECT1_0 = "NO";
    defparam add_2532_7.INJECT1_1 = "NO";
    LUT4 mic_cfg_width_6__I_0_1524_i12_3_lut (.A(n10_adj_3230), .B(mic_acc_last_N_2665[5]), 
         .C(mic_cfg_width[5]), .Z(n12_adj_3229)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[34:73])
    defparam mic_cfg_width_6__I_0_1524_i12_3_lut.init = 16'h8e8e;
    LUT4 i14641_4_lut (.A(n13984), .B(n13992), .C(n13985), .D(n13993), 
         .Z(n25582)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14641_4_lut.init = 16'h7bde;
    LUT4 i1_2_lut_adj_233 (.A(mod_settle[0]), .B(n16938), .Z(frame_req_N_2700)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(692[22:40])
    defparam i1_2_lut_adj_233.init = 16'heeee;
    PFUMX i4565 (.BLUT(n15326), .ALUT(n25800), .C0(ev_fall_N_2683), .Z(n15327));
    LUT4 i1_3_lut_4_lut_adj_234 (.A(mic_warm), .B(mic_phase[2]), .C(mic_phase[1]), 
         .D(n127), .Z(n20521)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_3_lut_4_lut_adj_234.init = 16'h0004;
    FD1P3AX status_bit_index_i0_i1 (.D(status_next_index[1]), .SP(spi1_sck_N_155_enable_9), 
            .CK(spi1_sck_N_155), .Q(status_bit_index[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam status_bit_index_i0_i1.GSR = "ENABLED";
    FD1P3JX pcm_read_count_latched_spi_i0_i4 (.D(pcm_count_spi[4]), .SP(spi1_sck_c_enable_169), 
            .PD(n17693), .CK(spi1_sck_c), .Q(pcm_read_count_latched_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam pcm_read_count_latched_spi_i0_i4.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_4_lut_adj_235 (.A(mic_phase[2]), .B(mic_phase[1]), 
         .C(mic_phase[3]), .D(mic_phase[0]), .Z(n13_adj_3286)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_235.init = 16'hfffe;
    LUT4 i6631_1_lut (.A(mic_run_state_1__N_2390[1]), .Z(mic_run_state_1__N_2390[0])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(924[9] 945[12])
    defparam i6631_1_lut.init = 16'h5555;
    LUT4 i15101_4_lut (.A(n16_adj_3282), .B(spi_byte_count[0]), .C(n16919), 
         .D(spi_bit_count_2__N_1415), .Z(spi1_sck_c_enable_66)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(475[21:44])
    defparam i15101_4_lut.init = 16'h0100;
    LUT4 n27418_bdd_4_lut_16201_4_lut (.A(status_next_index[1]), .B(status_hold[36]), 
         .C(status_hold[37]), .D(n27418), .Z(n26950)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27418_bdd_4_lut_16201_4_lut.init = 16'hf588;
    LUT4 n10819_bdd_4_lut (.A(mic_acc_state[0]), .B(n21127), .C(mic_acc_state[2]), 
         .D(mic_acc_state[1]), .Z(n28391)) /* synthesis lut_function=((B (C+(D))+!B (C))+!A) */ ;
    defparam n10819_bdd_4_lut.init = 16'hfdf5;
    LUT4 i1_2_lut_3_lut_adj_236 (.A(spi_command[0]), .B(spi_command[1]), 
         .C(n16894), .Z(n16895)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[61:83])
    defparam i1_2_lut_3_lut_adj_236.init = 16'hfefe;
    LUT4 i15142_2_lut_3_lut (.A(spi_command[0]), .B(spi_command[1]), .C(n16907), 
         .Z(spi_audio_read_cmd)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(174[38:60])
    defparam i15142_2_lut_3_lut.init = 16'h0404;
    LUT4 i4693_2_lut_3_lut (.A(mod_toggle_sync), .B(mod_toggle_seen), .C(n10668), 
         .Z(n15455)) /* synthesis lut_function=(A ((C)+!B)+!A (B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i4693_2_lut_3_lut.init = 16'hf6f6;
    LUT4 n27226_bdd_4_lut (.A(n27226), .B(n27199), .C(n27097), .D(status_next_bit_N_2754[3]), 
         .Z(n27229)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27226_bdd_4_lut.init = 16'haad8;
    CCU2D spi_byte_count_3044_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_bit_count[1]), .B1(n22122), .C1(spi_byte_count[0]), 
          .D1(GND_net), .COUT(n24535), .S1(n50));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044_add_4_1.INIT0 = 16'hF000;
    defparam spi_byte_count_3044_add_4_1.INIT1 = 16'h7878;
    defparam spi_byte_count_3044_add_4_1.INJECT1_0 = "NO";
    defparam spi_byte_count_3044_add_4_1.INJECT1_1 = "NO";
    LUT4 spi_command_5__bdd_4_lut_16489 (.A(n16919), .B(spi_byte_count[8]), 
         .C(spi_byte_count[4]), .D(spi_byte_count[5]), .Z(n28076)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam spi_command_5__bdd_4_lut_16489.init = 16'hfffe;
    CCU2D sub_1017_add_2_17 (.A0(mic_gate_wait[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24534), .S0(n6812));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(950[38:59])
    defparam sub_1017_add_2_17.INIT0 = 16'h5555;
    defparam sub_1017_add_2_17.INIT1 = 16'h0000;
    defparam sub_1017_add_2_17.INJECT1_0 = "NO";
    defparam sub_1017_add_2_17.INJECT1_1 = "NO";
    CCU2D add_2532_5 (.A0(\mic_acc_q[3] [2]), .B0(n20537), .C0(n4576), 
          .D0(GND_net), .A1(\mic_acc_q[3] [3]), .B1(n20537), .C1(n4576), 
          .D1(GND_net), .CIN(n24405), .COUT(n24406), .S0(n5720), .S1(n5719));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam add_2532_5.INIT0 = 16'h7848;
    defparam add_2532_5.INIT1 = 16'h7848;
    defparam add_2532_5.INJECT1_0 = "NO";
    defparam add_2532_5.INJECT1_1 = "NO";
    LUT4 i14620_4_lut (.A(n13988), .B(n13990), .C(n13989), .D(n13991), 
         .Z(n25560)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14620_4_lut.init = 16'h7bde;
    CCU2D add_2532_3 (.A0(\mic_acc_q[3] [0]), .B0(n20537), .C0(n4576), 
          .D0(GND_net), .A1(\mic_acc_q[3] [1]), .B1(n20537), .C1(n4576), 
          .D1(GND_net), .CIN(n24404), .COUT(n24405), .S0(n5722), .S1(n5721));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam add_2532_3.INIT0 = 16'h7484;
    defparam add_2532_3.INIT1 = 16'h7848;
    defparam add_2532_3.INJECT1_0 = "NO";
    defparam add_2532_3.INJECT1_1 = "NO";
    CCU2D sub_1017_add_2_15 (.A0(mic_gate_wait[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24533), .COUT(n24534), .S0(n6814), .S1(n6813));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(950[38:59])
    defparam sub_1017_add_2_15.INIT0 = 16'h5555;
    defparam sub_1017_add_2_15.INIT1 = 16'h5555;
    defparam sub_1017_add_2_15.INJECT1_0 = "NO";
    defparam sub_1017_add_2_15.INJECT1_1 = "NO";
    CCU2D sub_1017_add_2_13 (.A0(mic_gate_wait[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24532), .COUT(n24533), .S0(n6816), .S1(n6815));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(950[38:59])
    defparam sub_1017_add_2_13.INIT0 = 16'h5555;
    defparam sub_1017_add_2_13.INIT1 = 16'h5555;
    defparam sub_1017_add_2_13.INJECT1_0 = "NO";
    defparam sub_1017_add_2_13.INJECT1_1 = "NO";
    CCU2D sub_1017_add_2_11 (.A0(mic_gate_wait[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24531), .COUT(n24532), .S0(n6818), .S1(n6817));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(950[38:59])
    defparam sub_1017_add_2_11.INIT0 = 16'h5555;
    defparam sub_1017_add_2_11.INIT1 = 16'h5555;
    defparam sub_1017_add_2_11.INJECT1_0 = "NO";
    defparam sub_1017_add_2_11.INJECT1_1 = "NO";
    CCU2D sub_1017_add_2_9 (.A0(mic_gate_wait[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24530), .COUT(n24531), .S0(n6820), .S1(n6819));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(950[38:59])
    defparam sub_1017_add_2_9.INIT0 = 16'h5555;
    defparam sub_1017_add_2_9.INIT1 = 16'h5555;
    defparam sub_1017_add_2_9.INJECT1_0 = "NO";
    defparam sub_1017_add_2_9.INJECT1_1 = "NO";
    CCU2D add_2532_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20537), .B1(n4576), .C1(GND_net), .D1(GND_net), .COUT(n24404));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam add_2532_1.INIT0 = 16'hF000;
    defparam add_2532_1.INIT1 = 16'h7777;
    defparam add_2532_1.INJECT1_0 = "NO";
    defparam add_2532_1.INJECT1_1 = "NO";
    LUT4 i13605_2_lut (.A(phase_step_s1), .B(global_phase_s2[0]), .Z(n45_adj_3160)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13605_2_lut.init = 16'h6666;
    LUT4 i9768_2_lut (.A(spi_channel_page), .B(spi_phase_only_N_2776), .Z(staging_wr_page)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[35:75])
    defparam i9768_2_lut.init = 16'h8888;
    LUT4 mic_cfg_width_6__I_0_1524_i10_3_lut (.A(n8_adj_3231), .B(mic_acc_last_N_2665[4]), 
         .C(mic_cfg_width[4]), .Z(n10_adj_3230)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[34:73])
    defparam mic_cfg_width_6__I_0_1524_i10_3_lut.init = 16'h8e8e;
    LUT4 n27430_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[40]), 
         .C(status_hold[41]), .D(n27430), .Z(n26954)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam n27430_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 ev_run_hold_s5_83__I_0_i8_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_7), .D(ev_run_hold_s5[7]), .Z(us_tx_next[7])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i8_3_lut_4_lut.init = 16'hbf40;
    LUT4 mic_acc_last_I_94_4_lut (.A(pll_clk_enable_597), .B(mic_acc_last_N_2663), 
         .C(n10819), .D(mic_acc_last), .Z(mic_acc_last_N_2659)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B ((D)+!C)+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(947[9] 967[12])
    defparam mic_acc_last_I_94_4_lut.init = 16'h5c0c;
    LUT4 n27184_bdd_4_lut_then_3_lut (.A(n14040), .B(n14041), .C(status_bit_index[0]), 
         .Z(n27968)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27184_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 mic_cfg_width_6__I_0_1524_i8_3_lut (.A(n6_adj_3232), .B(mic_acc_last_N_2665[3]), 
         .C(mic_cfg_width[3]), .Z(n8_adj_3231)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[34:73])
    defparam mic_cfg_width_6__I_0_1524_i8_3_lut.init = 16'h8e8e;
    CCU2D sub_1017_add_2_7 (.A0(mic_gate_wait[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24529), .COUT(n24530), .S0(n6822), .S1(n6821));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(950[38:59])
    defparam sub_1017_add_2_7.INIT0 = 16'h5555;
    defparam sub_1017_add_2_7.INIT1 = 16'h5555;
    defparam sub_1017_add_2_7.INJECT1_0 = "NO";
    defparam sub_1017_add_2_7.INJECT1_1 = "NO";
    LUT4 i15160_2_lut_3_lut_4_lut (.A(fpga_cs_n_c), .B(n22119), .C(n16855), 
         .D(n16922), .Z(spi1_sck_c_enable_130)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;
    defparam i15160_2_lut_3_lut_4_lut.init = 16'h0040;
    LUT4 i2_4_lut_adj_237 (.A(n25754), .B(n4_adj_3149), .C(n81_adj_3287), 
         .D(spi_byte_count[7]), .Z(n32_adj_3179)) /* synthesis lut_function=(!(A ((C (D))+!B)+!A ((C+!(D))+!B))) */ ;
    defparam i2_4_lut_adj_237.init = 16'h0c88;
    CCU2D sub_1017_add_2_5 (.A0(mic_gate_wait[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24528), .COUT(n24529), .S0(n6824), .S1(n6823));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(950[38:59])
    defparam sub_1017_add_2_5.INIT0 = 16'h5555;
    defparam sub_1017_add_2_5.INIT1 = 16'h5555;
    defparam sub_1017_add_2_5.INJECT1_0 = "NO";
    defparam sub_1017_add_2_5.INJECT1_1 = "NO";
    CCU2D add_2478_9 (.A0(\mic_acc_q[0] [6]), .B0(n20521), .C0(n4518), 
          .D0(GND_net), .A1(\mic_acc_q[0] [7]), .B1(n20521), .C1(n4518), 
          .D1(GND_net), .CIN(n24399), .S0(n5692), .S1(n5691));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(842[17:43])
    defparam add_2478_9.INIT0 = 16'h7848;
    defparam add_2478_9.INIT1 = 16'h7848;
    defparam add_2478_9.INJECT1_0 = "NO";
    defparam add_2478_9.INJECT1_1 = "NO";
    FD1P3AX status_bit_index_i0_i2 (.D(status_next_index[2]), .SP(spi1_sck_N_155_enable_9), 
            .CK(spi1_sck_N_155), .Q(status_bit_index[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam status_bit_index_i0_i2.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i3 (.D(status_next_index[3]), .SP(spi1_sck_N_155_enable_9), 
            .CK(spi1_sck_N_155), .Q(status_bit_index[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam status_bit_index_i0_i3.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i4 (.D(status_next_index[4]), .SP(spi1_sck_N_155_enable_9), 
            .CK(spi1_sck_N_155), .Q(status_bit_index[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam status_bit_index_i0_i4.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i5 (.D(status_next_index[5]), .SP(spi1_sck_N_155_enable_9), 
            .CK(spi1_sck_N_155), .Q(status_bit_index[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam status_bit_index_i0_i5.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i6 (.D(status_next_index[6]), .SP(spi1_sck_N_155_enable_9), 
            .CK(spi1_sck_N_155), .Q(status_bit_index[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam status_bit_index_i0_i6.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i7 (.D(status_bit_index_8__N_1377[7]), .SP(spi1_sck_N_155_enable_9), 
            .CK(spi1_sck_N_155), .Q(status_bit_index[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam status_bit_index_i0_i7.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i8 (.D(status_bit_index_8__N_1377[8]), .SP(spi1_sck_N_155_enable_9), 
            .CK(spi1_sck_N_155), .Q(status_bit_index[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[14:90])
    defparam status_bit_index_i0_i8.GSR = "ENABLED";
    FD1P3IX mic_window_count__i1 (.D(n10734), .SP(pll_clk_enable_187), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_window_count[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_window_count__i1.GSR = "DISABLED";
    FD1P3IX mic_window_count__i2 (.D(n10735), .SP(pll_clk_enable_187), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_window_count[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_window_count__i2.GSR = "DISABLED";
    FD1P3IX mic_window_count__i3 (.D(n10736), .SP(pll_clk_enable_187), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_window_count[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_window_count__i3.GSR = "DISABLED";
    FD1P3IX mic_window_count__i4 (.D(n10737), .SP(pll_clk_enable_187), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_window_count[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_window_count__i4.GSR = "DISABLED";
    FD1P3IX mic_window_count__i5 (.D(n10738), .SP(pll_clk_enable_187), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_window_count[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_window_count__i5.GSR = "DISABLED";
    FD1P3IX mic_window_count__i6 (.D(n10739), .SP(pll_clk_enable_187), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(mic_window_count[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_window_count__i6.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i1 (.D(mic_done_N_2647[1]), .SP(pll_clk_enable_193), 
            .CD(n13586), .CK(pll_clk), .Q(mic_gate_index[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_index__i1.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i2 (.D(mic_done_N_2647[2]), .SP(pll_clk_enable_193), 
            .CD(n13586), .CK(pll_clk), .Q(mic_gate_index[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_index__i2.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i3 (.D(mic_done_N_2647[3]), .SP(pll_clk_enable_193), 
            .CD(n13586), .CK(pll_clk), .Q(mic_gate_index[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_index__i3.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i4 (.D(mic_done_N_2647[4]), .SP(pll_clk_enable_193), 
            .CD(n13586), .CK(pll_clk), .Q(mic_gate_index[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_index__i4.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i5 (.D(mic_done_N_2647[5]), .SP(pll_clk_enable_193), 
            .CD(n13586), .CK(pll_clk), .Q(mic_gate_index[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_index__i5.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i6 (.D(mic_done_N_2647[6]), .SP(pll_clk_enable_193), 
            .CD(n13586), .CK(pll_clk), .Q(mic_gate_index[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_index__i6.GSR = "DISABLED";
    FD1P3IX rgb_hold__i1 (.D(rgb_values[1]), .SP(pll_clk_enable_216), .CD(n13578), 
            .CK(pll_clk), .Q(rgb_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i1.GSR = "DISABLED";
    FD1P3IX rgb_hold__i2 (.D(rgb_values[2]), .SP(pll_clk_enable_216), .CD(n13578), 
            .CK(pll_clk), .Q(rgb_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i2.GSR = "DISABLED";
    FD1P3IX rgb_hold__i3 (.D(rgb_values[3]), .SP(pll_clk_enable_216), .CD(n13578), 
            .CK(pll_clk), .Q(rgb_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i3.GSR = "DISABLED";
    FD1P3IX rgb_hold__i4 (.D(rgb_values[4]), .SP(pll_clk_enable_216), .CD(n13578), 
            .CK(pll_clk), .Q(rgb_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i4.GSR = "DISABLED";
    FD1P3IX rgb_hold__i5 (.D(rgb_values[5]), .SP(pll_clk_enable_216), .CD(n13578), 
            .CK(pll_clk), .Q(rgb_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i5.GSR = "DISABLED";
    FD1P3IX rgb_hold__i6 (.D(rgb_values[6]), .SP(pll_clk_enable_216), .CD(n13578), 
            .CK(pll_clk), .Q(rgb_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i6.GSR = "DISABLED";
    FD1P3IX rgb_hold__i7 (.D(rgb_values[7]), .SP(pll_clk_enable_216), .CD(n13578), 
            .CK(pll_clk), .Q(rgb_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i7.GSR = "DISABLED";
    FD1P3IX rgb_hold__i8 (.D(rgb_values[8]), .SP(pll_clk_enable_216), .CD(n13578), 
            .CK(pll_clk), .Q(rgb_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i8.GSR = "DISABLED";
    FD1P3IX rgb_hold__i9 (.D(rgb_values[9]), .SP(pll_clk_enable_216), .CD(n13578), 
            .CK(pll_clk), .Q(rgb_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i9.GSR = "DISABLED";
    FD1P3IX rgb_hold__i10 (.D(rgb_values[10]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i10.GSR = "DISABLED";
    FD1P3IX rgb_hold__i11 (.D(rgb_values[11]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i11.GSR = "DISABLED";
    FD1P3IX rgb_hold__i12 (.D(rgb_values[12]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i12.GSR = "DISABLED";
    FD1P3IX rgb_hold__i13 (.D(rgb_values[13]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i13.GSR = "DISABLED";
    FD1P3IX rgb_hold__i14 (.D(rgb_values[14]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i14.GSR = "DISABLED";
    FD1P3IX rgb_hold__i15 (.D(rgb_values[15]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i15.GSR = "DISABLED";
    FD1P3IX rgb_hold__i16 (.D(rgb_values[16]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i16.GSR = "DISABLED";
    FD1P3IX rgb_hold__i17 (.D(rgb_values[17]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i17.GSR = "DISABLED";
    FD1P3IX rgb_hold__i18 (.D(rgb_values[18]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i18.GSR = "DISABLED";
    FD1P3IX rgb_hold__i19 (.D(rgb_values[19]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i19.GSR = "DISABLED";
    FD1P3IX rgb_hold__i20 (.D(rgb_values[20]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i20.GSR = "DISABLED";
    FD1P3IX rgb_hold__i21 (.D(rgb_values[21]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i21.GSR = "DISABLED";
    FD1P3IX rgb_hold__i22 (.D(rgb_values[22]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i22.GSR = "DISABLED";
    FD1P3IX rgb_hold__i23 (.D(rgb_values[23]), .SP(pll_clk_enable_216), 
            .CD(n13578), .CK(pll_clk), .Q(rgb_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rgb_hold__i23.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i2 (.D(n5721), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i2.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i3 (.D(n5720), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i3.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i4 (.D(n5719), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i4.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i5 (.D(n5718), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i5.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i6 (.D(n5717), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i6.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i7 (.D(n5716), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i7.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i8 (.D(n5715), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i8.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i9 (.D(n5714), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i9.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i10 (.D(n5713), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i10.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i11 (.D(n5712), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i11.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i12 (.D(n5711), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i12.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i13 (.D(n5710), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i13.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i14 (.D(n5709), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i14.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i15 (.D(n5708), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i15.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i16 (.D(n5707), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i16.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i17 (.D(n5706), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i17.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i18 (.D(n5705), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i18.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i19 (.D(n5704), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i19.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i20 (.D(n5703), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i20.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i21 (.D(n5702), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i21.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i22 (.D(n5701), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i22.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i23 (.D(n5700), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i23.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i24 (.D(n5699), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i24.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i25 (.D(n5698), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i25.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i26 (.D(n5697), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i26.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i27 (.D(n5696), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i27.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i28 (.D(n5695), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i28.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i29 (.D(n5694), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i29.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i30 (.D(n5693), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i30.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i31 (.D(n5692), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i31.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i32 (.D(n5691), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_q_3___i32.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i1 (.D(ev_rd_data[1]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i1.GSR = "DISABLED";
    LUT4 i15224_4_lut (.A(n9_adj_3257), .B(n14_adj_3254), .C(status_bit_index[3]), 
         .D(status_bit_index[2]), .Z(spi1_sck_N_155_enable_9)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i15224_4_lut.init = 16'h7fff;
    LUT4 i2196_2_lut_3_lut (.A(frame_req), .B(swap_pending), .C(build_idle_N_2714), 
         .Z(pll_clk_enable_538)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam i2196_2_lut_3_lut.init = 16'h2020;
    LUT4 frame_settle_3__bdd_4_lut (.A(frame_settle[3]), .B(frame_settle[1]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(n27526)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;
    defparam frame_settle_3__bdd_4_lut.init = 16'haaa9;
    LUT4 i1_3_lut_adj_238 (.A(status_bit_index[7]), .B(status_bit_index[0]), 
         .C(status_bit_index[6]), .Z(n9_adj_3257)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i1_3_lut_adj_238.init = 16'h8080;
    CCU2D add_2478_7 (.A0(\mic_acc_q[0] [4]), .B0(n20521), .C0(n4518), 
          .D0(GND_net), .A1(\mic_acc_q[0] [5]), .B1(n20521), .C1(n4518), 
          .D1(GND_net), .CIN(n24398), .COUT(n24399), .S0(n5694), .S1(n5693));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(842[17:43])
    defparam add_2478_7.INIT0 = 16'h7848;
    defparam add_2478_7.INIT1 = 16'h7848;
    defparam add_2478_7.INJECT1_0 = "NO";
    defparam add_2478_7.INJECT1_1 = "NO";
    CCU2D sub_1017_add_2_3 (.A0(mic_gate_wait[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24527), .COUT(n24528), .S0(n6826), .S1(n6825));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(950[38:59])
    defparam sub_1017_add_2_3.INIT0 = 16'h5555;
    defparam sub_1017_add_2_3.INIT1 = 16'h5555;
    defparam sub_1017_add_2_3.INJECT1_0 = "NO";
    defparam sub_1017_add_2_3.INJECT1_1 = "NO";
    LUT4 i1_2_lut_4_lut_adj_239 (.A(ws2812_enable_N_2591), .B(frame_toggle_sync), 
         .C(frame_toggle_seen), .D(n7_adj_3256), .Z(n16758)) /* synthesis lut_function=(!((B ((D)+!C)+!B (C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(663[18] 679[12])
    defparam i1_2_lut_4_lut_adj_239.init = 16'h0082;
    CCU2D sub_1017_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24527), .S1(n6827));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(950[38:59])
    defparam sub_1017_add_2_1.INIT0 = 16'hF000;
    defparam sub_1017_add_2_1.INIT1 = 16'h5555;
    defparam sub_1017_add_2_1.INJECT1_0 = "NO";
    defparam sub_1017_add_2_1.INJECT1_1 = "NO";
    CCU2D sub_828_add_2_7 (.A0(pcm_wr_ptr[5]), .B0(pcm_rd_sync2[5]), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24526), .S0(pcm_fifo_N_2607[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(853[25:52])
    defparam sub_828_add_2_7.INIT0 = 16'h5999;
    defparam sub_828_add_2_7.INIT1 = 16'h0000;
    defparam sub_828_add_2_7.INJECT1_0 = "NO";
    defparam sub_828_add_2_7.INJECT1_1 = "NO";
    LUT4 i6_4_lut_adj_240 (.A(status_bit_index[1]), .B(status_bit_index[5]), 
         .C(status_bit_index[4]), .D(status_bit_index[8]), .Z(n14_adj_3254)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_240.init = 16'h8000;
    CCU2D sub_828_add_2_5 (.A0(pcm_wr_ptr[3]), .B0(pcm_rd_sync2[3]), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_wr_ptr[4]), .B1(pcm_rd_sync2[4]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24525), .COUT(n24526));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(853[25:52])
    defparam sub_828_add_2_5.INIT0 = 16'h5999;
    defparam sub_828_add_2_5.INIT1 = 16'h5999;
    defparam sub_828_add_2_5.INJECT1_0 = "NO";
    defparam sub_828_add_2_5.INJECT1_1 = "NO";
    CCU2D add_2478_5 (.A0(\mic_acc_q[0] [2]), .B0(n20521), .C0(n4518), 
          .D0(GND_net), .A1(\mic_acc_q[0] [3]), .B1(n20521), .C1(n4518), 
          .D1(GND_net), .CIN(n24397), .COUT(n24398), .S0(n5696), .S1(n5695));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(842[17:43])
    defparam add_2478_5.INIT0 = 16'h7848;
    defparam add_2478_5.INIT1 = 16'h7848;
    defparam add_2478_5.INJECT1_0 = "NO";
    defparam add_2478_5.INJECT1_1 = "NO";
    LUT4 i10000_4_lut (.A(ev_rd_data[53]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[53])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10000_4_lut.init = 16'h1222;
    LUT4 equal_1614_i15_2_lut_3_lut (.A(spi_byte_count[7]), .B(spi_byte_count[8]), 
         .C(spi_byte_count[6]), .Z(n15_adj_3283)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam equal_1614_i15_2_lut_3_lut.init = 16'hfefe;
    LUT4 i1_2_lut_adj_241 (.A(pll_clk_enable_391), .B(n25502), .Z(pll_clk_enable_187)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_241.init = 16'heeee;
    LUT4 i1_2_lut_adj_242 (.A(spi_byte_count[0]), .B(spi_byte_count[1]), 
         .Z(n16966)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i1_2_lut_adj_242.init = 16'hbbbb;
    LUT4 i15059_2_lut_3_lut_4_lut (.A(fpga_cs_n_c), .B(n22119), .C(n16855), 
         .D(n16966), .Z(spi1_sck_c_enable_122)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;
    defparam i15059_2_lut_3_lut_4_lut.init = 16'h0040;
    CCU2D sub_828_add_2_3 (.A0(pcm_wr_ptr[1]), .B0(pcm_rd_sync2[1]), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_wr_ptr[2]), .B1(pcm_rd_sync2[2]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24524), .COUT(n24525));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(853[25:52])
    defparam sub_828_add_2_3.INIT0 = 16'h5999;
    defparam sub_828_add_2_3.INIT1 = 16'h5999;
    defparam sub_828_add_2_3.INJECT1_0 = "NO";
    defparam sub_828_add_2_3.INJECT1_1 = "NO";
    FD1S3IX wrap_s2_1246 (.D(wrap_s2_N_2598), .CK(pll_clk), .CD(n17690), 
            .Q(wrap_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam wrap_s2_1246.GSR = "DISABLED";
    CCU2D sub_828_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(pcm_wr_ptr[0]), .B1(pcm_rd_sync2[0]), .C1(GND_net), .D1(GND_net), 
          .COUT(n24524));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(853[25:52])
    defparam sub_828_add_2_1.INIT0 = 16'h0000;
    defparam sub_828_add_2_1.INIT1 = 16'h5999;
    defparam sub_828_add_2_1.INJECT1_0 = "NO";
    defparam sub_828_add_2_1.INJECT1_1 = "NO";
    FD1P3IX mic_cfg_gap_i15 (.D(mic_cfg_gap_w[15]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i15.GSR = "DISABLED";
    CCU2D feed_wr_7__I_0_1375_add_2_9 (.A0(feed_wr[7]), .B0(feed_rd[7]), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24523), .S0(feed_fill[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[29:46])
    defparam feed_wr_7__I_0_1375_add_2_9.INIT0 = 16'h5999;
    defparam feed_wr_7__I_0_1375_add_2_9.INIT1 = 16'h0000;
    defparam feed_wr_7__I_0_1375_add_2_9.INJECT1_0 = "NO";
    defparam feed_wr_7__I_0_1375_add_2_9.INJECT1_1 = "NO";
    PFUMX i16434 (.BLUT(n27982), .ALUT(n27983), .C0(status_bit_index[0]), 
          .Z(n27984));
    LUT4 mic_cfg_width_6__I_0_1524_i6_3_lut (.A(n4_adj_3233), .B(mic_acc_last_N_2665[2]), 
         .C(mic_cfg_width[2]), .Z(n6_adj_3232)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[34:73])
    defparam mic_cfg_width_6__I_0_1524_i6_3_lut.init = 16'h8e8e;
    FD1P3IX mic_cfg_gap_i14 (.D(mic_cfg_gap_w[14]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i14.GSR = "DISABLED";
    LUT4 i14972_4_lut (.A(n8_adj_3177), .B(spi_byte_count[6]), .C(spi_byte_count[5]), 
         .D(spi_byte_count[4]), .Z(n25754)) /* synthesis lut_function=(A (B+(C))+!A (B+(C (D)))) */ ;
    defparam i14972_4_lut.init = 16'hfcec;
    FD1P3IX mic_cfg_gap_i13 (.D(mic_cfg_gap_w[13]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i13.GSR = "DISABLED";
    FD1P3IX mic_cfg_gap_i12 (.D(mic_cfg_gap_w[12]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i12.GSR = "DISABLED";
    FD1P3IX mic_cfg_gap_i11 (.D(mic_cfg_gap_w[11]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i11.GSR = "DISABLED";
    CCU2D add_2478_3 (.A0(\mic_acc_q[0] [0]), .B0(n20521), .C0(n4518), 
          .D0(GND_net), .A1(\mic_acc_q[0] [1]), .B1(n20521), .C1(n4518), 
          .D1(GND_net), .CIN(n24396), .COUT(n24397), .S0(n5698), .S1(n5697));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(842[17:43])
    defparam add_2478_3.INIT0 = 16'h7484;
    defparam add_2478_3.INIT1 = 16'h7848;
    defparam add_2478_3.INJECT1_0 = "NO";
    defparam add_2478_3.INJECT1_1 = "NO";
    FD1P3IX mic_cfg_gap_i10 (.D(mic_cfg_gap_w[10]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i10.GSR = "DISABLED";
    CCU2D add_2478_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20521), .B1(n4518), .C1(GND_net), .D1(GND_net), .COUT(n24396));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(842[17:43])
    defparam add_2478_1.INIT0 = 16'hF000;
    defparam add_2478_1.INIT1 = 16'h7777;
    defparam add_2478_1.INJECT1_0 = "NO";
    defparam add_2478_1.INJECT1_1 = "NO";
    FD1P3IX mic_cfg_gap_i9 (.D(mic_cfg_gap_w[9]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i9.GSR = "DISABLED";
    LUT4 i13524_1_lut (.A(spi_bit_count[0]), .Z(n20_adj_3167)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam i13524_1_lut.init = 16'h5555;
    FD1P3IX mic_cfg_gap_i8 (.D(mic_cfg_gap_w[8]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i8.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_243 (.A(spi_rgb_start[7]), .B(spi_byte_count[8]), 
         .Z(n4_adj_3149)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_adj_243.init = 16'h2222;
    LUT4 i1_2_lut_adj_244 (.A(mod_settle[1]), .B(mod_settle[0]), .Z(n16190)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_244.init = 16'h9999;
    FD1S3IX ev_run_hold_s5__i2 (.D(ev_rd_data[2]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i2.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i3 (.D(ev_rd_data[3]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i3.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i4 (.D(ev_rd_data[4]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i4.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i5 (.D(ev_rd_data[5]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i5.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i6 (.D(ev_rd_data[6]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i6.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i7 (.D(ev_rd_data[7]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i7.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i8 (.D(ev_rd_data[8]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i8.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i9 (.D(ev_rd_data[9]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i9.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i10 (.D(ev_rd_data[10]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i10.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i11 (.D(ev_rd_data[11]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i11.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i12 (.D(ev_rd_data[12]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i12.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i13 (.D(ev_rd_data[13]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i13.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i14 (.D(ev_rd_data[14]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i14.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i15 (.D(ev_rd_data[15]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i15.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i16 (.D(ev_rd_data[16]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i16.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i17 (.D(ev_rd_data[17]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i17.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i18 (.D(ev_rd_data[18]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i18.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i19 (.D(ev_rd_data[19]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i19.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i20 (.D(ev_rd_data[20]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i20.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i21 (.D(ev_rd_data[21]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i21.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i22 (.D(ev_rd_data[22]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i22.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i23 (.D(ev_rd_data[23]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i23.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i24 (.D(ev_rd_data[24]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i24.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i25 (.D(ev_rd_data[25]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i25.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i26 (.D(ev_rd_data[26]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i26.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i27 (.D(ev_rd_data[27]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i27.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i28 (.D(ev_rd_data[28]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i28.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i29 (.D(ev_rd_data[29]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i29.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i30 (.D(ev_rd_data[30]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i30.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i31 (.D(ev_rd_data[31]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i31.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i32 (.D(ev_rd_data[32]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i32.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i33 (.D(ev_rd_data[33]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i33.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i34 (.D(ev_rd_data[34]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i34.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i35 (.D(ev_rd_data[35]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i35.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i36 (.D(ev_rd_data[36]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i36.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i37 (.D(ev_rd_data[37]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i37.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i38 (.D(ev_rd_data[38]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i38.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i39 (.D(ev_rd_data[39]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i39.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i40 (.D(ev_rd_data[40]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i40.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i41 (.D(ev_rd_data[41]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i41.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i42 (.D(ev_rd_data[42]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i42.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i43 (.D(ev_rd_data[43]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i43.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i44 (.D(ev_rd_data[44]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i44.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i45 (.D(ev_rd_data[45]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i45.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i46 (.D(ev_rd_data[46]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i46.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i47 (.D(ev_rd_data[47]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i47.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i48 (.D(ev_rd_data[48]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i48.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i49 (.D(ev_rd_data[49]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i49.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i50 (.D(ev_rd_data[50]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i50.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i51 (.D(ev_rd_data[51]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i51.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i52 (.D(ev_rd_data[52]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i52.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i53 (.D(ev_rd_data[53]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i53.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i54 (.D(ev_rd_data[54]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i54.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i55 (.D(ev_rd_data[55]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i55.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i56 (.D(ev_rd_data[56]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i56.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i57 (.D(ev_rd_data[57]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i57.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i58 (.D(ev_rd_data[58]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i58.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i59 (.D(ev_rd_data[59]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i59.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i60 (.D(ev_rd_data[60]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i60.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i61 (.D(ev_rd_data[61]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i61.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i62 (.D(ev_rd_data[62]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i62.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i63 (.D(ev_rd_data[63]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i63.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i64 (.D(ev_rd_data[64]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i64.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i65 (.D(ev_rd_data[65]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i65.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i66 (.D(ev_rd_data[66]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i66.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i67 (.D(ev_rd_data[67]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i67.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i68 (.D(ev_rd_data[68]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i68.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i69 (.D(ev_rd_data[69]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i69.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i70 (.D(ev_rd_data[70]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i70.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i71 (.D(ev_rd_data[71]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i71.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i72 (.D(ev_rd_data[72]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i72.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i73 (.D(ev_rd_data[73]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i73.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i74 (.D(ev_rd_data[74]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i74.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i75 (.D(ev_rd_data[75]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i75.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i76 (.D(ev_rd_data[76]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i76.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i77 (.D(ev_rd_data[77]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i77.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i78 (.D(ev_rd_data[78]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i78.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i79 (.D(ev_rd_data[79]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i79.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i80 (.D(ev_rd_data[80]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i80.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i81 (.D(ev_rd_data[81]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i81.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i82 (.D(ev_rd_data[82]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i82.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i83 (.D(ev_rd_data[83]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i83.GSR = "DISABLED";
    FD1P3IX frame_settle__i1 (.D(n3390), .SP(ws2812_enable_N_2591), .CD(n15270), 
            .CK(pll_clk), .Q(frame_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam frame_settle__i1.GSR = "DISABLED";
    FD1P3IX frame_settle__i2 (.D(n3389), .SP(ws2812_enable_N_2591), .CD(n15270), 
            .CK(pll_clk), .Q(frame_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam frame_settle__i2.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i1 (.D(n164), .CK(pll_clk), .Q(fpga_time[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i1.GSR = "DISABLED";
    FD1P3JX mic_lo_n_i0 (.D(mic_lo_n_next[0]), .SP(pll_clk_enable_273), 
            .PD(n22401), .CK(pll_clk), .Q(mic_lo_n[0])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_lo_n_i0.GSR = "DISABLED";
    FD1P3JX mic_cfg_count_i0 (.D(mic_cfg_count_w_6__N_850[0]), .SP(pll_clk_enable_597), 
            .PD(n24822), .CK(pll_clk), .Q(mic_cfg_count[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_count_i0.GSR = "DISABLED";
    FD1P3JX mic_warm_1300 (.D(mic_warm_N_1898), .SP(pll_clk_enable_258), 
            .PD(n17650), .CK(pll_clk), .Q(mic_warm)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_warm_1300.GSR = "DISABLED";
    FD1P3IX frame_req_mod_1279 (.D(frame_req_mod_N_2737), .SP(pll_clk_enable_259), 
            .CD(n16096), .CK(pll_clk), .Q(frame_req_mod)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam frame_req_mod_1279.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i0 (.D(ev_rd_data[0]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1633), 
            .Q(ev_run_hold_s5[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam ev_run_hold_s5__i0.GSR = "DISABLED";
    FD1P3IX pcm_read_count_latched_spi_i0_i0 (.D(pcm_count_spi[0]), .SP(spi1_sck_c_enable_169), 
            .CD(n17693), .CK(spi1_sck_c), .Q(pcm_read_count_latched_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam pcm_read_count_latched_spi_i0_i0.GSR = "DISABLED";
    FD1P3IX mic_block_count_3057__i0 (.D(n85), .SP(pll_clk_enable_262), 
            .CD(n17704), .CK(pll_clk), .Q(mic_block_count[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_block_count_3057__i0.GSR = "DISABLED";
    FD1P3IX pcm_decim_count_3054__i0 (.D(n45_adj_3235), .SP(pll_clk_enable_391), 
            .CD(n23736), .CK(pll_clk), .Q(pcm_decim_count[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(860[40:62])
    defparam pcm_decim_count_3054__i0.GSR = "DISABLED";
    FD1P3IX mic_gate_active_1337 (.D(mic_gate_active_N_2286), .SP(pll_clk_enable_264), 
            .CD(mic_acc_busy_N_2272), .CK(pll_clk), .Q(mic_gate_active)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_active_1337.GSR = "DISABLED";
    LUT4 mic_cfg_width_6__I_0_1524_i4_4_lut (.A(mic_acc_last_N_2665[0]), .B(mic_acc_last_N_2665[1]), 
         .C(mic_cfg_width[1]), .D(mic_cfg_width[0]), .Z(n4_adj_3233)) /* synthesis lut_function=(A (B+!(C))+!A !(B (C (D))+!B (C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[34:73])
    defparam mic_cfg_width_6__I_0_1524_i4_4_lut.init = 16'h8ecf;
    FD1P3IX mic_cfg_gap_i7 (.D(mic_cfg_gap_w[7]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i7.GSR = "DISABLED";
    LUT4 i2_3_lut_4_lut_adj_245 (.A(accepted_update_flags_spi[0]), .B(feed_pop), 
         .C(pll_clk_enable_2), .D(n16758), .Z(pll_clk_enable_3)) /* synthesis lut_function=(A (B+(C+(D)))+!A (B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(692[18] 701[12])
    defparam i2_3_lut_4_lut_adj_245.init = 16'hfefc;
    LUT4 i1_2_lut_4_lut_adj_246 (.A(ws2812_frame_done), .B(ws2812_frames_left[0]), 
         .C(ws2812_frames_left[1]), .D(ws2812_enable_N_2578), .Z(pll_clk_enable_94)) /* synthesis lut_function=(A (B+(C+(D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[13:68])
    defparam i1_2_lut_4_lut_adj_246.init = 16'hffa8;
    CCU2D feed_wr_7__I_0_1375_add_2_7 (.A0(feed_wr[5]), .B0(feed_rd[5]), 
          .C0(GND_net), .D0(GND_net), .A1(feed_wr[6]), .B1(feed_rd[6]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24522), .COUT(n24523), .S0(feed_fill[5]), 
          .S1(feed_fill[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[29:46])
    defparam feed_wr_7__I_0_1375_add_2_7.INIT0 = 16'h5999;
    defparam feed_wr_7__I_0_1375_add_2_7.INIT1 = 16'h5999;
    defparam feed_wr_7__I_0_1375_add_2_7.INJECT1_0 = "NO";
    defparam feed_wr_7__I_0_1375_add_2_7.INJECT1_1 = "NO";
    LUT4 i3409_2_lut (.A(pcm_rd_ptr_spi[0]), .B(pcm_payload_bit[4]), .Z(pcm_spi_sample_15__N_1371[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[18:55])
    defparam i3409_2_lut.init = 16'h6666;
    LUT4 i4572_4_lut (.A(ev_sum[7]), .B(staging_q[7]), .C(staging_rd_page_N_2718), 
         .D(ev_in_clear), .Z(n15334)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam i4572_4_lut.init = 16'hcac0;
    FD1S3AX fpga_time_3049__i2 (.D(n163), .CK(pll_clk), .Q(fpga_time[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i2.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i3 (.D(n162), .CK(pll_clk), .Q(fpga_time[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i3.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i4 (.D(n161), .CK(pll_clk), .Q(fpga_time[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i4.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i5 (.D(n160), .CK(pll_clk), .Q(fpga_time[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i5.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i6 (.D(n159), .CK(pll_clk), .Q(fpga_time[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i6.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i7 (.D(n158), .CK(pll_clk), .Q(fpga_time[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i7.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i8 (.D(n157), .CK(pll_clk), .Q(fpga_time[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i8.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i9 (.D(n156), .CK(pll_clk), .Q(fpga_time[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i9.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i10 (.D(n155), .CK(pll_clk), .Q(fpga_time[10])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i10.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i11 (.D(n154), .CK(pll_clk), .Q(fpga_time[11])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i11.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i12 (.D(n153), .CK(pll_clk), .Q(fpga_time[12])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i12.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i13 (.D(n152), .CK(pll_clk), .Q(fpga_time[13])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i13.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i14 (.D(n151), .CK(pll_clk), .Q(fpga_time[14])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i14.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i15 (.D(n150), .CK(pll_clk), .Q(fpga_time[15])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i15.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i16 (.D(n149), .CK(pll_clk), .Q(fpga_time[16])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i16.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i17 (.D(n148), .CK(pll_clk), .Q(fpga_time[17])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i17.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i18 (.D(n147), .CK(pll_clk), .Q(fpga_time[18])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i18.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i19 (.D(n146), .CK(pll_clk), .Q(fpga_time[19])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i19.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i20 (.D(n145), .CK(pll_clk), .Q(fpga_time[20])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i20.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i21 (.D(n144), .CK(pll_clk), .Q(fpga_time[21])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i21.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i22 (.D(n143), .CK(pll_clk), .Q(fpga_time[22])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i22.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i23 (.D(n142), .CK(pll_clk), .Q(fpga_time[23])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i23.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i24 (.D(n141), .CK(pll_clk), .Q(fpga_time[24])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i24.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i25 (.D(n140), .CK(pll_clk), .Q(fpga_time[25])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i25.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i26 (.D(n139), .CK(pll_clk), .Q(fpga_time[26])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i26.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i27 (.D(n138), .CK(pll_clk), .Q(fpga_time[27])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i27.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i28 (.D(n137), .CK(pll_clk), .Q(fpga_time[28])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i28.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i29 (.D(n136), .CK(pll_clk), .Q(fpga_time[29])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i29.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i30 (.D(n135), .CK(pll_clk), .Q(fpga_time[30])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i30.GSR = "DISABLED";
    FD1S3AX fpga_time_3049__i31 (.D(n134), .CK(pll_clk), .Q(fpga_time[31])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049__i31.GSR = "DISABLED";
    FD1P3AX status_hold__i2 (.D(accepted_sequence[9]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i2.GSR = "DISABLED";
    FD1P3IX mic_cfg_count_i4 (.D(mic_cfg_hold_spi[4]), .SP(pll_clk_enable_597), 
            .CD(n17672), .CK(pll_clk), .Q(mic_cfg_count[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_count_i4.GSR = "DISABLED";
    FD1P3IX mic_cfg_count_i3 (.D(mic_cfg_hold_spi[3]), .SP(pll_clk_enable_597), 
            .CD(n17672), .CK(pll_clk), .Q(mic_cfg_count[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_count_i3.GSR = "DISABLED";
    FD1P3IX mic_cfg_count_i2 (.D(mic_cfg_hold_spi[2]), .SP(pll_clk_enable_597), 
            .CD(n17672), .CK(pll_clk), .Q(mic_cfg_count[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_count_i2.GSR = "DISABLED";
    FD1P3IX mic_cfg_count_i1 (.D(mic_cfg_hold_spi[1]), .SP(pll_clk_enable_597), 
            .CD(n17672), .CK(pll_clk), .Q(mic_cfg_count[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_count_i1.GSR = "DISABLED";
    FD1P3JX mic_lo_n_i6 (.D(mic_lo_n_next[6]), .SP(pll_clk_enable_273), 
            .PD(n22401), .CK(pll_clk), .Q(mic_lo_n[6])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_lo_n_i6.GSR = "DISABLED";
    FD1P3JX mic_lo_n_i5 (.D(mic_lo_n_next[5]), .SP(pll_clk_enable_273), 
            .PD(n22401), .CK(pll_clk), .Q(mic_lo_n[5])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_lo_n_i5.GSR = "DISABLED";
    FD1P3JX mic_lo_n_i1 (.D(mic_lo_n_next[1]), .SP(pll_clk_enable_273), 
            .PD(n22401), .CK(pll_clk), .Q(mic_lo_n[1])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_lo_n_i1.GSR = "DISABLED";
    FD1P3IX pcm_acc_i8 (.D(pcm_acc_next[8]), .SP(pll_clk_enable_280), .CD(n19096), 
            .CK(pll_clk), .Q(pcm_acc[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pcm_acc_i8.GSR = "DISABLED";
    FD1P3IX pcm_acc_i7 (.D(pcm_acc_next[7]), .SP(pll_clk_enable_280), .CD(n19096), 
            .CK(pll_clk), .Q(pcm_acc[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pcm_acc_i7.GSR = "DISABLED";
    FD1P3IX pcm_acc_i6 (.D(pcm_acc_next[6]), .SP(pll_clk_enable_280), .CD(n19096), 
            .CK(pll_clk), .Q(pcm_acc[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pcm_acc_i6.GSR = "DISABLED";
    FD1P3IX pcm_acc_i5 (.D(pcm_acc_next[5]), .SP(pll_clk_enable_280), .CD(n19096), 
            .CK(pll_clk), .Q(pcm_acc[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pcm_acc_i5.GSR = "DISABLED";
    FD1P3IX pcm_acc_i4 (.D(pcm_acc_next[4]), .SP(pll_clk_enable_280), .CD(n19096), 
            .CK(pll_clk), .Q(pcm_acc[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pcm_acc_i4.GSR = "DISABLED";
    FD1P3IX pcm_acc_i1 (.D(pcm_acc_next[1]), .SP(pll_clk_enable_280), .CD(n19096), 
            .CK(pll_clk), .Q(pcm_acc[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pcm_acc_i1.GSR = "DISABLED";
    FD1P3IX pcm_acc_i0 (.D(pcm_acc_next[0]), .SP(pll_clk_enable_280), .CD(n19096), 
            .CK(pll_clk), .Q(pcm_acc[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pcm_acc_i0.GSR = "DISABLED";
    FD1P3IX mic_acc_word_i0 (.D(mic_acc_word_2__N_2312[0]), .SP(pll_clk_enable_281), 
            .CD(n28392), .CK(pll_clk), .Q(mic_acc_word[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_word_i0.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_247 (.A(fpga_cs_n_c), .B(n22119), .Z(n22120)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(85[24:33])
    defparam i1_2_lut_adj_247.init = 16'h4444;
    LUT4 i1_2_lut_adj_248 (.A(mic_phase[0]), .B(mic_phase[1]), .Z(n22402)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_248.init = 16'heeee;
    FD1P3AX status_hold__i3 (.D(accepted_sequence[10]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i3.GSR = "DISABLED";
    FD1P3AX status_hold__i4 (.D(accepted_sequence[11]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i4.GSR = "DISABLED";
    FD1P3AX status_hold__i5 (.D(accepted_sequence[12]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i5.GSR = "DISABLED";
    FD1P3AX status_hold__i6 (.D(accepted_sequence[13]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i6.GSR = "DISABLED";
    FD1P3AX status_hold__i7 (.D(accepted_sequence[14]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i7.GSR = "DISABLED";
    FD1P3AX status_hold__i8 (.D(accepted_sequence[15]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i8.GSR = "DISABLED";
    FD1P3AX status_hold__i9 (.D(accepted_sequence[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i9.GSR = "DISABLED";
    FD1P3AX status_hold__i10 (.D(accepted_sequence[1]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i10.GSR = "DISABLED";
    FD1P3AX status_hold__i11 (.D(accepted_sequence[2]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i11.GSR = "DISABLED";
    FD1P3AX status_hold__i12 (.D(accepted_sequence[3]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i12.GSR = "DISABLED";
    FD1P3AX status_hold__i13 (.D(accepted_sequence[4]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i13.GSR = "DISABLED";
    FD1P3AX status_hold__i14 (.D(accepted_sequence[5]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i14.GSR = "DISABLED";
    FD1P3AX status_hold__i15 (.D(accepted_sequence[6]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i15.GSR = "DISABLED";
    FD1P3AX status_hold__i16 (.D(accepted_sequence[7]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i16.GSR = "DISABLED";
    FD1P3AX status_hold__i17 (.D(fpga_time[24]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i17.GSR = "DISABLED";
    FD1P3AX status_hold__i18 (.D(fpga_time[25]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i18.GSR = "DISABLED";
    FD1P3AX status_hold__i19 (.D(fpga_time[26]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i19.GSR = "DISABLED";
    FD1P3AX status_hold__i20 (.D(fpga_time[27]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i20.GSR = "DISABLED";
    FD1P3AX status_hold__i21 (.D(fpga_time[28]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i21.GSR = "DISABLED";
    FD1P3AX status_hold__i22 (.D(fpga_time[29]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i22.GSR = "DISABLED";
    FD1P3AX status_hold__i23 (.D(fpga_time[30]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i23.GSR = "DISABLED";
    FD1P3AX status_hold__i24 (.D(fpga_time[31]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i24.GSR = "DISABLED";
    FD1P3AX status_hold__i25 (.D(fpga_time[16]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i25.GSR = "DISABLED";
    FD1P3AX status_hold__i26 (.D(fpga_time[17]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i26.GSR = "DISABLED";
    FD1P3AX status_hold__i27 (.D(fpga_time[18]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i27.GSR = "DISABLED";
    FD1P3AX status_hold__i28 (.D(fpga_time[19]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i28.GSR = "DISABLED";
    FD1P3AX status_hold__i29 (.D(fpga_time[20]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i29.GSR = "DISABLED";
    FD1P3AX status_hold__i30 (.D(fpga_time[21]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i30.GSR = "DISABLED";
    FD1P3AX status_hold__i31 (.D(fpga_time[22]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i31.GSR = "DISABLED";
    FD1P3AX status_hold__i32 (.D(fpga_time[23]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i32.GSR = "DISABLED";
    FD1P3AX status_hold__i33 (.D(fpga_time[8]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i33.GSR = "DISABLED";
    FD1P3AX status_hold__i34 (.D(fpga_time[9]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i34.GSR = "DISABLED";
    FD1P3AX status_hold__i35 (.D(fpga_time[10]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i35.GSR = "DISABLED";
    FD1P3AX status_hold__i36 (.D(fpga_time[11]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i36.GSR = "DISABLED";
    FD1P3AX status_hold__i37 (.D(fpga_time[12]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i37.GSR = "DISABLED";
    FD1P3AX status_hold__i38 (.D(fpga_time[13]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i38.GSR = "DISABLED";
    FD1P3AX status_hold__i39 (.D(fpga_time[14]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i39.GSR = "DISABLED";
    FD1P3AX status_hold__i40 (.D(fpga_time[15]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i40.GSR = "DISABLED";
    FD1P3AX status_hold__i41 (.D(fpga_time[0]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i41.GSR = "DISABLED";
    FD1P3AX status_hold__i42 (.D(fpga_time[1]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i42.GSR = "DISABLED";
    FD1P3AX status_hold__i43 (.D(fpga_time[2]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i43.GSR = "DISABLED";
    FD1P3AX status_hold__i44 (.D(fpga_time[3]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i44.GSR = "DISABLED";
    FD1P3AX status_hold__i45 (.D(fpga_time[4]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i45.GSR = "DISABLED";
    FD1P3AX status_hold__i46 (.D(fpga_time[5]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i46.GSR = "DISABLED";
    FD1P3AX status_hold__i47 (.D(fpga_time[6]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i47.GSR = "DISABLED";
    FD1P3AX status_hold__i48 (.D(fpga_time[7]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i48.GSR = "DISABLED";
    FD1P3AX status_hold__i49 (.D(status_flags_wire_15__N_1355[13]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i49.GSR = "DISABLED";
    FD1P3AX status_hold__i50 (.D(mod_active), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i50.GSR = "DISABLED";
    FD1P3AX status_hold__i51 (.D(invalid_frame_sync), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i51.GSR = "DISABLED";
    FD1P3AX status_hold__i52 (.D(running), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i52.GSR = "DISABLED";
    FD1P3AX status_hold__i53 (.D(fifo_depth_wire[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i53.GSR = "DISABLED";
    FD1P3AX status_hold__i54 (.D(fifo_credit_wire[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[104])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i54.GSR = "DISABLED";
    FD1P3AX status_hold__i55 (.D(feed_fill[0]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[112])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i55.GSR = "DISABLED";
    FD1P3AX status_hold__i56 (.D(feed_fill[1]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[113])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i56.GSR = "DISABLED";
    FD1P3AX status_hold__i57 (.D(feed_fill[2]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[114])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i57.GSR = "DISABLED";
    FD1P3AX status_hold__i58 (.D(feed_fill[3]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[115])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i58.GSR = "DISABLED";
    FD1P3AX status_hold__i59 (.D(feed_fill[4]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[116])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i59.GSR = "DISABLED";
    FD1P3AX status_hold__i60 (.D(feed_fill[5]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[117])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i60.GSR = "DISABLED";
    FD1P3AX status_hold__i61 (.D(feed_fill[6]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[118])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i61.GSR = "DISABLED";
    FD1P3AX status_hold__i62 (.D(feed_fill[7]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[119])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam status_hold__i62.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i1 (.D(accepted_sequence_15__N_995[1]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i1.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i2 (.D(accepted_sequence_15__N_995[2]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i3 (.D(accepted_sequence_15__N_995[3]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i4 (.D(accepted_sequence_15__N_995[4]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i5 (.D(accepted_sequence_15__N_995[5]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i6 (.D(accepted_sequence_15__N_995[6]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i7 (.D(accepted_sequence_15__N_995[7]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i8 (.D(accepted_sequence_15__N_995[8]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i9 (.D(accepted_sequence_15__N_995[9]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i10 (.D(accepted_sequence_15__N_995[10]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i11 (.D(accepted_sequence_15__N_995[11]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i12 (.D(accepted_sequence_15__N_995[12]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i13 (.D(accepted_sequence_15__N_995[13]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i14 (.D(accepted_sequence_15__N_995[14]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i15 (.D(accepted_sequence_15__N_995[15]), .SP(pll_clk_enable_296), 
            .CK(pll_clk), .Q(accepted_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam accepted_sequence_i15.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i1 (.D(global_phase_s2[1]), .SP(pll_clk_enable_304), 
            .CK(pll_clk), .Q(run_addr_s3[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam run_addr_s3_i1.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i2 (.D(global_phase_s2[2]), .SP(pll_clk_enable_304), 
            .CK(pll_clk), .Q(run_addr_s3[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam run_addr_s3_i2.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i3 (.D(global_phase_s2[3]), .SP(pll_clk_enable_304), 
            .CK(pll_clk), .Q(run_addr_s3[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam run_addr_s3_i3.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i4 (.D(global_phase_s2[4]), .SP(pll_clk_enable_304), 
            .CK(pll_clk), .Q(run_addr_s3[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam run_addr_s3_i4.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i5 (.D(global_phase_s2[5]), .SP(pll_clk_enable_304), 
            .CK(pll_clk), .Q(run_addr_s3[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam run_addr_s3_i5.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i6 (.D(global_phase_s2[6]), .SP(pll_clk_enable_304), 
            .CK(pll_clk), .Q(run_addr_s3[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam run_addr_s3_i6.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i7 (.D(global_phase_s2[7]), .SP(pll_clk_enable_304), 
            .CK(pll_clk), .Q(run_addr_s3[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam run_addr_s3_i7.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i8 (.D(active_bank_N_893), .SP(pll_clk_enable_304), 
            .CK(pll_clk), .Q(run_addr_s3[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam run_addr_s3_i8.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i1 (.D(feed_wr_spi[1]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam feed_wr_i0_i1.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i2 (.D(feed_wr_spi[2]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam feed_wr_i0_i2.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i3 (.D(feed_wr_spi[3]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam feed_wr_i0_i3.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i4 (.D(feed_wr_spi[4]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam feed_wr_i0_i4.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i5 (.D(feed_wr_spi[5]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam feed_wr_i0_i5.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i6 (.D(feed_wr_spi[6]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam feed_wr_i0_i6.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i7 (.D(feed_wr_spi[7]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam feed_wr_i0_i7.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i1 (.D(n15323), .SP(pll_clk_enable_311), .CK(pll_clk), 
            .Q(rmw_slot[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rmw_slot_i0_i1.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i2 (.D(n15325), .SP(pll_clk_enable_311), .CK(pll_clk), 
            .Q(rmw_slot[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rmw_slot_i0_i2.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i3 (.D(n15327), .SP(pll_clk_enable_311), .CK(pll_clk), 
            .Q(rmw_slot[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rmw_slot_i0_i3.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i4 (.D(n25149), .SP(pll_clk_enable_311), .CK(pll_clk), 
            .Q(rmw_slot[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rmw_slot_i0_i4.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i5 (.D(n15331), .SP(pll_clk_enable_311), .CK(pll_clk), 
            .Q(rmw_slot[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rmw_slot_i0_i5.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i6 (.D(n15333), .SP(pll_clk_enable_311), .CK(pll_clk), 
            .Q(rmw_slot[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rmw_slot_i0_i6.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i7 (.D(n15335), .SP(pll_clk_enable_311), .CK(pll_clk), 
            .Q(rmw_slot[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam rmw_slot_i0_i7.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i1 (.D(feed_q[1]), .SP(pll_clk_enable_318), 
            .CD(n13582), .CK(pll_clk), .Q(pending_mod_level[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_mod_level__i1.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i2 (.D(feed_q[2]), .SP(pll_clk_enable_318), 
            .CD(n13582), .CK(pll_clk), .Q(pending_mod_level[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_mod_level__i2.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i3 (.D(feed_q[3]), .SP(pll_clk_enable_318), 
            .CD(n13582), .CK(pll_clk), .Q(pending_mod_level[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_mod_level__i3.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i4 (.D(feed_q[4]), .SP(pll_clk_enable_318), 
            .CD(n13582), .CK(pll_clk), .Q(pending_mod_level[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_mod_level__i4.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i5 (.D(feed_q[5]), .SP(pll_clk_enable_318), 
            .CD(n13582), .CK(pll_clk), .Q(pending_mod_level[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_mod_level__i5.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i6 (.D(feed_q[6]), .SP(pll_clk_enable_318), 
            .CD(n13582), .CK(pll_clk), .Q(pending_mod_level[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_mod_level__i6.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i7 (.D(feed_q[7]), .SP(pll_clk_enable_318), 
            .CD(n13582), .CK(pll_clk), .Q(pending_mod_level[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_mod_level__i7.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i1 (.D(accepted_sequence_spi[1]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i1.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i2 (.D(accepted_sequence_spi[2]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i2.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i3 (.D(accepted_sequence_spi[3]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i3.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i4 (.D(accepted_sequence_spi[4]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i4.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i5 (.D(accepted_sequence_spi[5]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i5.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i6 (.D(accepted_sequence_spi[6]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i6.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i7 (.D(accepted_sequence_spi[7]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i7.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i8 (.D(accepted_sequence_spi[8]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i8.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i9 (.D(accepted_sequence_spi[9]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i9.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i10 (.D(accepted_sequence_spi[10]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i10.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i11 (.D(accepted_sequence_spi[11]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i11.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i12 (.D(accepted_sequence_spi[12]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i12.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i13 (.D(accepted_sequence_spi[13]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i13.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i14 (.D(accepted_sequence_spi[14]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i14.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i15 (.D(accepted_sequence_spi[15]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(pending_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam pending_sequence_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i1 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i2 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i3 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i4 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i5 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i6 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i7 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i8 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i9 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i10 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i11 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i12 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i13 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i14 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i15 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_0_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_l_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i1 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i2 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i3 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i4 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i5 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i6 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i7 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i8 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i9 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i10 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i11 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i12 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i13 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i14 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i15 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_363), 
            .CK(pll_clk), .Q(mic_shift_1_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_l_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i2 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i3 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i4 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i5 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i6 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i7 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i8 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i9 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i10 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i11 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i12 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i13 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i14 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i15 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_0_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_0_r__i15.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i2 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i3 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i4 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i5 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i6 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i7 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i8 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i9 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i10 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i11 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i12 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i13 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i14 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i15 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_391), 
            .CK(pll_clk), .Q(mic_shift_1_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_shift_1_r__i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i1 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i2 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i3 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i4 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i5 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i6 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i7 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i8 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i9 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i10 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i11 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i12 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i13 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i14 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i15 (.D(mic_shift_1_r[14]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i16 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i16.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i17 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i17.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i18 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i18.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i19 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i19.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i20 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i20.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i21 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i21.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i22 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i22.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i23 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i23.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i24 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i24.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i25 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i26 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i26.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i27 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i27.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i28 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i28.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i29 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i29.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i30 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i30.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i31 (.D(mic_shift_1_l[15]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i31.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i32 (.D(mic_data_0_c), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i32.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i33 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i33.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i34 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i34.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i35 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i35.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i36 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i36.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i37 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i37.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i38 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i38.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i39 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i39.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i40 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i40.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i41 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i41.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i42 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i42.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i43 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i43.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i44 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i44.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i45 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i45.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i46 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i46.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i47 (.D(mic_shift_0_r[14]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i47.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i48 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i48.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i49 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i49.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i50 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i50.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i51 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i51.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i52 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i52.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i53 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i53.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i54 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i54.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i55 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i55.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i56 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i56.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i57 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i57.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i58 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i58.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i59 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i59.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i60 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i60.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i61 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i61.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i62 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i62.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i63 (.D(mic_shift_0_l[15]), .SP(pll_clk_enable_454), 
            .CK(pll_clk), .Q(mic_latest[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_latest_i0_i63.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i2 (.D(\mic_acc_i[3] [1]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[3] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i2.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i3 (.D(\mic_acc_i[3] [2]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[3] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i3.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i4 (.D(\mic_acc_i[3] [3]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[3] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i4.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i5 (.D(\mic_acc_i[3] [4]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[3] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i5.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i6 (.D(\mic_acc_i[3] [5]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[3] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i6.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i7 (.D(\mic_acc_i[3] [6]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[3] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i7.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i8 (.D(\mic_acc_i[3] [7]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[3] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i8.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i9 (.D(\mic_acc_i[2] [0]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[2] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i9.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i10 (.D(\mic_acc_i[2] [1]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[2] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i10.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i11 (.D(\mic_acc_i[2] [2]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[2] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i11.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i12 (.D(\mic_acc_i[2] [3]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[2] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i12.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i13 (.D(\mic_acc_i[2] [4]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[2] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i13.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i14 (.D(\mic_acc_i[2] [5]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[2] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i14.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i15 (.D(\mic_acc_i[2] [6]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[2] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i15.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i16 (.D(\mic_acc_i[2] [7]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[2] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i16.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i17 (.D(\mic_acc_i[1] [0]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[1] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i17.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i18 (.D(\mic_acc_i[1] [1]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[1] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i18.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i19 (.D(\mic_acc_i[1] [2]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[1] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i19.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i20 (.D(\mic_acc_i[1] [3]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[1] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i20.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i21 (.D(\mic_acc_i[1] [4]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[1] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i21.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i22 (.D(\mic_acc_i[1] [5]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[1] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i22.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i23 (.D(\mic_acc_i[1] [6]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[1] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i23.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i24 (.D(\mic_acc_i[1] [7]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[1] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i24.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i25 (.D(\mic_acc_i[0] [0]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[0] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i25.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i26 (.D(\mic_acc_i[0] [1]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[0] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i26.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i27 (.D(\mic_acc_i[0] [2]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[0] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i27.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i28 (.D(\mic_acc_i[0] [3]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[0] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i28.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i29 (.D(\mic_acc_i[0] [4]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[0] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i29.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i30 (.D(\mic_acc_i[0] [5]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[0] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i30.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i31 (.D(\mic_acc_i[0] [6]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[0] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i31.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i32 (.D(\mic_acc_i[0] [7]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_i[0] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_i_3___i32.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i2 (.D(\mic_acc_q[3] [1]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[3] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i2.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i3 (.D(\mic_acc_q[3] [2]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[3] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i3.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i4 (.D(\mic_acc_q[3] [3]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[3] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i4.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i5 (.D(\mic_acc_q[3] [4]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[3] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i5.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i6 (.D(\mic_acc_q[3] [5]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[3] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i6.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i7 (.D(\mic_acc_q[3] [6]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[3] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i7.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i8 (.D(\mic_acc_q[3] [7]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[3] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i8.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i9 (.D(\mic_acc_q[2] [0]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[2] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i9.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i10 (.D(\mic_acc_q[2] [1]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[2] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i10.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i11 (.D(\mic_acc_q[2] [2]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[2] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i11.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i12 (.D(\mic_acc_q[2] [3]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[2] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i12.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i13 (.D(\mic_acc_q[2] [4]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[2] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i13.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i14 (.D(\mic_acc_q[2] [5]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[2] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i14.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i15 (.D(\mic_acc_q[2] [6]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[2] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i15.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i16 (.D(\mic_acc_q[2] [7]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[2] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i16.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i17 (.D(\mic_acc_q[1] [0]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[1] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i17.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i18 (.D(\mic_acc_q[1] [1]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[1] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i18.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i19 (.D(\mic_acc_q[1] [2]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[1] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i19.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i20 (.D(\mic_acc_q[1] [3]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[1] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i20.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i21 (.D(\mic_acc_q[1] [4]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[1] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i21.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i22 (.D(\mic_acc_q[1] [5]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[1] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i22.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i23 (.D(\mic_acc_q[1] [6]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[1] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i23.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i24 (.D(\mic_acc_q[1] [7]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[1] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i24.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i25 (.D(\mic_acc_q[0] [0]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[0] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i25.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i26 (.D(\mic_acc_q[0] [1]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[0] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i26.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i27 (.D(\mic_acc_q[0] [2]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[0] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i27.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i28 (.D(\mic_acc_q[0] [3]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[0] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i28.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i29 (.D(\mic_acc_q[0] [4]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[0] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i29.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i30 (.D(\mic_acc_q[0] [5]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[0] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i30.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i31 (.D(\mic_acc_q[0] [6]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[0] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i31.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i32 (.D(\mic_acc_q[0] [7]), .SP(pll_clk_enable_516), 
            .CK(pll_clk), .Q(\mic_win_q[0] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_win_q_3___i32.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i1 (.D(n17604), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i2 (.D(n17606), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i3 (.D(n17608), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i4 (.D(n17610), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i5 (.D(n17612), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i6 (.D(n17614), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i7 (.D(n17616), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i8 (.D(n17618), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i9 (.D(n17620), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i10 (.D(n17622), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i11 (.D(n17624), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i12 (.D(n17626), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i13 (.D(n17628), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i14 (.D(n17630), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i15 (.D(n17632), .SP(pll_clk_enable_531), .CK(pll_clk), 
            .Q(mic_gate_wait[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_gate_wait_i0_i15.GSR = "DISABLED";
    FD1P3IX mic_cfg_settle_i1 (.D(n25947), .SP(mic_acc_busy_N_2279), .CD(mic_cfg_settle_3__N_2126), 
            .CK(pll_clk), .Q(mic_cfg_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_settle_i1.GSR = "DISABLED";
    FD1P3IX mic_cfg_settle_i3 (.D(n25949), .SP(mic_acc_busy_N_2279), .CD(mic_cfg_settle_3__N_2126), 
            .CK(pll_clk), .Q(mic_cfg_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_settle_i3.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i1 (.D(mic_acc_word[1]), .SP(mic_ram_we_N_2673), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_addr_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i2 (.D(mic_acc_word[2]), .SP(mic_ram_we_N_2673), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_addr_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i3 (.D(mic_acc_gate[0]), .SP(mic_ram_we_N_2673), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_addr_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i4 (.D(mic_acc_gate[1]), .SP(mic_ram_we_N_2673), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_addr_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i5 (.D(mic_acc_gate[2]), .SP(mic_ram_we_N_2673), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_addr_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i6 (.D(mic_acc_gate[3]), .SP(mic_ram_we_N_2673), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_addr_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i7 (.D(mic_acc_gate[4]), .SP(mic_ram_we_N_2673), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_addr_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i8 (.D(mic_acc_gate[5]), .SP(mic_ram_we_N_2673), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_addr_i0_i8.GSR = "DISABLED";
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_16103_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[30]), .C(status_hold[31]), .D(status_next_index[1]), 
         .Z(n27388)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_16103_4_lut_4_lut.init = 16'h55d8;
    FD1S3AX mic_acc_state_i1 (.D(mic_acc_state_2__N_744[1]), .CK(pll_clk), 
            .Q(mic_acc_state[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_state_i1.GSR = "DISABLED";
    LUT4 i3544_2_lut (.A(mod_settle[1]), .B(mod_settle[0]), .Z(n4_adj_3272)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(693[27:44])
    defparam i3544_2_lut.init = 16'heeee;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_15824_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[114]), .C(status_hold[115]), .D(status_next_index[1]), 
         .Z(n27046)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_15824_4_lut_4_lut.init = 16'h55d8;
    LUT4 i1_2_lut_4_lut_adj_249 (.A(spi_command[6]), .B(spi_command[7]), 
         .C(spi_command[5]), .D(spi_command[4]), .Z(n14_adj_3244)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(174[38:60])
    defparam i1_2_lut_4_lut_adj_249.init = 16'hfeff;
    LUT4 n14036_bdd_3_lut (.A(n14036), .B(n14037), .C(status_bit_index[0]), 
         .Z(n28131)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n14036_bdd_3_lut.init = 16'hacac;
    PFUMX i4563 (.BLUT(n15324), .ALUT(n25798), .C0(ev_fall_N_2683), .Z(n15325));
    LUT4 i2_3_lut_adj_250 (.A(n4_adj_3281), .B(pcm_payload_bit[6]), .C(pcm_rd_ptr_spi[2]), 
         .Z(pcm_spi_sample_15__N_1371[2])) /* synthesis lut_function=(A (B (C)+!B !(C))+!A !(B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[18:55])
    defparam i2_3_lut_adj_250.init = 16'h9696;
    FD1P3AX build_level_i0_i1 (.D(pending_mod_level[1]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(build_level[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_i0_i1.GSR = "DISABLED";
    FD1P3AX build_level_i0_i2 (.D(pending_mod_level[2]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(build_level[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_i0_i2.GSR = "DISABLED";
    FD1P3AX build_level_i0_i3 (.D(pending_mod_level[3]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(build_level[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_i0_i3.GSR = "DISABLED";
    FD1P3AX build_level_i0_i4 (.D(pending_mod_level[4]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(build_level[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_i0_i4.GSR = "DISABLED";
    FD1P3AX build_level_i0_i5 (.D(pending_mod_level[5]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(build_level[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_i0_i5.GSR = "DISABLED";
    FD1P3AX build_level_i0_i6 (.D(pending_mod_level[6]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(build_level[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_i0_i6.GSR = "DISABLED";
    FD1P3AX build_level_i0_i7 (.D(pending_mod_level[7]), .SP(pll_clk_enable_538), 
            .CK(pll_clk), .Q(build_level[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam build_level_i0_i7.GSR = "DISABLED";
    FD1S3AX us_tx_i2 (.D(us_tx_next[1]), .CK(pll_clk), .Q(us_tx_c_1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i2.GSR = "DISABLED";
    FD1S3AX us_tx_i3 (.D(us_tx_next[2]), .CK(pll_clk), .Q(us_tx_c_2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i3.GSR = "DISABLED";
    FD1S3AX us_tx_i4 (.D(us_tx_next[3]), .CK(pll_clk), .Q(us_tx_c_3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i4.GSR = "DISABLED";
    FD1S3AX us_tx_i5 (.D(us_tx_next[4]), .CK(pll_clk), .Q(us_tx_c_4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i5.GSR = "DISABLED";
    FD1S3AX us_tx_i6 (.D(us_tx_next[5]), .CK(pll_clk), .Q(us_tx_c_5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i6.GSR = "DISABLED";
    FD1S3AX us_tx_i7 (.D(us_tx_next[6]), .CK(pll_clk), .Q(us_tx_c_6)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i7.GSR = "DISABLED";
    FD1S3AX us_tx_i8 (.D(us_tx_next[7]), .CK(pll_clk), .Q(us_tx_c_7)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i8.GSR = "DISABLED";
    FD1S3AX us_tx_i9 (.D(us_tx_next[8]), .CK(pll_clk), .Q(us_tx_c_8)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i9.GSR = "DISABLED";
    FD1S3AX us_tx_i10 (.D(us_tx_next[9]), .CK(pll_clk), .Q(us_tx_c_9)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i10.GSR = "DISABLED";
    FD1S3AX us_tx_i11 (.D(us_tx_next[10]), .CK(pll_clk), .Q(us_tx_c_10)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i11.GSR = "DISABLED";
    FD1S3AX us_tx_i12 (.D(us_tx_next[11]), .CK(pll_clk), .Q(us_tx_c_11)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i12.GSR = "DISABLED";
    FD1S3AX us_tx_i13 (.D(us_tx_next[12]), .CK(pll_clk), .Q(us_tx_c_12)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i13.GSR = "DISABLED";
    FD1S3AX us_tx_i14 (.D(us_tx_next[13]), .CK(pll_clk), .Q(us_tx_c_13)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i14.GSR = "DISABLED";
    FD1S3AX us_tx_i15 (.D(us_tx_next[14]), .CK(pll_clk), .Q(us_tx_c_14)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i15.GSR = "DISABLED";
    FD1S3AX us_tx_i16 (.D(us_tx_next[15]), .CK(pll_clk), .Q(us_tx_c_15)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i16.GSR = "DISABLED";
    FD1S3AX us_tx_i17 (.D(us_tx_next[16]), .CK(pll_clk), .Q(us_tx_c_16)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i17.GSR = "DISABLED";
    FD1S3AX us_tx_i18 (.D(us_tx_next[17]), .CK(pll_clk), .Q(us_tx_c_17)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i18.GSR = "DISABLED";
    FD1S3AX us_tx_i19 (.D(us_tx_next[18]), .CK(pll_clk), .Q(us_tx_c_18)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i19.GSR = "DISABLED";
    FD1S3AX us_tx_i20 (.D(us_tx_next[19]), .CK(pll_clk), .Q(us_tx_c_19)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i20.GSR = "DISABLED";
    FD1S3AX us_tx_i21 (.D(us_tx_next[20]), .CK(pll_clk), .Q(us_tx_c_20)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i21.GSR = "DISABLED";
    FD1S3AX us_tx_i22 (.D(us_tx_next[21]), .CK(pll_clk), .Q(us_tx_c_21)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i22.GSR = "DISABLED";
    FD1S3AX us_tx_i23 (.D(us_tx_next[22]), .CK(pll_clk), .Q(us_tx_c_22)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i23.GSR = "DISABLED";
    FD1S3AX us_tx_i24 (.D(us_tx_next[23]), .CK(pll_clk), .Q(us_tx_c_23)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i24.GSR = "DISABLED";
    FD1S3AX us_tx_i25 (.D(us_tx_next[24]), .CK(pll_clk), .Q(us_tx_c_24)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i25.GSR = "DISABLED";
    FD1S3AX us_tx_i26 (.D(us_tx_next[25]), .CK(pll_clk), .Q(us_tx_c_25)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i26.GSR = "DISABLED";
    FD1S3AX us_tx_i27 (.D(us_tx_next[26]), .CK(pll_clk), .Q(us_tx_c_26)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i27.GSR = "DISABLED";
    FD1S3AX us_tx_i28 (.D(us_tx_next[27]), .CK(pll_clk), .Q(us_tx_c_27)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i28.GSR = "DISABLED";
    FD1S3AX us_tx_i29 (.D(us_tx_next[28]), .CK(pll_clk), .Q(us_tx_c_28)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i29.GSR = "DISABLED";
    FD1S3AX us_tx_i30 (.D(us_tx_next[29]), .CK(pll_clk), .Q(us_tx_c_29)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i30.GSR = "DISABLED";
    FD1S3AX us_tx_i31 (.D(us_tx_next[30]), .CK(pll_clk), .Q(us_tx_c_30)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i31.GSR = "DISABLED";
    FD1S3AX us_tx_i32 (.D(us_tx_next[31]), .CK(pll_clk), .Q(us_tx_c_31)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i32.GSR = "DISABLED";
    FD1S3AX us_tx_i33 (.D(us_tx_next[32]), .CK(pll_clk), .Q(us_tx_c_32)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i33.GSR = "DISABLED";
    FD1S3AX us_tx_i34 (.D(us_tx_next[33]), .CK(pll_clk), .Q(us_tx_c_33)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i34.GSR = "DISABLED";
    FD1S3AX us_tx_i35 (.D(us_tx_next[34]), .CK(pll_clk), .Q(us_tx_c_34)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i35.GSR = "DISABLED";
    FD1S3AX us_tx_i36 (.D(us_tx_next[35]), .CK(pll_clk), .Q(us_tx_c_35)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i36.GSR = "DISABLED";
    FD1S3AX us_tx_i37 (.D(us_tx_next[36]), .CK(pll_clk), .Q(us_tx_c_36)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i37.GSR = "DISABLED";
    FD1S3AX us_tx_i38 (.D(us_tx_next[37]), .CK(pll_clk), .Q(us_tx_c_37)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i38.GSR = "DISABLED";
    FD1S3AX us_tx_i39 (.D(us_tx_next[38]), .CK(pll_clk), .Q(us_tx_c_38)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i39.GSR = "DISABLED";
    FD1S3AX us_tx_i40 (.D(us_tx_next[39]), .CK(pll_clk), .Q(us_tx_c_39)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i40.GSR = "DISABLED";
    FD1S3AX us_tx_i41 (.D(us_tx_next[40]), .CK(pll_clk), .Q(us_tx_c_40)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i41.GSR = "DISABLED";
    FD1S3AX us_tx_i42 (.D(us_tx_next[41]), .CK(pll_clk), .Q(us_tx_c_41)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i42.GSR = "DISABLED";
    FD1S3AX us_tx_i43 (.D(us_tx_next[42]), .CK(pll_clk), .Q(us_tx_c_42)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i43.GSR = "DISABLED";
    FD1S3AX us_tx_i44 (.D(us_tx_next[43]), .CK(pll_clk), .Q(us_tx_c_43)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i44.GSR = "DISABLED";
    FD1S3AX us_tx_i45 (.D(us_tx_next[44]), .CK(pll_clk), .Q(us_tx_c_44)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i45.GSR = "DISABLED";
    FD1S3AX us_tx_i46 (.D(us_tx_next[45]), .CK(pll_clk), .Q(us_tx_c_45)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i46.GSR = "DISABLED";
    FD1S3AX us_tx_i47 (.D(us_tx_next[46]), .CK(pll_clk), .Q(us_tx_c_46)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i47.GSR = "DISABLED";
    FD1S3AX us_tx_i48 (.D(us_tx_next[47]), .CK(pll_clk), .Q(us_tx_c_47)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i48.GSR = "DISABLED";
    FD1S3AX us_tx_i49 (.D(us_tx_next[48]), .CK(pll_clk), .Q(us_tx_c_48)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i49.GSR = "DISABLED";
    FD1S3AX us_tx_i50 (.D(us_tx_next[49]), .CK(pll_clk), .Q(us_tx_c_49)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i50.GSR = "DISABLED";
    FD1S3AX us_tx_i51 (.D(us_tx_next[50]), .CK(pll_clk), .Q(us_tx_c_50)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i51.GSR = "DISABLED";
    FD1S3AX us_tx_i52 (.D(us_tx_next[51]), .CK(pll_clk), .Q(us_tx_c_51)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i52.GSR = "DISABLED";
    FD1S3AX us_tx_i53 (.D(us_tx_next[52]), .CK(pll_clk), .Q(us_tx_c_52)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i53.GSR = "DISABLED";
    FD1S3AX us_tx_i54 (.D(us_tx_next[53]), .CK(pll_clk), .Q(us_tx_c_53)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i54.GSR = "DISABLED";
    FD1S3AX us_tx_i55 (.D(us_tx_next[54]), .CK(pll_clk), .Q(us_tx_c_54)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i55.GSR = "DISABLED";
    FD1S3AX us_tx_i56 (.D(us_tx_next[55]), .CK(pll_clk), .Q(us_tx_c_55)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i56.GSR = "DISABLED";
    FD1S3AX us_tx_i57 (.D(us_tx_next[56]), .CK(pll_clk), .Q(us_tx_c_56)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i57.GSR = "DISABLED";
    FD1S3AX us_tx_i58 (.D(us_tx_next[57]), .CK(pll_clk), .Q(us_tx_c_57)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i58.GSR = "DISABLED";
    FD1S3AX us_tx_i59 (.D(us_tx_next[58]), .CK(pll_clk), .Q(us_tx_c_58)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i59.GSR = "DISABLED";
    FD1S3AX us_tx_i60 (.D(us_tx_next[59]), .CK(pll_clk), .Q(us_tx_c_59)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i60.GSR = "DISABLED";
    FD1S3AX us_tx_i61 (.D(us_tx_next[60]), .CK(pll_clk), .Q(us_tx_c_60)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i61.GSR = "DISABLED";
    FD1S3AX us_tx_i62 (.D(us_tx_next[61]), .CK(pll_clk), .Q(us_tx_c_61)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i62.GSR = "DISABLED";
    FD1S3AX us_tx_i63 (.D(us_tx_next[62]), .CK(pll_clk), .Q(us_tx_c_62)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i63.GSR = "DISABLED";
    FD1S3AX us_tx_i64 (.D(us_tx_next[63]), .CK(pll_clk), .Q(us_tx_c_63)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i64.GSR = "DISABLED";
    FD1S3AX us_tx_i65 (.D(us_tx_next[64]), .CK(pll_clk), .Q(us_tx_c_64)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i65.GSR = "DISABLED";
    FD1S3AX us_tx_i66 (.D(us_tx_next[65]), .CK(pll_clk), .Q(us_tx_c_65)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i66.GSR = "DISABLED";
    FD1S3AX us_tx_i67 (.D(us_tx_next[66]), .CK(pll_clk), .Q(us_tx_c_66)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i67.GSR = "DISABLED";
    FD1S3AX us_tx_i68 (.D(us_tx_next[67]), .CK(pll_clk), .Q(us_tx_c_67)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i68.GSR = "DISABLED";
    FD1S3AX us_tx_i69 (.D(us_tx_next[68]), .CK(pll_clk), .Q(us_tx_c_68)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i69.GSR = "DISABLED";
    FD1S3AX us_tx_i70 (.D(us_tx_next[69]), .CK(pll_clk), .Q(us_tx_c_69)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i70.GSR = "DISABLED";
    FD1S3AX us_tx_i71 (.D(us_tx_next[70]), .CK(pll_clk), .Q(us_tx_c_70)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i71.GSR = "DISABLED";
    FD1S3AX us_tx_i72 (.D(us_tx_next[71]), .CK(pll_clk), .Q(us_tx_c_71)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i72.GSR = "DISABLED";
    FD1S3AX us_tx_i73 (.D(us_tx_next[72]), .CK(pll_clk), .Q(us_tx_c_72)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i73.GSR = "DISABLED";
    FD1S3AX us_tx_i74 (.D(us_tx_next[73]), .CK(pll_clk), .Q(us_tx_c_73)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i74.GSR = "DISABLED";
    FD1S3AX us_tx_i75 (.D(us_tx_next[74]), .CK(pll_clk), .Q(us_tx_c_74)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i75.GSR = "DISABLED";
    FD1S3AX us_tx_i76 (.D(us_tx_next[75]), .CK(pll_clk), .Q(us_tx_c_75)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i76.GSR = "DISABLED";
    FD1S3AX us_tx_i77 (.D(us_tx_next[76]), .CK(pll_clk), .Q(us_tx_c_76)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i77.GSR = "DISABLED";
    FD1S3AX us_tx_i78 (.D(us_tx_next[77]), .CK(pll_clk), .Q(us_tx_c_77)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i78.GSR = "DISABLED";
    FD1S3AX us_tx_i79 (.D(us_tx_next[78]), .CK(pll_clk), .Q(us_tx_c_78)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i79.GSR = "DISABLED";
    FD1S3AX us_tx_i80 (.D(us_tx_next[79]), .CK(pll_clk), .Q(us_tx_c_79)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i80.GSR = "DISABLED";
    FD1S3AX us_tx_i81 (.D(us_tx_next[80]), .CK(pll_clk), .Q(us_tx_c_80)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i81.GSR = "DISABLED";
    FD1S3AX us_tx_i82 (.D(us_tx_next[81]), .CK(pll_clk), .Q(us_tx_c_81)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i82.GSR = "DISABLED";
    FD1S3AX us_tx_i83 (.D(us_tx_next[82]), .CK(pll_clk), .Q(us_tx_c_82)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i83.GSR = "DISABLED";
    FD1S3AX us_tx_i84 (.D(us_tx_next[83]), .CK(pll_clk), .Q(us_tx_c_83)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[12:52])
    defparam us_tx_i84.GSR = "DISABLED";
    FD1S3AX pll_lock_sync_i1 (.D(pll_lock_sync[0]), .CK(pll_clk), .Q(status_flags_wire_15__N_1355[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(129[12:77])
    defparam pll_lock_sync_i1.GSR = "DISABLED";
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_16028_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[62]), .C(status_hold[63]), .D(status_next_index[1]), 
         .Z(n27208)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_16028_4_lut_4_lut.init = 16'h55d8;
    LUT4 i1_2_lut_3_lut_adj_251 (.A(spi_command[0]), .B(spi_command[1]), 
         .C(n16894), .Z(spi_mod_feed_N_2470)) /* synthesis lut_function=(A+((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(173[32:61])
    defparam i1_2_lut_3_lut_adj_251.init = 16'hfbfb;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_16108_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[118]), .C(status_hold[119]), .D(status_next_index[1]), 
         .Z(n27400)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_16108_4_lut_4_lut.init = 16'h55d8;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_15941_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[54]), .C(status_hold[55]), .D(status_next_index[1]), 
         .Z(n27196)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_15941_4_lut_4_lut.init = 16'h55d8;
    LUT4 i2_3_lut_adj_252 (.A(n6_adj_3288), .B(pcm_payload_bit[7]), .C(pcm_rd_ptr_spi[3]), 
         .Z(pcm_spi_sample_15__N_1371[3])) /* synthesis lut_function=(A (B (C)+!B !(C))+!A !(B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[18:55])
    defparam i2_3_lut_adj_252.init = 16'h9696;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_15853_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[46]), .C(status_hold[47]), .D(status_next_index[1]), 
         .Z(n27058)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_15853_4_lut_4_lut.init = 16'h55d8;
    LUT4 spi1_miso_N_2443_2__bdd_4_lut_15931_4_lut_4_lut (.A(status_bit_index[3]), 
         .B(n28008), .C(n27939), .D(status_bit_index[2]), .Z(n27124)) /* synthesis lut_function=(!(A (D)+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(429[27:54])
    defparam spi1_miso_N_2443_2__bdd_4_lut_15931_4_lut_4_lut.init = 16'h50ee;
    LUT4 i7_4_lut_adj_253 (.A(n13_adj_3162), .B(n11_adj_3163), .C(spi_byte_count[8]), 
         .D(n25468), .Z(n16855)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i7_4_lut_adj_253.init = 16'h0800;
    LUT4 i1_2_lut_3_lut_adj_254 (.A(spi_bit_count_2__N_1415), .B(fpga_cs_n_c), 
         .C(n32_adj_3179), .Z(spi_payload_byte)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_254.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_255 (.A(staging_rd_page_N_2719), .B(ev_fall_N_2683), 
         .C(staging_rd_page_N_2718), .Z(staging_rd_page)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(322[35] 323[54])
    defparam i1_2_lut_3_lut_adj_255.init = 16'hfefe;
    LUT4 i5_4_lut_adj_256 (.A(n20658), .B(spi_byte_count[4]), .C(spi_byte_count[2]), 
         .D(expected_next_8__N_1396[3]), .Z(n13_adj_3162)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i5_4_lut_adj_256.init = 16'h1000;
    LUT4 i4_4_lut_adj_257 (.A(spi_byte_count[7]), .B(n8_adj_3190), .C(spi_byte_count[3]), 
         .D(spi_rgb_start[7]), .Z(n25468)) /* synthesis lut_function=(A (B (C (D)))+!A !((C+(D))+!B)) */ ;
    defparam i4_4_lut_adj_257.init = 16'h8004;
    LUT4 i1_2_lut_3_lut_adj_258 (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(n16_adj_3282), .Z(n16929)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[30:53])
    defparam i1_2_lut_3_lut_adj_258.init = 16'hfefe;
    LUT4 i9999_4_lut (.A(ev_rd_data[54]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[54])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9999_4_lut.init = 16'h1222;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_15946_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[58]), .C(status_hold[59]), .D(status_next_index[1]), 
         .Z(n27202)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_15946_4_lut_4_lut.init = 16'h55d8;
    LUT4 i3429_3_lut (.A(pcm_rd_ptr_spi[2]), .B(pcm_payload_bit[6]), .C(n4_adj_3281), 
         .Z(n6_adj_3288)) /* synthesis lut_function=(A (B+(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[18:55])
    defparam i3429_3_lut.init = 16'he8e8;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_15936_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[50]), .C(status_hold[51]), .D(status_next_index[1]), 
         .Z(n27094)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_15936_4_lut_4_lut.init = 16'h55d8;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_16043_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[18]), .C(status_hold[19]), .D(status_next_index[1]), 
         .Z(n27310)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_16043_4_lut_4_lut.init = 16'h55d8;
    LUT4 frame_settle_3__bdd_2_lut_3_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n27526), .Z(n27527)) /* synthesis lut_function=(A ((C)+!B)+!A (B+(C))) */ ;
    defparam frame_settle_3__bdd_2_lut_3_lut.init = 16'hf6f6;
    LUT4 i1_2_lut_3_lut_4_lut_adj_259 (.A(spi_byte_count[0]), .B(n7), .C(spi_byte_count[2]), 
         .D(spi_byte_count[1]), .Z(spi1_sck_c_enable_100)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_259.init = 16'h8000;
    LUT4 i1_2_lut_3_lut_4_lut_adj_260 (.A(mic_phase[3]), .B(mic_phase[2]), 
         .C(mic_phase[1]), .D(pcm_fifo_N_2613), .Z(n22386)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_3_lut_4_lut_adj_260.init = 16'h0800;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_16078_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[22]), .C(status_hold[23]), .D(status_next_index[1]), 
         .Z(n27328)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_16078_4_lut_4_lut.init = 16'h55d8;
    LUT4 i3_4_lut_adj_261 (.A(spi_byte_count[6]), .B(n25492), .C(spi_rgb_start[7]), 
         .D(spi_byte_count[5]), .Z(n8_adj_3190)) /* synthesis lut_function=(!(A (((D)+!C)+!B)+!A ((C+!(D))+!B))) */ ;
    defparam i3_4_lut_adj_261.init = 16'h0480;
    LUT4 run_addr_s3_8__I_0_i9_3_lut_4_lut (.A(run_addr_s3[8]), .B(active_bank), 
         .C(staging_rd_page_N_2719), .D(phase_step_s3), .Z(event_rd_addr[8])) /* synthesis lut_function=(A (((D)+!C)+!B)+!A !(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i9_3_lut_4_lut.init = 16'haa3a;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[42]), .C(status_hold[43]), .D(status_next_index[1]), 
         .Z(n27430)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_4_lut_4_lut.init = 16'h55d8;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_16118_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[34]), .C(status_hold[35]), .D(status_next_index[1]), 
         .Z(n27406)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_16118_4_lut_4_lut.init = 16'h55d8;
    LUT4 i1_4_lut_adj_262 (.A(spi_rgb_start[7]), .B(n21228), .C(n81_adj_3287), 
         .D(spi_byte_count[7]), .Z(n25492)) /* synthesis lut_function=((B (C (D))+!B (C+!(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam i1_4_lut_adj_262.init = 16'hf577;
    LUT4 run_addr_s3_8__I_0_i8_3_lut_4_lut (.A(run_addr_s3[7]), .B(rmw_slot[7]), 
         .C(staging_rd_page_N_2719), .D(phase_step_s3), .Z(event_rd_addr[7])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i8_3_lut_4_lut.init = 16'haaca;
    LUT4 i10478_4_lut (.A(spi_byte_count[6]), .B(spi_byte_count[5]), .C(spi_byte_count[2]), 
         .D(n20565), .Z(n21228)) /* synthesis lut_function=(A+(B (C+(D)))) */ ;
    defparam i10478_4_lut.init = 16'heeea;
    LUT4 run_addr_s3_8__I_0_i7_3_lut_4_lut (.A(run_addr_s3[6]), .B(rmw_slot[6]), 
         .C(staging_rd_page_N_2719), .D(phase_step_s3), .Z(event_rd_addr[6])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i7_3_lut_4_lut.init = 16'haaca;
    LUT4 i9913_2_lut (.A(spi_byte_count[0]), .B(spi_byte_count[1]), .Z(n20658)) /* synthesis lut_function=(A (B)) */ ;
    defparam i9913_2_lut.init = 16'h8888;
    LUT4 i9998_4_lut (.A(ev_rd_data[55]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[55])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9998_4_lut.init = 16'h1222;
    LUT4 i15098_3_lut_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[0]), 
         .C(spi_byte_count[1]), .D(n7), .Z(spi1_sck_c_enable_76)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i15098_3_lut_4_lut.init = 16'h0200;
    LUT4 i13526_2_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), .Z(n19_adj_3168)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam i13526_2_lut.init = 16'h6666;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_16128_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[38]), .C(status_hold[39]), .D(status_next_index[1]), 
         .Z(n27418)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_16128_4_lut_4_lut.init = 16'h55d8;
    LUT4 i13533_3_lut (.A(spi_bit_count[2]), .B(spi_bit_count[1]), .C(spi_bit_count[0]), 
         .Z(n18_adj_3169)) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam i13533_3_lut.init = 16'h6a6a;
    LUT4 run_addr_s3_8__I_0_i6_3_lut_4_lut (.A(run_addr_s3[5]), .B(rmw_slot[5]), 
         .C(staging_rd_page_N_2719), .D(phase_step_s3), .Z(event_rd_addr[5])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i6_3_lut_4_lut.init = 16'haaca;
    LUT4 run_addr_s3_8__I_0_i5_3_lut_4_lut (.A(run_addr_s3[4]), .B(rmw_slot[4]), 
         .C(staging_rd_page_N_2719), .D(phase_step_s3), .Z(event_rd_addr[4])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i5_3_lut_4_lut.init = 16'haaca;
    LUT4 i3_2_lut (.A(spi_bit_count_2__N_1415), .B(fpga_cs_n_c), .Z(n11_adj_3163)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i3_2_lut.init = 16'h2222;
    LUT4 n27184_bdd_4_lut_else_3_lut (.A(n14024), .B(n14025), .C(status_bit_index[0]), 
         .Z(n27967)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27184_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 compact_end_I_0_2_lut_3_lut_4_lut (.A(compact_end), .B(spi_command[0]), 
         .C(spi_command[1]), .D(n16894), .Z(audio_reset_end)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(587[28:62])
    defparam compact_end_I_0_2_lut_3_lut_4_lut.init = 16'h0002;
    LUT4 status_next_bit_N_2754_0__bdd_4_lut_16093_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[26]), .C(status_hold[27]), .D(status_next_index[1]), 
         .Z(n27370)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_N_2754_0__bdd_4_lut_16093_4_lut_4_lut.init = 16'h55d8;
    LUT4 n27124_bdd_4_lut_4_lut (.A(status_bit_index[3]), .B(n27945), .C(n27942), 
         .D(n27124), .Z(n27127)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(429[27:54])
    defparam n27124_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 n27154_bdd_4_lut (.A(n27154), .B(\mic_win_q[0] [6]), .C(\mic_win_i[0] [6]), 
         .D(mic_acc_word[0]), .Z(n27157)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27154_bdd_4_lut.init = 16'haad8;
    LUT4 i1_4_lut_adj_263 (.A(spi_byte_count[1]), .B(n35_adj_3269), .C(n6_adj_3252), 
         .D(spi_byte_count[3]), .Z(n22119)) /* synthesis lut_function=(A (B ((D)+!C))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(574[34:54])
    defparam i1_4_lut_adj_263.init = 16'hcc4c;
    LUT4 mic_acc_word_2__bdd_4_lut_15964 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [6]), 
         .C(\mic_win_q[1] [6]), .D(mic_acc_word[0]), .Z(n27154)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15964.init = 16'he4aa;
    LUT4 run_addr_s3_8__I_0_i4_3_lut_4_lut (.A(run_addr_s3[3]), .B(rmw_slot[3]), 
         .C(staging_rd_page_N_2719), .D(phase_step_s3), .Z(event_rd_addr[3])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i4_3_lut_4_lut.init = 16'haaca;
    LUT4 run_addr_s3_8__I_0_i3_3_lut_4_lut (.A(run_addr_s3[2]), .B(rmw_slot[2]), 
         .C(staging_rd_page_N_2719), .D(phase_step_s3), .Z(event_rd_addr[2])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i3_3_lut_4_lut.init = 16'haaca;
    LUT4 i1_2_lut_4_lut_adj_264 (.A(frame_settle[3]), .B(frame_settle[1]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(n7_adj_3256)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(663[22:42])
    defparam i1_2_lut_4_lut_adj_264.init = 16'hfeff;
    LUT4 run_addr_s3_8__I_0_i2_3_lut_4_lut (.A(run_addr_s3[1]), .B(rmw_slot[1]), 
         .C(staging_rd_page_N_2719), .D(phase_step_s3), .Z(event_rd_addr[1])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i2_3_lut_4_lut.init = 16'haaca;
    LUT4 run_addr_s3_8__I_0_i1_3_lut_4_lut (.A(run_addr_s3[0]), .B(rmw_slot[0]), 
         .C(staging_rd_page_N_2719), .D(phase_step_s3), .Z(event_rd_addr[0])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i1_3_lut_4_lut.init = 16'haaca;
    LUT4 i2_2_lut_adj_265 (.A(n38_adj_3151), .B(n15_adj_3283), .Z(n6_adj_3252)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i2_2_lut_adj_265.init = 16'h2222;
    LUT4 i1_4_lut_adj_266 (.A(spi_byte_count[5]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[2]), .D(spi_byte_count[0]), .Z(n38_adj_3151)) /* synthesis lut_function=(!(A (B+(C))+!A (B (C)+!B !(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(480[21] 528[24])
    defparam i1_4_lut_adj_266.init = 16'h1617;
    LUT4 i1_2_lut_3_lut_4_lut_adj_267 (.A(ev_fall_N_2683), .B(frame_req), 
         .C(swap_pending), .D(build_idle_N_2714), .Z(n17726)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_267.init = 16'h0400;
    LUT4 i1_2_lut_3_lut_4_lut_adj_268 (.A(spi_command[0]), .B(spi_command[5]), 
         .C(spi_command[4]), .D(n13_adj_3166), .Z(n25474)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_268.init = 16'h0010;
    LUT4 i1_2_lut_3_lut_4_lut_adj_269 (.A(n31), .B(frame_end), .C(spi_bit_count_2__N_1415), 
         .D(fpga_cs_n_c), .Z(spi1_sck_c_enable_69)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i1_2_lut_3_lut_4_lut_adj_269.init = 16'h0040;
    LUT4 i1_2_lut_3_lut_4_lut_adj_270 (.A(spi_channel_page), .B(spi_command[0]), 
         .C(spi_command[1]), .D(n16894), .Z(spi_channel_page_N_2479)) /* synthesis lut_function=(!(A (((D)+!C)+!B)+!A !(((D)+!C)+!B))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_270.init = 16'h5595;
    LUT4 i1_2_lut_4_lut_adj_271 (.A(frame_settle[3]), .B(frame_settle[1]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(ws2812_enable_N_2591)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(663[22:42])
    defparam i1_2_lut_4_lut_adj_271.init = 16'hfffe;
    LUT4 i1_2_lut_4_lut_adj_272 (.A(pll_clk_enable_597), .B(mic_run_state_1__N_2390[1]), 
         .C(mic_acc_busy_N_2272), .D(mic_done_N_2646), .Z(pll_clk_enable_31)) /* synthesis lut_function=(A+(B+(C (D)))) */ ;
    defparam i1_2_lut_4_lut_adj_272.init = 16'hfeee;
    LUT4 i6621_2_lut_4_lut (.A(frame_settle[3]), .B(frame_settle[1]), .C(frame_settle[2]), 
         .D(frame_settle[0]), .Z(n17390)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(663[22:42])
    defparam i6621_2_lut_4_lut.init = 16'h00fe;
    LUT4 n14039_bdd_3_lut_16656 (.A(n14039), .B(n14038), .C(status_bit_index[0]), 
         .Z(n28128)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n14039_bdd_3_lut_16656.init = 16'hcaca;
    LUT4 i10023_4_lut (.A(ev_rd_data[30]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[30])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10023_4_lut.init = 16'h1222;
    LUT4 i14774_4_lut_4_lut_4_lut (.A(status_next_index[5]), .B(status_next_index[4]), 
         .C(n26549), .D(n27397), .Z(n25716)) /* synthesis lut_function=(!(A (B+!(D))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam i14774_4_lut_4_lut_4_lut.init = 16'h7250;
    LUT4 n27178_bdd_4_lut_then_3_lut (.A(mic_ram_rd_data[4]), .B(mic_ram_rd_data[6]), 
         .C(status_bit_index[1]), .Z(n27971)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27178_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 i4562_4_lut (.A(ev_sum[2]), .B(staging_q[2]), .C(staging_rd_page_N_2718), 
         .D(ev_in_clear), .Z(n15324)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam i4562_4_lut.init = 16'hcac0;
    LUT4 i1_3_lut_4_lut_adj_273 (.A(spi_bit_count_2__N_1415), .B(spi_mod_feed_N_2470), 
         .C(n17_adj_3270), .D(n17_adj_3278), .Z(n35_adj_3269)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(85[24:33])
    defparam i1_3_lut_4_lut_adj_273.init = 16'h8000;
    LUT4 i1_2_lut_4_lut_adj_274 (.A(spi_command[5]), .B(spi_command[4]), 
         .C(spi_command[6]), .D(spi_command[7]), .Z(n18)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i1_2_lut_4_lut_adj_274.init = 16'h0004;
    LUT4 n27178_bdd_4_lut_else_3_lut (.A(mic_ram_rd_data[5]), .B(mic_ram_rd_data[7]), 
         .C(status_bit_index[1]), .Z(n27970)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27178_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 i2_3_lut_4_lut_adj_275 (.A(spi_bit_count_2__N_1415), .B(spi_mod_feed_N_2470), 
         .C(n8_adj_3177), .D(spi_byte_count[1]), .Z(n25414)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(85[24:33])
    defparam i2_3_lut_4_lut_adj_275.init = 16'h0800;
    LUT4 i3117_2_lut_3_lut (.A(feed_pop), .B(pll_clk_enable_2), .C(n10668), 
         .Z(pll_clk_enable_318)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i3117_2_lut_3_lut.init = 16'hfefe;
    LUT4 i1_2_lut_adj_276 (.A(spi_command[5]), .B(spi_command[4]), .Z(n16910)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(174[38:60])
    defparam i1_2_lut_adj_276.init = 16'hbbbb;
    LUT4 i1_4_lut_adj_277 (.A(spi_byte_count[6]), .B(spi_byte_count[3]), 
         .C(n4_adj_3147), .D(spi_byte_count[2]), .Z(n81_adj_3287)) /* synthesis lut_function=(A (B (C+(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i1_4_lut_adj_277.init = 16'ha8a0;
    LUT4 i2_3_lut_4_lut_adj_278 (.A(pcm_rd_ptr_spi[0]), .B(pcm_payload_bit[4]), 
         .C(pcm_rd_ptr_spi[1]), .D(pcm_payload_bit[5]), .Z(pcm_spi_sample_15__N_1371[1])) /* synthesis lut_function=(A (B (C (D)+!C !(D))+!B !(C (D)+!C !(D)))+!A !(C (D)+!C !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[18:55])
    defparam i2_3_lut_4_lut_adj_278.init = 16'h8778;
    umh_toggle_ram84 event_ram (.pll_clk(pll_clk), .ev_we(ev_we), .VCC_net(VCC_net), 
            .GND_net(GND_net), .rmw_slot({rmw_slot}), .event_rd_addr({event_rd_addr}), 
            .active_bank_N_892(active_bank_N_892), .ev_wr_data({ev_wr_data}), 
            .ev_rd_data({ev_rd_data})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[22] 342[6])
    L6MUX21 i16511 (.D0(n27969), .D1(n28132), .SD(status_bit_index[2]), 
            .Z(n28133));
    CCU2D add_1585_19 (.A0(phase_frac[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24322), .COUT(n24323), .S0(phase_frac_sum[17]), 
          .S1(phase_frac_sum[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_19.INIT0 = 16'h5aaa;
    defparam add_1585_19.INIT1 = 16'h5aaa;
    defparam add_1585_19.INJECT1_0 = "NO";
    defparam add_1585_19.INJECT1_1 = "NO";
    CCU2D add_1585_17 (.A0(phase_frac[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24321), .COUT(n24322), .S0(phase_frac_sum[15]), 
          .S1(phase_frac_sum[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_17.INIT0 = 16'h5555;
    defparam add_1585_17.INIT1 = 16'h5aaa;
    defparam add_1585_17.INJECT1_0 = "NO";
    defparam add_1585_17.INJECT1_1 = "NO";
    CCU2D add_1585_15 (.A0(phase_frac[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24320), .COUT(n24321), .S0(phase_frac_sum[13]), 
          .S1(phase_frac_sum[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_15.INIT0 = 16'h5555;
    defparam add_1585_15.INIT1 = 16'h5555;
    defparam add_1585_15.INJECT1_0 = "NO";
    defparam add_1585_15.INJECT1_1 = "NO";
    CCU2D add_1585_13 (.A0(phase_frac[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24319), .COUT(n24320), .S0(phase_frac_sum[11]), 
          .S1(phase_frac_sum[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_13.INIT0 = 16'h5aaa;
    defparam add_1585_13.INIT1 = 16'h5555;
    defparam add_1585_13.INJECT1_0 = "NO";
    defparam add_1585_13.INJECT1_1 = "NO";
    CCU2D add_1585_11 (.A0(phase_frac[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24318), .COUT(n24319), .S0(phase_frac_sum[9]), 
          .S1(phase_frac_sum[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_11.INIT0 = 16'h5aaa;
    defparam add_1585_11.INIT1 = 16'h5555;
    defparam add_1585_11.INJECT1_0 = "NO";
    defparam add_1585_11.INJECT1_1 = "NO";
    PFUMX i16215 (.BLUT(n27588), .ALUT(n27583), .C0(spi_version[2]), .Z(n27589));
    CCU2D add_1585_23 (.A0(phase_frac[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24324), .COUT(n24325), .S0(phase_frac_sum[21]), 
          .S1(phase_frac_sum[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_23.INIT0 = 16'h5555;
    defparam add_1585_23.INIT1 = 16'h5aaa;
    defparam add_1585_23.INJECT1_0 = "NO";
    defparam add_1585_23.INJECT1_1 = "NO";
    CCU2D add_1585_7 (.A0(phase_frac[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24316), .COUT(n24317), .S0(phase_frac_sum[5]), 
          .S1(phase_frac_sum[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_7.INIT0 = 16'h5aaa;
    defparam add_1585_7.INIT1 = 16'h5555;
    defparam add_1585_7.INJECT1_0 = "NO";
    defparam add_1585_7.INJECT1_1 = "NO";
    LUT4 i10022_4_lut (.A(ev_rd_data[31]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[31])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10022_4_lut.init = 16'h1222;
    LUT4 n10819_bdd_2_lut (.A(n10819), .B(n28391), .Z(n28392)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam n10819_bdd_2_lut.init = 16'h4444;
    LUT4 i3422_3_lut_4_lut (.A(pcm_rd_ptr_spi[0]), .B(pcm_payload_bit[4]), 
         .C(pcm_payload_bit[5]), .D(pcm_rd_ptr_spi[1]), .Z(n4_adj_3281)) /* synthesis lut_function=(A (B (C+(D))+!B (C (D)))+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[18:55])
    defparam i3422_3_lut_4_lut.init = 16'hf880;
    LUT4 i9997_4_lut (.A(ev_rd_data[56]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[56])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9997_4_lut.init = 16'h1222;
    LUT4 i1_2_lut_4_lut_adj_279 (.A(fpga_time[13]), .B(fpga_time[11]), .C(mic_warm), 
         .D(mic_ultrasonic), .Z(pll_clk_enable_258)) /* synthesis lut_function=(A (B+(C+(D)))+!A (C+(D))) */ ;
    defparam i1_2_lut_4_lut_adj_279.init = 16'hfff8;
    LUT4 i10021_4_lut (.A(ev_rd_data[32]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[32])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10021_4_lut.init = 16'h1222;
    LUT4 i10020_4_lut (.A(ev_rd_data[33]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[33])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10020_4_lut.init = 16'h1222;
    CCU2D feed_wr_7__I_0_1375_add_2_5 (.A0(feed_wr[3]), .B0(feed_rd[3]), 
          .C0(GND_net), .D0(GND_net), .A1(feed_wr[4]), .B1(feed_rd[4]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24521), .COUT(n24522), .S0(feed_fill[3]), 
          .S1(feed_fill[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[29:46])
    defparam feed_wr_7__I_0_1375_add_2_5.INIT0 = 16'h5999;
    defparam feed_wr_7__I_0_1375_add_2_5.INIT1 = 16'h5999;
    defparam feed_wr_7__I_0_1375_add_2_5.INJECT1_0 = "NO";
    defparam feed_wr_7__I_0_1375_add_2_5.INJECT1_1 = "NO";
    PFUMX i16213 (.BLUT(n27586), .ALUT(n27585), .C0(n7_adj_3219), .Z(n27587));
    FD1S3IX ev_state_FSM_i2 (.D(n15019), .CK(pll_clk), .CD(n10668), .Q(ev_in_clear));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam ev_state_FSM_i2.GSR = "DISABLED";
    LUT4 i10019_4_lut (.A(ev_rd_data[34]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[34])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10019_4_lut.init = 16'h1222;
    CCU2D feed_wr_7__I_0_1375_add_2_3 (.A0(feed_wr[1]), .B0(feed_rd[1]), 
          .C0(GND_net), .D0(GND_net), .A1(feed_wr[2]), .B1(feed_rd[2]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24520), .COUT(n24521), .S0(feed_fill[1]), 
          .S1(feed_fill[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[29:46])
    defparam feed_wr_7__I_0_1375_add_2_3.INIT0 = 16'h5999;
    defparam feed_wr_7__I_0_1375_add_2_3.INIT1 = 16'h5999;
    defparam feed_wr_7__I_0_1375_add_2_3.INJECT1_0 = "NO";
    defparam feed_wr_7__I_0_1375_add_2_3.INJECT1_1 = "NO";
    LUT4 i10080_4_lut (.A(mod_settle[3]), .B(feed_rd_7__N_1692), .C(mod_settle[2]), 
         .D(n4_adj_3272), .Z(n3592)) /* synthesis lut_function=(A (B+(C+(D)))+!A (B+!(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(692[18] 701[12])
    defparam i10080_4_lut.init = 16'heeed;
    CCU2D add_558_7 (.A0(n23788), .B0(compact_end), .C0(pcm_read_count_latched_spi[4]), 
          .D0(n16895), .A1(GND_net), .B1(audio_reset_end), .C1(pcm_wr_sync2_spi[5]), 
          .D1(pcm_rd_ptr_spi[5]), .CIN(n24391), .S0(pcm_rd_ptr_spi_5__N_1610[4]), 
          .S1(pcm_rd_ptr_spi_5__N_1610[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[31:74])
    defparam add_558_7.INIT0 = 16'h5a9a;
    defparam add_558_7.INIT1 = 16'h596a;
    defparam add_558_7.INJECT1_0 = "NO";
    defparam add_558_7.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_280 (.A(mic_warm), .B(mic_phase[1]), .C(mic_phase[0]), 
         .Z(n5636)) /* synthesis lut_function=(!(A+(B (C)+!B !(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_3_lut_adj_280.init = 16'h1414;
    LUT4 i3317_2_lut_3_lut (.A(n19096), .B(pcm_fifo_N_2607[5]), .C(pcm_wr_ptr[4]), 
         .Z(n14051)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;
    defparam i3317_2_lut_3_lut.init = 16'h0202;
    LUT4 i2803_2_lut_3_lut (.A(build_idle_N_2714), .B(frame_req), .C(swap_pending), 
         .Z(n13154)) /* synthesis lut_function=(A ((C)+!B)) */ ;
    defparam i2803_2_lut_3_lut.init = 16'ha2a2;
    CCU2D feed_wr_7__I_0_1375_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(feed_wr[0]), .B1(feed_rd[0]), .C1(GND_net), 
          .D1(GND_net), .COUT(n24520), .S1(feed_fill[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[29:46])
    defparam feed_wr_7__I_0_1375_add_2_1.INIT0 = 16'h0000;
    defparam feed_wr_7__I_0_1375_add_2_1.INIT1 = 16'h5999;
    defparam feed_wr_7__I_0_1375_add_2_1.INJECT1_0 = "NO";
    defparam feed_wr_7__I_0_1375_add_2_1.INJECT1_1 = "NO";
    LUT4 i10018_4_lut (.A(ev_rd_data[35]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[35])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10018_4_lut.init = 16'h1222;
    LUT4 i3318_2_lut_3_lut (.A(n19096), .B(pcm_fifo_N_2607[5]), .C(pcm_wr_ptr[4]), 
         .Z(n14052)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i3318_2_lut_3_lut.init = 16'h2020;
    LUT4 i1_2_lut_adj_281 (.A(pll_clk_enable_538), .B(n10668), .Z(n16096)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_281.init = 16'heeee;
    LUT4 i4096_2_lut_3_lut (.A(phase_step_s2), .B(wrap_s2), .C(swap_pending), 
         .Z(pll_clk_enable_304)) /* synthesis lut_function=(A+(B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(621[18] 626[12])
    defparam i4096_2_lut_3_lut.init = 16'heaea;
    CCU2D sub_1610_add_2_7 (.A0(status_bit_index[8]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24519), .S0(pcm_payload_bit[8]));
    defparam sub_1610_add_2_7.INIT0 = 16'h5555;
    defparam sub_1610_add_2_7.INIT1 = 16'h0000;
    defparam sub_1610_add_2_7.INJECT1_0 = "NO";
    defparam sub_1610_add_2_7.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_282 (.A(feed_pop), .B(pll_clk_enable_2), .Z(frame_req_mod_N_2737)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(692[18] 701[12])
    defparam i1_2_lut_adj_282.init = 16'heeee;
    FD1S3IX ev_state_FSM_i3 (.D(n16841), .CK(pll_clk), .CD(n10668), .Q(n10024));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam ev_state_FSM_i3.GSR = "DISABLED";
    FD1S3IX ev_state_FSM_i4 (.D(n10058), .CK(pll_clk), .CD(n10668), .Q(staging_rd_page_N_2718));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam ev_state_FSM_i4.GSR = "DISABLED";
    FD1S3IX ev_state_FSM_i5 (.D(n25425), .CK(pll_clk), .CD(n10668), .Q(staging_rd_page_N_2719));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam ev_state_FSM_i5.GSR = "DISABLED";
    FD1S3IX ev_state_FSM_i6 (.D(ev_rd_grant), .CK(pll_clk), .CD(n10668), 
            .Q(ev_fall_N_2683));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam ev_state_FSM_i6.GSR = "DISABLED";
    CCU2D add_558_5 (.A0(n23792), .B0(compact_end), .C0(pcm_read_count_latched_spi[2]), 
          .D0(n16895), .A1(n23790), .B1(compact_end), .C1(pcm_read_count_latched_spi[3]), 
          .D1(n16895), .CIN(n24390), .COUT(n24391), .S0(pcm_rd_ptr_spi_5__N_1610[2]), 
          .S1(pcm_rd_ptr_spi_5__N_1610[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[31:74])
    defparam add_558_5.INIT0 = 16'h5a9a;
    defparam add_558_5.INIT1 = 16'h5a9a;
    defparam add_558_5.INJECT1_0 = "NO";
    defparam add_558_5.INJECT1_1 = "NO";
    CCU2D sub_1610_add_2_5 (.A0(status_bit_index[6]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[7]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24518), .COUT(n24519), .S0(pcm_payload_bit[6]), 
          .S1(pcm_payload_bit[7]));
    defparam sub_1610_add_2_5.INIT0 = 16'h5555;
    defparam sub_1610_add_2_5.INIT1 = 16'h5aaa;
    defparam sub_1610_add_2_5.INJECT1_0 = "NO";
    defparam sub_1610_add_2_5.INJECT1_1 = "NO";
    LUT4 active_bank_I_0_1360_2_lut_3_lut (.A(active_bank), .B(wrap_s2), 
         .C(swap_pending), .Z(active_bank_N_893)) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(621[18] 626[12])
    defparam active_bank_I_0_1360_2_lut_3_lut.init = 16'h6a6a;
    CCU2D add_558_3 (.A0(n23796), .B0(compact_end), .C0(pcm_read_count_latched_spi[0]), 
          .D0(n16895), .A1(n23794), .B1(compact_end), .C1(pcm_read_count_latched_spi[1]), 
          .D1(n16895), .CIN(n24389), .COUT(n24390), .S0(pcm_rd_ptr_spi_5__N_1610[0]), 
          .S1(pcm_rd_ptr_spi_5__N_1610[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[31:74])
    defparam add_558_3.INIT0 = 16'h5a9a;
    defparam add_558_3.INIT1 = 16'h5a9a;
    defparam add_558_3.INJECT1_0 = "NO";
    defparam add_558_3.INJECT1_1 = "NO";
    CCU2D sub_1610_add_2_3 (.A0(status_bit_index[4]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[5]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24517), .COUT(n24518), .S0(pcm_payload_bit[4]), 
          .S1(pcm_payload_bit[5]));
    defparam sub_1610_add_2_3.INIT0 = 16'h5555;
    defparam sub_1610_add_2_3.INIT1 = 16'h5555;
    defparam sub_1610_add_2_3.INJECT1_0 = "NO";
    defparam sub_1610_add_2_3.INJECT1_1 = "NO";
    LUT4 spi_byte_count_0__bdd_2_lut (.A(spi_byte_count[0]), .B(n27574), 
         .Z(spi1_sck_c_enable_71)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam spi_byte_count_0__bdd_2_lut.init = 16'h4444;
    CCU2D add_558_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(compact_end), .B1(n16895), .C1(GND_net), .D1(GND_net), 
          .COUT(n24389));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[31:74])
    defparam add_558_1.INIT0 = 16'hF000;
    defparam add_558_1.INIT1 = 16'hffff;
    defparam add_558_1.INJECT1_0 = "NO";
    defparam add_558_1.INJECT1_1 = "NO";
    CCU2D sub_1610_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[3]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24517), .S1(pcm_payload_bit[3]));
    defparam sub_1610_add_2_1.INIT0 = 16'hF000;
    defparam sub_1610_add_2_1.INIT1 = 16'h5555;
    defparam sub_1610_add_2_1.INJECT1_0 = "NO";
    defparam sub_1610_add_2_1.INJECT1_1 = "NO";
    CCU2D pcm_wr_sync2_spi_5__I_0_add_2_7 (.A0(pcm_wr_sync2_spi[5]), .B0(pcm_rd_ptr_spi[5]), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24516), .S0(pcm_count_spi[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[33:66])
    defparam pcm_wr_sync2_spi_5__I_0_add_2_7.INIT0 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_7.INIT1 = 16'h0000;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_7.INJECT1_0 = "NO";
    defparam pcm_wr_sync2_spi_5__I_0_add_2_7.INJECT1_1 = "NO";
    LUT4 i15062_4_lut (.A(n7_adj_3217), .B(n21213), .C(spi_byte_count[0]), 
         .D(spi_byte_count[4]), .Z(spi1_sck_c_enable_36)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i15062_4_lut.init = 16'h0100;
    LUT4 i15197_2_lut_3_lut_4_lut (.A(invalid_frame_spi_N_2546), .B(n20546), 
         .C(n21266), .D(spi1_sck_c_enable_146), .Z(spi1_sck_c_enable_145)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;
    defparam i15197_2_lut_3_lut_4_lut.init = 16'h4000;
    CCU2D global_phase_s2_3047_add_4_8 (.A0(global_phase_s2[6]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[7]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24384), .S0(n39_adj_3174), 
          .S1(n38_adj_3175));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047_add_4_8.INIT0 = 16'hfaaa;
    defparam global_phase_s2_3047_add_4_8.INIT1 = 16'hfaaa;
    defparam global_phase_s2_3047_add_4_8.INJECT1_0 = "NO";
    defparam global_phase_s2_3047_add_4_8.INJECT1_1 = "NO";
    LUT4 frame_toggle_sync_I_0_2_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .Z(ws2812_enable_N_2592)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(660[13:51])
    defparam frame_toggle_sync_I_0_2_lut.init = 16'h6666;
    LUT4 i10017_4_lut (.A(ev_rd_data[36]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[36])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10017_4_lut.init = 16'h1222;
    LUT4 i10016_4_lut (.A(ev_rd_data[37]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[37])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10016_4_lut.init = 16'h1222;
    CCU2D pcm_wr_sync2_spi_5__I_0_add_2_5 (.A0(pcm_wr_sync2_spi[3]), .B0(pcm_rd_ptr_spi[3]), 
          .C0(GND_net), .D0(GND_net), .A1(pcm_wr_sync2_spi[4]), .B1(pcm_rd_ptr_spi[4]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24515), .COUT(n24516), .S0(pcm_count_spi[3]), 
          .S1(pcm_count_spi[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[33:66])
    defparam pcm_wr_sync2_spi_5__I_0_add_2_5.INIT0 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_5.INIT1 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_5.INJECT1_0 = "NO";
    defparam pcm_wr_sync2_spi_5__I_0_add_2_5.INJECT1_1 = "NO";
    LUT4 i2_2_lut_3_lut_4_lut_adj_283 (.A(spi_command[1]), .B(n25474), .C(spi_command[3]), 
         .D(spi_command[2]), .Z(n25491)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i2_2_lut_3_lut_4_lut_adj_283.init = 16'h0080;
    LUT4 i1_2_lut_3_lut_adj_284 (.A(n11907), .B(n4_adj_3271), .C(n11905), 
         .Z(n16176)) /* synthesis lut_function=(!(A (C)+!A (B (C)+!B !(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(492[50:75])
    defparam i1_2_lut_3_lut_adj_284.init = 16'h1e1e;
    CCU2D global_phase_s2_3047_add_4_6 (.A0(global_phase_s2[4]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[5]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24383), .COUT(n24384), .S0(n41_adj_3172), 
          .S1(n40_adj_3173));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047_add_4_6.INIT0 = 16'hfaaa;
    defparam global_phase_s2_3047_add_4_6.INIT1 = 16'hfaaa;
    defparam global_phase_s2_3047_add_4_6.INJECT1_0 = "NO";
    defparam global_phase_s2_3047_add_4_6.INJECT1_1 = "NO";
    CCU2D pcm_wr_sync2_spi_5__I_0_add_2_3 (.A0(pcm_wr_sync2_spi[1]), .B0(pcm_rd_ptr_spi[1]), 
          .C0(GND_net), .D0(GND_net), .A1(pcm_wr_sync2_spi[2]), .B1(pcm_rd_ptr_spi[2]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24514), .COUT(n24515), .S0(pcm_count_spi[1]), 
          .S1(pcm_count_spi[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[33:66])
    defparam pcm_wr_sync2_spi_5__I_0_add_2_3.INIT0 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_3.INIT1 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_3.INJECT1_0 = "NO";
    defparam pcm_wr_sync2_spi_5__I_0_add_2_3.INJECT1_1 = "NO";
    CCU2D global_phase_s2_3047_add_4_4 (.A0(global_phase_s2[2]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[3]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24382), .COUT(n24383), .S0(n43_adj_3170), 
          .S1(n42_adj_3171));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047_add_4_4.INIT0 = 16'hfaaa;
    defparam global_phase_s2_3047_add_4_4.INIT1 = 16'hfaaa;
    defparam global_phase_s2_3047_add_4_4.INJECT1_0 = "NO";
    defparam global_phase_s2_3047_add_4_4.INJECT1_1 = "NO";
    CCU2D pcm_wr_sync2_spi_5__I_0_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_wr_sync2_spi[0]), .B1(pcm_rd_ptr_spi[0]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24514), .S1(pcm_count_spi[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[33:66])
    defparam pcm_wr_sync2_spi_5__I_0_add_2_1.INIT0 = 16'h0000;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_1.INIT1 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_1.INJECT1_0 = "NO";
    defparam pcm_wr_sync2_spi_5__I_0_add_2_1.INJECT1_1 = "NO";
    LUT4 i15179_2_lut_3_lut_3_lut_4_lut (.A(n5_adj_3249), .B(mic_acc_state[1]), 
         .C(mic_acc_state[2]), .D(mic_acc_state[0]), .Z(pll_clk_enable_603)) /* synthesis lut_function=(!(A (B+(C+!(D)))+!A !(B+(C+(D))))) */ ;
    defparam i15179_2_lut_3_lut_3_lut_4_lut.init = 16'h5754;
    LUT4 i1_2_lut_3_lut_4_lut_adj_285 (.A(n22120), .B(n16855), .C(spi_byte_count[1]), 
         .D(spi_byte_count[0]), .Z(spi1_sck_c_enable_138)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_285.init = 16'h0800;
    LUT4 i10015_4_lut (.A(ev_rd_data[38]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[38])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10015_4_lut.init = 16'h1222;
    LUT4 i1_4_lut_adj_286 (.A(mic_run_state_1__N_2390[1]), .B(n22369), .C(n10677), 
         .D(mic_warm), .Z(n25502)) /* synthesis lut_function=(A+(B (C+!(D))+!B (C (D)))) */ ;
    defparam i1_4_lut_adj_286.init = 16'hfaee;
    LUT4 spi_command_3__bdd_4_lut_16310 (.A(spi_command[3]), .B(n25413), 
         .C(spi_command[0]), .D(spi_command[1]), .Z(n27586)) /* synthesis lut_function=(A (B (C (D)+!C !(D))+!B !(C+(D)))+!A (B+!(C (D)+!C !(D)))) */ ;
    defparam spi_command_3__bdd_4_lut_16310.init = 16'hc55e;
    LUT4 i10014_4_lut (.A(ev_rd_data[39]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[39])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10014_4_lut.init = 16'h1222;
    LUT4 i15119_2_lut_3_lut (.A(mic_acc_gate_6__N_2263), .B(mic_gate_active), 
         .C(mic_acc_busy_N_2282), .Z(mic_gate_active_N_2286)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(947[9] 967[12])
    defparam i15119_2_lut_3_lut.init = 16'h0202;
    LUT4 mic_acc_state_2__I_0_i5_2_lut_3_lut (.A(mic_acc_state[1]), .B(mic_acc_state[2]), 
         .C(mic_acc_state[0]), .Z(n5)) /* synthesis lut_function=((B+(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(985[13:17])
    defparam mic_acc_state_2__I_0_i5_2_lut_3_lut.init = 16'hfdfd;
    LUT4 i1_2_lut_3_lut_adj_287 (.A(pll_clk_enable_597), .B(mic_run_state_1__N_2390[1]), 
         .C(mic_gate_active_N_2286), .Z(n25494)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_3_lut_adj_287.init = 16'hfefe;
    LUT4 mux_673_i14_3_lut_4_lut (.A(pending_sequence[13]), .B(accepted_sequence_spi[13]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[13])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i14_3_lut_4_lut.init = 16'hacaa;
    LUT4 spi_command_3__bdd_3_lut_16311 (.A(spi_command[3]), .B(spi_command[0]), 
         .C(spi_command[1]), .Z(n27585)) /* synthesis lut_function=(!(A+(B (C)+!B !(C)))) */ ;
    defparam spi_command_3__bdd_3_lut_16311.init = 16'h1414;
    LUT4 i10013_4_lut (.A(ev_rd_data[40]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[40])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10013_4_lut.init = 16'h1222;
    LUT4 ws2812_toggle_spi_I_0_2_lut (.A(ws2812_toggle_spi), .B(invalid_frame_spi_N_2546), 
         .Z(ws2812_toggle_spi_N_2555)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(556[30] 570[24])
    defparam ws2812_toggle_spi_I_0_2_lut.init = 16'h6666;
    LUT4 i10012_4_lut (.A(ev_rd_data[41]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[41])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10012_4_lut.init = 16'h1222;
    LUT4 i10011_4_lut (.A(ev_rd_data[42]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[42])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10011_4_lut.init = 16'h1222;
    FD1P3IX mic_acc_i_3___i2 (.D(n5688), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i2.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i3 (.D(n5687), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i3.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i4 (.D(n5686), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i4.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i5 (.D(n5685), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i5.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i6 (.D(n5684), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i6.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i7 (.D(n5683), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i7.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i8 (.D(n5682), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i8.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i9 (.D(n5681), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i9.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i10 (.D(n5680), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i10.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i11 (.D(n5679), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i11.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i12 (.D(n5678), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i12.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i13 (.D(n5677), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i13.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i14 (.D(n5676), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i14.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i15 (.D(n5675), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i15.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i16 (.D(n5674), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i16.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i17 (.D(n5673), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i17.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i18 (.D(n5672), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i18.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i19 (.D(n5671), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i19.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i20 (.D(n5670), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i20.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i21 (.D(n5669), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i21.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i22 (.D(n5668), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i22.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i23 (.D(n5667), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i23.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i24 (.D(n5666), .SP(pll_clk_enable_584), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i24.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i25 (.D(n5665), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i25.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i26 (.D(n5664), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i26.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i27 (.D(n5663), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i27.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i28 (.D(n5662), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i28.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i29 (.D(n5661), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i29.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i30 (.D(n5660), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i30.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i31 (.D(n5659), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i31.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i32 (.D(n5658), .SP(pll_clk_enable_592), .CD(mic_run_state_1__N_2390[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_i_3___i32.GSR = "DISABLED";
    INV i16816 (.A(spi1_sck_c), .Z(spi1_sck_N_155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(86[24:32])
    LUT4 frame_req_I_0_1555_2_lut (.A(frame_req), .B(swap_pending), .Z(build_level_eff_7__N_1772[0])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(723[21:47])
    defparam frame_req_I_0_1555_2_lut.init = 16'h2222;
    LUT4 i10010_4_lut (.A(ev_rd_data[43]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[43])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10010_4_lut.init = 16'h1222;
    LUT4 i3_4_lut_adj_288 (.A(wrap_s2), .B(feed_pop_N_2749), .C(fifo_credit_wire[0]), 
         .D(mod_active), .Z(feed_pop)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(372[28] 373[49])
    defparam i3_4_lut_adj_288.init = 16'h8000;
    LUT4 i7_4_lut_adj_289 (.A(n9_adj_3192), .B(n11_adj_3189), .C(n10_adj_3191), 
         .D(n12_adj_3188), .Z(feed_pop_N_2749)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(373[29:49])
    defparam i7_4_lut_adj_289.init = 16'hfffe;
    PFUMX i16426 (.BLUT(n27970), .ALUT(n27971), .C0(status_bit_index[0]), 
          .Z(n27972));
    LUT4 i2_2_lut_adj_290 (.A(mic_cfg_width[1]), .B(mic_cfg_width[6]), .Z(n9)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i2_2_lut_adj_290.init = 16'heeee;
    FD1P3IX mic_cfg_gap_i6 (.D(mic_cfg_gap_w[6]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i6.GSR = "DISABLED";
    LUT4 i10460_2_lut_3_lut (.A(n16732), .B(mic_cfg_hold_spi[46]), .C(mic_cfg_hold_spi[41]), 
         .Z(mic_cfg_width_w[1])) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (C)))) */ ;
    defparam i10460_2_lut_3_lut.init = 16'h6060;
    LUT4 i10009_4_lut (.A(ev_rd_data[44]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[44])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10009_4_lut.init = 16'h1222;
    LUT4 i1_4_lut_adj_291 (.A(feed_wr[0]), .B(feed_wr[6]), .C(feed_rd[0]), 
         .D(feed_rd[6]), .Z(n9_adj_3192)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(373[29:49])
    defparam i1_4_lut_adj_291.init = 16'h7bde;
    LUT4 mux_673_i13_3_lut_4_lut (.A(pending_sequence[12]), .B(accepted_sequence_spi[12]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[12])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i13_3_lut_4_lut.init = 16'hacaa;
    LUT4 i1_2_lut_adj_292 (.A(mic_cfg_width[3]), .B(mic_acc_busy_N_2282), 
         .Z(n8)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_292.init = 16'heeee;
    LUT4 i3_4_lut_adj_293 (.A(feed_wr[3]), .B(feed_wr[5]), .C(feed_rd[3]), 
         .D(feed_rd[5]), .Z(n11_adj_3189)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(373[29:49])
    defparam i3_4_lut_adj_293.init = 16'h7bde;
    FD1P3IX mic_cfg_gap_i5 (.D(mic_cfg_gap_w[5]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i5.GSR = "DISABLED";
    LUT4 i2_4_lut_adj_294 (.A(feed_wr[2]), .B(feed_wr[4]), .C(feed_rd[2]), 
         .D(feed_rd[4]), .Z(n10_adj_3191)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(373[29:49])
    defparam i2_4_lut_adj_294.init = 16'h7bde;
    LUT4 i10008_4_lut (.A(ev_rd_data[45]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[45])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10008_4_lut.init = 16'h1222;
    LUT4 i4_4_lut_adj_295 (.A(feed_wr[1]), .B(feed_wr[7]), .C(feed_rd[1]), 
         .D(feed_rd[7]), .Z(n12_adj_3188)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(373[29:49])
    defparam i4_4_lut_adj_295.init = 16'h7bde;
    LUT4 spi_command_3__bdd_3_lut_16211 (.A(spi_command[3]), .B(n7_adj_3219), 
         .C(spi_command[1]), .Z(n27584)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;
    defparam spi_command_3__bdd_3_lut_16211.init = 16'h1010;
    FD1P3IX mic_cfg_gap_i4 (.D(mic_cfg_gap_w[4]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i4.GSR = "DISABLED";
    LUT4 spi_command_3__bdd_4_lut_16212 (.A(spi_command[3]), .B(spi_command[2]), 
         .C(spi_command[0]), .D(spi_command[1]), .Z(n27583)) /* synthesis lut_function=(!(A+(B+(C (D)+!C !(D))))) */ ;
    defparam spi_command_3__bdd_4_lut_16212.init = 16'h0110;
    FD1P3IX mic_cfg_gap_i3 (.D(mic_cfg_gap_w[3]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i3.GSR = "DISABLED";
    LUT4 i10007_4_lut (.A(ev_rd_data[46]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[46])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10007_4_lut.init = 16'h1222;
    FD1P3IX mic_cfg_gap_i2 (.D(mic_cfg_gap_w[2]), .SP(pll_clk_enable_597), 
            .CD(n17676), .CK(pll_clk), .Q(mic_cfg_gap[2])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_cfg_gap_i2.GSR = "DISABLED";
    LUT4 i2_2_lut_adj_296 (.A(spi_byte_count[3]), .B(n16904), .Z(n7_adj_3217)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i2_2_lut_adj_296.init = 16'heeee;
    LUT4 i2_3_lut_adj_297 (.A(swap_pending), .B(frame_req), .C(build_idle_N_2714), 
         .Z(fifo_credit_wire[0])) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;
    defparam i2_3_lut_adj_297.init = 16'h1010;
    PFUMX i16509 (.BLUT(n28003), .ALUT(n28131), .C0(pcm_spi_sample_15__N_1371[4]), 
          .Z(n28132));
    LUT4 i2_2_lut_adj_298 (.A(mic_window_count[3]), .B(mic_window_count[0]), 
         .Z(n7_adj_3259)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[21:47])
    defparam i2_2_lut_adj_298.init = 16'heeee;
    LUT4 i10006_4_lut (.A(ev_rd_data[47]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[47])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10006_4_lut.init = 16'h1222;
    LUT4 i6948_2_lut_3_lut_4_lut (.A(frame_req), .B(swap_pending), .C(build_idle_N_2714), 
         .D(frame_req_mod), .Z(n17719)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i6948_2_lut_3_lut_4_lut.init = 16'h2000;
    FD1S3AX feed_rd_3050__i1 (.D(n63), .CK(pll_clk), .Q(feed_rd[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050__i1.GSR = "DISABLED";
    LUT4 i10319_2_lut_3_lut (.A(n16732), .B(mic_cfg_hold_spi[46]), .C(mic_cfg_hold_spi[42]), 
         .Z(mic_cfg_width_w[2])) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (C)))) */ ;
    defparam i10319_2_lut_3_lut.init = 16'h6060;
    LUT4 i3624_2_lut_3_lut (.A(n11907), .B(n4_adj_3271), .C(n11905), .Z(n8_adj_3273)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(492[50:75])
    defparam i3624_2_lut_3_lut.init = 16'hfefe;
    PFUMX i16424 (.BLUT(n27967), .ALUT(n27968), .C0(pcm_spi_sample_15__N_1371[4]), 
          .Z(n27969));
    LUT4 i15232_4_lut (.A(n13), .B(mic_lo_n_next_6__N_642[6]), .C(n25678), 
         .D(n20_adj_3265), .Z(mic_lo_i_r_N_2629)) /* synthesis lut_function=((B (C)+!B !(D))+!A) */ ;
    defparam i15232_4_lut.init = 16'hd5f7;
    LUT4 i3_2_lut_adj_299 (.A(mic_cfg_width[2]), .B(mic_cfg_width[4]), .Z(n10)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i3_2_lut_adj_299.init = 16'heeee;
    LUT4 i14737_4_lut (.A(n25636), .B(mic_lo_n_next_6__N_642[5]), .C(mic_lo_n_next_6__N_642[3]), 
         .D(mic_lo_n_next_6__N_642[4]), .Z(n25678)) /* synthesis lut_function=(A (B+(C+(D)))+!A (B+(D))) */ ;
    defparam i14737_4_lut.init = 16'hffec;
    LUT4 i1_4_lut_adj_300 (.A(mic_lo_n_next_6__N_642[5]), .B(n25517), .C(mic_lo_n_next_6__N_642[3]), 
         .D(mic_lo_n_next_6__N_642[4]), .Z(n20_adj_3265)) /* synthesis lut_function=(A+(B (C (D)))) */ ;
    defparam i1_4_lut_adj_300.init = 16'heaaa;
    LUT4 i3182_2_lut (.A(ev_fall), .B(ev_fall_N_2683), .Z(n13599)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i3182_2_lut.init = 16'h8888;
    LUT4 i15_4_lut (.A(n25_adj_3148), .B(n27), .C(n26), .D(n28), .Z(mic_acc_busy_N_2282)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(949[21:43])
    defparam i15_4_lut.init = 16'hfffe;
    LUT4 i14950_3_lut_3_lut_3_lut (.A(mic_acc_state[0]), .B(mic_acc_state[1]), 
         .C(mic_acc_state[2]), .Z(n25730)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(985[13:17])
    defparam i14950_3_lut_3_lut_3_lut.init = 16'h0404;
    LUT4 i9_4_lut_adj_301 (.A(mic_gate_wait[0]), .B(mic_gate_wait[1]), .C(mic_gate_wait[6]), 
         .D(mic_gate_wait[4]), .Z(n25_adj_3148)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(949[21:43])
    defparam i9_4_lut_adj_301.init = 16'hfffe;
    LUT4 spi_command_5__bdd_3_lut_16759_4_lut (.A(spi_command[5]), .B(spi_command[6]), 
         .C(spi_command[7]), .D(n28078), .Z(n16854)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam spi_command_5__bdd_3_lut_16759_4_lut.init = 16'h0100;
    LUT4 i6619_2_lut (.A(mod_settle[0]), .B(n16938), .Z(n17388)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(692[18] 701[12])
    defparam i6619_2_lut.init = 16'h4444;
    FD1S3AX feed_rd_3050__i2 (.D(n62), .CK(pll_clk), .Q(feed_rd[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050__i2.GSR = "DISABLED";
    FD1S3AX feed_rd_3050__i3 (.D(n61), .CK(pll_clk), .Q(feed_rd[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050__i3.GSR = "DISABLED";
    FD1S3AX feed_rd_3050__i4 (.D(n60), .CK(pll_clk), .Q(feed_rd[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050__i4.GSR = "DISABLED";
    FD1S3AX feed_rd_3050__i5 (.D(n59), .CK(pll_clk), .Q(feed_rd[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050__i5.GSR = "DISABLED";
    FD1S3AX feed_rd_3050__i6 (.D(n58), .CK(pll_clk), .Q(feed_rd[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050__i6.GSR = "DISABLED";
    FD1S3AX feed_rd_3050__i7 (.D(n57), .CK(pll_clk), .Q(feed_rd[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam feed_rd_3050__i7.GSR = "DISABLED";
    FD1P3AX mic_acc_gate__i2 (.D(mic_gate_index[1]), .SP(mic_acc_busy_N_2273), 
            .CK(pll_clk), .Q(mic_acc_gate[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_gate__i2.GSR = "DISABLED";
    CCU2D feed_wr_spi_3046_add_4_9 (.A0(feed_wr_spi[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24380), .S0(n38));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046_add_4_9.INIT0 = 16'hfaaa;
    defparam feed_wr_spi_3046_add_4_9.INIT1 = 16'h0000;
    defparam feed_wr_spi_3046_add_4_9.INJECT1_0 = "NO";
    defparam feed_wr_spi_3046_add_4_9.INJECT1_1 = "NO";
    CCU2D feed_wr_spi_3046_add_4_7 (.A0(feed_wr_spi[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(feed_wr_spi[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24379), .COUT(n24380), .S0(n40), .S1(n39));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046_add_4_7.INIT0 = 16'hfaaa;
    defparam feed_wr_spi_3046_add_4_7.INIT1 = 16'hfaaa;
    defparam feed_wr_spi_3046_add_4_7.INJECT1_0 = "NO";
    defparam feed_wr_spi_3046_add_4_7.INJECT1_1 = "NO";
    CCU2D feed_wr_spi_3046_add_4_5 (.A0(feed_wr_spi[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(feed_wr_spi[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24378), .COUT(n24379), .S0(n42), .S1(n41));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046_add_4_5.INIT0 = 16'hfaaa;
    defparam feed_wr_spi_3046_add_4_5.INIT1 = 16'hfaaa;
    defparam feed_wr_spi_3046_add_4_5.INJECT1_0 = "NO";
    defparam feed_wr_spi_3046_add_4_5.INJECT1_1 = "NO";
    CCU2D feed_wr_spi_3046_add_4_3 (.A0(feed_wr_spi[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(feed_wr_spi[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24377), .COUT(n24378), .S0(n44), .S1(n43));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[60:78])
    defparam feed_wr_spi_3046_add_4_3.INIT0 = 16'hfaaa;
    defparam feed_wr_spi_3046_add_4_3.INIT1 = 16'hfaaa;
    defparam feed_wr_spi_3046_add_4_3.INJECT1_0 = "NO";
    defparam feed_wr_spi_3046_add_4_3.INJECT1_1 = "NO";
    PFUMX i16505 (.BLUT(n27985), .ALUT(n28128), .C0(pcm_spi_sample_15__N_1371[4]), 
          .Z(n28129));
    LUT4 mux_673_i12_3_lut_4_lut (.A(pending_sequence[11]), .B(accepted_sequence_spi[11]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[11])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i12_3_lut_4_lut.init = 16'hacaa;
    LUT4 i1_2_lut_4_lut_adj_302 (.A(spi_bit_count_2__N_1415), .B(spi_command[5]), 
         .C(n13_adj_3166), .D(n28078), .Z(feed_we)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;
    defparam i1_2_lut_4_lut_adj_302.init = 16'h0200;
    LUT4 i14695_3_lut (.A(mic_lo_n_next_6__N_642[2]), .B(mic_lo_n_next_6__N_642[1]), 
         .C(mic_lo_n_next_6__N_642[0]), .Z(n25636)) /* synthesis lut_function=(A+(B (C))) */ ;
    defparam i14695_3_lut.init = 16'heaea;
    LUT4 i13630_2_lut_3_lut_4_lut (.A(feed_wr[6]), .B(mod_settle[0]), .C(n16938), 
         .D(n21096), .Z(n23771)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam i13630_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 n27587_bdd_3_lut (.A(n27587), .B(n27584), .C(spi_command[2]), 
         .Z(n27588)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n27587_bdd_3_lut.init = 16'hcaca;
    LUT4 i11_4_lut_adj_303 (.A(mic_gate_wait[10]), .B(mic_gate_wait[2]), 
         .C(mic_gate_wait[12]), .D(mic_gate_wait[7]), .Z(n27)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(949[21:43])
    defparam i11_4_lut_adj_303.init = 16'hfffe;
    LUT4 i13625_2_lut_3_lut_4_lut (.A(feed_wr[7]), .B(mod_settle[0]), .C(n16938), 
         .D(n21096), .Z(n23769)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam i13625_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 i10318_2_lut_3_lut (.A(n16732), .B(mic_cfg_hold_spi[46]), .C(mic_cfg_hold_spi[43]), 
         .Z(mic_cfg_width_w[3])) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (C)))) */ ;
    defparam i10318_2_lut_3_lut.init = 16'h6060;
    LUT4 mux_1041_i1_4_lut (.A(n4_adj_3268), .B(n10805), .C(n10819), .D(mic_acc_state[0]), 
         .Z(mic_acc_state_2__N_2327[0])) /* synthesis lut_function=(A (B (C (D)))+!A (B ((D)+!C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(947[9] 967[12])
    defparam mux_1041_i1_4_lut.init = 16'hc505;
    L6MUX21 i16507 (.D0(n28129), .D1(n28026), .SD(status_bit_index[2]), 
            .Z(n28130));
    LUT4 i3_4_lut_adj_304 (.A(status_bit_index[2]), .B(status_bit_index[0]), 
         .C(n43_adj_3193), .D(status_bit_index[1]), .Z(n25409)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(148[17:33])
    defparam i3_4_lut_adj_304.init = 16'hfffe;
    LUT4 mux_2189_i1_3_lut_4_lut (.A(frame_req), .B(swap_pending), .C(event_level[0]), 
         .D(ev_rd_grant), .Z(n10943)) /* synthesis lut_function=(A (B (C (D))+!B (C+!(D)))+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam mux_2189_i1_3_lut_4_lut.init = 16'hf022;
    LUT4 mux_673_i11_3_lut_4_lut (.A(pending_sequence[10]), .B(accepted_sequence_spi[10]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[10])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i11_3_lut_4_lut.init = 16'hacaa;
    LUT4 sub_225_inv_0_i3_1_lut (.A(status_next_index[2]), .Z(status_next_bit_N_2754[2])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam sub_225_inv_0_i3_1_lut.init = 16'h5555;
    CCU2D add_2496_9 (.A0(\mic_acc_q[2] [6]), .B0(n20521), .C0(n4576), 
          .D0(GND_net), .A1(\mic_acc_q[2] [7]), .B1(n20521), .C1(n4576), 
          .D1(GND_net), .CIN(n24500), .S0(n5708), .S1(n5707));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(843[17:43])
    defparam add_2496_9.INIT0 = 16'h7848;
    defparam add_2496_9.INIT1 = 16'h7848;
    defparam add_2496_9.INJECT1_0 = "NO";
    defparam add_2496_9.INJECT1_1 = "NO";
    LUT4 i4508_2_lut_3_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n10668), .Z(n15270)) /* synthesis lut_function=(A ((C)+!B)+!A (B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i4508_2_lut_3_lut.init = 16'hf6f6;
    LUT4 i4_4_lut_adj_305 (.A(mic_phase[1]), .B(n16750), .C(mic_phase[2]), 
         .D(n6_adj_3276), .Z(mic_bc_valid_N_2618)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i4_4_lut_adj_305.init = 16'h1000;
    CCU2D add_2496_7 (.A0(\mic_acc_q[2] [4]), .B0(n20521), .C0(n4576), 
          .D0(GND_net), .A1(\mic_acc_q[2] [5]), .B1(n20521), .C1(n4576), 
          .D1(GND_net), .CIN(n24499), .COUT(n24500), .S0(n5710), .S1(n5709));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(843[17:43])
    defparam add_2496_7.INIT0 = 16'h7848;
    defparam add_2496_7.INIT1 = 16'h7848;
    defparam add_2496_7.INJECT1_0 = "NO";
    defparam add_2496_7.INJECT1_1 = "NO";
    LUT4 i10317_2_lut_3_lut (.A(n16732), .B(mic_cfg_hold_spi[46]), .C(mic_cfg_hold_spi[44]), 
         .Z(mic_cfg_width_w[4])) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (C)))) */ ;
    defparam i10317_2_lut_3_lut.init = 16'h6060;
    CCU2D add_2496_5 (.A0(\mic_acc_q[2] [2]), .B0(n20521), .C0(n4576), 
          .D0(GND_net), .A1(\mic_acc_q[2] [3]), .B1(n20521), .C1(n4576), 
          .D1(GND_net), .CIN(n24498), .COUT(n24499), .S0(n5712), .S1(n5711));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(843[17:43])
    defparam add_2496_5.INIT0 = 16'h7848;
    defparam add_2496_5.INIT1 = 16'h7848;
    defparam add_2496_5.INJECT1_0 = "NO";
    defparam add_2496_5.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_adj_306 (.A(mic_warm), .B(mic_phase[0]), .C(mic_phase[3]), 
         .Z(n6_adj_3276)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_3_lut_adj_306.init = 16'h4040;
    CCU2D add_2496_3 (.A0(\mic_acc_q[2] [0]), .B0(n20521), .C0(n4576), 
          .D0(GND_net), .A1(\mic_acc_q[2] [1]), .B1(n20521), .C1(n4576), 
          .D1(GND_net), .CIN(n24497), .COUT(n24498), .S0(n5714), .S1(n5713));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(843[17:43])
    defparam add_2496_3.INIT0 = 16'h7484;
    defparam add_2496_3.INIT1 = 16'h7848;
    defparam add_2496_3.INJECT1_0 = "NO";
    defparam add_2496_3.INJECT1_1 = "NO";
    LUT4 i10_4_lut (.A(mic_gate_wait[8]), .B(mic_gate_wait[3]), .C(mic_gate_wait[13]), 
         .D(mic_gate_wait[5]), .Z(n26)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(949[21:43])
    defparam i10_4_lut.init = 16'hfffe;
    LUT4 i10277_4_lut (.A(mic_cfg_width[5]), .B(n9), .C(n8), .D(n10), 
         .Z(mic_acc_last_N_2672)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;
    defparam i10277_4_lut.init = 16'h0001;
    LUT4 i3_4_lut_adj_307 (.A(status_bit_index[3]), .B(status_bit_index[4]), 
         .C(status_bit_index[5]), .D(status_bit_index[6]), .Z(n43_adj_3193)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(148[17:33])
    defparam i3_4_lut_adj_307.init = 16'hfffe;
    CCU2D add_2496_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20521), .B1(n4576), .C1(GND_net), .D1(GND_net), .COUT(n24497));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(843[17:43])
    defparam add_2496_1.INIT0 = 16'hF000;
    defparam add_2496_1.INIT1 = 16'h7777;
    defparam add_2496_1.INJECT1_0 = "NO";
    defparam add_2496_1.INJECT1_1 = "NO";
    LUT4 i14626_3_lut (.A(compact_end), .B(spi_audio_read_cmd), .C(n16895), 
         .Z(spi1_sck_c_enable_143)) /* synthesis lut_function=(A (B+!(C))) */ ;
    defparam i14626_3_lut.init = 16'h8a8a;
    LUT4 i15191_2_lut (.A(n21266), .B(spi1_sck_c_enable_146), .Z(spi1_sck_c_enable_162)) /* synthesis lut_function=(A (B)) */ ;
    defparam i15191_2_lut.init = 16'h8888;
    LUT4 i10316_2_lut_3_lut (.A(n16732), .B(mic_cfg_hold_spi[46]), .C(mic_cfg_hold_spi[45]), 
         .Z(mic_cfg_width_w[5])) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (C)))) */ ;
    defparam i10316_2_lut_3_lut.init = 16'h6060;
    LUT4 i12_4_lut_adj_308 (.A(mic_gate_wait[11]), .B(mic_gate_wait[9]), 
         .C(mic_gate_wait[14]), .D(mic_gate_wait[15]), .Z(n28)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(949[21:43])
    defparam i12_4_lut_adj_308.init = 16'hfffe;
    LUT4 i1_2_lut_3_lut_adj_309 (.A(mic_warm), .B(mic_phase[0]), .C(n22369), 
         .Z(pll_clk_enable_516)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_3_lut_adj_309.init = 16'h4040;
    LUT4 i6871_2_lut_3_lut (.A(pll_clk_enable_597), .B(mic_acc_busy_N_2272), 
         .C(mic_done_N_2646), .Z(n17704)) /* synthesis lut_function=(!((B (C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam i6871_2_lut_3_lut.init = 16'h2a2a;
    LUT4 n28133_bdd_3_lut (.A(n28133), .B(n28130), .C(status_bit_index[1]), 
         .Z(n28134)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n28133_bdd_3_lut.init = 16'hcaca;
    LUT4 i2442_1_lut (.A(mic_gate_active), .Z(mic_gate_active_N_2284)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i2442_1_lut.init = 16'h5555;
    LUT4 spi_audio_read_cmd_bdd_3_lut_16364_4_lut (.A(n8_adj_3177), .B(n16922), 
         .C(spi_byte_count[4]), .D(spi_byte_count[5]), .Z(n27762)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(456[57:94])
    defparam spi_audio_read_cmd_bdd_3_lut_16364_4_lut.init = 16'h1000;
    LUT4 i820_2_lut (.A(mic_data_1_c), .B(mic_lo_i_r), .Z(n4547)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam i820_2_lut.init = 16'h6666;
    CCU2D add_2469_9 (.A0(\mic_acc_i[0] [6]), .B0(n20521), .C0(n4489), 
          .D0(GND_net), .A1(\mic_acc_i[0] [7]), .B1(n20521), .C1(n4489), 
          .D1(GND_net), .CIN(n24492), .S0(n5659), .S1(n5658));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(842[17:43])
    defparam add_2469_9.INIT0 = 16'h7848;
    defparam add_2469_9.INIT1 = 16'h7848;
    defparam add_2469_9.INJECT1_0 = "NO";
    defparam add_2469_9.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_adj_310 (.A(n22373), .B(n104), .C(mic_warm), .Z(n20537)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_3_lut_adj_310.init = 16'h0808;
    LUT4 i13626_2_lut_3_lut_4_lut (.A(feed_wr[4]), .B(mod_settle[0]), .C(n16938), 
         .D(n21096), .Z(n23775)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam i13626_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 i4_4_lut_adj_311 (.A(mic_window_count[4]), .B(mic_window_count[6]), 
         .C(mic_window_count[5]), .D(n6), .Z(n8_adj_3258)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;
    defparam i4_4_lut_adj_311.init = 16'hffbf;
    LUT4 i2_4_lut_4_lut (.A(n16895), .B(n16907), .C(spi_command[1]), .Z(n21183)) /* synthesis lut_function=(A (B+!(C))) */ ;
    defparam i2_4_lut_4_lut.init = 16'h8a8a;
    LUT4 i13628_2_lut_3_lut_4_lut (.A(feed_wr[5]), .B(mod_settle[0]), .C(n16938), 
         .D(n21096), .Z(n23773)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(683[24:38])
    defparam i13628_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 i14984_2_lut (.A(mic_run_state[1]), .B(mic_run_state[0]), .Z(n25761)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[69:94])
    defparam i14984_2_lut.init = 16'heeee;
    PFUMX i16420 (.BLUT(n27961), .ALUT(n27962), .C0(mic_acc_state[2]), 
          .Z(pll_clk_enable_604));
    LUT4 i13453_2_lut_3_lut (.A(n22373), .B(n104), .C(mic_sample_count[0]), 
         .Z(n25)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i13453_2_lut_3_lut.init = 16'h7878;
    CCU2D add_2469_7 (.A0(\mic_acc_i[0] [4]), .B0(n20521), .C0(n4489), 
          .D0(GND_net), .A1(\mic_acc_i[0] [5]), .B1(n20521), .C1(n4489), 
          .D1(GND_net), .CIN(n24491), .COUT(n24492), .S0(n5661), .S1(n5660));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(842[17:43])
    defparam add_2469_7.INIT0 = 16'h7848;
    defparam add_2469_7.INIT1 = 16'h7848;
    defparam add_2469_7.INJECT1_0 = "NO";
    defparam add_2469_7.INJECT1_1 = "NO";
    LUT4 i14886_3_lut_4_lut (.A(mic_phase[2]), .B(mic_phase[1]), .C(mic_ultrasonic), 
         .D(n127), .Z(n25757)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i14886_3_lut_4_lut.init = 16'h0010;
    LUT4 i1_2_lut_3_lut_adj_312 (.A(spi_byte_count[0]), .B(n7), .C(spi_byte_count[2]), 
         .Z(spi1_sck_c_enable_115)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_312.init = 16'h0808;
    LUT4 i14661_2_lut_3_lut (.A(mic_phase[2]), .B(mic_phase[1]), .C(mic_phase[0]), 
         .Z(n25602)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i14661_2_lut_3_lut.init = 16'hfefe;
    LUT4 mic_acc_state_0__bdd_4_lut (.A(mic_acc_state[0]), .B(n21127), .C(mic_acc_last), 
         .D(mic_acc_state[1]), .Z(n27630)) /* synthesis lut_function=(A ((C+!(D))+!B)+!A (D)) */ ;
    defparam mic_acc_state_0__bdd_4_lut.init = 16'hf7aa;
    LUT4 mic_acc_state_0__bdd_4_lut_16235 (.A(mic_acc_state[0]), .B(n21127), 
         .C(n5_adj_3249), .D(n3), .Z(n24753)) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A (B+(C+!(D)))) */ ;
    defparam mic_acc_state_0__bdd_4_lut_16235.init = 16'hfc5d;
    LUT4 i9996_4_lut (.A(ev_rd_data[57]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[57])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9996_4_lut.init = 16'h1222;
    LUT4 i10293_2_lut (.A(status_bit_index[0]), .B(status_bit_index[1]), 
         .Z(n21039)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10293_2_lut.init = 16'h8888;
    LUT4 n27172_bdd_4_lut_then_3_lut (.A(mic_ram_rd_data[0]), .B(mic_ram_rd_data[2]), 
         .C(status_bit_index[1]), .Z(n27983)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27172_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 mux_673_i10_3_lut_4_lut (.A(pending_sequence[9]), .B(accepted_sequence_spi[9]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[9])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i10_3_lut_4_lut.init = 16'hacaa;
    LUT4 i2_4_lut_adj_313 (.A(pcm_rd_ptr_spi[3]), .B(n4_adj_3245), .C(pcm_payload_bit[7]), 
         .D(n6_adj_3288), .Z(pcm_spi_sample_15__N_1371[4])) /* synthesis lut_function=(!(A (B (C+(D))+!B !(C+(D)))+!A (B (C (D))+!B !(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[18:55])
    defparam i2_4_lut_adj_313.init = 16'h366c;
    LUT4 i1_2_lut_adj_314 (.A(pcm_payload_bit[8]), .B(pcm_rd_ptr_spi[4]), 
         .Z(n4_adj_3245)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[18:55])
    defparam i1_2_lut_adj_314.init = 16'h6666;
    LUT4 n27172_bdd_4_lut_else_3_lut (.A(mic_ram_rd_data[1]), .B(mic_ram_rd_data[3]), 
         .C(status_bit_index[1]), .Z(n27982)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27172_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 i9794_2_lut (.A(spi_command[0]), .B(spi_command[1]), .Z(n20539)) /* synthesis lut_function=(A (B)) */ ;
    defparam i9794_2_lut.init = 16'h8888;
    LUT4 i9987_4_lut (.A(ev_rd_data[66]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[66])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9987_4_lut.init = 16'h1222;
    LUT4 i9986_4_lut (.A(ev_rd_data[67]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[67])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9986_4_lut.init = 16'h1222;
    LUT4 i15126_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_0)) /* synthesis lut_function=(!(A+(B+(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15126_2_lut_3_lut.init = 16'h0101;
    LUT4 i15103_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_1)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15103_2_lut_3_lut.init = 16'h1010;
    LUT4 i9985_4_lut (.A(ev_rd_data[68]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[68])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9985_4_lut.init = 16'h1222;
    LUT4 i9984_4_lut (.A(ev_rd_data[69]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[69])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9984_4_lut.init = 16'h1222;
    CCU2D add_1585_5 (.A0(phase_frac[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24315), .COUT(n24316), .S0(phase_frac_sum[3]), 
          .S1(phase_frac_sum[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_5.INIT0 = 16'h5aaa;
    defparam add_1585_5.INIT1 = 16'h5aaa;
    defparam add_1585_5.INJECT1_0 = "NO";
    defparam add_1585_5.INJECT1_1 = "NO";
    CCU2D add_2469_5 (.A0(\mic_acc_i[0] [2]), .B0(n20521), .C0(n4489), 
          .D0(GND_net), .A1(\mic_acc_i[0] [3]), .B1(n20521), .C1(n4489), 
          .D1(GND_net), .CIN(n24490), .COUT(n24491), .S0(n5663), .S1(n5662));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(842[17:43])
    defparam add_2469_5.INIT0 = 16'h7848;
    defparam add_2469_5.INIT1 = 16'h7848;
    defparam add_2469_5.INJECT1_0 = "NO";
    defparam add_2469_5.INJECT1_1 = "NO";
    LUT4 i15134_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_4)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15134_2_lut_3_lut.init = 16'h0404;
    LUT4 i10292_4_lut (.A(wrap_s2), .B(ev_fall_N_2683), .C(swap_pending), 
         .D(n4_adj_3243), .Z(swap_pending_N_2707)) /* synthesis lut_function=(A (B (D))+!A (B (C+(D))+!B (C))) */ ;
    defparam i10292_4_lut.init = 16'hdc50;
    LUT4 i15128_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_5)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15128_2_lut_3_lut.init = 16'h4040;
    LUT4 i9983_4_lut (.A(ev_rd_data[70]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[70])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9983_4_lut.init = 16'h1222;
    LUT4 i2_3_lut_adj_315 (.A(mic_run_state[1]), .B(mic_run_state[0]), .C(swap_now_s3), 
         .Z(mic_run_state_1__N_2390[1])) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i2_3_lut_adj_315.init = 16'h4040;
    LUT4 i10379_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_7)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i10379_2_lut_3_lut.init = 16'h8080;
    LUT4 i9995_4_lut (.A(ev_rd_data[58]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[58])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9995_4_lut.init = 16'h1222;
    LUT4 i15123_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_6)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i15123_2_lut_3_lut.init = 16'h0808;
    LUT4 i9982_4_lut (.A(ev_rd_data[71]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[71])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9982_4_lut.init = 16'h1222;
    PFUMX i4561 (.BLUT(n15322), .ALUT(n25796), .C0(ev_fall_N_2683), .Z(n15323));
    LUT4 i15163_2_lut (.A(mic_warm), .B(mic_phase[0]), .Z(n92)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15163_2_lut.init = 16'h1111;
    LUT4 n27250_bdd_4_lut (.A(n27250), .B(\mic_win_q[2] [0]), .C(\mic_win_i[2] [0]), 
         .D(mic_acc_word[0]), .Z(n27253)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27250_bdd_4_lut.init = 16'haad8;
    LUT4 i1_2_lut_adj_316 (.A(n10677), .B(mic_warm), .Z(pll_clk_enable_99)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_316.init = 16'h8888;
    LUT4 i9981_4_lut (.A(ev_rd_data[72]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[72])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9981_4_lut.init = 16'h1222;
    CCU2D add_2469_3 (.A0(\mic_acc_i[0] [0]), .B0(n20521), .C0(n4489), 
          .D0(GND_net), .A1(\mic_acc_i[0] [1]), .B1(n20521), .C1(n4489), 
          .D1(GND_net), .CIN(n24489), .COUT(n24490), .S0(n5665), .S1(n5664));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(842[17:43])
    defparam add_2469_3.INIT0 = 16'h7484;
    defparam add_2469_3.INIT1 = 16'h7848;
    defparam add_2469_3.INJECT1_0 = "NO";
    defparam add_2469_3.INJECT1_1 = "NO";
    LUT4 i9980_4_lut (.A(ev_rd_data[73]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[73])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9980_4_lut.init = 16'h1222;
    LUT4 i9979_4_lut (.A(ev_rd_data[74]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[74])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9979_4_lut.init = 16'h1222;
    CCU2D add_2469_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20521), .B1(n4489), .C1(GND_net), .D1(GND_net), .COUT(n24489));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(842[17:43])
    defparam add_2469_1.INIT0 = 16'hF000;
    defparam add_2469_1.INIT1 = 16'h7777;
    defparam add_2469_1.INJECT1_0 = "NO";
    defparam add_2469_1.INJECT1_1 = "NO";
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_17 (.A0(mic_cfg_hold_spi[39]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24484), .S0(mic_cfg_gap_w[15]), .S1(mic_cfg_gap_w[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_17.INIT0 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_17.INIT1 = 16'hffff;
    defparam mic_cfg_hold_spi_39__I_0_add_2_17.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_17.INJECT1_1 = "NO";
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_15 (.A0(mic_cfg_hold_spi[37]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_cfg_hold_spi[38]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24483), .COUT(n24484), .S0(mic_cfg_gap_w[13]), 
          .S1(mic_cfg_gap_w[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_15.INIT0 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_15.INIT1 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_15.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_15.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_317 (.A(mic_window_count[2]), .B(mic_window_count[1]), 
         .Z(n6)) /* synthesis lut_function=((B)+!A) */ ;
    defparam i1_2_lut_adj_317.init = 16'hdddd;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_13 (.A0(mic_cfg_hold_spi[35]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_cfg_hold_spi[36]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24482), .COUT(n24483), .S0(mic_cfg_gap_w[11]), 
          .S1(mic_cfg_gap_w[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_13.INIT0 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_13.INIT1 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_13.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_13.INJECT1_1 = "NO";
    MUX21 i1558457_i1 (.D0(n27157), .D1(n27091), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[6]));
    LUT4 mic_acc_word_2__bdd_4_lut_15984 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [0]), 
         .C(\mic_win_q[3] [0]), .D(mic_acc_word[0]), .Z(n27250)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15984.init = 16'he4aa;
    LUT4 i6_4_lut_adj_318 (.A(mic_warm_half[6]), .B(n12_adj_3251), .C(n8_adj_3253), 
         .D(mic_warm_half[5]), .Z(mic_ultrasonic_N_1894)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam i6_4_lut_adj_318.init = 16'h0040;
    LUT4 i9978_4_lut (.A(ev_rd_data[75]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[75])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9978_4_lut.init = 16'h1222;
    LUT4 i9977_4_lut (.A(ev_rd_data[76]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[76])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9977_4_lut.init = 16'h1222;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_11 (.A0(mic_cfg_hold_spi[33]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_cfg_hold_spi[34]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24481), .COUT(n24482), .S0(mic_cfg_gap_w[9]), 
          .S1(mic_cfg_gap_w[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_11.INIT0 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_11.INIT1 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_11.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_11.INJECT1_1 = "NO";
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_9 (.A0(mic_cfg_hold_spi[31]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_cfg_hold_spi[32]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24480), .COUT(n24481), .S0(mic_cfg_gap_w[7]), 
          .S1(mic_cfg_gap_w[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_9.INIT0 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_9.INIT1 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_9.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_9.INJECT1_1 = "NO";
    CCU2D global_phase_s2_3047_add_4_2 (.A0(phase_step_s1), .B0(global_phase_s2[0]), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[1]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .COUT(n24382), .S1(n44_adj_3159));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[32:54])
    defparam global_phase_s2_3047_add_4_2.INIT0 = 16'h7000;
    defparam global_phase_s2_3047_add_4_2.INIT1 = 16'hfaaa;
    defparam global_phase_s2_3047_add_4_2.INJECT1_0 = "NO";
    defparam global_phase_s2_3047_add_4_2.INJECT1_1 = "NO";
    PFUMX i16418 (.BLUT(n27958), .ALUT(n27959), .C0(pcm_spi_sample_15__N_1371[4]), 
          .Z(n27960));
    LUT4 i2_3_lut_adj_319 (.A(mod_settle[3]), .B(mod_settle[1]), .C(mod_settle[2]), 
         .Z(n16938)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(692[22:40])
    defparam i2_3_lut_adj_319.init = 16'hfefe;
    LUT4 i5_4_lut_adj_320 (.A(mic_warm_half[2]), .B(mic_warm_half[1]), .C(mic_warm_half[4]), 
         .D(mic_warm_half[0]), .Z(n12_adj_3251)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam i5_4_lut_adj_320.init = 16'h8000;
    spi_mic_stream mic_stream_i (.sck_N_3124(sck_N_3124), .spi_mic_cs_n_c(spi_mic_cs_n_c), 
            .mic_latest({mic_latest}), .spi_mic_miso_c(spi_mic_miso_c), 
            .GND_net(GND_net), .n28627(n28627)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1028[20] 1031[6])
    LUT4 spi_command_7__I_0_1442_i13_2_lut (.A(spi_command[6]), .B(spi_command[7]), 
         .Z(n13_adj_3166)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(173[32:61])
    defparam spi_command_7__I_0_1442_i13_2_lut.init = 16'heeee;
    LUT4 i1_2_lut_adj_321 (.A(mic_warm_half[7]), .B(mic_warm_half[3]), .Z(n8_adj_3253)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(826[34:54])
    defparam i1_2_lut_adj_321.init = 16'h8888;
    LUT4 i1_2_lut_adj_322 (.A(fpga_time[13]), .B(fpga_time[11]), .Z(mic_boot_done)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam i1_2_lut_adj_322.init = 16'h8888;
    LUT4 i15149_3_lut (.A(n10912), .B(ws2812_enable), .C(n13623), .Z(pll_clk_enable_138)) /* synthesis lut_function=(!(A (B)+!A !((C)+!B))) */ ;
    defparam i15149_3_lut.init = 16'h7373;
    LUT4 i4564_4_lut (.A(ev_sum[3]), .B(staging_q[3]), .C(staging_rd_page_N_2718), 
         .D(ev_in_clear), .Z(n15326)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam i4564_4_lut.init = 16'hcac0;
    LUT4 n27256_bdd_4_lut (.A(n27256), .B(\mic_win_q[0] [4]), .C(\mic_win_i[0] [4]), 
         .D(mic_acc_word[0]), .Z(n27259)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27256_bdd_4_lut.init = 16'haad8;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_7 (.A0(mic_cfg_hold_spi[29]), .B0(mic_cfg_hold_spi[45]), 
          .C0(n16732), .D0(mic_cfg_hold_spi[46]), .A1(mic_cfg_hold_spi[30]), 
          .B1(mic_cfg_hold_spi[46]), .C1(GND_net), .D1(GND_net), .CIN(n24479), 
          .COUT(n24480), .S0(mic_cfg_gap_w[5]), .S1(mic_cfg_gap_w[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_7.INIT0 = 16'h5995;
    defparam mic_cfg_hold_spi_39__I_0_add_2_7.INIT1 = 16'h5999;
    defparam mic_cfg_hold_spi_39__I_0_add_2_7.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_7.INJECT1_1 = "NO";
    LUT4 i9976_4_lut (.A(ev_rd_data[77]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[77])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9976_4_lut.init = 16'h1222;
    LUT4 i6_4_lut_adj_323 (.A(n25474), .B(n12), .C(spi_command[1]), .D(n22120), 
         .Z(n7)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_adj_323.init = 16'h0800;
    LUT4 mic_acc_word_2__bdd_4_lut_15989 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [4]), 
         .C(\mic_win_q[1] [4]), .D(mic_acc_word[0]), .Z(n27256)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15989.init = 16'he4aa;
    LUT4 i1_2_lut_3_lut_4_lut_adj_324 (.A(wrap_s2), .B(swap_pending), .C(ws2812_settle[1]), 
         .D(n28057), .Z(pll_clk_enable_296)) /* synthesis lut_function=(A (B+!(C+!(D)))+!A !(C+!(D))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_324.init = 16'h8f88;
    LUT4 i9975_4_lut (.A(ev_rd_data[78]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[78])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9975_4_lut.init = 16'h1222;
    LUT4 i15075_2_lut_3_lut_4_lut (.A(ev_ch[5]), .B(ev_ch[6]), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(ev_ch_hi_2)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15075_2_lut_3_lut_4_lut.init = 16'h0010;
    LUT4 ev_run_hold_s5_83__I_0_i7_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_6), .D(ev_run_hold_s5[6]), .Z(us_tx_next[6])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i7_3_lut_4_lut.init = 16'hbf40;
    LUT4 mic_acc_state_1__bdd_4_lut (.A(mic_acc_state[1]), .B(n27630), .C(mic_acc_state[0]), 
         .D(mic_acc_state[2]), .Z(n27632)) /* synthesis lut_function=(!(A (B+(D))+!A (B (C+!(D))+!B (C (D)+!C !(D))))) */ ;
    defparam mic_acc_state_1__bdd_4_lut.init = 16'h0532;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_5 (.A0(mic_cfg_hold_spi[27]), .B0(mic_cfg_hold_spi[43]), 
          .C0(n16732), .D0(mic_cfg_hold_spi[46]), .A1(mic_cfg_hold_spi[28]), 
          .B1(mic_cfg_hold_spi[44]), .C1(n16732), .D1(mic_cfg_hold_spi[46]), 
          .CIN(n24478), .COUT(n24479), .S0(mic_cfg_gap_w[3]), .S1(mic_cfg_gap_w[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_5.INIT0 = 16'h5995;
    defparam mic_cfg_hold_spi_39__I_0_add_2_5.INIT1 = 16'h5995;
    defparam mic_cfg_hold_spi_39__I_0_add_2_5.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_5.INJECT1_1 = "NO";
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_3 (.A0(mic_cfg_hold_spi[25]), .B0(mic_cfg_hold_spi[41]), 
          .C0(n16732), .D0(mic_cfg_hold_spi[46]), .A1(mic_cfg_hold_spi[26]), 
          .B1(mic_cfg_hold_spi[42]), .C1(n16732), .D1(mic_cfg_hold_spi[46]), 
          .CIN(n24477), .COUT(n24478), .S0(mic_cfg_gap_w[1]), .S1(mic_cfg_gap_w[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_3.INIT0 = 16'h5995;
    defparam mic_cfg_hold_spi_39__I_0_add_2_3.INIT1 = 16'h5995;
    defparam mic_cfg_hold_spi_39__I_0_add_2_3.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_3.INJECT1_1 = "NO";
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_1 (.A0(GND_net), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_cfg_hold_spi[24]), .B1(mic_cfg_hold_spi[40]), 
          .C1(n16732), .D1(mic_cfg_hold_spi[46]), .COUT(n24477), .S1(mic_cfg_gap_w[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_1.INIT0 = 16'h0000;
    defparam mic_cfg_hold_spi_39__I_0_add_2_1.INIT1 = 16'h599a;
    defparam mic_cfg_hold_spi_39__I_0_add_2_1.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_1.INJECT1_1 = "NO";
    PFUMX i16416 (.BLUT(n27955), .ALUT(n27956), .C0(spi_command[0]), .Z(n21266));
    LUT4 i9974_4_lut (.A(ev_rd_data[79]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[79])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9974_4_lut.init = 16'h1222;
    LUT4 n27262_bdd_4_lut (.A(n27262), .B(\mic_win_q[0] [3]), .C(\mic_win_i[0] [3]), 
         .D(mic_acc_word[0]), .Z(n27265)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27262_bdd_4_lut.init = 16'haad8;
    LUT4 i9973_4_lut (.A(ev_rd_data[80]), .B(ev_in_clear), .C(ev_ch_hi_10), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[80])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9973_4_lut.init = 16'h1222;
    CCU2D add_1585_3 (.A0(phase_frac[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24314), .COUT(n24315), .S0(phase_frac_sum[1]), 
          .S1(phase_frac_sum[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_3.INIT0 = 16'h5555;
    defparam add_1585_3.INIT1 = 16'h5aaa;
    defparam add_1585_3.INJECT1_0 = "NO";
    defparam add_1585_3.INJECT1_1 = "NO";
    LUT4 mic_acc_word_2__bdd_4_lut_15994 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [3]), 
         .C(\mic_win_q[1] [3]), .D(mic_acc_word[0]), .Z(n27262)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_15994.init = 16'he4aa;
    CCU2D add_1585_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24314), .S1(phase_frac_sum[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_1.INIT0 = 16'hF000;
    defparam add_1585_1.INIT1 = 16'h5555;
    defparam add_1585_1.INJECT1_0 = "NO";
    defparam add_1585_1.INJECT1_1 = "NO";
    LUT4 i9972_4_lut (.A(ev_rd_data[81]), .B(ev_in_clear), .C(ev_ch_hi_10), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[81])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9972_4_lut.init = 16'h1222;
    FD1P3AX mic_acc_gate__i3 (.D(mic_gate_index[2]), .SP(mic_acc_busy_N_2273), 
            .CK(pll_clk), .Q(mic_acc_gate[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_gate__i3.GSR = "DISABLED";
    FD1P3AX mic_acc_gate__i4 (.D(mic_gate_index[3]), .SP(mic_acc_busy_N_2273), 
            .CK(pll_clk), .Q(mic_acc_gate[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_gate__i4.GSR = "DISABLED";
    FD1P3AX mic_acc_gate__i5 (.D(mic_gate_index[4]), .SP(mic_acc_busy_N_2273), 
            .CK(pll_clk), .Q(mic_acc_gate[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_gate__i5.GSR = "DISABLED";
    FD1P3AX mic_acc_gate__i6 (.D(mic_gate_index[5]), .SP(mic_acc_busy_N_2273), 
            .CK(pll_clk), .Q(mic_acc_gate[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_gate__i6.GSR = "DISABLED";
    FD1S3AX spi_byte_count_3044__i1 (.D(n49), .CK(spi1_sck_c), .Q(spi_byte_count[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044__i1.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3044__i2 (.D(n48), .CK(spi1_sck_c), .Q(spi_byte_count[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044__i2.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3044__i3 (.D(n47), .CK(spi1_sck_c), .Q(spi_byte_count[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044__i3.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3044__i4 (.D(n46), .CK(spi1_sck_c), .Q(spi_byte_count[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044__i4.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3044__i5 (.D(n45_adj_3220), .CK(spi1_sck_c), 
            .Q(spi_byte_count[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044__i5.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3044__i6 (.D(n44_adj_3221), .CK(spi1_sck_c), 
            .Q(spi_byte_count[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044__i6.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3044__i7 (.D(n43_adj_3222), .CK(spi1_sck_c), 
            .Q(spi_byte_count[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044__i7.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3044__i8 (.D(n42_adj_3223), .CK(spi1_sck_c), 
            .Q(spi_byte_count[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[35:56])
    defparam spi_byte_count_3044__i8.GSR = "ENABLED";
    FD1S3AX spi_rgb_captured_3043__i1 (.D(n14), .CK(spi1_sck_c), .Q(spi_rgb_captured[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(525[49:72])
    defparam spi_rgb_captured_3043__i1.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3042__i1 (.D(n39_adj_3210), .CK(spi1_sck_c), 
            .Q(spi_channel_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042__i1.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3042__i2 (.D(n38_adj_3211), .CK(spi1_sck_c), 
            .Q(spi_channel_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042__i2.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3042__i3 (.D(n37_adj_3212), .CK(spi1_sck_c), 
            .Q(spi_channel_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042__i3.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3042__i4 (.D(n36_adj_3213), .CK(spi1_sck_c), 
            .Q(spi_channel_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042__i4.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3042__i5 (.D(n35_adj_3214), .CK(spi1_sck_c), 
            .Q(spi_channel_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042__i5.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3042__i6 (.D(n34_adj_3215), .CK(spi1_sck_c), 
            .Q(spi_channel_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[58:82])
    defparam spi_channel_index_3042__i6.GSR = "ENABLED";
    FD1P3AX pcm_wr_ptr_3053__i1 (.D(n34_adj_3203), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(pcm_wr_ptr[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(855[39:56])
    defparam pcm_wr_ptr_3053__i1.GSR = "DISABLED";
    CCU2D fpga_time_3049_add_4_33 (.A0(fpga_time[31]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24476), .S0(n134));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_33.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_33.INIT1 = 16'h0000;
    defparam fpga_time_3049_add_4_33.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_33.INJECT1_1 = "NO";
    FD1P3AX pcm_wr_ptr_3053__i2 (.D(n33_adj_3204), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(pcm_wr_ptr[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(855[39:56])
    defparam pcm_wr_ptr_3053__i2.GSR = "DISABLED";
    FD1P3AX pcm_wr_ptr_3053__i3 (.D(n32_adj_3205), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(pcm_wr_ptr[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(855[39:56])
    defparam pcm_wr_ptr_3053__i3.GSR = "DISABLED";
    FD1P3AX pcm_wr_ptr_3053__i4 (.D(n31_adj_3206), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(pcm_wr_ptr[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(855[39:56])
    defparam pcm_wr_ptr_3053__i4.GSR = "DISABLED";
    FD1P3AX pcm_wr_ptr_3053__i5 (.D(n30_adj_3207), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(pcm_wr_ptr[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(855[39:56])
    defparam pcm_wr_ptr_3053__i5.GSR = "DISABLED";
    CCU2D fpga_time_3049_add_4_31 (.A0(fpga_time[29]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[30]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24475), .COUT(n24476), .S0(n136), .S1(n135));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_31.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_31.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_31.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_31.INJECT1_1 = "NO";
    LUT4 ws2812_settle_2__bdd_3_lut (.A(ws2812_settle[2]), .B(ws2812_settle[1]), 
         .C(ws2812_settle[0]), .Z(n27633)) /* synthesis lut_function=(A (B+(C))+!A !(B+(C))) */ ;
    defparam ws2812_settle_2__bdd_3_lut.init = 16'ha9a9;
    LUT4 i9971_4_lut (.A(ev_rd_data[82]), .B(ev_in_clear), .C(ev_ch_hi_10), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[82])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9971_4_lut.init = 16'h1222;
    LUT4 ev_run_hold_s5_83__I_0_i6_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_5), .D(ev_run_hold_s5[5]), .Z(us_tx_next[5])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i6_3_lut_4_lut.init = 16'hbf40;
    CCU2D fpga_time_3049_add_4_29 (.A0(fpga_time[27]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[28]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24474), .COUT(n24475), .S0(n138), .S1(n137));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_29.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_29.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_29.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_29.INJECT1_1 = "NO";
    CCU2D fpga_time_3049_add_4_27 (.A0(fpga_time[25]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[26]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24473), .COUT(n24474), .S0(n140), .S1(n139));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_27.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_27.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_27.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_27.INJECT1_1 = "NO";
    LUT4 i9970_4_lut (.A(ev_rd_data[83]), .B(ev_in_clear), .C(ev_ch_hi_10), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[83])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9970_4_lut.init = 16'h1222;
    LUT4 n27268_bdd_4_lut (.A(n27268), .B(\mic_win_q[2] [3]), .C(\mic_win_i[2] [3]), 
         .D(mic_acc_word[0]), .Z(n27271)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27268_bdd_4_lut.init = 16'haad8;
    CCU2D fpga_time_3049_add_4_25 (.A0(fpga_time[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[24]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24472), .COUT(n24473), .S0(n142), .S1(n141));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_25.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_25.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_25.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_25.INJECT1_1 = "NO";
    LUT4 mic_acc_word_2__bdd_4_lut_16063 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [3]), 
         .C(\mic_win_q[3] [3]), .D(mic_acc_word[0]), .Z(n27268)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16063.init = 16'he4aa;
    LUT4 i9784_4_lut (.A(ev_rd_data[0]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[0])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9784_4_lut.init = 16'h1222;
    LUT4 ws2812_settle_3__bdd_4_lut (.A(ws2812_settle[3]), .B(ws2812_settle[1]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(n27634)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;
    defparam ws2812_settle_3__bdd_4_lut.init = 16'haaa9;
    LUT4 n27166_bdd_4_lut_else_3_lut (.A(n14022), .B(n14023), .C(status_bit_index[0]), 
         .Z(n27985)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27166_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 i15051_2_lut_3_lut_4_lut (.A(ev_ch[5]), .B(ev_ch[6]), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(ev_ch_hi_6)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15051_2_lut_3_lut_4_lut.init = 16'h0020;
    CCU2D fpga_time_3049_add_4_23 (.A0(fpga_time[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24471), .COUT(n24472), .S0(n144), .S1(n143));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_23.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_23.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_23.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_23.INJECT1_1 = "NO";
    LUT4 i15069_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_3)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i15069_2_lut_3_lut_4_lut.init = 16'h0008;
    LUT4 i10052_4_lut (.A(ev_rd_data[1]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[1])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10052_4_lut.init = 16'h1222;
    LUT4 i10051_4_lut (.A(ev_rd_data[2]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[2])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10051_4_lut.init = 16'h1222;
    CCU2D fpga_time_3049_add_4_21 (.A0(fpga_time[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24470), .COUT(n24471), .S0(n146), .S1(n145));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_21.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_21.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_21.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_21.INJECT1_1 = "NO";
    LUT4 i15158_2_lut_3_lut (.A(spi_byte_count[0]), .B(n7), .C(spi_byte_count[2]), 
         .Z(spi1_sck_c_enable_108)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i15158_2_lut_3_lut.init = 16'h0404;
    LUT4 i15048_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_7)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i15048_2_lut_3_lut_4_lut.init = 16'h0800;
    LUT4 i9994_4_lut (.A(ev_rd_data[59]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[59])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9994_4_lut.init = 16'h1222;
    PFUMX i16344 (.BLUT(n27823), .ALUT(n27822), .C0(mic_warm), .Z(mic_clk_N_2454));
    LUT4 i2_3_lut_4_lut_adj_325 (.A(spi_byte_count[0]), .B(spi_byte_count[1]), 
         .C(n7), .D(spi_byte_count[2]), .Z(spi1_sck_c_enable_84)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i2_3_lut_4_lut_adj_325.init = 16'h2000;
    LUT4 i10050_4_lut (.A(ev_rd_data[3]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[3])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10050_4_lut.init = 16'h1222;
    LUT4 mux_3310_i2_3_lut (.A(n14001), .B(n14002), .C(n13998), .Z(rd_data_7__N_2897[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3310_i2_3_lut.init = 16'hcaca;
    CCU2D fpga_time_3049_add_4_19 (.A0(fpga_time[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24469), .COUT(n24470), .S0(n148), .S1(n147));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_19.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_19.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_19.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_19.INJECT1_1 = "NO";
    LUT4 i10049_4_lut (.A(ev_rd_data[4]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[4])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10049_4_lut.init = 16'h1222;
    LUT4 mux_3310_i3_3_lut (.A(n14003), .B(n14004), .C(n13998), .Z(rd_data_7__N_2897[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3310_i3_3_lut.init = 16'hcaca;
    LUT4 mux_3310_i4_3_lut (.A(n14005), .B(n14006), .C(n13998), .Z(rd_data_7__N_2897[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3310_i4_3_lut.init = 16'hcaca;
    PFUMX i16414 (.BLUT(n27952), .ALUT(n27953), .C0(mic_acc_state[2]), 
          .Z(n27954));
    LUT4 mux_3310_i5_3_lut (.A(n14007), .B(n14008), .C(n13998), .Z(rd_data_7__N_2897[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3310_i5_3_lut.init = 16'hcaca;
    LUT4 i10048_4_lut (.A(ev_rd_data[5]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[5])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10048_4_lut.init = 16'h1222;
    CCU2D fpga_time_3049_add_4_17 (.A0(fpga_time[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24468), .COUT(n24469), .S0(n150), .S1(n149));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_17.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_17.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_17.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_17.INJECT1_1 = "NO";
    CCU2D fpga_time_3049_add_4_15 (.A0(fpga_time[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24467), .COUT(n24468), .S0(n152), .S1(n151));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_15.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_15.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_15.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_15.INJECT1_1 = "NO";
    LUT4 mic_cfg_width_6__I_0_1524_i14_3_lut (.A(n12_adj_3229), .B(mic_acc_last_N_2665[6]), 
         .C(mic_cfg_width[6]), .Z(mic_acc_last_N_2664)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[34:73])
    defparam mic_cfg_width_6__I_0_1524_i14_3_lut.init = 16'h8e8e;
    FD1P3IX mic_ram_wr_data_i0_i0 (.D(mic_ram_wr_data_15__N_2296[0]), .SP(pll_clk_enable_603), 
            .CD(n17760), .CK(pll_clk), .Q(mic_ram_wr_data[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_ram_wr_data_i0_i0.GSR = "DISABLED";
    LUT4 mux_3310_i6_3_lut (.A(n14009), .B(n14010), .C(n13998), .Z(rd_data_7__N_2897[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3310_i6_3_lut.init = 16'hcaca;
    LUT4 i10047_4_lut (.A(ev_rd_data[6]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[6])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10047_4_lut.init = 16'h1222;
    LUT4 mux_3310_i7_3_lut (.A(n14011), .B(n14012), .C(n13998), .Z(rd_data_7__N_2897[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3310_i7_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_adj_326 (.A(mic_lo_n_next_6__N_642[0]), .B(mic_lo_n_next_6__N_642[2]), 
         .C(mic_lo_n_next_6__N_642[1]), .Z(n25517)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_3_lut_adj_326.init = 16'hfefe;
    LUT4 i15079_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_3)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15079_2_lut_3_lut.init = 16'h2020;
    LUT4 mux_3310_i8_3_lut (.A(n14013), .B(n14014), .C(n13998), .Z(rd_data_7__N_2897[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3310_i8_3_lut.init = 16'hcaca;
    MUX21 i15676103_i1 (.D0(n27379), .D1(n27367), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[1]));
    MUX21 i1560266_i1 (.D0(n27265), .D1(n27271), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[3]));
    MUX21 i15740135_i1 (.D0(n28186), .D1(n28134), .SD(pcm_payload_bit[3]), 
          .Z(n26518));
    MUX21 i1559663_i1 (.D0(n27259), .D1(n27073), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[4]));
    LUT4 mux_673_i9_3_lut_4_lut (.A(pending_sequence[8]), .B(accepted_sequence_spi[8]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[8])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i9_3_lut_4_lut.init = 16'hacaa;
    LUT4 n27160_bdd_4_lut_16232_then_3_lut (.A(mic_ram_rd_data[12]), .B(mic_ram_rd_data[14]), 
         .C(status_bit_index[1]), .Z(n27989)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27160_bdd_4_lut_16232_then_3_lut.init = 16'hacac;
    LUT4 n27160_bdd_4_lut_16232_else_3_lut (.A(mic_ram_rd_data[13]), .B(mic_ram_rd_data[15]), 
         .C(status_bit_index[1]), .Z(n27988)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27160_bdd_4_lut_16232_else_3_lut.init = 16'hacac;
    LUT4 i15085_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_2)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15085_2_lut_3_lut.init = 16'h0202;
    LUT4 n27148_bdd_4_lut_then_3_lut (.A(mic_ram_rd_data[8]), .B(mic_ram_rd_data[10]), 
         .C(status_bit_index[1]), .Z(n27992)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27148_bdd_4_lut_then_3_lut.init = 16'hacac;
    MUX21 i15670100_i1 (.D0(n27235), .D1(n27253), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[0]));
    MUX21 i1559060_i1 (.D0(n27055), .D1(n27079), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[5]));
    LUT4 i10046_4_lut (.A(ev_rd_data[7]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[7])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10046_4_lut.init = 16'h1222;
    LUT4 i10045_4_lut (.A(ev_rd_data[8]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[8])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10045_4_lut.init = 16'h1222;
    LUT4 n27148_bdd_4_lut_else_3_lut (.A(mic_ram_rd_data[9]), .B(mic_ram_rd_data[11]), 
         .C(status_bit_index[1]), .Z(n27991)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27148_bdd_4_lut_else_3_lut.init = 16'hacac;
    MUX21 i15716123_i1 (.D0(n27223), .D1(n27229), .SD(status_next_bit_N_2754[4]), 
          .Z(n26549));
    LUT4 i10044_4_lut (.A(ev_rd_data[9]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[9])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10044_4_lut.init = 16'h1222;
    CCU2D fpga_time_3049_add_4_13 (.A0(fpga_time[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24466), .COUT(n24467), .S0(n154), .S1(n153));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_13.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_13.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_13.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_13.INJECT1_1 = "NO";
    MUX21 i1566497_i1 (.D0(n27049), .D1(n27403), .SD(status_next_bit_N_2754[2]), 
          .Z(n26601));
    LUT4 i9993_4_lut (.A(ev_rd_data[60]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[60])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9993_4_lut.init = 16'h1222;
    MUX21 i1557854_i1 (.D0(n27103), .D1(n27109), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[7]));
    LUT4 mod_toggle_sync_I_0_2_lut (.A(mod_toggle_sync), .B(mod_toggle_seen), 
         .Z(feed_rd_7__N_1692)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(689[13:47])
    defparam mod_toggle_sync_I_0_2_lut.init = 16'h6666;
    MUX21 i1560869_i1 (.D0(n27355), .D1(n27427), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[2]));
    LUT4 i10043_4_lut (.A(ev_rd_data[10]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[10])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10043_4_lut.init = 16'h1222;
    LUT4 i1_4_lut_adj_327 (.A(mic_run_state_1__N_2390[1]), .B(n25757), .C(n10677), 
         .D(mic_warm), .Z(pll_clk_enable_159)) /* synthesis lut_function=(A+(B (C+!(D))+!B (C (D)))) */ ;
    defparam i1_4_lut_adj_327.init = 16'hfaee;
    PFUMX spi1_miso_I_23 (.BLUT(spi1_miso_N_2433), .ALUT(pcm_audio_miso), 
          .C0(spi_audio_read_cmd), .Z(spi1_miso_N_2432));
    VLO i1 (.Z(GND_net));
    TSALL TSALL_INST (.TSALL(GND_net));
    CCU2D fpga_time_3049_add_4_11 (.A0(fpga_time[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24465), .COUT(n24466), .S0(n156), .S1(n155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_11.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_11.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_11.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_11.INJECT1_1 = "NO";
    CCU2D fpga_time_3049_add_4_9 (.A0(fpga_time[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24464), .COUT(n24465), .S0(n158), .S1(n157));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_9.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_9.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_9.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_9.INJECT1_1 = "NO";
    CCU2D fpga_time_3049_add_4_7 (.A0(fpga_time[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24463), .COUT(n24464), .S0(n160), .S1(n159));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_7.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_7.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_7.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_7.INJECT1_1 = "NO";
    CCU2D fpga_time_3049_add_4_5 (.A0(fpga_time[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24462), .COUT(n24463), .S0(n162), .S1(n161));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_5.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_5.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_5.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_5.INJECT1_1 = "NO";
    PFUMX i16308 (.BLUT(n27764), .ALUT(n27762), .C0(spi_audio_read_cmd), 
          .Z(n27765));
    CCU2D fpga_time_3049_add_4_3 (.A0(fpga_time[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24461), .COUT(n24462), .S0(n164), .S1(n163));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_3.INIT0 = 16'hfaaa;
    defparam fpga_time_3049_add_4_3.INIT1 = 16'hfaaa;
    defparam fpga_time_3049_add_4_3.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_3.INJECT1_1 = "NO";
    PUR PUR_INST (.PUR(VCC_net));
    defparam PUR_INST.RST_PULSE = 1;
    CCU2D fpga_time_3049_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(n21086), .B1(n24623), .C1(fpga_time[0]), 
          .D1(GND_net), .COUT(n24461), .S1(n165));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(638[35:52])
    defparam fpga_time_3049_add_4_1.INIT0 = 16'hF000;
    defparam fpga_time_3049_add_4_1.INIT1 = 16'h7878;
    defparam fpga_time_3049_add_4_1.INJECT1_0 = "NO";
    defparam fpga_time_3049_add_4_1.INJECT1_1 = "NO";
    LUT4 i15212_2_lut (.A(mic_warm), .B(mic_ultrasonic), .Z(mic_warm_half_7__N_1887)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15212_2_lut.init = 16'h1111;
    LUT4 i10042_4_lut (.A(ev_rd_data[11]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[11])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10042_4_lut.init = 16'h1222;
    LUT4 i1_3_lut_adj_328 (.A(n11897), .B(n11899), .C(n12_adj_3247), .Z(n16168)) /* synthesis lut_function=(!(A (B+(C))+!A !(B+(C)))) */ ;
    defparam i1_3_lut_adj_328.init = 16'h5656;
    LUT4 i10041_4_lut (.A(ev_rd_data[12]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[12])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10041_4_lut.init = 16'h1222;
    LUT4 ev_run_hold_s5_83__I_0_i5_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_4), .D(ev_run_hold_s5[4]), .Z(us_tx_next[4])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i5_3_lut_4_lut.init = 16'hbf40;
    LUT4 shift_right_238_i11_4_lut (.A(n21039), .B(n28017), .C(status_bit_index[2]), 
         .D(pcm_read_count_latched_spi[4]), .Z(n11_adj_3275)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(431[11:70])
    defparam shift_right_238_i11_4_lut.init = 16'hcac0;
    LUT4 i2_2_lut_adj_329 (.A(link_idle[3]), .B(n4), .Z(link_timeout)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(383[25:35])
    defparam i2_2_lut_adj_329.init = 16'h8888;
    LUT4 cs_sync_d_I_0_1569_2_lut (.A(cs_sync_d), .B(cs_sync), .Z(cs_fall)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(388[20:41])
    defparam cs_sync_d_I_0_1569_2_lut.init = 16'h2222;
    LUT4 i3984_2_lut (.A(link_idle[0]), .B(pll_clk_enable_141), .Z(n11_adj_3152)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i3984_2_lut.init = 16'h6666;
    LUT4 i10040_4_lut (.A(ev_rd_data[13]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[13])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10040_4_lut.init = 16'h1222;
    CCU2D add_2505_9 (.A0(\mic_acc_i[1] [6]), .B0(n20537), .C0(n4489), 
          .D0(GND_net), .A1(\mic_acc_i[1] [7]), .B1(n20537), .C1(n4489), 
          .D1(GND_net), .CIN(n24459), .S0(n5667), .S1(n5666));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam add_2505_9.INIT0 = 16'h7848;
    defparam add_2505_9.INIT1 = 16'h7848;
    defparam add_2505_9.INJECT1_0 = "NO";
    defparam add_2505_9.INJECT1_1 = "NO";
    LUT4 i15194_4_lut (.A(fpga_cs_n_c), .B(spi_bit_count_2__N_1415), .C(frame_end), 
         .D(n31), .Z(spi1_sck_c_enable_146)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;
    defparam i15194_4_lut.init = 16'h4000;
    LUT4 stop_toggle_sync_I_0_2_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .Z(stop_event)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(382[23:61])
    defparam stop_toggle_sync_I_0_2_lut.init = 16'h6666;
    PFUMX i16410 (.BLUT(n27946), .ALUT(n27947), .C0(status_next_index[4]), 
          .Z(n27948));
    LUT4 mux_673_i8_3_lut_4_lut (.A(pending_sequence[7]), .B(accepted_sequence_spi[7]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[7])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i8_3_lut_4_lut.init = 16'hacaa;
    CCU2D add_2505_7 (.A0(\mic_acc_i[1] [4]), .B0(n20537), .C0(n4489), 
          .D0(GND_net), .A1(\mic_acc_i[1] [5]), .B1(n20537), .C1(n4489), 
          .D1(GND_net), .CIN(n24458), .COUT(n24459), .S0(n5669), .S1(n5668));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam add_2505_7.INIT0 = 16'h7848;
    defparam add_2505_7.INIT1 = 16'h7848;
    defparam add_2505_7.INJECT1_0 = "NO";
    defparam add_2505_7.INJECT1_1 = "NO";
    L6MUX21 i16552 (.D0(n27930), .D1(n28184), .SD(status_bit_index[2]), 
            .Z(n28185));
    LUT4 mux_1041_i2_4_lut (.A(mic_gate_active), .B(n10805), .C(n10819), 
         .D(mic_acc_state[1]), .Z(mic_acc_state_2__N_2327[1])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(947[9] 967[12])
    defparam mux_1041_i2_4_lut.init = 16'hca0a;
    LUT4 i15088_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_0)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15088_2_lut_3_lut_4_lut.init = 16'h0001;
    LUT4 i2_3_lut_adj_330 (.A(n8_adj_3273), .B(n11901), .C(n11903), .Z(n12_adj_3247)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(492[50:75])
    defparam i2_3_lut_adj_330.init = 16'hfefe;
    LUT4 equal_1615_i17_2_lut_3_lut (.A(n8_adj_3177), .B(n16922), .C(n16_adj_3282), 
         .Z(n17_adj_3278)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(456[57:94])
    defparam equal_1615_i17_2_lut_3_lut.init = 16'hfefe;
    LUT4 n27130_bdd_4_lut_16167_else_3_lut (.A(n14020), .B(n14021), .C(status_bit_index[0]), 
         .Z(n28003)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27130_bdd_4_lut_16167_else_3_lut.init = 16'hacac;
    LUT4 i1_2_lut_adj_331 (.A(mic_cfg_toggle_spi), .B(n16085), .Z(mic_cfg_toggle_spi_N_2681)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_331.init = 16'h9999;
    LUT4 i10039_4_lut (.A(ev_rd_data[14]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[14])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10039_4_lut.init = 16'h1222;
    CCU2D add_2505_5 (.A0(\mic_acc_i[1] [2]), .B0(n20537), .C0(n4489), 
          .D0(GND_net), .A1(\mic_acc_i[1] [3]), .B1(n20537), .C1(n4489), 
          .D1(GND_net), .CIN(n24457), .COUT(n24458), .S0(n5671), .S1(n5670));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam add_2505_5.INIT0 = 16'h7848;
    defparam add_2505_5.INIT1 = 16'h7848;
    defparam add_2505_5.INJECT1_0 = "NO";
    defparam add_2505_5.INJECT1_1 = "NO";
    LUT4 equal_1614_i13_2_lut (.A(spi_byte_count[4]), .B(spi_byte_count[5]), 
         .Z(n4_adj_3147)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(456[97:138])
    defparam equal_1614_i13_2_lut.init = 16'heeee;
    LUT4 i9774_2_lut (.A(mic_acc_busy_N_2272), .B(mic_done_N_2646), .Z(n20519)) /* synthesis lut_function=((B)+!A) */ ;
    defparam i9774_2_lut.init = 16'hdddd;
    CCU2D add_2505_3 (.A0(\mic_acc_i[1] [0]), .B0(n20537), .C0(n4489), 
          .D0(GND_net), .A1(\mic_acc_i[1] [1]), .B1(n20537), .C1(n4489), 
          .D1(GND_net), .CIN(n24456), .COUT(n24457), .S0(n5673), .S1(n5672));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam add_2505_3.INIT0 = 16'h7484;
    defparam add_2505_3.INIT1 = 16'h7848;
    defparam add_2505_3.INJECT1_0 = "NO";
    defparam add_2505_3.INJECT1_1 = "NO";
    LUT4 i3112_4_lut (.A(link_timeout), .B(cs_fall), .C(link_tick_d), 
         .D(fpga_time[10]), .Z(pll_clk_enable_141)) /* synthesis lut_function=(A (B)+!A (B+!(C+!(D)))) */ ;
    defparam i3112_4_lut.init = 16'hcdcc;
    PFUMX i16408 (.BLUT(n27943), .ALUT(n27944), .C0(status_bit_index[0]), 
          .Z(n27945));
    LUT4 i1_4_lut_adj_332 (.A(mic_warm), .B(mic_phase[2]), .C(mic_phase[1]), 
         .D(mic_phase[0]), .Z(n5635)) /* synthesis lut_function=(!(A+(B (C (D))+!B !(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_4_lut_adj_332.init = 16'h1444;
    LUT4 i10038_4_lut (.A(ev_rd_data[15]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[15])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10038_4_lut.init = 16'h1222;
    CCU2D add_2505_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20537), .B1(n4489), .C1(GND_net), .D1(GND_net), .COUT(n24456));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(848[17:43])
    defparam add_2505_1.INIT0 = 16'hF000;
    defparam add_2505_1.INIT1 = 16'h7777;
    defparam add_2505_1.INJECT1_0 = "NO";
    defparam add_2505_1.INJECT1_1 = "NO";
    LUT4 i10037_4_lut (.A(ev_rd_data[16]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[16])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10037_4_lut.init = 16'h1222;
    PFUMX i16550 (.BLUT(n28033), .ALUT(n28183), .C0(pcm_spi_sample_15__N_1371[4]), 
          .Z(n28184));
    INV i16817 (.A(spi_mic_sck_c), .Z(sck_N_3124));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(95[24:35])
    CCU2D add_1059_7 (.A0(mic_gate_index[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_index[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24451), .S0(mic_done_N_2647[5]), .S1(mic_done_N_2647[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam add_1059_7.INIT0 = 16'h5aaa;
    defparam add_1059_7.INIT1 = 16'h5aaa;
    defparam add_1059_7.INJECT1_0 = "NO";
    defparam add_1059_7.INJECT1_1 = "NO";
    LUT4 n27118_bdd_4_lut_then_3_lut (.A(mic_block_count[12]), .B(mic_block_count[14]), 
         .C(status_bit_index[1]), .Z(n28007)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27118_bdd_4_lut_then_3_lut.init = 16'hacac;
    CCU2D add_1059_5 (.A0(mic_gate_index[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_index[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24450), .COUT(n24451), .S0(mic_done_N_2647[3]), 
          .S1(mic_done_N_2647[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam add_1059_5.INIT0 = 16'h5aaa;
    defparam add_1059_5.INIT1 = 16'h5aaa;
    defparam add_1059_5.INJECT1_0 = "NO";
    defparam add_1059_5.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_333 (.A(mic_warm), .B(mic_phase[3]), .C(n65), .D(mic_phase[0]), 
         .Z(n5634)) /* synthesis lut_function=(!(A+(B (C (D))+!B !(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_4_lut_adj_333.init = 16'h1444;
    LUT4 n27118_bdd_4_lut_else_3_lut (.A(mic_block_count[13]), .B(mic_block_count[15]), 
         .C(status_bit_index[1]), .Z(n28006)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27118_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 i1_2_lut_adj_334 (.A(mic_phase[2]), .B(mic_phase[1]), .Z(n65)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam i1_2_lut_adj_334.init = 16'h8888;
    PFUMX i16406 (.BLUT(n27940), .ALUT(n27941), .C0(status_bit_index[0]), 
          .Z(n27942));
    LUT4 i2_3_lut_adj_335 (.A(mic_cfg_settle[2]), .B(mic_cfg_settle[3]), 
         .C(mic_cfg_settle[1]), .Z(n17)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_3_lut_adj_335.init = 16'hfefe;
    LUT4 i10036_4_lut (.A(ev_rd_data[17]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[17])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10036_4_lut.init = 16'h1222;
    LUT4 i5_2_lut (.A(mic_cfg_toggle_seen), .B(mic_cfg_toggle_sync), .Z(mic_cfg_settle_3__N_2126)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[58:77])
    defparam i5_2_lut.init = 16'h6666;
    CCU2D add_1059_3 (.A0(mic_gate_index[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_index[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24449), .COUT(n24450), .S0(mic_done_N_2647[1]), 
          .S1(mic_done_N_2647[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam add_1059_3.INIT0 = 16'h5aaa;
    defparam add_1059_3.INIT1 = 16'h5aaa;
    defparam add_1059_3.INJECT1_0 = "NO";
    defparam add_1059_3.INJECT1_1 = "NO";
    LUT4 status_next_bit_I_0_i94_4_lut (.A(n78_adj_3261), .B(status_hold[88]), 
         .C(status_next_index[4]), .D(n24835), .Z(n94)) /* synthesis lut_function=(A (B (C+(D))+!B (C))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam status_next_bit_I_0_i94_4_lut.init = 16'haca0;
    CCU2D add_1059_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(mic_gate_index[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24449), .S1(mic_done_N_2647[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam add_1059_1.INIT0 = 16'hF000;
    defparam add_1059_1.INIT1 = 16'h5555;
    defparam add_1059_1.INJECT1_0 = "NO";
    defparam add_1059_1.INJECT1_1 = "NO";
    LUT4 i13107_3_lut (.A(pcm_rd_ptr_spi[4]), .B(pcm_wr_sync2_spi[4]), .C(audio_reset_end), 
         .Z(n23788)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[31:74])
    defparam i13107_3_lut.init = 16'hcaca;
    LUT4 i2953_2_lut (.A(mic_acc_busy_N_2272), .B(mic_done_N_2646), .Z(mic_done_N_2640)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(969[9] 1010[16])
    defparam i2953_2_lut.init = 16'h8888;
    LUT4 i10035_4_lut (.A(ev_rd_data[18]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[18])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10035_4_lut.init = 16'h1222;
    LUT4 spi1_miso_I_28_i49_then_4_lut (.A(status_bit_index[1]), .B(mic_run_state[0]), 
         .C(status_bit_index[0]), .D(mic_run_state[1]), .Z(n28010)) /* synthesis lut_function=(A (B (C+!(D))+!B (C))+!A !(B+(C+!(D)))) */ ;
    defparam spi1_miso_I_28_i49_then_4_lut.init = 16'ha1a8;
    LUT4 n16096_bdd_4_lut_16794 (.A(n7_adj_3256), .B(mod_active), .C(frame_toggle_seen), 
         .D(frame_toggle_sync), .Z(n28590)) /* synthesis lut_function=(!(A+(B+!(C (D)+!C !(D))))) */ ;
    defparam n16096_bdd_4_lut_16794.init = 16'h1001;
    CCU2D add_1050_7 (.A0(mic_gate_fill[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_fill[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24447), .S0(mic_acc_last_N_2665[5]), .S1(mic_acc_last_N_2665[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(995[42:62])
    defparam add_1050_7.INIT0 = 16'h5aaa;
    defparam add_1050_7.INIT1 = 16'h5aaa;
    defparam add_1050_7.INJECT1_0 = "NO";
    defparam add_1050_7.INJECT1_1 = "NO";
    LUT4 i3344_2_lut (.A(link_idle[1]), .B(link_idle[0]), .Z(n3184)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(644[26:42])
    defparam i3344_2_lut.init = 16'h6666;
    LUT4 i9992_4_lut (.A(ev_rd_data[61]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[61])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9992_4_lut.init = 16'h1222;
    LUT4 i10034_4_lut (.A(ev_rd_data[19]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[19])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10034_4_lut.init = 16'h1222;
    LUT4 m1_lut (.Z(n28627)) /* synthesis lut_function=1, syn_instantiated=1 */ ;
    defparam m1_lut.init = 16'hffff;
    LUT4 i10033_4_lut (.A(ev_rd_data[20]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[20])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10033_4_lut.init = 16'h1222;
    PFUMX i16404 (.BLUT(n27937), .ALUT(n27938), .C0(status_bit_index[0]), 
          .Z(n27939));
    LUT4 reduce_or_1675_i1_3_lut (.A(n10024), .B(ev_sum[8]), .C(ev_in_clear), 
         .Z(n10058)) /* synthesis lut_function=(A+(B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam reduce_or_1675_i1_3_lut.init = 16'heaea;
    LUT4 spi1_miso_I_28_i49_else_4_lut (.A(status_bit_index[1]), .B(mic_run_state[0]), 
         .C(status_bit_index[0]), .D(mic_run_state[1]), .Z(n28009)) /* synthesis lut_function=(!(A ((C+(D))+!B)+!A (B+(C+!(D))))) */ ;
    defparam spi1_miso_I_28_i49_else_4_lut.init = 16'h0108;
    LUT4 i14673_2_lut (.A(status_next_index[0]), .B(status_next_index[1]), 
         .Z(n25614)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14673_2_lut.init = 16'h8888;
    LUT4 sub_225_inv_0_i5_1_lut (.A(status_next_index[4]), .Z(status_next_bit_N_2754[4])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[49:80])
    defparam sub_225_inv_0_i5_1_lut.init = 16'h5555;
    LUT4 i3358_2_lut (.A(link_idle[3]), .B(n4), .Z(n3182)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(644[26:42])
    defparam i3358_2_lut.init = 16'h6666;
    LUT4 i5_4_lut_adj_336 (.A(n15_adj_3155), .B(n10_adj_3263), .C(n11), 
         .D(spi_byte_count[4]), .Z(n12)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i5_4_lut_adj_336.init = 16'h0080;
    LUT4 i9991_4_lut (.A(ev_rd_data[62]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[62])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9991_4_lut.init = 16'h1222;
    LUT4 i1_2_lut_adj_337 (.A(frame_toggle_spi), .B(n21266), .Z(frame_toggle_spi_N_2508)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_337.init = 16'h9999;
    LUT4 mic_cfg_count_6__I_0_i14_3_lut (.A(n12_adj_3224), .B(mic_done_N_2647[6]), 
         .C(mic_cfg_count[6]), .Z(mic_done_N_2646)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_cfg_count_6__I_0_i14_3_lut.init = 16'h8e8e;
    LUT4 mic_cfg_count_6__I_0_i12_3_lut (.A(n10_adj_3225), .B(mic_done_N_2647[5]), 
         .C(mic_cfg_count[5]), .Z(n12_adj_3224)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_cfg_count_6__I_0_i12_3_lut.init = 16'h8e8e;
    CCU2D add_1585_9 (.A0(phase_frac[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24317), .COUT(n24318), .S0(phase_frac_sum[7]), 
          .S1(phase_frac_sum[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1585_9.INIT0 = 16'h5555;
    defparam add_1585_9.INIT1 = 16'h5555;
    defparam add_1585_9.INJECT1_0 = "NO";
    defparam add_1585_9.INJECT1_1 = "NO";
    LUT4 mic_cfg_count_6__I_0_i10_3_lut (.A(n8_adj_3226), .B(mic_done_N_2647[4]), 
         .C(mic_cfg_count[4]), .Z(n10_adj_3225)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_cfg_count_6__I_0_i10_3_lut.init = 16'h8e8e;
    LUT4 i1_4_lut_adj_338 (.A(staging_rd_page_N_2718), .B(staging_rd_page_N_2719), 
         .C(n10061), .D(phase_step_s3), .Z(n25425)) /* synthesis lut_function=(A+(B (C+(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[9] 774[16])
    defparam i1_4_lut_adj_338.init = 16'hfefa;
    LUT4 i9990_4_lut (.A(ev_rd_data[63]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[63])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9990_4_lut.init = 16'h1222;
    LUT4 mux_673_i7_3_lut_4_lut (.A(pending_sequence[6]), .B(accepted_sequence_spi[6]), 
         .C(ws2812_settle[1]), .D(n28057), .Z(accepted_sequence_15__N_995[6])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(712[13] 718[16])
    defparam mux_673_i7_3_lut_4_lut.init = 16'hacaa;
    CCU2D add_1050_5 (.A0(mic_gate_fill[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_fill[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24446), .COUT(n24447), .S0(mic_acc_last_N_2665[3]), 
          .S1(mic_acc_last_N_2665[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(995[42:62])
    defparam add_1050_5.INIT0 = 16'h5aaa;
    defparam add_1050_5.INIT1 = 16'h5aaa;
    defparam add_1050_5.INJECT1_0 = "NO";
    defparam add_1050_5.INJECT1_1 = "NO";
    LUT4 i10032_4_lut (.A(ev_rd_data[21]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[21])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10032_4_lut.init = 16'h1222;
    CCU2D add_1050_3 (.A0(mic_gate_fill[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_fill[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24445), .COUT(n24446), .S0(mic_acc_last_N_2665[1]), 
          .S1(mic_acc_last_N_2665[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(995[42:62])
    defparam add_1050_3.INIT0 = 16'h5aaa;
    defparam add_1050_3.INIT1 = 16'h5aaa;
    defparam add_1050_3.INJECT1_0 = "NO";
    defparam add_1050_3.INJECT1_1 = "NO";
    CCU2D add_1050_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(mic_gate_fill[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24445), .S1(mic_acc_last_N_2665[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(995[42:62])
    defparam add_1050_1.INIT0 = 16'hF000;
    defparam add_1050_1.INIT1 = 16'h5555;
    defparam add_1050_1.INJECT1_0 = "NO";
    defparam add_1050_1.INJECT1_1 = "NO";
    LUT4 i10031_4_lut (.A(ev_rd_data[22]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[22])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10031_4_lut.init = 16'h1222;
    ws2812_stream ws2812_i (.reset_count({Open_0, Open_1, Open_2, Open_3, 
            Open_4, Open_5, Open_6, Open_7, Open_8, Open_9, Open_10, 
            Open_11, Open_12, Open_13, reset_count[0]}), .pll_clk(pll_clk), 
            .pll_clk_enable_138(pll_clk_enable_138), .ws2812_enable(ws2812_enable), 
            .GND_net(GND_net), .rgb_hold({rgb_hold}), .n10912(n10912), 
            .ws2812_frame_done(ws2812_frame_done), .rgb_data_c(rgb_data_c), 
            .n13359(n13359), .n24165(n24165), .n13623(n13623)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1021[19] 1027[6])
    LUT4 mic_cfg_count_6__I_0_i8_3_lut (.A(n6_adj_3227), .B(mic_done_N_2647[3]), 
         .C(mic_cfg_count[3]), .Z(n8_adj_3226)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_cfg_count_6__I_0_i8_3_lut.init = 16'h8e8e;
    CCU2D add_1048_16 (.A0(mic_ram_rd_data[14]), .B0(mic_acc_win[7]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[15]), .B1(mic_acc_win[7]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24443), .S0(mic_ram_wr_data_15__N_2296[14]), 
          .S1(mic_ram_wr_data_15__N_2296[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(988[36:68])
    defparam add_1048_16.INIT0 = 16'h5666;
    defparam add_1048_16.INIT1 = 16'h5666;
    defparam add_1048_16.INJECT1_0 = "NO";
    defparam add_1048_16.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i4_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_3), .D(ev_run_hold_s5[3]), .Z(us_tx_next[3])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i4_3_lut_4_lut.init = 16'hbf40;
    CCU2D add_1048_14 (.A0(mic_ram_rd_data[12]), .B0(mic_acc_win[7]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[13]), .B1(mic_acc_win[7]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24442), .COUT(n24443), .S0(mic_ram_wr_data_15__N_2296[12]), 
          .S1(mic_ram_wr_data_15__N_2296[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(988[36:68])
    defparam add_1048_14.INIT0 = 16'h5666;
    defparam add_1048_14.INIT1 = 16'h5666;
    defparam add_1048_14.INJECT1_0 = "NO";
    defparam add_1048_14.INJECT1_1 = "NO";
    LUT4 i14922_2_lut (.A(n25409), .B(status_miso_q), .Z(n25721)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[9] 431[79])
    defparam i14922_2_lut.init = 16'h8888;
    LUT4 i3_4_lut_adj_339 (.A(n104), .B(pll_clk_enable_99), .C(mic_phase[2]), 
         .D(mic_phase[1]), .Z(n13362)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i3_4_lut_adj_339.init = 16'h0020;
    LUT4 mic_cfg_count_6__I_0_i6_3_lut (.A(n4_adj_3228), .B(mic_done_N_2647[2]), 
         .C(mic_cfg_count[2]), .Z(n6_adj_3227)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_cfg_count_6__I_0_i6_3_lut.init = 16'h8e8e;
    LUT4 i10030_4_lut (.A(ev_rd_data[23]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[23])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10030_4_lut.init = 16'h1222;
    PFUMX i16546 (.BLUT(n27934), .ALUT(n28180), .C0(pcm_spi_sample_15__N_1371[4]), 
          .Z(n28181));
    LUT4 i10029_4_lut (.A(ev_rd_data[24]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[24])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10029_4_lut.init = 16'h1222;
    CCU2D add_1048_12 (.A0(mic_ram_rd_data[10]), .B0(mic_acc_win[7]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[11]), .B1(mic_acc_win[7]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24441), .COUT(n24442), .S0(mic_ram_wr_data_15__N_2296[10]), 
          .S1(mic_ram_wr_data_15__N_2296[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(988[36:68])
    defparam add_1048_12.INIT0 = 16'h5666;
    defparam add_1048_12.INIT1 = 16'h5666;
    defparam add_1048_12.INJECT1_0 = "NO";
    defparam add_1048_12.INJECT1_1 = "NO";
    LUT4 i9989_4_lut (.A(ev_rd_data[64]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[64])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9989_4_lut.init = 16'h1222;
    CCU2D add_1048_10 (.A0(mic_ram_rd_data[8]), .B0(mic_acc_win[7]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[9]), .B1(mic_acc_win[7]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24440), .COUT(n24441), .S0(mic_ram_wr_data_15__N_2296[8]), 
          .S1(mic_ram_wr_data_15__N_2296[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(988[36:68])
    defparam add_1048_10.INIT0 = 16'h5666;
    defparam add_1048_10.INIT1 = 16'h5666;
    defparam add_1048_10.INJECT1_0 = "NO";
    defparam add_1048_10.INJECT1_1 = "NO";
    LUT4 mic_cfg_count_6__I_0_i4_4_lut (.A(mic_done_N_2647[0]), .B(mic_done_N_2647[1]), 
         .C(mic_cfg_count[1]), .D(mic_cfg_count[0]), .Z(n4_adj_3228)) /* synthesis lut_function=(A (B+!(C))+!A !(B (C (D))+!B (C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1007[17:33])
    defparam mic_cfg_count_6__I_0_i4_4_lut.init = 16'h8ecf;
    LUT4 i10028_4_lut (.A(ev_rd_data[25]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[25])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10028_4_lut.init = 16'h1222;
    LUT4 i3115_2_lut (.A(ws2812_enable_N_2578), .B(n13578), .Z(pll_clk_enable_216)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i3115_2_lut.init = 16'heeee;
    LUT4 i3351_2_lut_3_lut (.A(link_idle[0]), .B(link_idle[1]), .C(link_idle[2]), 
         .Z(n3183)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(383[25:35])
    defparam i3351_2_lut_3_lut.init = 16'h7878;
    LUT4 i15183_2_lut_3_lut (.A(mic_acc_state[1]), .B(mic_acc_state[0]), 
         .C(mic_acc_state[2]), .Z(mic_acc_busy_N_2272)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;
    defparam i15183_2_lut_3_lut.init = 16'h1010;
    LUT4 i15057_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_4)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15057_2_lut_3_lut_4_lut.init = 16'h0100;
    LUT4 ev_run_hold_s5_83__I_0_i1_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_0), .D(ev_run_hold_s5[0]), .Z(us_tx_next[0])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i1_3_lut_4_lut.init = 16'hbf40;
    LUT4 n16096_bdd_4_lut (.A(n16096), .B(ws2812_enable_N_2591), .C(frame_req_mod_N_2737), 
         .D(n28590), .Z(pll_clk_enable_259)) /* synthesis lut_function=(A+(B (C+(D))+!B (C))) */ ;
    defparam n16096_bdd_4_lut.init = 16'hfefa;
    CCU2D add_1048_8 (.A0(mic_ram_rd_data[6]), .B0(mic_acc_win[6]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[7]), .B1(mic_acc_win[7]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24439), .COUT(n24440), .S0(mic_ram_wr_data_15__N_2296[6]), 
          .S1(mic_ram_wr_data_15__N_2296[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(988[36:68])
    defparam add_1048_8.INIT0 = 16'h5666;
    defparam add_1048_8.INIT1 = 16'h5666;
    defparam add_1048_8.INJECT1_0 = "NO";
    defparam add_1048_8.INJECT1_1 = "NO";
    CCU2D add_1048_6 (.A0(mic_ram_rd_data[4]), .B0(mic_acc_win[4]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[5]), .B1(mic_acc_win[5]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24438), .COUT(n24439), .S0(mic_ram_wr_data_15__N_2296[4]), 
          .S1(mic_ram_wr_data_15__N_2296[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(988[36:68])
    defparam add_1048_6.INIT0 = 16'h5666;
    defparam add_1048_6.INIT1 = 16'h5666;
    defparam add_1048_6.INJECT1_0 = "NO";
    defparam add_1048_6.INJECT1_1 = "NO";
    CCU2D add_1048_4 (.A0(mic_ram_rd_data[2]), .B0(mic_acc_win[2]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[3]), .B1(mic_acc_win[3]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24437), .COUT(n24438), .S0(mic_ram_wr_data_15__N_2296[2]), 
          .S1(mic_ram_wr_data_15__N_2296[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(988[36:68])
    defparam add_1048_4.INIT0 = 16'h5666;
    defparam add_1048_4.INIT1 = 16'h5666;
    defparam add_1048_4.INJECT1_0 = "NO";
    defparam add_1048_4.INJECT1_1 = "NO";
    LUT4 i9988_4_lut (.A(ev_rd_data[65]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[65])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9988_4_lut.init = 16'h1222;
    PFUMX i16398 (.BLUT(n27928), .ALUT(n27929), .C0(pcm_spi_sample_15__N_1371[4]), 
          .Z(n27930));
    LUT4 i824_2_lut (.A(mic_data_1_c), .B(mic_lo_q_r), .Z(n4576)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(849[17:43])
    defparam i824_2_lut.init = 16'h6666;
    CCU2D add_1048_2 (.A0(mic_ram_rd_data[0]), .B0(mic_acc_win[0]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[1]), .B1(mic_acc_win[1]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24437), .S1(mic_ram_wr_data_15__N_2296[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(988[36:68])
    defparam add_1048_2.INIT0 = 16'h7000;
    defparam add_1048_2.INIT1 = 16'h5666;
    defparam add_1048_2.INJECT1_0 = "NO";
    defparam add_1048_2.INJECT1_1 = "NO";
    LUT4 i62_3_lut (.A(n43_adj_3279), .B(spi1_miso_N_2436), .C(status_bit_index[6]), 
         .Z(n37_adj_3277)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(435[25] 438[29])
    defparam i62_3_lut.init = 16'hcaca;
    LUT4 i6617_4_lut (.A(rgb_values_complete_spi), .B(accepted_update_flags_spi[1]), 
         .C(n10654), .D(n16758), .Z(ws2812_enable_N_2578)) /* synthesis lut_function=(A (B (C+(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(717[17:115])
    defparam i6617_4_lut.init = 16'ha8a0;
    LUT4 i10027_4_lut (.A(ev_rd_data[26]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[26])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10027_4_lut.init = 16'h1222;
    CCU2D time_divider_3048_add_4_7 (.A0(time_divider[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24435), .S0(n30));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(637[25:44])
    defparam time_divider_3048_add_4_7.INIT0 = 16'hfaaa;
    defparam time_divider_3048_add_4_7.INIT1 = 16'h0000;
    defparam time_divider_3048_add_4_7.INJECT1_0 = "NO";
    defparam time_divider_3048_add_4_7.INJECT1_1 = "NO";
    LUT4 staging_rd_page_N_2719_I_0_2_lut (.A(staging_rd_page_N_2719), .B(phase_step_s3), 
         .Z(ev_rd_grant)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[33:70])
    defparam staging_rd_page_N_2719_I_0_2_lut.init = 16'h2222;
    LUT4 i14895_2_lut (.A(status_miso_q), .B(n25409), .Z(n25788)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(435[25] 438[29])
    defparam i14895_2_lut.init = 16'h8888;
    CCU2D time_divider_3048_add_4_5 (.A0(time_divider[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24434), .COUT(n24435), .S0(n32), 
          .S1(n31_adj_3176));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(637[25:44])
    defparam time_divider_3048_add_4_5.INIT0 = 16'hfaaa;
    defparam time_divider_3048_add_4_5.INIT1 = 16'hfaaa;
    defparam time_divider_3048_add_4_5.INJECT1_0 = "NO";
    defparam time_divider_3048_add_4_5.INJECT1_1 = "NO";
    GSR GSR_INST (.GSR(fpga_cs_n_N_2452));
    LUT4 i1_3_lut_adj_340 (.A(n14797), .B(spi1_sck_c_enable_67), .C(n31), 
         .Z(spi1_sck_c_enable_161)) /* synthesis lut_function=(A (B)+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(471[18] 576[12])
    defparam i1_3_lut_adj_340.init = 16'h8c8c;
    umh_mic_iq_ram mic_iq_ram (.pll_clk(pll_clk), .mic_ram_we(mic_ram_we), 
            .VCC_net(VCC_net), .GND_net(GND_net), .mic_ram_wr_addr({mic_ram_wr_addr}), 
            .mic_ram_rd_addr({mic_ram_rd_addr}), .mic_ram_wr_data({mic_ram_wr_data}), 
            .mic_ram_rd_data({mic_ram_rd_data})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(344[20] 348[6])
    LUT4 i10026_4_lut (.A(ev_rd_data[27]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[27])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10026_4_lut.init = 16'h1222;
    umh_channel_ram18 staging_ram (.\rmw_slot[0] (rmw_slot[0]), .\build_level_eff[0] (build_level_eff[0]), 
            .spi_channel_index({spi_channel_index}), .staging_wr_page(staging_wr_page), 
            .ev_ch({ev_ch}), .staging_rd_page(staging_rd_page), .spi1_mosi_c_0(spi1_mosi_c_0), 
            .\spi_rx_shift[0] (spi_rx_shift[0]), .\spi_rx_shift[1] (spi_rx_shift[1]), 
            .\spi_rx_shift[2] (spi_rx_shift[2]), .\spi_rx_shift[3] (spi_rx_shift[3]), 
            .\spi_rx_shift[4] (spi_rx_shift[4]), .\spi_rx_shift[5] (spi_rx_shift[5]), 
            .\spi_rx_shift[6] (spi_rx_shift[6]), .\staging_q[1] (staging_q[1]), 
            .\staging_q[2] (staging_q[2]), .\staging_q[3] (staging_q[3]), 
            .\staging_q[5] (staging_q[5]), .\staging_q[6] (staging_q[6]), 
            .\staging_q[7] (staging_q[7]), .spi1_sck_c(spi1_sck_c), .spi_payload_byte(spi_payload_byte), 
            .VCC_net(VCC_net), .pll_clk(pll_clk), .GND_net(GND_net), .\build_level[1] (build_level[1]), 
            .build_mod(build_mod), .n15(n15), .\event_level[1] (event_level[1]), 
            .\ev_sum[4] (ev_sum[4]), .staging_rd_page_N_2718(staging_rd_page_N_2718), 
            .ev_in_clear(ev_in_clear), .\build_level[0] (build_level[0]), 
            .\event_level[0] (event_level[0]), .\build_level[4] (build_level[4]), 
            .\event_level[4] (event_level[4]), .n25784(n25784), .ev_fall_N_2683(ev_fall_N_2683), 
            .n25149(n25149), .n25736(n25736), .n25313(n25313)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[23] 329[6])
    FD1P3IX mic_acc_busy_1338 (.D(mic_acc_busy_N_2273), .SP(pll_clk_enable_604), 
            .CD(n27632), .CK(pll_clk), .Q(mic_acc_busy)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[12] 1011[8])
    defparam mic_acc_busy_1338.GSR = "DISABLED";
    LUT4 i13452_2_lut (.A(reset_count[0]), .B(n13359), .Z(n24165)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam i13452_2_lut.init = 16'h2222;
    LUT4 ev_run_hold_s5_83__I_0_i2_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_1), .D(ev_run_hold_s5[1]), .Z(us_tx_next[1])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1015[46:67])
    defparam ev_run_hold_s5_83__I_0_i2_3_lut_4_lut.init = 16'hbf40;
    CCU2D time_divider_3048_add_4_3 (.A0(time_divider[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24433), .COUT(n24434), .S0(n34), 
          .S1(n33));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(637[25:44])
    defparam time_divider_3048_add_4_3.INIT0 = 16'hfaaa;
    defparam time_divider_3048_add_4_3.INIT1 = 16'hfaaa;
    defparam time_divider_3048_add_4_3.INJECT1_0 = "NO";
    defparam time_divider_3048_add_4_3.INJECT1_1 = "NO";
    LUT4 i22_3_lut (.A(spi_byte_count[1]), .B(spi_byte_count[2]), .C(spi_byte_count[3]), 
         .Z(n11)) /* synthesis lut_function=(!(A ((C)+!B)+!A (B (C)+!B !(C)))) */ ;
    defparam i22_3_lut.init = 16'h1c1c;
    
endmodule
//
// Verilog Description of module umh_level_fifo
//

module umh_level_fifo (n13984, spi1_sck_c, feed_wr_spi, n13986, n13980, 
            n13982, feed_q, pll_clk, rd_data_7__N_2897, n56, spi1_mosi_c_0, 
            \spi_rx_shift[0] , \spi_rx_shift[1] , \spi_rx_shift[2] , \spi_rx_shift[3] , 
            \spi_rx_shift[4] , \spi_rx_shift[5] , \spi_rx_shift[6] , n13999, 
            n14001, n14003, n14005, n14007, n14009, n14011, n14013, 
            feed_we, VCC_net, GND_net, n13988, n13990, n13992, n13994) /* synthesis syn_module_defined=1 */ ;
    output n13984;
    input spi1_sck_c;
    input [7:0]feed_wr_spi;
    output n13986;
    output n13980;
    output n13982;
    output [7:0]feed_q;
    input pll_clk;
    input [7:0]rd_data_7__N_2897;
    input [7:0]n56;
    input spi1_mosi_c_0;
    input \spi_rx_shift[0] ;
    input \spi_rx_shift[1] ;
    input \spi_rx_shift[2] ;
    input \spi_rx_shift[3] ;
    input \spi_rx_shift[4] ;
    input \spi_rx_shift[5] ;
    input \spi_rx_shift[6] ;
    output n13999;
    output n14001;
    output n14003;
    output n14005;
    output n14007;
    output n14009;
    output n14011;
    output n14013;
    input feed_we;
    input VCC_net;
    input GND_net;
    output n13988;
    output n13990;
    output n13992;
    output n13994;
    
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(86[24:32])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(108[10:17])
    
    FD1S3AX mem_3289 (.D(feed_wr_spi[2]), .CK(spi1_sck_c), .Q(n13984));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3289.GSR = "DISABLED";
    FD1S3AX mem_3291 (.D(feed_wr_spi[3]), .CK(spi1_sck_c), .Q(n13986));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3291.GSR = "DISABLED";
    FD1S3AX mem_3285 (.D(feed_wr_spi[0]), .CK(spi1_sck_c), .Q(n13980));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3285.GSR = "DISABLED";
    FD1S3AX mem_3287 (.D(feed_wr_spi[1]), .CK(spi1_sck_c), .Q(n13982));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3287.GSR = "DISABLED";
    FD1S3AX rd_data_i0 (.D(rd_data_7__N_2897[0]), .CK(pll_clk), .Q(feed_q[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=374, LSE_RLINE=377 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i0.GSR = "DISABLED";
    DP8KC mem0 (.DIA0(spi1_mosi_c_0), .DIA1(\spi_rx_shift[0] ), .DIA2(\spi_rx_shift[1] ), 
          .DIA3(\spi_rx_shift[2] ), .DIA4(\spi_rx_shift[3] ), .DIA5(\spi_rx_shift[4] ), 
          .DIA6(\spi_rx_shift[5] ), .DIA7(\spi_rx_shift[6] ), .DIA8(GND_net), 
          .ADA0(VCC_net), .ADA1(GND_net), .ADA2(GND_net), .ADA3(feed_wr_spi[0]), 
          .ADA4(feed_wr_spi[1]), .ADA5(feed_wr_spi[2]), .ADA6(feed_wr_spi[3]), 
          .ADA7(feed_wr_spi[4]), .ADA8(feed_wr_spi[5]), .ADA9(feed_wr_spi[6]), 
          .ADA10(feed_wr_spi[7]), .ADA11(GND_net), .ADA12(GND_net), .CEA(VCC_net), 
          .OCEA(VCC_net), .CLKA(spi1_sck_c), .WEA(feed_we), .CSA0(GND_net), 
          .CSA1(GND_net), .CSA2(GND_net), .RSTA(GND_net), .DIB0(GND_net), 
          .DIB1(GND_net), .DIB2(GND_net), .DIB3(GND_net), .DIB4(GND_net), 
          .DIB5(GND_net), .DIB6(GND_net), .DIB7(GND_net), .DIB8(GND_net), 
          .ADB0(VCC_net), .ADB1(GND_net), .ADB2(GND_net), .ADB3(n56[0]), 
          .ADB4(n56[1]), .ADB5(n56[2]), .ADB6(n56[3]), .ADB7(n56[4]), 
          .ADB8(n56[5]), .ADB9(n56[6]), .ADB10(n56[7]), .ADB11(GND_net), 
          .ADB12(GND_net), .CEB(VCC_net), .OCEB(VCC_net), .CLKB(pll_clk), 
          .WEB(GND_net), .CSB0(GND_net), .CSB1(GND_net), .CSB2(GND_net), 
          .RSTB(GND_net), .DOB0(n13999), .DOB1(n14001), .DOB2(n14003), 
          .DOB3(n14005), .DOB4(n14007), .DOB5(n14009), .DOB6(n14011), 
          .DOB7(n14013));
    defparam mem0.DATA_WIDTH_A = 9;
    defparam mem0.DATA_WIDTH_B = 9;
    defparam mem0.REGMODE_A = "NOREG";
    defparam mem0.REGMODE_B = "NOREG";
    defparam mem0.CSDECODE_A = "0b000";
    defparam mem0.CSDECODE_B = "0b000";
    defparam mem0.WRITEMODE_A = "WRITETHROUGH";
    defparam mem0.WRITEMODE_B = "WRITETHROUGH";
    defparam mem0.GSR = "DISABLED";
    defparam mem0.RESETMODE = "SYNC";
    defparam mem0.ASYNC_RESET_RELEASE = "SYNC";
    defparam mem0.INIT_DATA = "STATIC";
    defparam mem0.INITVAL_00 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_01 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_02 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_03 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_04 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_05 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_06 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_07 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_08 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_09 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_10 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_11 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_12 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_13 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_14 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_15 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_16 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_17 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_18 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_19 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    FD1S3AX mem_3293 (.D(feed_wr_spi[4]), .CK(spi1_sck_c), .Q(n13988));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3293.GSR = "DISABLED";
    FD1S3AX mem_3295 (.D(feed_wr_spi[5]), .CK(spi1_sck_c), .Q(n13990));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3295.GSR = "DISABLED";
    FD1S3AX mem_3297 (.D(feed_wr_spi[6]), .CK(spi1_sck_c), .Q(n13992));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3297.GSR = "DISABLED";
    FD1S3AX mem_3299 (.D(feed_wr_spi[7]), .CK(spi1_sck_c), .Q(n13994));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3299.GSR = "DISABLED";
    FD1S3AX rd_data_i1 (.D(rd_data_7__N_2897[1]), .CK(pll_clk), .Q(feed_q[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=374, LSE_RLINE=377 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i1.GSR = "DISABLED";
    FD1S3AX rd_data_i2 (.D(rd_data_7__N_2897[2]), .CK(pll_clk), .Q(feed_q[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=374, LSE_RLINE=377 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i2.GSR = "DISABLED";
    FD1S3AX rd_data_i3 (.D(rd_data_7__N_2897[3]), .CK(pll_clk), .Q(feed_q[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=374, LSE_RLINE=377 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i3.GSR = "DISABLED";
    FD1S3AX rd_data_i4 (.D(rd_data_7__N_2897[4]), .CK(pll_clk), .Q(feed_q[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=374, LSE_RLINE=377 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i4.GSR = "DISABLED";
    FD1S3AX rd_data_i5 (.D(rd_data_7__N_2897[5]), .CK(pll_clk), .Q(feed_q[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=374, LSE_RLINE=377 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i5.GSR = "DISABLED";
    FD1S3AX rd_data_i6 (.D(rd_data_7__N_2897[6]), .CK(pll_clk), .Q(feed_q[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=374, LSE_RLINE=377 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i6.GSR = "DISABLED";
    FD1S3AX rd_data_i7 (.D(rd_data_7__N_2897[7]), .CK(pll_clk), .Q(feed_q[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=374, LSE_RLINE=377 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i7.GSR = "DISABLED";
    
endmodule
//
// Verilog Description of module umh_toggle_ram84
//

module umh_toggle_ram84 (pll_clk, ev_we, VCC_net, GND_net, rmw_slot, 
            event_rd_addr, active_bank_N_892, ev_wr_data, ev_rd_data) /* synthesis syn_module_defined=1 */ ;
    input pll_clk;
    input ev_we;
    input VCC_net;
    input GND_net;
    input [7:0]rmw_slot;
    input [8:0]event_rd_addr;
    input active_bank_N_892;
    input [83:0]ev_wr_data;
    output [83:0]ev_rd_data;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(108[10:17])
    
    PDPW8KC mem1 (.DI0(ev_wr_data[48]), .DI1(ev_wr_data[49]), .DI2(ev_wr_data[50]), 
            .DI3(ev_wr_data[51]), .DI4(ev_wr_data[52]), .DI5(ev_wr_data[53]), 
            .DI6(ev_wr_data[54]), .DI7(ev_wr_data[55]), .DI8(ev_wr_data[56]), 
            .DI9(ev_wr_data[57]), .DI10(ev_wr_data[58]), .DI11(ev_wr_data[59]), 
            .DI12(ev_wr_data[60]), .DI13(ev_wr_data[61]), .DI14(ev_wr_data[62]), 
            .DI15(ev_wr_data[63]), .DI16(ev_wr_data[64]), .DI17(ev_wr_data[65]), 
            .ADW0(rmw_slot[0]), .ADW1(rmw_slot[1]), .ADW2(rmw_slot[2]), 
            .ADW3(rmw_slot[3]), .ADW4(rmw_slot[4]), .ADW5(rmw_slot[5]), 
            .ADW6(rmw_slot[6]), .ADW7(rmw_slot[7]), .ADW8(active_bank_N_892), 
            .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), .CLKW(pll_clk), 
            .CSW0(GND_net), .CSW1(GND_net), .CSW2(GND_net), .ADR0(GND_net), 
            .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), .ADR4(event_rd_addr[0]), 
            .ADR5(event_rd_addr[1]), .ADR6(event_rd_addr[2]), .ADR7(event_rd_addr[3]), 
            .ADR8(event_rd_addr[4]), .ADR9(event_rd_addr[5]), .ADR10(event_rd_addr[6]), 
            .ADR11(event_rd_addr[7]), .ADR12(event_rd_addr[8]), .CER(VCC_net), 
            .OCER(VCC_net), .CLKR(pll_clk), .CSR0(GND_net), .CSR1(GND_net), 
            .CSR2(GND_net), .RST(GND_net), .DO0(ev_rd_data[57]), .DO1(ev_rd_data[58]), 
            .DO2(ev_rd_data[59]), .DO3(ev_rd_data[60]), .DO4(ev_rd_data[61]), 
            .DO5(ev_rd_data[62]), .DO6(ev_rd_data[63]), .DO7(ev_rd_data[64]), 
            .DO8(ev_rd_data[65]), .DO9(ev_rd_data[48]), .DO10(ev_rd_data[49]), 
            .DO11(ev_rd_data[50]), .DO12(ev_rd_data[51]), .DO13(ev_rd_data[52]), 
            .DO14(ev_rd_data[53]), .DO15(ev_rd_data[54]), .DO16(ev_rd_data[55]), 
            .DO17(ev_rd_data[56]));
    defparam mem1.DATA_WIDTH_W = 18;
    defparam mem1.DATA_WIDTH_R = 18;
    defparam mem1.REGMODE = "NOREG";
    defparam mem1.CSDECODE_W = "0b000";
    defparam mem1.CSDECODE_R = "0b000";
    defparam mem1.GSR = "DISABLED";
    defparam mem1.RESETMODE = "SYNC";
    defparam mem1.ASYNC_RESET_RELEASE = "SYNC";
    defparam mem1.INIT_DATA = "STATIC";
    defparam mem1.INITVAL_00 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_01 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_02 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_03 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_04 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_05 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_06 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_07 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_08 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_09 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_0A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_0B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_0C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_0D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_0E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_0F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_10 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_11 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_12 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_13 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_14 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_15 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_16 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_17 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_18 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_19 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_1A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_1B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_1C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_1D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_1E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem1.INITVAL_1F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    PDPW8KC mem2 (.DI0(ev_wr_data[30]), .DI1(ev_wr_data[31]), .DI2(ev_wr_data[32]), 
            .DI3(ev_wr_data[33]), .DI4(ev_wr_data[34]), .DI5(ev_wr_data[35]), 
            .DI6(ev_wr_data[36]), .DI7(ev_wr_data[37]), .DI8(ev_wr_data[38]), 
            .DI9(ev_wr_data[39]), .DI10(ev_wr_data[40]), .DI11(ev_wr_data[41]), 
            .DI12(ev_wr_data[42]), .DI13(ev_wr_data[43]), .DI14(ev_wr_data[44]), 
            .DI15(ev_wr_data[45]), .DI16(ev_wr_data[46]), .DI17(ev_wr_data[47]), 
            .ADW0(rmw_slot[0]), .ADW1(rmw_slot[1]), .ADW2(rmw_slot[2]), 
            .ADW3(rmw_slot[3]), .ADW4(rmw_slot[4]), .ADW5(rmw_slot[5]), 
            .ADW6(rmw_slot[6]), .ADW7(rmw_slot[7]), .ADW8(active_bank_N_892), 
            .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), .CLKW(pll_clk), 
            .CSW0(GND_net), .CSW1(GND_net), .CSW2(GND_net), .ADR0(GND_net), 
            .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), .ADR4(event_rd_addr[0]), 
            .ADR5(event_rd_addr[1]), .ADR6(event_rd_addr[2]), .ADR7(event_rd_addr[3]), 
            .ADR8(event_rd_addr[4]), .ADR9(event_rd_addr[5]), .ADR10(event_rd_addr[6]), 
            .ADR11(event_rd_addr[7]), .ADR12(event_rd_addr[8]), .CER(VCC_net), 
            .OCER(VCC_net), .CLKR(pll_clk), .CSR0(GND_net), .CSR1(GND_net), 
            .CSR2(GND_net), .RST(GND_net), .DO0(ev_rd_data[39]), .DO1(ev_rd_data[40]), 
            .DO2(ev_rd_data[41]), .DO3(ev_rd_data[42]), .DO4(ev_rd_data[43]), 
            .DO5(ev_rd_data[44]), .DO6(ev_rd_data[45]), .DO7(ev_rd_data[46]), 
            .DO8(ev_rd_data[47]), .DO9(ev_rd_data[30]), .DO10(ev_rd_data[31]), 
            .DO11(ev_rd_data[32]), .DO12(ev_rd_data[33]), .DO13(ev_rd_data[34]), 
            .DO14(ev_rd_data[35]), .DO15(ev_rd_data[36]), .DO16(ev_rd_data[37]), 
            .DO17(ev_rd_data[38]));
    defparam mem2.DATA_WIDTH_W = 18;
    defparam mem2.DATA_WIDTH_R = 18;
    defparam mem2.REGMODE = "NOREG";
    defparam mem2.CSDECODE_W = "0b000";
    defparam mem2.CSDECODE_R = "0b000";
    defparam mem2.GSR = "DISABLED";
    defparam mem2.RESETMODE = "SYNC";
    defparam mem2.ASYNC_RESET_RELEASE = "SYNC";
    defparam mem2.INIT_DATA = "STATIC";
    defparam mem2.INITVAL_00 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_01 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_02 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_03 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_04 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_05 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_06 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_07 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_08 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_09 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_0A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_0B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_0C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_0D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_0E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_0F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_10 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_11 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_12 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_13 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_14 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_15 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_16 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_17 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_18 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_19 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_1A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_1B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_1C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_1D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_1E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem2.INITVAL_1F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    PDPW8KC mem0 (.DI0(ev_wr_data[66]), .DI1(ev_wr_data[67]), .DI2(ev_wr_data[68]), 
            .DI3(ev_wr_data[69]), .DI4(ev_wr_data[70]), .DI5(ev_wr_data[71]), 
            .DI6(ev_wr_data[72]), .DI7(ev_wr_data[73]), .DI8(ev_wr_data[74]), 
            .DI9(ev_wr_data[75]), .DI10(ev_wr_data[76]), .DI11(ev_wr_data[77]), 
            .DI12(ev_wr_data[78]), .DI13(ev_wr_data[79]), .DI14(ev_wr_data[80]), 
            .DI15(ev_wr_data[81]), .DI16(ev_wr_data[82]), .DI17(ev_wr_data[83]), 
            .ADW0(rmw_slot[0]), .ADW1(rmw_slot[1]), .ADW2(rmw_slot[2]), 
            .ADW3(rmw_slot[3]), .ADW4(rmw_slot[4]), .ADW5(rmw_slot[5]), 
            .ADW6(rmw_slot[6]), .ADW7(rmw_slot[7]), .ADW8(active_bank_N_892), 
            .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), .CLKW(pll_clk), 
            .CSW0(GND_net), .CSW1(GND_net), .CSW2(GND_net), .ADR0(GND_net), 
            .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), .ADR4(event_rd_addr[0]), 
            .ADR5(event_rd_addr[1]), .ADR6(event_rd_addr[2]), .ADR7(event_rd_addr[3]), 
            .ADR8(event_rd_addr[4]), .ADR9(event_rd_addr[5]), .ADR10(event_rd_addr[6]), 
            .ADR11(event_rd_addr[7]), .ADR12(event_rd_addr[8]), .CER(VCC_net), 
            .OCER(VCC_net), .CLKR(pll_clk), .CSR0(GND_net), .CSR1(GND_net), 
            .CSR2(GND_net), .RST(GND_net), .DO0(ev_rd_data[75]), .DO1(ev_rd_data[76]), 
            .DO2(ev_rd_data[77]), .DO3(ev_rd_data[78]), .DO4(ev_rd_data[79]), 
            .DO5(ev_rd_data[80]), .DO6(ev_rd_data[81]), .DO7(ev_rd_data[82]), 
            .DO8(ev_rd_data[83]), .DO9(ev_rd_data[66]), .DO10(ev_rd_data[67]), 
            .DO11(ev_rd_data[68]), .DO12(ev_rd_data[69]), .DO13(ev_rd_data[70]), 
            .DO14(ev_rd_data[71]), .DO15(ev_rd_data[72]), .DO16(ev_rd_data[73]), 
            .DO17(ev_rd_data[74]));
    defparam mem0.DATA_WIDTH_W = 18;
    defparam mem0.DATA_WIDTH_R = 18;
    defparam mem0.REGMODE = "NOREG";
    defparam mem0.CSDECODE_W = "0b000";
    defparam mem0.CSDECODE_R = "0b000";
    defparam mem0.GSR = "DISABLED";
    defparam mem0.RESETMODE = "SYNC";
    defparam mem0.ASYNC_RESET_RELEASE = "SYNC";
    defparam mem0.INIT_DATA = "STATIC";
    defparam mem0.INITVAL_00 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_01 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_02 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_03 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_04 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_05 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_06 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_07 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_08 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_09 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_10 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_11 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_12 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_13 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_14 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_15 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_16 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_17 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_18 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_19 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    PDPW8KC mem4 (.DI0(ev_wr_data[0]), .DI1(ev_wr_data[1]), .DI2(ev_wr_data[2]), 
            .DI3(ev_wr_data[3]), .DI4(ev_wr_data[4]), .DI5(ev_wr_data[5]), 
            .DI6(ev_wr_data[6]), .DI7(ev_wr_data[7]), .DI8(ev_wr_data[8]), 
            .DI9(ev_wr_data[9]), .DI10(ev_wr_data[10]), .DI11(ev_wr_data[11]), 
            .DI12(GND_net), .DI13(GND_net), .DI14(GND_net), .DI15(GND_net), 
            .DI16(GND_net), .DI17(GND_net), .ADW0(rmw_slot[0]), .ADW1(rmw_slot[1]), 
            .ADW2(rmw_slot[2]), .ADW3(rmw_slot[3]), .ADW4(rmw_slot[4]), 
            .ADW5(rmw_slot[5]), .ADW6(rmw_slot[6]), .ADW7(rmw_slot[7]), 
            .ADW8(active_bank_N_892), .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), 
            .CLKW(pll_clk), .CSW0(GND_net), .CSW1(GND_net), .CSW2(GND_net), 
            .ADR0(GND_net), .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), 
            .ADR4(event_rd_addr[0]), .ADR5(event_rd_addr[1]), .ADR6(event_rd_addr[2]), 
            .ADR7(event_rd_addr[3]), .ADR8(event_rd_addr[4]), .ADR9(event_rd_addr[5]), 
            .ADR10(event_rd_addr[6]), .ADR11(event_rd_addr[7]), .ADR12(event_rd_addr[8]), 
            .CER(VCC_net), .OCER(VCC_net), .CLKR(pll_clk), .CSR0(GND_net), 
            .CSR1(GND_net), .CSR2(GND_net), .RST(GND_net), .DO0(ev_rd_data[9]), 
            .DO1(ev_rd_data[10]), .DO2(ev_rd_data[11]), .DO9(ev_rd_data[0]), 
            .DO10(ev_rd_data[1]), .DO11(ev_rd_data[2]), .DO12(ev_rd_data[3]), 
            .DO13(ev_rd_data[4]), .DO14(ev_rd_data[5]), .DO15(ev_rd_data[6]), 
            .DO16(ev_rd_data[7]), .DO17(ev_rd_data[8]));
    defparam mem4.DATA_WIDTH_W = 18;
    defparam mem4.DATA_WIDTH_R = 18;
    defparam mem4.REGMODE = "NOREG";
    defparam mem4.CSDECODE_W = "0b000";
    defparam mem4.CSDECODE_R = "0b000";
    defparam mem4.GSR = "DISABLED";
    defparam mem4.RESETMODE = "SYNC";
    defparam mem4.ASYNC_RESET_RELEASE = "SYNC";
    defparam mem4.INIT_DATA = "STATIC";
    defparam mem4.INITVAL_00 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_01 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_02 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_03 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_04 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_05 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_06 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_07 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_08 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_09 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_0A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_0B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_0C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_0D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_0E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_0F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_10 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_11 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_12 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_13 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_14 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_15 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_16 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_17 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_18 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_19 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_1A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_1B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_1C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_1D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_1E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem4.INITVAL_1F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    PDPW8KC mem3 (.DI0(ev_wr_data[12]), .DI1(ev_wr_data[13]), .DI2(ev_wr_data[14]), 
            .DI3(ev_wr_data[15]), .DI4(ev_wr_data[16]), .DI5(ev_wr_data[17]), 
            .DI6(ev_wr_data[18]), .DI7(ev_wr_data[19]), .DI8(ev_wr_data[20]), 
            .DI9(ev_wr_data[21]), .DI10(ev_wr_data[22]), .DI11(ev_wr_data[23]), 
            .DI12(ev_wr_data[24]), .DI13(ev_wr_data[25]), .DI14(ev_wr_data[26]), 
            .DI15(ev_wr_data[27]), .DI16(ev_wr_data[28]), .DI17(ev_wr_data[29]), 
            .ADW0(rmw_slot[0]), .ADW1(rmw_slot[1]), .ADW2(rmw_slot[2]), 
            .ADW3(rmw_slot[3]), .ADW4(rmw_slot[4]), .ADW5(rmw_slot[5]), 
            .ADW6(rmw_slot[6]), .ADW7(rmw_slot[7]), .ADW8(active_bank_N_892), 
            .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), .CLKW(pll_clk), 
            .CSW0(GND_net), .CSW1(GND_net), .CSW2(GND_net), .ADR0(GND_net), 
            .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), .ADR4(event_rd_addr[0]), 
            .ADR5(event_rd_addr[1]), .ADR6(event_rd_addr[2]), .ADR7(event_rd_addr[3]), 
            .ADR8(event_rd_addr[4]), .ADR9(event_rd_addr[5]), .ADR10(event_rd_addr[6]), 
            .ADR11(event_rd_addr[7]), .ADR12(event_rd_addr[8]), .CER(VCC_net), 
            .OCER(VCC_net), .CLKR(pll_clk), .CSR0(GND_net), .CSR1(GND_net), 
            .CSR2(GND_net), .RST(GND_net), .DO0(ev_rd_data[21]), .DO1(ev_rd_data[22]), 
            .DO2(ev_rd_data[23]), .DO3(ev_rd_data[24]), .DO4(ev_rd_data[25]), 
            .DO5(ev_rd_data[26]), .DO6(ev_rd_data[27]), .DO7(ev_rd_data[28]), 
            .DO8(ev_rd_data[29]), .DO9(ev_rd_data[12]), .DO10(ev_rd_data[13]), 
            .DO11(ev_rd_data[14]), .DO12(ev_rd_data[15]), .DO13(ev_rd_data[16]), 
            .DO14(ev_rd_data[17]), .DO15(ev_rd_data[18]), .DO16(ev_rd_data[19]), 
            .DO17(ev_rd_data[20]));
    defparam mem3.DATA_WIDTH_W = 18;
    defparam mem3.DATA_WIDTH_R = 18;
    defparam mem3.REGMODE = "NOREG";
    defparam mem3.CSDECODE_W = "0b000";
    defparam mem3.CSDECODE_R = "0b000";
    defparam mem3.GSR = "DISABLED";
    defparam mem3.RESETMODE = "SYNC";
    defparam mem3.ASYNC_RESET_RELEASE = "SYNC";
    defparam mem3.INIT_DATA = "STATIC";
    defparam mem3.INITVAL_00 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_01 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_02 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_03 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_04 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_05 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_06 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_07 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_08 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_09 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_0A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_0B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_0C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_0D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_0E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_0F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_10 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_11 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_12 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_13 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_14 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_15 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_16 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_17 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_18 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_19 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_1A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_1B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_1C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_1D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_1E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem3.INITVAL_1F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    
endmodule
//
// Verilog Description of module spi_mic_stream
//

module spi_mic_stream (sck_N_3124, spi_mic_cs_n_c, mic_latest, spi_mic_miso_c, 
            GND_net, n28627) /* synthesis syn_module_defined=1 */ ;
    input sck_N_3124;
    input spi_mic_cs_n_c;
    input [63:0]mic_latest;
    output spi_mic_miso_c;
    input GND_net;
    input n28627;
    
    wire sck_N_3124 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(10[12:16])
    
    wire n26636;
    wire [6:0]out_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(20[12:21])
    
    wire n27924, n28032;
    wire [95:0]hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(10[12:16])
    wire [6:0]bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[11:20])
    
    wire n28028, n28027;
    wire [6:0]n20;
    
    wire sck_N_3124_enable_72, n17790, n17788, n17786, n17784, n17782, 
        n17780, n28031, n28030, n27922, n27926, n28019, n28018, 
        n27925, sck_N_3124_enable_8;
    wire [6:0]bit_count_6__N_3125;
    
    wire n27932, n27931, n28022, n28021, n28029, n27492, n27493, 
        n26622, n26629, n27491, n28023, n27950, n27949, n13, n26699, 
        n27494, n28020, n27495, n28012, n28013, n28014, n28000, 
        n28001, n28002, n27997, n27998, n27999, n27994, n27995, 
        n27996, n27965, n27964, n25417, n26615, n27490, n27966, 
        n27933, n27979, n27980, n27981, n27976, n27977, n27978, 
        n26643, n25780, n12, n27974, n27973, n24512, n24511, n27975, 
        n24510;
    wire [6:0]bit_count_6__N_3133;
    
    wire n78, n27951, n27927, n27295, n27289, n27923, n27286, 
        n27292;
    
    MUX21 i1563281_i1 (.D0(n27924), .D1(n28032), .SD(out_index[2]), .Z(n26636));
    LUT4 i14763_then_3_lut (.A(hold[10]), .B(bit_count[1]), .C(hold[8]), 
         .Z(n28028)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14763_then_3_lut.init = 16'he2e2;
    LUT4 i14763_else_3_lut (.A(hold[11]), .B(bit_count[1]), .C(hold[9]), 
         .Z(n28027)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14763_else_3_lut.init = 16'he2e2;
    LUT4 i10269_2_lut (.A(n20[6]), .B(sck_N_3124_enable_72), .Z(n17790)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10269_2_lut.init = 16'h2222;
    LUT4 i10268_2_lut (.A(n20[5]), .B(sck_N_3124_enable_72), .Z(n17788)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10268_2_lut.init = 16'h2222;
    LUT4 i10267_2_lut (.A(n20[4]), .B(sck_N_3124_enable_72), .Z(n17786)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10267_2_lut.init = 16'h2222;
    LUT4 i10266_2_lut (.A(n20[3]), .B(sck_N_3124_enable_72), .Z(n17784)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10266_2_lut.init = 16'h2222;
    LUT4 i10265_2_lut (.A(n20[2]), .B(sck_N_3124_enable_72), .Z(n17782)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10265_2_lut.init = 16'h2222;
    LUT4 i10263_2_lut (.A(n20[1]), .B(sck_N_3124_enable_72), .Z(n17780)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10263_2_lut.init = 16'h2222;
    LUT4 n27064_bdd_4_lut_then_3_lut (.A(hold[76]), .B(bit_count[1]), .C(hold[78]), 
         .Z(n28031)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27064_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27064_bdd_4_lut_else_3_lut (.A(hold[77]), .B(bit_count[1]), .C(hold[79]), 
         .Z(n28030)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27064_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 n27238_bdd_4_lut_else_3_lut (.A(hold[73]), .B(bit_count[1]), .C(hold[75]), 
         .Z(n27922)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27238_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 n27412_bdd_4_lut_then_3_lut (.A(hold[36]), .B(bit_count[1]), .C(hold[38]), 
         .Z(n27926)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27412_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 i14757_then_3_lut (.A(hold[2]), .B(bit_count[1]), .C(hold[0]), 
         .Z(n28019)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14757_then_3_lut.init = 16'he2e2;
    LUT4 i14757_else_3_lut (.A(hold[3]), .B(bit_count[1]), .C(hold[1]), 
         .Z(n28018)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14757_else_3_lut.init = 16'he2e2;
    FD1P3DX hold__i1 (.D(mic_latest[0]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i1.GSR = "DISABLED";
    LUT4 n27412_bdd_4_lut_else_3_lut (.A(hold[37]), .B(bit_count[1]), .C(hold[39]), 
         .Z(n27925)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27412_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    FD1P3DX bit_count_i0 (.D(bit_count_6__N_3125[0]), .SP(sck_N_3124_enable_8), 
            .CK(sck_N_3124), .CD(spi_mic_cs_n_c), .Q(bit_count[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i0.GSR = "DISABLED";
    PFUMX i16466 (.BLUT(n28030), .ALUT(n28031), .C0(bit_count[0]), .Z(n28032));
    LUT4 n27346_bdd_4_lut_then_3_lut (.A(hold[60]), .B(bit_count[1]), .C(hold[62]), 
         .Z(n27932)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27346_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27346_bdd_4_lut_else_3_lut (.A(hold[61]), .B(bit_count[1]), .C(hold[63]), 
         .Z(n27931)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27346_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 i14760_then_3_lut (.A(hold[6]), .B(bit_count[1]), .C(hold[4]), 
         .Z(n28022)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14760_then_3_lut.init = 16'he2e2;
    LUT4 i14760_else_3_lut (.A(hold[7]), .B(bit_count[1]), .C(hold[5]), 
         .Z(n28021)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14760_else_3_lut.init = 16'he2e2;
    PFUMX i16464 (.BLUT(n28027), .ALUT(n28028), .C0(bit_count[0]), .Z(n28029));
    LUT4 n27492_bdd_2_lut (.A(n27492), .B(bit_count[6]), .Z(n27493)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam n27492_bdd_2_lut.init = 16'h2222;
    FD1P3DX bit_count_i6 (.D(n17790), .SP(sck_N_3124_enable_8), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i6.GSR = "DISABLED";
    FD1P3DX bit_count_i5 (.D(n17788), .SP(sck_N_3124_enable_8), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i5.GSR = "DISABLED";
    FD1P3DX bit_count_i4 (.D(n17786), .SP(sck_N_3124_enable_8), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i4.GSR = "DISABLED";
    FD1P3DX bit_count_i3 (.D(n17784), .SP(sck_N_3124_enable_8), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i3.GSR = "DISABLED";
    FD1P3DX bit_count_i2 (.D(n17782), .SP(sck_N_3124_enable_8), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i2.GSR = "DISABLED";
    FD1P3DX bit_count_i1 (.D(n17780), .SP(sck_N_3124_enable_8), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i1.GSR = "DISABLED";
    LUT4 n26622_bdd_3_lut_16250 (.A(n26622), .B(n26629), .C(bit_count[3]), 
         .Z(n27491)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n26622_bdd_3_lut_16250.init = 16'hacac;
    PFUMX i16460 (.BLUT(n28021), .ALUT(n28022), .C0(bit_count[0]), .Z(n28023));
    LUT4 i14769_then_3_lut (.A(hold[26]), .B(bit_count[1]), .C(hold[24]), 
         .Z(n27950)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14769_then_3_lut.init = 16'he2e2;
    LUT4 i14769_else_3_lut (.A(hold[27]), .B(bit_count[1]), .C(hold[25]), 
         .Z(n27949)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14769_else_3_lut.init = 16'he2e2;
    LUT4 bit_count_5__bdd_3_lut_16356 (.A(bit_count[6]), .B(n13), .C(n26699), 
         .Z(n27494)) /* synthesis lut_function=(A (C)+!A (B)) */ ;
    defparam bit_count_5__bdd_3_lut_16356.init = 16'he4e4;
    PFUMX i16458 (.BLUT(n28018), .ALUT(n28019), .C0(bit_count[0]), .Z(n28020));
    LUT4 n17056_bdd_3_lut_16359 (.A(sck_N_3124_enable_72), .B(spi_mic_cs_n_c), 
         .C(n27495), .Z(spi_mic_miso_c)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;
    defparam n17056_bdd_3_lut_16359.init = 16'h1010;
    PFUMX i16454 (.BLUT(n28012), .ALUT(n28013), .C0(bit_count[0]), .Z(n28014));
    PFUMX i16446 (.BLUT(n28000), .ALUT(n28001), .C0(bit_count[0]), .Z(n28002));
    PFUMX i16444 (.BLUT(n27997), .ALUT(n27998), .C0(bit_count[0]), .Z(n27999));
    PFUMX i16442 (.BLUT(n27994), .ALUT(n27995), .C0(bit_count[0]), .Z(n27996));
    LUT4 n27298_bdd_4_lut_then_3_lut (.A(hold[56]), .B(bit_count[1]), .C(hold[58]), 
         .Z(n27965)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27298_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27298_bdd_4_lut_else_3_lut (.A(hold[57]), .B(bit_count[1]), .C(hold[59]), 
         .Z(n27964)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27298_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 i2_3_lut (.A(bit_count[2]), .B(hold[64]), .C(bit_count[1]), .Z(n25417)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(24[49:58])
    defparam i2_3_lut.init = 16'h8080;
    LUT4 n26622_bdd_4_lut_16249_4_lut (.A(bit_count[0]), .B(n25417), .C(n26615), 
         .D(bit_count[3]), .Z(n27490)) /* synthesis lut_function=(A (C (D))+!A (B (C+!(D))+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(20[24:41])
    defparam n26622_bdd_4_lut_16249_4_lut.init = 16'hf044;
    MUX21 i1562678_i1 (.D0(n27966), .D1(n27933), .SD(out_index[2]), .Z(n26629));
    PFUMX i16432 (.BLUT(n27979), .ALUT(n27980), .C0(bit_count[0]), .Z(n27981));
    PFUMX i16430 (.BLUT(n27976), .ALUT(n27977), .C0(bit_count[0]), .Z(n27978));
    LUT4 i14947_2_lut (.A(n26643), .B(bit_count[3]), .Z(n25780)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[24:36])
    defparam i14947_2_lut.init = 16'h8888;
    LUT4 i15177_4_lut (.A(bit_count[2]), .B(n12), .C(bit_count[6]), .D(bit_count[1]), 
         .Z(sck_N_3124_enable_72)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(23[15:34])
    defparam i15177_4_lut.init = 16'h0001;
    LUT4 i5_4_lut (.A(bit_count[0]), .B(bit_count[5]), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n12)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(23[15:34])
    defparam i5_4_lut.init = 16'hfffe;
    FD1P3DX hold__i2 (.D(mic_latest[1]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i2.GSR = "DISABLED";
    LUT4 n27280_bdd_4_lut_then_3_lut (.A(hold[84]), .B(bit_count[1]), .C(hold[86]), 
         .Z(n27974)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27280_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27280_bdd_4_lut_else_3_lut (.A(hold[85]), .B(bit_count[1]), .C(hold[87]), 
         .Z(n27973)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27280_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    CCU2D add_13_7 (.A0(bit_count[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24512), .S0(n20[5]), .S1(n20[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(34[22:38])
    defparam add_13_7.INIT0 = 16'h5aaa;
    defparam add_13_7.INIT1 = 16'h5aaa;
    defparam add_13_7.INJECT1_0 = "NO";
    defparam add_13_7.INJECT1_1 = "NO";
    CCU2D add_13_5 (.A0(bit_count[3]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24511), .COUT(n24512), .S0(n20[3]), .S1(n20[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(34[22:38])
    defparam add_13_5.INIT0 = 16'h5aaa;
    defparam add_13_5.INIT1 = 16'h5aaa;
    defparam add_13_5.INJECT1_0 = "NO";
    defparam add_13_5.INJECT1_1 = "NO";
    PFUMX i16428 (.BLUT(n27973), .ALUT(n27974), .C0(bit_count[0]), .Z(n27975));
    LUT4 n27274_bdd_4_lut_then_3_lut (.A(hold[80]), .B(bit_count[1]), .C(hold[82]), 
         .Z(n27977)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27274_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    CCU2D add_13_3 (.A0(bit_count[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24510), .COUT(n24511), .S0(n20[1]), .S1(n20[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(34[22:38])
    defparam add_13_3.INIT0 = 16'h5aaa;
    defparam add_13_3.INIT1 = 16'h5aaa;
    defparam add_13_3.INJECT1_0 = "NO";
    defparam add_13_3.INJECT1_1 = "NO";
    CCU2D add_13_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24510), .S1(bit_count_6__N_3133[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(34[22:38])
    defparam add_13_1.INIT0 = 16'hF000;
    defparam add_13_1.INIT1 = 16'h5555;
    defparam add_13_1.INJECT1_0 = "NO";
    defparam add_13_1.INJECT1_1 = "NO";
    LUT4 n27274_bdd_4_lut_else_3_lut (.A(hold[81]), .B(bit_count[1]), .C(hold[83]), 
         .Z(n27976)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27274_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    FD1P3DX hold__i3 (.D(mic_latest[2]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i3.GSR = "DISABLED";
    FD1P3DX hold__i4 (.D(mic_latest[3]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i4.GSR = "DISABLED";
    FD1P3DX hold__i5 (.D(mic_latest[4]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i5.GSR = "DISABLED";
    FD1P3DX hold__i6 (.D(mic_latest[5]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i6.GSR = "DISABLED";
    FD1P3DX hold__i7 (.D(mic_latest[6]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i7.GSR = "DISABLED";
    FD1P3DX hold__i8 (.D(mic_latest[7]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i8.GSR = "DISABLED";
    FD1P3DX hold__i9 (.D(mic_latest[8]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i9.GSR = "DISABLED";
    FD1P3DX hold__i10 (.D(mic_latest[9]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i10.GSR = "DISABLED";
    FD1P3DX hold__i11 (.D(mic_latest[10]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i11.GSR = "DISABLED";
    FD1P3DX hold__i12 (.D(mic_latest[11]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i12.GSR = "DISABLED";
    FD1P3DX hold__i13 (.D(mic_latest[12]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i13.GSR = "DISABLED";
    FD1P3DX hold__i14 (.D(mic_latest[13]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i14.GSR = "DISABLED";
    FD1P3DX hold__i15 (.D(mic_latest[14]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i15.GSR = "DISABLED";
    FD1P3DX hold__i16 (.D(mic_latest[15]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i16.GSR = "DISABLED";
    FD1P3DX hold__i17 (.D(mic_latest[16]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[24])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i17.GSR = "DISABLED";
    FD1P3DX hold__i18 (.D(mic_latest[17]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[25])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i18.GSR = "DISABLED";
    FD1P3DX hold__i19 (.D(mic_latest[18]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[26])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i19.GSR = "DISABLED";
    FD1P3DX hold__i20 (.D(mic_latest[19]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[27])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i20.GSR = "DISABLED";
    FD1P3DX hold__i21 (.D(mic_latest[20]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[28])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i21.GSR = "DISABLED";
    FD1P3DX hold__i22 (.D(mic_latest[21]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[29])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i22.GSR = "DISABLED";
    FD1P3DX hold__i23 (.D(mic_latest[22]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[30])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i23.GSR = "DISABLED";
    FD1P3DX hold__i24 (.D(mic_latest[23]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[31])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i24.GSR = "DISABLED";
    FD1P3DX hold__i25 (.D(mic_latest[24]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[32])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i25.GSR = "DISABLED";
    FD1P3DX hold__i26 (.D(mic_latest[25]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[33])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i26.GSR = "DISABLED";
    FD1P3DX hold__i27 (.D(mic_latest[26]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[34])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i27.GSR = "DISABLED";
    FD1P3DX hold__i28 (.D(mic_latest[27]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[35])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i28.GSR = "DISABLED";
    FD1P3DX hold__i29 (.D(mic_latest[28]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[36])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i29.GSR = "DISABLED";
    FD1P3DX hold__i30 (.D(mic_latest[29]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[37])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i30.GSR = "DISABLED";
    FD1P3DX hold__i31 (.D(mic_latest[30]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[38])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i31.GSR = "DISABLED";
    FD1P3DX hold__i32 (.D(mic_latest[31]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[39])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i32.GSR = "DISABLED";
    FD1P3DX hold__i33 (.D(mic_latest[32]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[48])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i33.GSR = "DISABLED";
    FD1P3DX hold__i34 (.D(mic_latest[33]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[49])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i34.GSR = "DISABLED";
    FD1P3DX hold__i35 (.D(mic_latest[34]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[50])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i35.GSR = "DISABLED";
    FD1P3DX hold__i36 (.D(mic_latest[35]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[51])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i36.GSR = "DISABLED";
    FD1P3DX hold__i37 (.D(mic_latest[36]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[52])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i37.GSR = "DISABLED";
    FD1P3DX hold__i38 (.D(mic_latest[37]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[53])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i38.GSR = "DISABLED";
    FD1P3DX hold__i39 (.D(mic_latest[38]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[54])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i39.GSR = "DISABLED";
    FD1P3DX hold__i40 (.D(mic_latest[39]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[55])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i40.GSR = "DISABLED";
    FD1P3DX hold__i41 (.D(mic_latest[40]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[56])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i41.GSR = "DISABLED";
    FD1P3DX hold__i42 (.D(mic_latest[41]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[57])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i42.GSR = "DISABLED";
    FD1P3DX hold__i43 (.D(mic_latest[42]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[58])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i43.GSR = "DISABLED";
    FD1P3DX hold__i44 (.D(mic_latest[43]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[59])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i44.GSR = "DISABLED";
    FD1P3DX hold__i45 (.D(mic_latest[44]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[60])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i45.GSR = "DISABLED";
    FD1P3DX hold__i46 (.D(mic_latest[45]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[61])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i46.GSR = "DISABLED";
    FD1P3DX hold__i47 (.D(mic_latest[46]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[62])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i47.GSR = "DISABLED";
    FD1P3DX hold__i48 (.D(mic_latest[47]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[63])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i48.GSR = "DISABLED";
    FD1P3DX hold__i49 (.D(n28627), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[64])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i49.GSR = "DISABLED";
    FD1P3DX hold__i50 (.D(mic_latest[48]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[72])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i50.GSR = "DISABLED";
    FD1P3DX hold__i51 (.D(mic_latest[49]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[73])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i51.GSR = "DISABLED";
    FD1P3DX hold__i52 (.D(mic_latest[50]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[74])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i52.GSR = "DISABLED";
    FD1P3DX hold__i53 (.D(mic_latest[51]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[75])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i53.GSR = "DISABLED";
    FD1P3DX hold__i54 (.D(mic_latest[52]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[76])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i54.GSR = "DISABLED";
    FD1P3DX hold__i55 (.D(mic_latest[53]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[77])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i55.GSR = "DISABLED";
    FD1P3DX hold__i56 (.D(mic_latest[54]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[78])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i56.GSR = "DISABLED";
    FD1P3DX hold__i57 (.D(mic_latest[55]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[79])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i57.GSR = "DISABLED";
    FD1P3DX hold__i58 (.D(mic_latest[56]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[80])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i58.GSR = "DISABLED";
    FD1P3DX hold__i59 (.D(mic_latest[57]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[81])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i59.GSR = "DISABLED";
    FD1P3DX hold__i60 (.D(mic_latest[58]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[82])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i60.GSR = "DISABLED";
    FD1P3DX hold__i61 (.D(mic_latest[59]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[83])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i61.GSR = "DISABLED";
    FD1P3DX hold__i62 (.D(mic_latest[60]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[84])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i62.GSR = "DISABLED";
    FD1P3DX hold__i63 (.D(mic_latest[61]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[85])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i63.GSR = "DISABLED";
    FD1P3DX hold__i64 (.D(mic_latest[62]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[86])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i64.GSR = "DISABLED";
    FD1P3DX hold__i65 (.D(mic_latest[63]), .SP(sck_N_3124_enable_72), .CK(sck_N_3124), 
            .CD(spi_mic_cs_n_c), .Q(hold[87])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1028, LSE_RLINE=1031 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i65.GSR = "DISABLED";
    MUX21 i1562075_i1 (.D0(n27999), .D1(n27996), .SD(out_index[2]), .Z(n26622));
    LUT4 i15166_3_lut (.A(bit_count[5]), .B(sck_N_3124_enable_72), .C(bit_count[6]), 
         .Z(sck_N_3124_enable_8)) /* synthesis lut_function=((B+!(C))+!A) */ ;
    defparam i15166_3_lut.init = 16'hdfdf;
    PFUMX i35 (.BLUT(n25780), .ALUT(n78), .C0(bit_count[4]), .Z(n13));
    LUT4 i9863_2_lut (.A(bit_count_6__N_3133[0]), .B(sck_N_3124_enable_72), 
         .Z(bit_count_6__N_3125[0])) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(33[14] 35[8])
    defparam i9863_2_lut.init = 16'heeee;
    LUT4 bit_count_6__I_0_22_inv_0_i3_1_lut (.A(bit_count[2]), .Z(out_index[2])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(20[24:41])
    defparam bit_count_6__I_0_22_inv_0_i3_1_lut.init = 16'h5555;
    LUT4 i14766_then_3_lut (.A(hold[14]), .B(bit_count[1]), .C(hold[12]), 
         .Z(n27980)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14766_then_3_lut.init = 16'he2e2;
    PFUMX i16422 (.BLUT(n27964), .ALUT(n27965), .C0(bit_count[0]), .Z(n27966));
    LUT4 i14766_else_3_lut (.A(hold[15]), .B(bit_count[1]), .C(hold[13]), 
         .Z(n27979)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14766_else_3_lut.init = 16'he2e2;
    LUT4 out_index_6__I_0_i78_4_lut_4_lut (.A(bit_count[3]), .B(n25417), 
         .C(n26636), .D(bit_count[0]), .Z(n78)) /* synthesis lut_function=(A (B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam out_index_6__I_0_i78_4_lut_4_lut.init = 16'hd850;
    PFUMX i16150 (.BLUT(n27494), .ALUT(n27493), .C0(bit_count[5]), .Z(n27495));
    MUX21 i1563884_i1 (.D0(n27978), .D1(n27975), .SD(out_index[2]), .Z(n26643));
    PFUMX i16412 (.BLUT(n27949), .ALUT(n27950), .C0(bit_count[0]), .Z(n27951));
    LUT4 n27142_bdd_4_lut_then_3_lut (.A(hold[52]), .B(bit_count[1]), .C(hold[54]), 
         .Z(n27995)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27142_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27142_bdd_4_lut_else_3_lut (.A(hold[53]), .B(bit_count[1]), .C(hold[55]), 
         .Z(n27994)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27142_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 n27136_bdd_4_lut_then_3_lut (.A(hold[48]), .B(bit_count[1]), .C(hold[50]), 
         .Z(n27998)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27136_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27136_bdd_4_lut_else_3_lut (.A(hold[49]), .B(bit_count[1]), .C(hold[51]), 
         .Z(n27997)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27136_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    MUX21 i1561472_i1 (.D0(n28014), .D1(n27927), .SD(out_index[2]), .Z(n26615));
    LUT4 i14772_then_3_lut (.A(hold[30]), .B(bit_count[1]), .C(hold[28]), 
         .Z(n28001)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14772_then_3_lut.init = 16'he2e2;
    LUT4 i14772_else_3_lut (.A(hold[31]), .B(bit_count[1]), .C(hold[29]), 
         .Z(n28000)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i14772_else_3_lut.init = 16'he2e2;
    MUX21 i15692111_i1 (.D0(n27295), .D1(n27289), .SD(out_index[2]), .Z(n26699));
    LUT4 n27238_bdd_4_lut_then_3_lut (.A(hold[72]), .B(bit_count[1]), .C(hold[74]), 
         .Z(n27923)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27238_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    PFUMX i16148 (.BLUT(n27491), .ALUT(n27490), .C0(bit_count[4]), .Z(n27492));
    LUT4 n27286_bdd_4_lut (.A(n27286), .B(n27981), .C(n28002), .D(bit_count[3]), 
         .Z(n27289)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27286_bdd_4_lut.init = 16'haad8;
    LUT4 n27112_bdd_4_lut_then_3_lut (.A(hold[32]), .B(bit_count[1]), .C(hold[34]), 
         .Z(n28013)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27112_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 bit_count_4__bdd_4_lut_16013 (.A(bit_count[4]), .B(n25417), .C(n28023), 
         .D(bit_count[3]), .Z(n27286)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam bit_count_4__bdd_4_lut_16013.init = 16'he4aa;
    LUT4 n27112_bdd_4_lut_else_3_lut (.A(hold[33]), .B(bit_count[1]), .C(hold[35]), 
         .Z(n28012)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27112_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    PFUMX i16400 (.BLUT(n27931), .ALUT(n27932), .C0(bit_count[0]), .Z(n27933));
    LUT4 n27292_bdd_4_lut (.A(n27292), .B(n28029), .C(n27951), .D(bit_count[3]), 
         .Z(n27295)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27292_bdd_4_lut.init = 16'haad8;
    PFUMX i16396 (.BLUT(n27925), .ALUT(n27926), .C0(bit_count[0]), .Z(n27927));
    LUT4 bit_count_4__bdd_4_lut_16743 (.A(bit_count[4]), .B(n25417), .C(n28020), 
         .D(bit_count[3]), .Z(n27292)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam bit_count_4__bdd_4_lut_16743.init = 16'he4aa;
    PFUMX i16394 (.BLUT(n27922), .ALUT(n27923), .C0(bit_count[0]), .Z(n27924));
    
endmodule
//
// Verilog Description of module TSALL
// module not written out since it is a black-box. 
//

//
// Verilog Description of module PUR
// module not written out since it is a black-box. 
//

//
// Verilog Description of module ws2812_stream
//

module ws2812_stream (reset_count, pll_clk, pll_clk_enable_138, ws2812_enable, 
            GND_net, rgb_hold, n10912, ws2812_frame_done, rgb_data_c, 
            n13359, n24165, n13623) /* synthesis syn_module_defined=1 */ ;
    output [14:0]reset_count;
    input pll_clk;
    input pll_clk_enable_138;
    input ws2812_enable;
    input GND_net;
    input [23:0]rgb_hold;
    output n10912;
    output ws2812_frame_done;
    output rgb_data_c;
    output n13359;
    input n24165;
    output n13623;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(108[10:17])
    
    wire enable_N_2961, n10874, n13;
    wire [3:0]n10212;
    
    wire pll_clk_enable_260, n17735, pll_clk_enable_561, n24419;
    wire [6:0]bit_cell_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[15:29])
    wire [6:0]n33;
    
    wire n15003, n24418, n24417;
    wire [4:0]bit_number;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(21[15:25])
    wire [4:0]n25;
    
    wire n24215, n27884, pll_clk_enable_265, data_out_N_3021, data_out_N_3014;
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(22[16:30])
    
    wire n10866, n10865, n10864, n10863, n10862, n10861, n10860, 
        n10859, n10858, n10857, n10856, n10855, n10854, n16913, 
        n10853, n10852, n10851, n10850, n10849, n10848;
    wire [14:0]reset_count_c;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[16:27])
    
    wire n10875, n10876, n10877, n10878, n10879, n10880, n10881, 
        n10882, n10883, n10884, n10885, n10886, n10887, n10888, 
        n10847, n21, n27725, n10846, n10845, n10844, n29_adj_3144, 
        n27726, pll_clk_enable_261, n17643, n17692, n15952, n27727, 
        n24729, n16, n20, n16_adj_3145, n18, n10228, n14999, n24508, 
        n11005, n24507, n24506, n24505, n24504, n24503, n24502, 
        n10873, n4, n27883, n25684, n26_adj_3146, n25670, n25654, 
        n24;
    
    FD1P3IX reset_count__i0 (.D(n10874), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i0.GSR = "DISABLED";
    LUT4 i15217_4_lut_3_lut (.A(n13), .B(n10212[2]), .C(pll_clk_enable_260), 
         .Z(n17735)) /* synthesis lut_function=(!(A (B+!(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i15217_4_lut_3_lut.init = 16'h7070;
    LUT4 i2200_2_lut_4_lut (.A(n10212[1]), .B(n13), .C(n10212[2]), .D(ws2812_enable), 
         .Z(pll_clk_enable_561)) /* synthesis lut_function=(!(A (B (C+!(D))+!B !(D))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i2200_2_lut_4_lut.init = 16'h3a00;
    CCU2D bit_cell_count_3060_add_4_7 (.A0(bit_cell_count[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(bit_cell_count[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24419), .S0(n33[5]), .S1(n33[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060_add_4_7.INIT0 = 16'hfaaa;
    defparam bit_cell_count_3060_add_4_7.INIT1 = 16'hfaaa;
    defparam bit_cell_count_3060_add_4_7.INJECT1_0 = "NO";
    defparam bit_cell_count_3060_add_4_7.INJECT1_1 = "NO";
    FD1S3JX state_FSM_i1 (.D(n15003), .CK(pll_clk), .PD(enable_N_2961), 
            .Q(n10212[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam state_FSM_i1.GSR = "DISABLED";
    CCU2D bit_cell_count_3060_add_4_5 (.A0(bit_cell_count[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(bit_cell_count[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24418), .COUT(n24419), .S0(n33[3]), 
          .S1(n33[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060_add_4_5.INIT0 = 16'hfaaa;
    defparam bit_cell_count_3060_add_4_5.INIT1 = 16'hfaaa;
    defparam bit_cell_count_3060_add_4_5.INJECT1_0 = "NO";
    defparam bit_cell_count_3060_add_4_5.INJECT1_1 = "NO";
    LUT4 i1_3_lut (.A(ws2812_enable), .B(n10212[2]), .C(n10212[1]), .Z(pll_clk_enable_260)) /* synthesis lut_function=(A (B+(C))) */ ;
    defparam i1_3_lut.init = 16'ha8a8;
    CCU2D bit_cell_count_3060_add_4_3 (.A0(bit_cell_count[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(bit_cell_count[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24417), .COUT(n24418), .S0(n33[1]), 
          .S1(n33[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060_add_4_3.INIT0 = 16'hfaaa;
    defparam bit_cell_count_3060_add_4_3.INIT1 = 16'hfaaa;
    defparam bit_cell_count_3060_add_4_3.INJECT1_0 = "NO";
    defparam bit_cell_count_3060_add_4_3.INJECT1_1 = "NO";
    LUT4 i13497_2_lut_3_lut (.A(bit_number[1]), .B(bit_number[0]), .C(bit_number[2]), 
         .Z(n25[2])) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13497_2_lut_3_lut.init = 16'h7878;
    LUT4 i13511_3_lut (.A(bit_number[4]), .B(bit_number[3]), .C(n24215), 
         .Z(n25[4])) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13511_3_lut.init = 16'h6a6a;
    CCU2D bit_cell_count_3060_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24417), .S1(n33[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060_add_4_1.INIT0 = 16'hF000;
    defparam bit_cell_count_3060_add_4_1.INIT1 = 16'h0555;
    defparam bit_cell_count_3060_add_4_1.INJECT1_0 = "NO";
    defparam bit_cell_count_3060_add_4_1.INJECT1_1 = "NO";
    LUT4 i13490_2_lut (.A(bit_number[1]), .B(bit_number[0]), .Z(n25[1])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13490_2_lut.init = 16'h6666;
    LUT4 i13488_1_lut (.A(bit_number[0]), .Z(n25[0])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13488_1_lut.init = 16'h5555;
    LUT4 i15247_4_lut (.A(n27884), .B(n13), .C(n10212[2]), .D(ws2812_enable), 
         .Z(pll_clk_enable_265)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i15247_4_lut.init = 16'h7fff;
    LUT4 i1747_4_lut (.A(n10212[1]), .B(data_out_N_3021), .C(n10212[2]), 
         .D(n13), .Z(data_out_N_3014)) /* synthesis lut_function=(!(A (B (C)+!B (C (D)))+!A (B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i1747_4_lut.init = 16'h0a3a;
    LUT4 mux_2150_i24_3_lut_4_lut (.A(rgb_hold[23]), .B(shift_register[22]), 
         .C(n10212[2]), .D(n13), .Z(n10866)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i24_3_lut_4_lut.init = 16'haaca;
    FD1P3IX bit_cell_count_3060__i6 (.D(n33[6]), .SP(pll_clk_enable_260), 
            .CD(n17735), .CK(pll_clk), .Q(bit_cell_count[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060__i6.GSR = "DISABLED";
    FD1P3IX bit_cell_count_3060__i5 (.D(n33[5]), .SP(pll_clk_enable_260), 
            .CD(n17735), .CK(pll_clk), .Q(bit_cell_count[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060__i5.GSR = "DISABLED";
    LUT4 mux_2150_i23_3_lut_4_lut (.A(rgb_hold[22]), .B(shift_register[21]), 
         .C(n10212[2]), .D(n13), .Z(n10865)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i23_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i22_3_lut_4_lut (.A(rgb_hold[21]), .B(shift_register[20]), 
         .C(n10212[2]), .D(n13), .Z(n10864)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i22_3_lut_4_lut.init = 16'haaca;
    FD1P3IX bit_cell_count_3060__i4 (.D(n33[4]), .SP(pll_clk_enable_260), 
            .CD(n17735), .CK(pll_clk), .Q(bit_cell_count[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060__i4.GSR = "DISABLED";
    LUT4 mux_2150_i21_3_lut_4_lut (.A(rgb_hold[20]), .B(shift_register[19]), 
         .C(n10212[2]), .D(n13), .Z(n10863)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i21_3_lut_4_lut.init = 16'haaca;
    FD1P3IX bit_cell_count_3060__i3 (.D(n33[3]), .SP(pll_clk_enable_260), 
            .CD(n17735), .CK(pll_clk), .Q(bit_cell_count[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060__i3.GSR = "DISABLED";
    LUT4 mux_2150_i20_3_lut_4_lut (.A(rgb_hold[19]), .B(shift_register[18]), 
         .C(n10212[2]), .D(n13), .Z(n10862)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i20_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i19_3_lut_4_lut (.A(rgb_hold[18]), .B(shift_register[17]), 
         .C(n10212[2]), .D(n13), .Z(n10861)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i19_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i18_3_lut_4_lut (.A(rgb_hold[17]), .B(shift_register[16]), 
         .C(n10212[2]), .D(n13), .Z(n10860)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i18_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i17_3_lut_4_lut (.A(rgb_hold[16]), .B(shift_register[15]), 
         .C(n10212[2]), .D(n13), .Z(n10859)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i17_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i16_3_lut_4_lut (.A(rgb_hold[15]), .B(shift_register[14]), 
         .C(n10212[2]), .D(n13), .Z(n10858)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i16_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i15_3_lut_4_lut (.A(rgb_hold[14]), .B(shift_register[13]), 
         .C(n10212[2]), .D(n13), .Z(n10857)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i15_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i14_3_lut_4_lut (.A(rgb_hold[13]), .B(shift_register[12]), 
         .C(n10212[2]), .D(n13), .Z(n10856)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i14_3_lut_4_lut.init = 16'haaca;
    FD1P3IX bit_cell_count_3060__i2 (.D(n33[2]), .SP(pll_clk_enable_260), 
            .CD(n17735), .CK(pll_clk), .Q(bit_cell_count[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060__i2.GSR = "DISABLED";
    FD1P3IX bit_cell_count_3060__i1 (.D(n33[1]), .SP(pll_clk_enable_260), 
            .CD(n17735), .CK(pll_clk), .Q(bit_cell_count[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060__i1.GSR = "DISABLED";
    LUT4 mux_2150_i13_3_lut_4_lut (.A(rgb_hold[12]), .B(shift_register[11]), 
         .C(n10212[2]), .D(n13), .Z(n10855)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i13_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i12_3_lut_4_lut (.A(rgb_hold[11]), .B(shift_register[10]), 
         .C(n10212[2]), .D(n13), .Z(n10854)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i12_3_lut_4_lut.init = 16'haaca;
    LUT4 i2_3_lut_4_lut (.A(bit_cell_count[1]), .B(bit_cell_count[0]), .C(bit_cell_count[2]), 
         .D(bit_cell_count[4]), .Z(n16913)) /* synthesis lut_function=((((D)+!C)+!B)+!A) */ ;
    defparam i2_3_lut_4_lut.init = 16'hff7f;
    LUT4 mux_2150_i11_3_lut_4_lut (.A(rgb_hold[10]), .B(shift_register[9]), 
         .C(n10212[2]), .D(n13), .Z(n10853)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i11_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i10_3_lut_4_lut (.A(rgb_hold[9]), .B(shift_register[8]), 
         .C(n10212[2]), .D(n13), .Z(n10852)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i10_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i9_3_lut_4_lut (.A(rgb_hold[8]), .B(shift_register[7]), 
         .C(n10212[2]), .D(n13), .Z(n10851)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i9_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i8_3_lut_4_lut (.A(rgb_hold[7]), .B(shift_register[6]), 
         .C(n10212[2]), .D(n13), .Z(n10850)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i8_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i7_3_lut_4_lut (.A(rgb_hold[6]), .B(shift_register[5]), 
         .C(n10212[2]), .D(n13), .Z(n10849)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i7_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i6_3_lut_4_lut (.A(rgb_hold[5]), .B(shift_register[4]), 
         .C(n10212[2]), .D(n13), .Z(n10848)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i6_3_lut_4_lut.init = 16'haaca;
    FD1P3IX reset_count__i1 (.D(n10875), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i1.GSR = "DISABLED";
    FD1P3IX reset_count__i2 (.D(n10876), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i2.GSR = "DISABLED";
    FD1P3IX reset_count__i3 (.D(n10877), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i3.GSR = "DISABLED";
    FD1P3IX reset_count__i4 (.D(n10878), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i4.GSR = "DISABLED";
    FD1P3IX reset_count__i5 (.D(n10879), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i5.GSR = "DISABLED";
    FD1P3IX reset_count__i6 (.D(n10880), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i6.GSR = "DISABLED";
    FD1P3IX reset_count__i7 (.D(n10881), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i7.GSR = "DISABLED";
    FD1P3IX reset_count__i8 (.D(n10882), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i8.GSR = "DISABLED";
    FD1P3IX reset_count__i9 (.D(n10883), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i9.GSR = "DISABLED";
    FD1P3IX reset_count__i10 (.D(n10884), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i10.GSR = "DISABLED";
    FD1P3IX reset_count__i11 (.D(n10885), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i11.GSR = "DISABLED";
    FD1P3IX reset_count__i12 (.D(n10886), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i12.GSR = "DISABLED";
    FD1P3IX reset_count__i13 (.D(n10887), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[13])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i13.GSR = "DISABLED";
    FD1P3IX reset_count__i14 (.D(n10888), .SP(pll_clk_enable_138), .CD(enable_N_2961), 
            .CK(pll_clk), .Q(reset_count_c[14])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i14.GSR = "DISABLED";
    LUT4 mux_2150_i5_3_lut_4_lut (.A(rgb_hold[4]), .B(shift_register[3]), 
         .C(n10212[2]), .D(n13), .Z(n10847)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i5_3_lut_4_lut.init = 16'haaca;
    LUT4 reset_count_11__bdd_4_lut_16284 (.A(reset_count_c[5]), .B(n21), 
         .C(reset_count_c[14]), .D(reset_count_c[8]), .Z(n27725)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;
    defparam reset_count_11__bdd_4_lut_16284.init = 16'hffdf;
    LUT4 mux_2150_i4_3_lut_4_lut (.A(rgb_hold[3]), .B(shift_register[2]), 
         .C(n10212[2]), .D(n13), .Z(n10846)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i4_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i3_3_lut_4_lut (.A(rgb_hold[2]), .B(shift_register[1]), 
         .C(n10212[2]), .D(n13), .Z(n10845)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i3_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2150_i2_3_lut_4_lut (.A(rgb_hold[1]), .B(shift_register[0]), 
         .C(n10212[2]), .D(n13), .Z(n10844)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2150_i2_3_lut_4_lut.init = 16'haaca;
    LUT4 i2816_3_lut_4_lut (.A(n29_adj_3144), .B(data_out_N_3021), .C(n10212[2]), 
         .D(n13), .Z(n10912)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A !(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i2816_3_lut_4_lut.init = 16'h5535;
    LUT4 reset_count_11__bdd_4_lut_16285 (.A(reset_count_c[12]), .B(reset_count_c[10]), 
         .C(reset_count_c[7]), .D(n27725), .Z(n27726)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;
    defparam reset_count_11__bdd_4_lut_16285.init = 16'hfffb;
    FD1P3IX bit_number_3058__i4 (.D(n25[4]), .SP(pll_clk_enable_261), .CD(n17643), 
            .CK(pll_clk), .Q(bit_number[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam bit_number_3058__i4.GSR = "DISABLED";
    FD1P3IX bit_number_3058__i3 (.D(n25[3]), .SP(pll_clk_enable_261), .CD(n17643), 
            .CK(pll_clk), .Q(bit_number[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam bit_number_3058__i3.GSR = "DISABLED";
    FD1P3IX bit_number_3058__i2 (.D(n25[2]), .SP(pll_clk_enable_261), .CD(n17643), 
            .CK(pll_clk), .Q(bit_number[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam bit_number_3058__i2.GSR = "DISABLED";
    FD1P3IX bit_number_3058__i1 (.D(n25[1]), .SP(pll_clk_enable_261), .CD(n17643), 
            .CK(pll_clk), .Q(bit_number[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam bit_number_3058__i1.GSR = "DISABLED";
    FD1P3IX shift_register_i0_i0 (.D(rgb_hold[0]), .SP(pll_clk_enable_561), 
            .CD(n17692), .CK(pll_clk), .Q(shift_register[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i0.GSR = "DISABLED";
    FD1S3IX frame_done_53 (.D(data_out_N_3021), .CK(pll_clk), .CD(n15952), 
            .Q(ws2812_frame_done)) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam frame_done_53.GSR = "DISABLED";
    FD1P3IX bit_cell_count_3060__i0 (.D(n33[0]), .SP(pll_clk_enable_260), 
            .CD(n17735), .CK(pll_clk), .Q(bit_cell_count[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3060__i0.GSR = "DISABLED";
    FD1P3IX bit_number_3058__i0 (.D(n25[0]), .SP(pll_clk_enable_261), .CD(n17643), 
            .CK(pll_clk), .Q(bit_number[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam bit_number_3058__i0.GSR = "DISABLED";
    OFS1P3IX data_out_reg_50 (.D(data_out_N_3014), .SP(pll_clk_enable_265), 
            .SCLK(pll_clk), .CD(enable_N_2961), .Q(rgb_data_c)) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam data_out_reg_50.GSR = "DISABLED";
    LUT4 reset_count_11__bdd_4_lut_16483 (.A(reset_count_c[9]), .B(reset_count_c[1]), 
         .C(reset_count_c[2]), .D(n27726), .Z(n27727)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;
    defparam reset_count_11__bdd_4_lut_16483.init = 16'hfffd;
    LUT4 i4241_4_lut (.A(n10212[0]), .B(n24729), .C(n29_adj_3144), .D(n16), 
         .Z(n15003)) /* synthesis lut_function=(A (B (C+(D))+!B (C))+!A (B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i4241_4_lut.init = 16'heca0;
    LUT4 i10_4_lut (.A(bit_cell_count[1]), .B(n20), .C(n16_adj_3145), 
         .D(n10212[2]), .Z(n24729)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i10_4_lut.init = 16'h8000;
    LUT4 i9_4_lut (.A(bit_cell_count[3]), .B(n18), .C(bit_cell_count[6]), 
         .D(bit_cell_count[0]), .Z(n20)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i9_4_lut.init = 16'h8000;
    LUT4 i5_2_lut (.A(bit_number[0]), .B(bit_number[2]), .Z(n16_adj_3145)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i5_2_lut.init = 16'h8888;
    LUT4 i7_4_lut (.A(bit_cell_count[2]), .B(bit_cell_count[5]), .C(bit_number[3]), 
         .D(bit_cell_count[4]), .Z(n18)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i7_4_lut.init = 16'h0002;
    LUT4 i15114_2_lut_3_lut (.A(ws2812_enable), .B(n10212[2]), .C(n13), 
         .Z(n15952)) /* synthesis lut_function=(((C)+!B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam i15114_2_lut_3_lut.init = 16'hf7f7;
    FD1S3IX state_FSM_i2 (.D(n10228), .CK(pll_clk), .CD(enable_N_2961), 
            .Q(n10212[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam state_FSM_i2.GSR = "DISABLED";
    FD1S3IX state_FSM_i3 (.D(n14999), .CK(pll_clk), .CD(enable_N_2961), 
            .Q(n10212[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam state_FSM_i3.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i1 (.D(n10844), .SP(pll_clk_enable_561), .CK(pll_clk), 
            .Q(shift_register[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i1.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i2 (.D(n10845), .SP(pll_clk_enable_561), .CK(pll_clk), 
            .Q(shift_register[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i2.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i3 (.D(n10846), .SP(pll_clk_enable_561), .CK(pll_clk), 
            .Q(shift_register[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i3.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i4 (.D(n10847), .SP(pll_clk_enable_561), .CK(pll_clk), 
            .Q(shift_register[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i4.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i5 (.D(n10848), .SP(pll_clk_enable_561), .CK(pll_clk), 
            .Q(shift_register[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i5.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i6 (.D(n10849), .SP(pll_clk_enable_561), .CK(pll_clk), 
            .Q(shift_register[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i6.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i7 (.D(n10850), .SP(pll_clk_enable_561), .CK(pll_clk), 
            .Q(shift_register[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i7.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i8 (.D(n10851), .SP(pll_clk_enable_561), .CK(pll_clk), 
            .Q(shift_register[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i8.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i9 (.D(n10852), .SP(pll_clk_enable_561), .CK(pll_clk), 
            .Q(shift_register[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i9.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i10 (.D(n10853), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i10.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i11 (.D(n10854), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i11.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i12 (.D(n10855), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i12.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i13 (.D(n10856), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[13])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i13.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i14 (.D(n10857), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[14])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i14.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i15 (.D(n10858), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[15])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i15.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i16 (.D(n10859), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[16])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i16.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i17 (.D(n10860), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[17])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i17.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i18 (.D(n10861), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[18])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i18.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i19 (.D(n10862), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[19])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i19.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i20 (.D(n10863), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[20])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i20.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i21 (.D(n10864), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[21])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i21.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i22 (.D(n10865), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[22])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i22.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i23 (.D(n10866), .SP(pll_clk_enable_561), 
            .CK(pll_clk), .Q(shift_register[23])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1021, LSE_RLINE=1027 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i23.GSR = "DISABLED";
    LUT4 i6921_2_lut_3_lut (.A(pll_clk_enable_561), .B(n10212[2]), .C(n13), 
         .Z(n17692)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam i6921_2_lut_3_lut.init = 16'h0808;
    CCU2D add_5_15 (.A0(reset_count_c[13]), .B0(n13359), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count_c[14]), .B1(n13359), .C1(GND_net), 
          .D1(GND_net), .CIN(n24508), .S0(n10887), .S1(n10888));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_15.INIT0 = 16'hd222;
    defparam add_5_15.INIT1 = 16'hd222;
    defparam add_5_15.INJECT1_0 = "NO";
    defparam add_5_15.INJECT1_1 = "NO";
    LUT4 i13504_2_lut_4_lut (.A(bit_number[3]), .B(bit_number[1]), .C(bit_number[0]), 
         .D(bit_number[2]), .Z(n25[3])) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13504_2_lut_4_lut.init = 16'h6aaa;
    LUT4 i2245_2_lut_3_lut (.A(data_out_N_3021), .B(n10212[2]), .C(n13), 
         .Z(n11005)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i2245_2_lut_3_lut.init = 16'h0808;
    CCU2D add_5_13 (.A0(reset_count_c[11]), .B0(n13359), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count_c[12]), .B1(n13359), .C1(GND_net), 
          .D1(GND_net), .CIN(n24507), .COUT(n24508), .S0(n10885), .S1(n10886));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_13.INIT0 = 16'hd222;
    defparam add_5_13.INIT1 = 16'hd222;
    defparam add_5_13.INJECT1_0 = "NO";
    defparam add_5_13.INJECT1_1 = "NO";
    CCU2D add_5_11 (.A0(reset_count_c[9]), .B0(n13359), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count_c[10]), .B1(n13359), .C1(GND_net), 
          .D1(GND_net), .CIN(n24506), .COUT(n24507), .S0(n10883), .S1(n10884));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_11.INIT0 = 16'hd222;
    defparam add_5_11.INIT1 = 16'hd222;
    defparam add_5_11.INJECT1_0 = "NO";
    defparam add_5_11.INJECT1_1 = "NO";
    LUT4 i13500_2_lut_3_lut (.A(bit_number[1]), .B(bit_number[0]), .C(bit_number[2]), 
         .Z(n24215)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13500_2_lut_3_lut.init = 16'h8080;
    CCU2D add_5_9 (.A0(reset_count_c[7]), .B0(n13359), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count_c[8]), .B1(n13359), .C1(GND_net), .D1(GND_net), 
          .CIN(n24505), .COUT(n24506), .S0(n10881), .S1(n10882));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_9.INIT0 = 16'hd222;
    defparam add_5_9.INIT1 = 16'hd222;
    defparam add_5_9.INJECT1_0 = "NO";
    defparam add_5_9.INJECT1_1 = "NO";
    CCU2D add_5_7 (.A0(reset_count_c[5]), .B0(n13359), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count_c[6]), .B1(n13359), .C1(GND_net), .D1(GND_net), 
          .CIN(n24504), .COUT(n24505), .S0(n10879), .S1(n10880));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_7.INIT0 = 16'hd222;
    defparam add_5_7.INIT1 = 16'hd222;
    defparam add_5_7.INJECT1_0 = "NO";
    defparam add_5_7.INJECT1_1 = "NO";
    CCU2D add_5_5 (.A0(reset_count_c[3]), .B0(n13359), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count_c[4]), .B1(n13359), .C1(GND_net), .D1(GND_net), 
          .CIN(n24503), .COUT(n24504), .S0(n10877), .S1(n10878));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_5.INIT0 = 16'hd222;
    defparam add_5_5.INIT1 = 16'hd222;
    defparam add_5_5.INJECT1_0 = "NO";
    defparam add_5_5.INJECT1_1 = "NO";
    CCU2D add_5_3 (.A0(reset_count_c[1]), .B0(n13359), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count_c[2]), .B1(n13359), .C1(GND_net), .D1(GND_net), 
          .CIN(n24502), .COUT(n24503), .S0(n10875), .S1(n10876));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_3.INIT0 = 16'hd222;
    defparam add_5_3.INIT1 = 16'hd222;
    defparam add_5_3.INJECT1_0 = "NO";
    defparam add_5_3.INJECT1_1 = "NO";
    CCU2D add_5_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n11005), .B1(n10912), .C1(n24165), .D1(GND_net), .COUT(n24502), 
          .S1(n10874));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_1.INIT0 = 16'hF000;
    defparam add_5_1.INIT1 = 16'he1e1;
    defparam add_5_1.INJECT1_0 = "NO";
    defparam add_5_1.INJECT1_1 = "NO";
    LUT4 i15121_2_lut_3_lut (.A(pll_clk_enable_561), .B(n10212[2]), .C(n13), 
         .Z(n17643)) /* synthesis lut_function=(A ((C)+!B)) */ ;
    defparam i15121_2_lut_3_lut.init = 16'ha2a2;
    LUT4 i15064_2_lut_4_lut (.A(data_out_N_3021), .B(n10212[2]), .C(n13), 
         .D(pll_clk_enable_561), .Z(pll_clk_enable_261)) /* synthesis lut_function=(A (B (C (D))+!B (D))+!A (D)) */ ;
    defparam i15064_2_lut_4_lut.init = 16'hf700;
    LUT4 i2817_3_lut (.A(n10212[0]), .B(n13), .C(n10212[2]), .Z(n13623)) /* synthesis lut_function=(!(A (B (C))+!A (B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i2817_3_lut.init = 16'h3a3a;
    LUT4 enable_I_0_1_lut (.A(ws2812_enable), .Z(enable_N_2961)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(27[13:20])
    defparam enable_I_0_1_lut.init = 16'h5555;
    LUT4 i2_4_lut (.A(bit_cell_count[3]), .B(n16913), .C(bit_cell_count[6]), 
         .D(bit_cell_count[5]), .Z(n13)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;
    defparam i2_4_lut.init = 16'hffdf;
    LUT4 i8_4_lut (.A(reset_count_c[3]), .B(reset_count_c[13]), .C(reset_count[0]), 
         .D(reset_count_c[4]), .Z(n21)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(37[25:52])
    defparam i8_4_lut.init = 16'hfffe;
    LUT4 i1729_2_lut (.A(n29_adj_3144), .B(n10212[0]), .Z(n10228)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i1729_2_lut.init = 16'h4444;
    LUT4 i4238_4_lut (.A(n10212[2]), .B(n10212[1]), .C(n13), .D(data_out_N_3021), 
         .Z(n14999)) /* synthesis lut_function=(A (B+(C+!(D)))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i4238_4_lut.init = 16'hecee;
    LUT4 i15207_4_lut (.A(n16), .B(bit_number[3]), .C(bit_number[2]), 
         .D(bit_number[0]), .Z(data_out_N_3021)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(52[29:48])
    defparam i15207_4_lut.init = 16'h2000;
    LUT4 i2150_2_lut (.A(n10212[2]), .B(n13), .Z(n10873)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i2150_2_lut.init = 16'h2222;
    LUT4 i1_2_lut (.A(bit_number[1]), .B(bit_number[4]), .Z(n16)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut.init = 16'h8888;
    LUT4 i15072_2_lut (.A(bit_cell_count[1]), .B(bit_cell_count[0]), .Z(n4)) /* synthesis lut_function=(!(A (B))) */ ;
    defparam i15072_2_lut.init = 16'h7777;
    LUT4 reset_count_11__bdd_3_lut_16486 (.A(reset_count_c[11]), .B(reset_count_c[6]), 
         .C(n27727), .Z(n29_adj_3144)) /* synthesis lut_function=((B+(C))+!A) */ ;
    defparam reset_count_11__bdd_3_lut_16486.init = 16'hfdfd;
    LUT4 bit_cell_count_6__bdd_4_lut_16376 (.A(shift_register[23]), .B(bit_cell_count[2]), 
         .C(bit_cell_count[5]), .D(bit_cell_count[4]), .Z(n27883)) /* synthesis lut_function=(A (((D)+!C)+!B)+!A (B+(C+!(D)))) */ ;
    defparam bit_cell_count_6__bdd_4_lut_16376.init = 16'hfe7f;
    LUT4 bit_cell_count_6__bdd_4_lut_16377 (.A(bit_cell_count[6]), .B(bit_cell_count[3]), 
         .C(n4), .D(n27883), .Z(n27884)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam bit_cell_count_6__bdd_4_lut_16377.init = 16'hfffe;
    LUT4 i1_4_lut (.A(reset_count_c[9]), .B(n10873), .C(n25684), .D(n26_adj_3146), 
         .Z(n13359)) /* synthesis lut_function=(A (B+!(C+!(D)))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i1_4_lut.init = 16'hcecc;
    LUT4 i14743_3_lut (.A(reset_count_c[2]), .B(n25670), .C(reset_count_c[6]), 
         .Z(n25684)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i14743_3_lut.init = 16'hfefe;
    LUT4 i12_4_lut (.A(n25654), .B(n24), .C(reset_count_c[4]), .D(reset_count_c[5]), 
         .Z(n26_adj_3146)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i12_4_lut.init = 16'h0400;
    LUT4 i14729_4_lut (.A(reset_count_c[14]), .B(reset_count_c[8]), .C(reset_count_c[11]), 
         .D(reset_count_c[3]), .Z(n25670)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;
    defparam i14729_4_lut.init = 16'hffdf;
    LUT4 i14713_2_lut (.A(reset_count[0]), .B(reset_count_c[13]), .Z(n25654)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14713_2_lut.init = 16'heeee;
    LUT4 i10_4_lut_adj_131 (.A(reset_count_c[1]), .B(reset_count_c[10]), 
         .C(reset_count_c[7]), .D(reset_count_c[12]), .Z(n24)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i10_4_lut_adj_131.init = 16'h0004;
    
endmodule
//
// Verilog Description of module umh_mic_iq_ram
//

module umh_mic_iq_ram (pll_clk, mic_ram_we, VCC_net, GND_net, mic_ram_wr_addr, 
            mic_ram_rd_addr, mic_ram_wr_data, mic_ram_rd_data) /* synthesis syn_module_defined=1 */ ;
    input pll_clk;
    input mic_ram_we;
    input VCC_net;
    input GND_net;
    input [8:0]mic_ram_wr_addr;
    input [8:0]mic_ram_rd_addr;
    input [15:0]mic_ram_wr_data;
    output [15:0]mic_ram_rd_data;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(108[10:17])
    
    PDPW8KC mem0 (.DI0(mic_ram_wr_data[0]), .DI1(mic_ram_wr_data[1]), .DI2(mic_ram_wr_data[2]), 
            .DI3(mic_ram_wr_data[3]), .DI4(mic_ram_wr_data[4]), .DI5(mic_ram_wr_data[5]), 
            .DI6(mic_ram_wr_data[6]), .DI7(mic_ram_wr_data[7]), .DI8(mic_ram_wr_data[8]), 
            .DI9(mic_ram_wr_data[9]), .DI10(mic_ram_wr_data[10]), .DI11(mic_ram_wr_data[11]), 
            .DI12(mic_ram_wr_data[12]), .DI13(mic_ram_wr_data[13]), .DI14(mic_ram_wr_data[14]), 
            .DI15(mic_ram_wr_data[15]), .DI16(GND_net), .DI17(GND_net), 
            .ADW0(mic_ram_wr_addr[0]), .ADW1(mic_ram_wr_addr[1]), .ADW2(mic_ram_wr_addr[2]), 
            .ADW3(mic_ram_wr_addr[3]), .ADW4(mic_ram_wr_addr[4]), .ADW5(mic_ram_wr_addr[5]), 
            .ADW6(mic_ram_wr_addr[6]), .ADW7(mic_ram_wr_addr[7]), .ADW8(mic_ram_wr_addr[8]), 
            .BE0(VCC_net), .BE1(VCC_net), .CEW(mic_ram_we), .CLKW(pll_clk), 
            .CSW0(GND_net), .CSW1(GND_net), .CSW2(GND_net), .ADR0(GND_net), 
            .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), .ADR4(mic_ram_rd_addr[0]), 
            .ADR5(mic_ram_rd_addr[1]), .ADR6(mic_ram_rd_addr[2]), .ADR7(mic_ram_rd_addr[3]), 
            .ADR8(mic_ram_rd_addr[4]), .ADR9(mic_ram_rd_addr[5]), .ADR10(mic_ram_rd_addr[6]), 
            .ADR11(mic_ram_rd_addr[7]), .ADR12(mic_ram_rd_addr[8]), .CER(VCC_net), 
            .OCER(VCC_net), .CLKR(pll_clk), .CSR0(GND_net), .CSR1(GND_net), 
            .CSR2(GND_net), .RST(GND_net), .DO0(mic_ram_rd_data[9]), .DO1(mic_ram_rd_data[10]), 
            .DO2(mic_ram_rd_data[11]), .DO3(mic_ram_rd_data[12]), .DO4(mic_ram_rd_data[13]), 
            .DO5(mic_ram_rd_data[14]), .DO6(mic_ram_rd_data[15]), .DO9(mic_ram_rd_data[0]), 
            .DO10(mic_ram_rd_data[1]), .DO11(mic_ram_rd_data[2]), .DO12(mic_ram_rd_data[3]), 
            .DO13(mic_ram_rd_data[4]), .DO14(mic_ram_rd_data[5]), .DO15(mic_ram_rd_data[6]), 
            .DO16(mic_ram_rd_data[7]), .DO17(mic_ram_rd_data[8]));
    defparam mem0.DATA_WIDTH_W = 18;
    defparam mem0.DATA_WIDTH_R = 18;
    defparam mem0.REGMODE = "NOREG";
    defparam mem0.CSDECODE_W = "0b000";
    defparam mem0.CSDECODE_R = "0b000";
    defparam mem0.GSR = "DISABLED";
    defparam mem0.RESETMODE = "SYNC";
    defparam mem0.ASYNC_RESET_RELEASE = "SYNC";
    defparam mem0.INIT_DATA = "STATIC";
    defparam mem0.INITVAL_00 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_01 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_02 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_03 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_04 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_05 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_06 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_07 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_08 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_09 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_10 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_11 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_12 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_13 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_14 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_15 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_16 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_17 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_18 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_19 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    
endmodule
//
// Verilog Description of module umh_channel_ram18
//

module umh_channel_ram18 (\rmw_slot[0] , \build_level_eff[0] , spi_channel_index, 
            staging_wr_page, ev_ch, staging_rd_page, spi1_mosi_c_0, 
            \spi_rx_shift[0] , \spi_rx_shift[1] , \spi_rx_shift[2] , \spi_rx_shift[3] , 
            \spi_rx_shift[4] , \spi_rx_shift[5] , \spi_rx_shift[6] , \staging_q[1] , 
            \staging_q[2] , \staging_q[3] , \staging_q[5] , \staging_q[6] , 
            \staging_q[7] , spi1_sck_c, spi_payload_byte, VCC_net, pll_clk, 
            GND_net, \build_level[1] , build_mod, n15, \event_level[1] , 
            \ev_sum[4] , staging_rd_page_N_2718, ev_in_clear, \build_level[0] , 
            \event_level[0] , \build_level[4] , \event_level[4] , n25784, 
            ev_fall_N_2683, n25149, n25736, n25313) /* synthesis syn_module_defined=1 */ ;
    input \rmw_slot[0] ;
    input \build_level_eff[0] ;
    input [6:0]spi_channel_index;
    input staging_wr_page;
    input [6:0]ev_ch;
    input staging_rd_page;
    input spi1_mosi_c_0;
    input \spi_rx_shift[0] ;
    input \spi_rx_shift[1] ;
    input \spi_rx_shift[2] ;
    input \spi_rx_shift[3] ;
    input \spi_rx_shift[4] ;
    input \spi_rx_shift[5] ;
    input \spi_rx_shift[6] ;
    output \staging_q[1] ;
    output \staging_q[2] ;
    output \staging_q[3] ;
    output \staging_q[5] ;
    output \staging_q[6] ;
    output \staging_q[7] ;
    input spi1_sck_c;
    input spi_payload_byte;
    input VCC_net;
    input pll_clk;
    input GND_net;
    input \build_level[1] ;
    input build_mod;
    output n15;
    output \event_level[1] ;
    input \ev_sum[4] ;
    input staging_rd_page_N_2718;
    input ev_in_clear;
    input \build_level[0] ;
    output \event_level[0] ;
    input \build_level[4] ;
    output \event_level[4] ;
    input n25784;
    input ev_fall_N_2683;
    output n25149;
    input n25736;
    output n25313;
    
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(86[24:32])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(108[10:17])
    
    wire n19;
    wire [7:0]staging_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[17:26])
    
    wire n8, n14, n10, n16;
    
    LUT4 i45_2_lut (.A(\rmw_slot[0] ), .B(\build_level_eff[0] ), .Z(n19)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(331[17:28])
    defparam i45_2_lut.init = 16'h6666;
    DP8KC mem0 (.DIA0(spi1_mosi_c_0), .DIA1(\spi_rx_shift[0] ), .DIA2(\spi_rx_shift[1] ), 
          .DIA3(\spi_rx_shift[2] ), .DIA4(\spi_rx_shift[3] ), .DIA5(\spi_rx_shift[4] ), 
          .DIA6(\spi_rx_shift[5] ), .DIA7(\spi_rx_shift[6] ), .DIA8(GND_net), 
          .ADA0(VCC_net), .ADA1(GND_net), .ADA2(GND_net), .ADA3(spi_channel_index[0]), 
          .ADA4(spi_channel_index[1]), .ADA5(spi_channel_index[2]), .ADA6(spi_channel_index[3]), 
          .ADA7(spi_channel_index[4]), .ADA8(spi_channel_index[5]), .ADA9(spi_channel_index[6]), 
          .ADA10(staging_wr_page), .ADA11(GND_net), .ADA12(GND_net), .CEA(VCC_net), 
          .OCEA(VCC_net), .CLKA(spi1_sck_c), .WEA(spi_payload_byte), .CSA0(GND_net), 
          .CSA1(GND_net), .CSA2(GND_net), .RSTA(GND_net), .DIB0(GND_net), 
          .DIB1(GND_net), .DIB2(GND_net), .DIB3(GND_net), .DIB4(GND_net), 
          .DIB5(GND_net), .DIB6(GND_net), .DIB7(GND_net), .DIB8(GND_net), 
          .ADB0(VCC_net), .ADB1(GND_net), .ADB2(GND_net), .ADB3(ev_ch[0]), 
          .ADB4(ev_ch[1]), .ADB5(ev_ch[2]), .ADB6(ev_ch[3]), .ADB7(ev_ch[4]), 
          .ADB8(ev_ch[5]), .ADB9(ev_ch[6]), .ADB10(staging_rd_page), .ADB11(GND_net), 
          .ADB12(GND_net), .CEB(VCC_net), .OCEB(VCC_net), .CLKB(pll_clk), 
          .WEB(GND_net), .CSB0(GND_net), .CSB1(GND_net), .CSB2(GND_net), 
          .RSTB(GND_net), .DOB0(staging_q[0]), .DOB1(\staging_q[1] ), 
          .DOB2(\staging_q[2] ), .DOB3(\staging_q[3] ), .DOB4(staging_q[4]), 
          .DOB5(\staging_q[5] ), .DOB6(\staging_q[6] ), .DOB7(\staging_q[7] ));
    defparam mem0.DATA_WIDTH_A = 9;
    defparam mem0.DATA_WIDTH_B = 9;
    defparam mem0.REGMODE_A = "NOREG";
    defparam mem0.REGMODE_B = "NOREG";
    defparam mem0.CSDECODE_A = "0b000";
    defparam mem0.CSDECODE_B = "0b000";
    defparam mem0.WRITEMODE_A = "READBEFOREWRITE";
    defparam mem0.WRITEMODE_B = "READBEFOREWRITE";
    defparam mem0.GSR = "DISABLED";
    defparam mem0.RESETMODE = "SYNC";
    defparam mem0.ASYNC_RESET_RELEASE = "SYNC";
    defparam mem0.INIT_DATA = "STATIC";
    defparam mem0.INITVAL_00 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_01 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_02 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_03 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_04 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_05 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_06 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_07 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_08 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_09 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_0F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_10 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_11 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_12 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_13 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_14 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_15 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_16 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_17 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_18 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_19 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam mem0.INITVAL_1F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    LUT4 i9684_4_lut (.A(\staging_q[1] ), .B(\build_level[1] ), .C(build_mod), 
         .D(n15), .Z(\event_level[1] )) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(363[17:26])
    defparam i9684_4_lut.init = 16'hca0a;
    LUT4 i22_4_lut (.A(\ev_sum[4] ), .B(staging_q[4]), .C(staging_rd_page_N_2718), 
         .D(ev_in_clear), .Z(n8)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(331[17:28])
    defparam i22_4_lut.init = 16'hcac0;
    LUT4 i9700_4_lut (.A(staging_q[0]), .B(\build_level[0] ), .C(build_mod), 
         .D(n15), .Z(\event_level[0] )) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(363[17:26])
    defparam i9700_4_lut.init = 16'hca0a;
    LUT4 i7_4_lut (.A(staging_q[0]), .B(n14), .C(n10), .D(\staging_q[6] ), 
         .Z(n15)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i7_4_lut.init = 16'hfffe;
    LUT4 i6_4_lut (.A(\staging_q[3] ), .B(\staging_q[2] ), .C(staging_q[4]), 
         .D(\staging_q[1] ), .Z(n14)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i6_4_lut.init = 16'hfffe;
    LUT4 i2_2_lut (.A(\staging_q[7] ), .B(\staging_q[5] ), .Z(n10)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i2_2_lut.init = 16'heeee;
    LUT4 i9692_4_lut (.A(staging_q[4]), .B(\build_level[4] ), .C(build_mod), 
         .D(n15), .Z(\event_level[4] )) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(363[17:26])
    defparam i9692_4_lut.init = 16'hca0a;
    PFUMX i21 (.BLUT(n8), .ALUT(n25784), .C0(ev_fall_N_2683), .Z(n25149));
    LUT4 i40_4_lut (.A(n19), .B(staging_q[0]), .C(staging_rd_page_N_2718), 
         .D(ev_in_clear), .Z(n16)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(331[17:28])
    defparam i40_4_lut.init = 16'hcac0;
    PFUMX i38 (.BLUT(n16), .ALUT(n25736), .C0(ev_fall_N_2683), .Z(n25313));
    
endmodule
