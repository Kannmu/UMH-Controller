// Verilog netlist produced by program LSE :  version Diamond (64-bit) 3.13.0.56.2
// Netlist written on Tue Oct 27 21:38:39 2026
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
    wire spi1_sck_N_161 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(148[17:33])
    wire sck_N_3140 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(10[12:16])
    
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
    
    wire expected_valid;
    wire [15:0]spi_extension_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(135[35:55])
    wire [15:0]spi_frame_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(136[17:35])
    wire [15:0]accepted_sequence_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(136[37:58])
    wire [1:0]accepted_update_flags_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(137[17:42])
    wire [6:0]spi_channel_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(138[17:34])
    
    wire spi_channel_page;
    wire [1:0]spi_compact_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(140[17:33])
    wire [1:0]mod_level_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(141[17:30])
    
    wire mod_toggle_spi;
    wire [9:0]feed_wr_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(143[17:28])
    
    wire frame_toggle_spi, stop_toggle_spi, invalid_frame_spi, ws2812_toggle_spi;
    wire [23:0]rgb_values;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(145[17:27])
    wire [1:0]spi_rgb_captured;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(146[17:33])
    
    wire rgb_values_complete_spi;
    wire [8:0]status_bit_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(148[17:33])
    
    wire spi_payload_byte, staging_wr_page, n24901;
    wire [8:0]spi_rgb_start;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(159[17:30])
    
    wire spi_audio_read_cmd, feed_we, ws2812_enable, ws2812_frame_done;
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
    
    wire link_tick_d, mic_warm, mic_ultrasonic;
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
    
    wire n24900, mic_lo_q_next;
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
    
    wire n14890, mic_cfg_toggle_spi, mic_cfg_toggle_meta, mic_cfg_toggle_sync, 
        mic_cfg_toggle_seen;
    wire [3:0]mic_cfg_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(285[17:31])
    
    wire n20734;
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
        ev_ch_lo_2, ev_ch_lo_1, ev_ch_lo_0, n29109, n29108, n29107, 
        n29106, n29105, n29104, n29103, n29101, n29100, n28061, 
        n28060, n28031, n28030, n27786, n27783, n27780, n27777, 
        staging_rd_page;
    wire [7:0]staging_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[17:26])
    
    wire ev_in_clear, ev_rd_grant, active_bank_N_898;
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
    
    wire mod_active, mod_fast;
    wire [7:0]pending_mod_level;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(362[17:34])
    wire [7:0]build_level;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(363[17:28])
    
    wire build_mod, frame_req_mod, mod_toggle_meta, mod_toggle_sync, 
        mod_toggle_seen;
    wire [3:0]mod_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[17:27])
    wire [9:0]feed_wr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[17:24])
    wire [9:0]feed_rd;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[26:33])
    wire [7:0]feed_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[17:23])
    wire [9:0]feed_fill;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(371[17:26])
    
    wire feed_pop;
    wire [7:0]event_level;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(390[17:28])
    
    wire stop_event, link_timeout, cs_meta, cs_sync, cs_sync_d, n28029, 
        n27774;
    wire [127:0]status_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(397[18:29])
    
    wire n27771, n71, n10129, n22794, n10126, n16947, n136, n74, 
        n19280, n27768, status_miso_q, cs_fall, cs_rise, n10796, 
        n10797, n10798;
    wire [15:0]fifo_credit_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[17:33])
    wire [15:0]fifo_depth_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[17:32])
    
    wire n10811;
    wire [8:0]status_next_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(427[17:34])
    
    wire status_next_bit, status_miso_bit;
    wire [8:0]pcm_payload_bit;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(437[16:31])
    
    wire n27762, n25978, n27759, pcm_audio_miso, payload_end, compact_end, 
        frame_end, audio_reset_end;
    wire [83:0]us_tx_next;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1029[17:27])
    
    wire n6, n27756, n10807, n4, n28484, spi1_sck_c_enable_135, 
        n28483, n28395;
    wire [6:0]mic_lo_n_next_6__N_648;
    
    wire n93, spi1_sck_c_enable_89, n27747;
    wire [6:0]mic_cfg_count_w_6__N_856;
    
    wire ev_ch_hi_10, build_idle_N_2724, staging_rd_page_N_2728, staging_rd_page_N_2729, 
        n28410, spi1_sck_c_enable_75, n10742, pll_clk_enable_267, n10751, 
        n17, n10728, n24820, n24899, pll_clk_enable_15, n27744;
    wire [15:0]status_flags_wire_15__N_1365;
    
    wire n10813, spi1_sck_c_enable_161, spi1_sck_c_enable_39, spi1_sck_c_enable_152, 
        n10812;
    wire [6:0]status_next_bit_N_2770;
    
    wire n28475, n28474;
    wire [5:0]pcm_spi_sample_15__N_1381;
    
    wire n10810, n25035, pll_clk_enable_169, n10809, n126;
    wire [4:0]pcm_audio_miso_N_2783;
    
    wire n25962, spi1_miso_N_2462, spi1_miso_N_2456, spi1_miso_N_2453, 
        spi1_miso_N_2452, fpga_cs_n_N_2472;
    wire [8:0]status_bit_index_8__N_1387;
    
    wire n25958, n24898, pll_clk_enable_6, n10808, n16190, n15211, 
        n13030, spi1_sck_c_enable_50, n25956, n24818, n24817, n24816, 
        n24814, n24813;
    wire [15:0]expected_next_8__N_1406;
    
    wire n25867, n28392, n17_adj_3164, spi_phase_only_N_2792, n17763, 
        n16, spi_bit_count_2__N_1425, n25948, spi_mod_feed_N_2490, n24897, 
        n28028, n17_adj_3165, n24, n25944, n20483, n23, n22, pll_clk_enable_172, 
        n25051, n29158, n4_adj_3166, n13458, n13455, n13723, n100, 
        n28027, spi_channel_page_N_2499, n28, n28449, spi1_sck_c_enable_164, 
        spi1_sck_c_enable_54, n15427, n15429, n15431, n28466, n25932, 
        n19, n18, n27717, n78, n55, n54, n53, n52, n13, n24175, 
        n24162, n24160, n12, n28465, pll_clk_enable_188, n14, invalid_frame_spi_N_2556, 
        n13_adj_3167, n17802, n51, mic_cfg_toggle_spi_N_2691, ws2812_toggle_spi_N_2565, 
        stop_toggle_spi_N_2529, frame_toggle_spi_N_2518, mod_toggle_spi_N_2510, 
        invalid_frame_spi_N_2536, n26906, n24812, n27714, n27711, 
        n24167, n27708, n24165, n24171, n28010, n107, n24811, 
        n16832, n17806, n3, n13_adj_3168, n79, n6_adj_3169, n21070, 
        n54_adj_3170, n17799, n17699, n8, n28463, n24173, n17477;
    wire [5:0]pcm_rd_ptr_spi_5__N_1626;
    
    wire n24156, n24154, n30, n27699, n28009, n28152, wrap_s2_N_2608, 
        n27696, n27693, n24169, active_bank_N_900, active_bank_N_899, 
        n17862, n17475, n28452, ev_run_hold_s5_83__N_1649, n25725, 
        n15, n17767, n28008, n10, n14159, n14158, n14156, n14155, 
        n14154, n14153, n14152, n14151, n14150, n14149, n14148, 
        n14147, n14146, n14145, n14144, n14143, n14142, n14141, 
        n14140, n14139, n14138, n14137, n14136, n14135, n14134, 
        n14133, n14132, n14131, n14130, n14129, n14128, n14127, 
        n14126, n14125, n14121, n14120, n14119, n14118, n14117, 
        n14116, n14115, n14114, n14113, n14112, n14111, n14110, 
        n14109, n14108, n14107, n14106, n14105, n14104, n14102, 
        n14101, n14100, n38, n27690, n28462, n28460, n28459, n13_adj_3171, 
        n28457, n28007, n3217, n3218, n3219, n14099, n14098, n14097, 
        n14096, n28456, n1, n10092, pll_clk_enable_98, pll_clk_enable_99, 
        n28454, n17717, n28006, n27913, n17694, n31, n4_adj_3172, 
        n24177, n24712, n17715, n14084, n27678, n28453, n3369, 
        ws2812_enable_N_2602, ws2812_enable_N_2601, n3424, n3425, n3_adj_3173, 
        n14095, n14094, n14093, n14092, n14091, n14090, n14089, 
        n14088, feed_rd_9__N_1712, frame_req_N_2710, n24883, n3639, 
        n9, n11, pll_clk_enable_100, frame_req_mod_N_2752, ws2812_settle_3__N_1717, 
        ws2812_enable_N_2606, n72, n27914, n24882, n24881, n17809, 
        n28451;
    wire [15:0]accepted_sequence_15__N_1001;
    
    wire n14087, n14086, n14085, n24880, pll_clk_enable_594, n17307, 
        n28450, ws2812_enable_N_2588, n10659, spi1_sck_c_enable_17, 
        n17703, n38_adj_3174, n39, n40, n41, n42, n43, n12_adj_3175, 
        n17705, n17707;
    wire [7:0]build_level_eff_7__N_1792;
    
    wire n13_adj_3176, n17709, n28448, n5, n24704, n16834, n7, 
        n23_adj_3177, pll_clk_enable_586, ev_fall_N_2693, swap_pending_N_2717, 
        n24711, n11_adj_3178, n24875, n4_adj_3179, n27663, n24874, 
        n24873, n13_adj_3180, n17697, n24158, n24714, n17701, n24716, 
        n17719, n17721, mic_ultrasonic_N_1914, n27660, n24872, n27314, 
        spi1_sck_c_enable_113, n28428, n24715, n18_adj_3181, n25972, 
        n28447, n20759, n28157, n24705, n24602, n24832, n27310, 
        n24867, n24212, n24866, n27651, n26174, n28693, n17689, 
        n24703, n24865, n27648, mic_lo_i_r_N_2639, n30_adj_3182, n31_adj_3183, 
        n32, n33, n34, n35, n24830, n17723, n27645, n28692, 
        n27642, n164, n165, n24864, n15115, n15113, n17726, n27639, 
        n44, n16841, n15109, n24834, spi1_sck_c_enable_51, n45, 
        n24374, n24829, pcm_fifo_N_2624, n4541, n24710, n12_adj_3184, 
        pll_clk_enable_603, n130, n4570, n24828, n24859, n24826, 
        n24858, n4599, n24857, n24825, n85, n84, n83, n82, n81, 
        n80, n24856, n24824, n4628, n79_adj_3185, n78_adj_3186, 
        n77, n76, n75, n74_adj_3187, n73, n72_adj_3188, n17695, 
        n163, n162, n161, n160, n159, n24855;
    wire [5:0]pcm_fifo_N_2617;
    
    wire n24854, n24853, n24852, n26172, pcm_fifo_N_2611, n8_adj_3189, 
        n78_adj_3190, n24851, n24823, n24850, n27636, n28440, n24706, 
        n13_adj_3191, n28441, spi1_sck_c_enable_68, n150, n151, n152, 
        n153, n154, n155, n156, n157, n158, n24849, n27810, 
        spi1_sck_c_enable_83, n21403, n26170, n24806, n24805, n24804, 
        n71_adj_3192, spi1_sck_c_enable_157, n24848, spi1_sck_c_enable_128, 
        n24847, pll_clk_enable_108, spi1_sck_c_enable_143, spi1_sck_c_enable_121, 
        spi1_sck_c_enable_105, n24846, spi1_sck_c_enable_160, n24822, 
        n24206, n28455, n15546, spi1_sck_c_enable_151, pll_clk_enable_253, 
        n8_adj_3193, n24845, n24844, n24843, n28686, n24821, n24842, 
        n28_adj_3194, n24841, n143, n144, n145, n146, n147, n148, 
        n149, n27, pll_clk_enable_117, n20918, pll_clk_enable_202, 
        pll_clk_enable_143, n134, n135, n136_adj_3195, n137, n138, 
        n139, n140, n141, n142, n26, n25, n27612, n24840, n27609, 
        n26168, n24839, n24838, n24837, pll_clk_enable_196, pll_clk_enable_540, 
        pll_clk_enable_150, pll_clk_enable_205, n17_adj_3196, n28434, 
        n17845, n24833, n70, n14_adj_3197, n16500, n15436, n15434, 
        n15432, n15430, n15428, n27606, n20909, n27603, n13_adj_3198, 
        n20751, n28685, n10_adj_3199, n40_adj_3200, n39_adj_3201, 
        n38_adj_3202, n37, n36, n35_adj_3203, n34_adj_3204, n28464, 
        n53_adj_3205, n21319, n27600, n24552, n17838, n12_adj_3206, 
        n28401, n26166, n25863, n25898, n21487, n45_adj_3207, n44_adj_3208, 
        n43_adj_3209, n42_adj_3210, n41_adj_3211, n40_adj_3212, n39_adj_3213, 
        n38_adj_3214, n62, pll_clk_enable_290, n35_adj_3215, n34_adj_3216, 
        n33_adj_3217, n32_adj_3218, n31_adj_3219, n30_adj_3220, n25805, 
        n20901, n40_adj_3221, n39_adj_3222, n38_adj_3223, n37_adj_3224, 
        n36_adj_3225, n35_adj_3226, n34_adj_3227, pll_clk_enable_90, 
        n11_adj_3228, n25850, n28684, n8_adj_3229, pll_clk_enable_168, 
        spi1_sck_c_enable_171, spi1_sck_c_enable_52, n25904, n28683, 
        n27591, n27588, pll_clk_enable_250, n28682, n15_adj_3230, 
        n14_adj_3231, n28495, n20896, mic_clk_N_2474, mic_warm_N_1918, 
        n5686, n5687, n5688, n5689, pll_clk_enable_259, pll_clk_enable_266, 
        pll_clk_enable_72, n21300, pll_clk_enable_268, n28443, mic_bc_valid_N_2628, 
        n5710, n5711, n5712, n5713, n5714, n5715, n5716, n5717, 
        n5718, n5719, n5720, n5721, n5722, n5723, n5724, n5725, 
        n5726, n5727, n5728, n5729, n5730, n5731, n5732, n5733, 
        n5734, n5735, n5736, n5737, n5738, n5739, n5740, n5741, 
        n5743, n5744, n5745, n5746, n5747, n5748, n5749, n5750, 
        n5751, n5752, n5753, n5754, n5755, n5756, n5757, n5758, 
        n5759, n5760, n5761, n5762, n5763, n5764, n5765, n5766, 
        n5767, n5768, n5769, n5770, n5771, n5772, n5773, n5774, 
        pll_clk_enable_605, pll_clk_enable_34, pll_clk_enable_97, pll_clk_enable_510, 
        n50, n49, n48, n47, n46, n45_adj_3232, n44_adj_3233, n43_adj_3234, 
        n42_adj_3235, n11_adj_3236, n28681, n25788, n21503, n12_adj_3237, 
        n10_adj_3238, n8_adj_3239, n6_adj_3240, n4_adj_3241, n28680, 
        n12_adj_3242, n10_adj_3243, n8_adj_3244, n6_adj_3245, n4_adj_3246, 
        n28032, n23802, n45_adj_3247, n44_adj_3248, n43_adj_3249, 
        n42_adj_3250, n41_adj_3251, n40_adj_3252, n39_adj_3253, n38_adj_3254, 
        n28431, n6_adj_3255, n24207, n25_adj_3256, n25885, n25879, 
        n24208, pll_clk_enable_246, n21289, n27573, n44_adj_3257, 
        pll_clk_enable_21, n13703, n13683, pll_clk_enable_297, pll_clk_enable_282, 
        n27570, n14_adj_3258, n26876, n28436, n26160, n24798, pll_clk_enable_11, 
        pll_clk_enable_24, n28435, n28433, n28432, n21455, n5_adj_3259, 
        n24183, n24995, n20788, n24797, n21440, n26302, n20745, 
        n28446, n7_adj_3260, n20, n25844, n18_adj_3261, n16_adj_3262, 
        n26156, n28404, n24209, n25021, n24796, n21415, spi1_sck_c_enable_21, 
        n7_adj_3263, n28473, n25237, n25895, n4_adj_3264, n20743, 
        mic_cfg_settle_3__N_2146, mic_acc_busy_N_2299, n26304, n28407, 
        n23777, n26146, n15263, n26145, n50_adj_3265, n49_adj_3266, 
        n124, n109, n48_adj_3267, n47_adj_3268, n16924, n46_adj_3269;
    wire [3:0]mic_cfg_settle_3__N_2142;
    
    wire n17711, n12_adj_3270, n29123, n12_adj_3271, n25210, n26306, 
        n26137, n17713, n13560, n24210, n26136, n27543, n20772, 
        n26133, n26132, n27540, n24795;
    wire [1:0]mic_run_state_1__N_2410;
    
    wire n24836, n12_adj_3272, n15433, n15435, n26129, n17740, n17733, 
        n11928, n24211, n13689, n18_adj_3273, n13685, n17_adj_3274, 
        n24179, n25667, n28935, n24947, n28934, n28933, n24946, 
        n15_adj_3275, n6_adj_3276, n14_adj_3277, n11_adj_3278, n10_adj_3279, 
        n24945, mic_acc_gate_6__N_2283, mic_gate_active_N_2304, mic_acc_busy_N_2302, 
        n6864, n6865, n6866, n6867, n6868, n6869, n6870, n6871, 
        n6872, n6873, n6874, n6875, n6876, n6877, n6878, n6879, 
        n28569, n12_adj_3280, n11_adj_3281, n14_adj_3282, n24790, 
        n12000, n24789, n24788, n11992, n11998, n24787, n11990, 
        n11988, n14_adj_3283, n26122, mic_acc_last_N_2682, mic_acc_last_N_2674, 
        n10_adj_3284, n10_adj_3285, n13249, n24943, n24782, n24942, 
        n21389, mic_acc_last_N_2673, n17708, n17710, n17712, n17714, 
        n17716, n17718, n17720, n17722, n17725, n11996, n24781, 
        n11994, mic_gate_active_N_2306, n27528, n25864, n24941, n17823;
    wire [2:0]mic_acc_state_2__N_2347;
    
    wire mic_acc_busy_N_2293, mic_acc_last_N_2669, n7025, n7026;
    wire [2:0]mic_acc_word_2__N_2332;
    wire [15:0]mic_ram_wr_data_15__N_2316;
    wire [6:0]mic_acc_last_N_2675;
    
    wire n34_adj_3286, n27525, n2;
    wire [6:0]mic_done_N_2657;
    
    wire mic_done_N_2656, n12_adj_3287, n73_adj_3288, n6_adj_3289, spi1_sck_c_enable_159, 
        n28896, n28895, n28894, n26103, n17036, n28893, n25072, 
        n17027, n28892, n17024, n27522, n17021, n24940, n24939, 
        n24780, n24775, n24774, n24773, n24771, n24770, n24769, 
        n24768, n24767, n24766, n24765, n24764, n24763, n27519, 
        mic_acc_busy_N_2292, mic_ram_we_N_2683, n24938, n28891, n4_adj_3290, 
        n17015, n10_adj_3291, n9_adj_3292, n25163, n28890;
    wire [2:0]mic_acc_state_2__N_750;
    
    wire n8_adj_3293, mic_done_N_2650, n25862, pll_clk_enable_247, n28562, 
        n28627, n16996, n10_adj_3294, n16995, n14083, spi1_sck_c_enable_162, 
        n24181, n27516, spi1_sck_c_enable_163, n16983, n27798, n97;
    wire [7:0]rd_data_7__N_2913;
    
    wire n10833, n28406, n11_adj_3295, n4_adj_3296, n16960, n24937, 
        n24936, n24935, n27242, n28458, n24934, pll_clk_enable_42, 
        pll_clk_enable_104, n24761;
    wire [14:0]reset_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[16:27])
    
    wire n24760, n24759, n24758, n24754, n24753, n24752, n24751, 
        n24750, n24749, n24748, n24747, n24746, n24745, n24744, 
        n24932, n8_adj_3297, n24931, n26100, n25040, n22498, n24742, 
        spi1_sck_c_enable_55, spi1_sck_c_enable_53, n24930, n28405, 
        n13550, n28403, spi1_sck_c_enable_97, n16974, n16953, n26099, 
        n24741, n24740, n8_adj_3298, n28461, n24928, n24739, n25810, 
        n24738, n24737, n24736, n25878, n24733, n24732, n24731, 
        n24729, n24728, n24727, n24726, n28402, pll_clk_enable_248, 
        n24722, n24721, n24720, n24718, n24709, n24708, n24707, 
        n24717, n24719, n24702, n24927, n17696, n17698, n17700, 
        n17702, n17704, n17706, n16829, n4_adj_3299, n24926, n24925, 
        n27501, n24924, n12_adj_3300, n27498, n26098, n27495, n8_adj_3301, 
        n27492, pll_clk_enable_5, n26096, n27489, n28158, n28156, 
        n28155, n4_adj_3302, n28153, n11_adj_3303, n26986, n22510, 
        n25901, n22516, n24923, n27486, n24922, n27483, n24921, 
        n28397, n24920, n24919, n24918, n25133, n25861, n27480, 
        n26056, n28789, n27471, n26050, n16_adj_3304, n28396, n24917, 
        n28788, n24916, n28787, n24915, n27468, n28482, n26048, 
        n8_adj_3305, n27459, n24914, spi1_sck_N_161_enable_9, n27456, 
        n16282, n16280, n16857, n16268, n16266, n24912, n26038, 
        n24701, n27441, pll_clk_enable_342, pll_clk_enable_370, n26091, 
        pll_clk_enable_601, pll_clk_enable_433, pll_clk_enable_495, n17018, 
        n16013, n16260, n16258, pll_clk_enable_602, n26032, n28394, 
        n10879, n27438, n28494, n28393, n22773, n4_adj_3306, n27789, 
        n10893, n28136, n24911, n28134, n24910, n24909, n16248, 
        n27429, n22781, n27426, n27423, n28504, n24908, n16952, 
        n16_adj_3307, n15108, n13500, n13496, n27420, n10986, n27417, 
        n70_adj_3308, n31_adj_3309, n78_adj_3310, n77_adj_3311, n26921, 
        n76_adj_3312, n75_adj_3313, n24904, n27414, n16180, n69, 
        n11017, n25936, n24903, n6_adj_3314, pll_clk_enable_517, n24902;
    
    VHI i2 (.Z(VCC_net));
    INV i17217 (.A(spi_mic_sck_c), .Z(sck_N_3140));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(95[24:35])
    FD1S3AY cs_meta_1212 (.D(fpga_cs_n_c), .CK(pll_clk), .Q(cs_meta)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(401[12] 405[8])
    defparam cs_meta_1212.GSR = "DISABLED";
    FD1P3AX spi_extension_length_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_extension_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_extension_length[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i5.GSR = "ENABLED";
    FD1S3AX mem_3327 (.D(n71), .CK(pll_clk), .Q(n14098));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3327.GSR = "DISABLED";
    FD1P3AX spi_extension_length_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_extension_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i4.GSR = "ENABLED";
    MUX21 i16023139_i1 (.D0(n28896), .D1(n28686), .SD(pcm_payload_bit[3]), 
          .Z(n26876));
    FD1P3AX spi_extension_length_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_extension_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_extension_length[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_extension_length[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_update_flags__i2 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_21), 
            .CK(spi1_sck_c), .Q(expected_next_8__N_1406[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_update_flags__i2.GSR = "ENABLED";
    LUT4 i1_2_lut (.A(n13_adj_3180), .B(mic_lo_n_next_6__N_648[5]), .Z(mic_lo_n_next[5])) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[33:52])
    defparam i1_2_lut.init = 16'h8888;
    MUX21 i15999127_i1 (.D0(n27489), .D1(n27471), .SD(status_next_index[6]), 
          .Z(status_next_bit));
    LUT4 ev_run_hold_s5_83__I_0_i72_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_71), .D(ev_run_hold_s5[71]), .Z(us_tx_next[71])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i72_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_adj_134 (.A(mod_settle[1]), .B(mod_settle[0]), .Z(n16280)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_134.init = 16'h9999;
    LUT4 stop_toggle_sync_I_0_2_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .Z(stop_event)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(393[23:61])
    defparam stop_toggle_sync_I_0_2_lut.init = 16'h6666;
    LUT4 i13840_2_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), .Z(n19)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[34:54])
    defparam i13840_2_lut.init = 16'h6666;
    FD1P3AX mic_sample_count_3082__i3 (.D(n22), .SP(pll_clk_enable_6), .CK(pll_clk), 
            .Q(mic_sample_count[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(879[37:60])
    defparam mic_sample_count_3082__i3.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i64_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_63), .D(ev_run_hold_s5[63]), .Z(us_tx_next[63])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i64_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX mic_sample_count_3082__i2 (.D(n23), .SP(pll_clk_enable_6), .CK(pll_clk), 
            .Q(mic_sample_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(879[37:60])
    defparam mic_sample_count_3082__i2.GSR = "DISABLED";
    FD1P3AX mic_sample_count_3082__i1 (.D(n24), .SP(pll_clk_enable_6), .CK(pll_clk), 
            .Q(mic_sample_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(879[37:60])
    defparam mic_sample_count_3082__i1.GSR = "DISABLED";
    FD1S3AY cs_sync_1213 (.D(cs_meta), .CK(pll_clk), .Q(cs_sync)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(401[12] 405[8])
    defparam cs_sync_1213.GSR = "DISABLED";
    FD1S3AY cs_sync_d_1214 (.D(cs_sync), .CK(pll_clk), .Q(cs_sync_d)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(401[12] 405[8])
    defparam cs_sync_d_1214.GSR = "DISABLED";
    FD1S3AX status_miso_q_1215 (.D(status_next_bit), .CK(spi1_sck_N_161), 
            .Q(status_miso_q)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[24:57])
    defparam status_miso_q_1215.GSR = "ENABLED";
    LUT4 i13847_3_lut (.A(spi_bit_count[2]), .B(spi_bit_count[1]), .C(spi_bit_count[0]), 
         .Z(n18)) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[34:54])
    defparam i13847_3_lut.init = 16'h6a6a;
    LUT4 i15484_4_lut (.A(fpga_cs_n_c), .B(spi_bit_count_2__N_1425), .C(frame_end), 
         .D(n31), .Z(spi1_sck_c_enable_159)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;
    defparam i15484_4_lut.init = 16'h4000;
    LUT4 mic_acc_last_I_92_4_lut (.A(pll_clk_enable_602), .B(mic_acc_last_N_2673), 
         .C(n10893), .D(mic_acc_last), .Z(mic_acc_last_N_2669)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B ((D)+!C)+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(962[9] 982[12])
    defparam mic_acc_last_I_92_4_lut.init = 16'h5c0c;
    LUT4 i4592_4_lut (.A(ev_sum[5]), .B(staging_q[5]), .C(staging_rd_page_N_2728), 
         .D(ev_in_clear), .Z(n15431)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i4592_4_lut.init = 16'hcac0;
    FD1S3AX spi_rx_shift_i1 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(spi_rx_shift[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_rx_shift_i1.GSR = "ENABLED";
    LUT4 i1_4_lut (.A(n28032), .B(n13_adj_3198), .C(spi_version[0]), .D(n28031), 
         .Z(n16960)) /* synthesis lut_function=(A (B)+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(550[30] 582[24])
    defparam i1_4_lut.init = 16'hc888;
    MUX21 i1586560_i1 (.D0(n27717), .D1(n27771), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[7]));
    MUX21 i15967111_i1 (.D0(n27759), .D1(n27693), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[2]));
    FD1P3IX running_1261 (.D(n29158), .SP(active_bank_N_900), .CD(n10742), 
            .CK(pll_clk), .Q(running)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam running_1261.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_135 (.A(frame_toggle_spi), .B(n21503), .Z(frame_toggle_spi_N_2518)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_135.init = 16'h9999;
    LUT4 i15481_2_lut (.A(n21503), .B(spi1_sck_c_enable_159), .Z(spi1_sck_c_enable_160)) /* synthesis lut_function=(A (B)) */ ;
    defparam i15481_2_lut.init = 16'h8888;
    MUX21 i1585957_i1 (.D0(n27501), .D1(n27573), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[1]));
    LUT4 i1_2_lut_adj_136 (.A(stop_toggle_spi), .B(n20751), .Z(stop_toggle_spi_N_2529)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_136.init = 16'h9999;
    FD1S3AX global_phase_s2_3074__i7 (.D(n38_adj_3174), .CK(pll_clk), .Q(global_phase_s2[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074__i7.GSR = "DISABLED";
    FD1S3AX global_phase_s2_3074__i6 (.D(n39), .CK(pll_clk), .Q(global_phase_s2[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074__i6.GSR = "DISABLED";
    FD1S3AX global_phase_s2_3074__i5 (.D(n40), .CK(pll_clk), .Q(global_phase_s2[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074__i5.GSR = "DISABLED";
    FD1P3AX spi_compact_data_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_compact_data[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_compact_data_i0_i0.GSR = "DISABLED";
    MUX21 i1593997_i1 (.D0(n27441), .D1(n27777), .SD(status_next_bit_N_2770[2]), 
          .Z(n26986));
    FD1S3AX global_phase_s2_3074__i4 (.D(n41), .CK(pll_clk), .Q(global_phase_s2[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074__i4.GSR = "DISABLED";
    LUT4 i15402_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_3)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15402_2_lut_3_lut.init = 16'h2020;
    FD1S3AX global_phase_s2_3074__i3 (.D(n42), .CK(pll_clk), .Q(global_phase_s2[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074__i3.GSR = "DISABLED";
    FD1S3AX global_phase_s2_3074__i2 (.D(n43), .CK(pll_clk), .Q(global_phase_s2[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074__i2.GSR = "DISABLED";
    MUX21 i1588369_i1 (.D0(n27543), .D1(n27789), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[4]));
    FD1S3AX feed_wr_spi_3073__i0 (.D(n55), .CK(spi1_sck_c), .Q(feed_wr_spi[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073__i0.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i0 (.D(spi_frame_sequence[0]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i0.GSR = "DISABLED";
    OB us_tx_pad_70 (.I(us_tx_c_70), .O(us_tx[70]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    FD1S3AX global_phase_s2_3074__i1 (.D(n44), .CK(pll_clk), .Q(global_phase_s2[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074__i1.GSR = "DISABLED";
    FD1P3AX spi_version_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_17), 
            .CK(spi1_sck_c), .Q(spi_version[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_version_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_17), 
            .CK(spi1_sck_c), .Q(spi_version[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_version_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_17), 
            .CK(spi1_sck_c), .Q(spi_version[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_version_i0_i5.GSR = "ENABLED";
    LUT4 i15_4_lut (.A(n25), .B(n27), .C(n26), .D(n28_adj_3194), .Z(mic_acc_busy_N_2302)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(964[21:43])
    defparam i15_4_lut.init = 16'hfffe;
    OB us_tx_pad_71 (.I(us_tx_c_71), .O(us_tx[71]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    FD1P3AX spi_version_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_17), 
            .CK(spi1_sck_c), .Q(spi_version[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_version_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_17), 
            .CK(spi1_sck_c), .Q(spi_version[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_version_i0_i3.GSR = "ENABLED";
    OB us_tx_pad_72 (.I(us_tx_c_72), .O(us_tx[72]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_73 (.I(us_tx_c_73), .O(us_tx[73]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_74 (.I(us_tx_c_74), .O(us_tx[74]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_75 (.I(us_tx_c_75), .O(us_tx[75]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_76 (.I(us_tx_c_76), .O(us_tx[76]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    FD1P3IX mod_active_1295 (.D(mod_level_spi[0]), .SP(pll_clk_enable_15), 
            .CD(n10742), .CK(pll_clk), .Q(mod_active)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mod_active_1295.GSR = "DISABLED";
    FD1P3AX spi_version_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_17), 
            .CK(spi1_sck_c), .Q(spi_version[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_version_i0_i2.GSR = "ENABLED";
    MUX21 i1587766_i1 (.D0(n27519), .D1(n27525), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[5]));
    FD1P3AX spi_version_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_17), 
            .CK(spi1_sck_c), .Q(spi_version[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_version_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_17), 
            .CK(spi1_sck_c), .Q(spi_version[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_version_i0_i0.GSR = "ENABLED";
    FD1S3AX global_phase_s2_3074__i0 (.D(n45), .CK(pll_clk), .Q(global_phase_s2[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074__i0.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i15 (.D(spi_frame_sequence[15]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i15.GSR = "DISABLED";
    FD1P3IX frame_req_1287 (.D(n29158), .SP(pll_clk_enable_5), .CD(n16190), 
            .CK(pll_clk), .Q(frame_req)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam frame_req_1287.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i14 (.D(spi_frame_sequence[14]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i13 (.D(spi_frame_sequence[13]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_sample_count_3082__i0 (.D(n25_adj_3256), .SP(pll_clk_enable_6), 
            .CK(pll_clk), .Q(mic_sample_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(879[37:60])
    defparam mic_sample_count_3082__i0.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_137 (.A(n13_adj_3180), .B(mic_lo_n_next_6__N_648[1]), 
         .Z(mic_lo_n_next[1])) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[33:52])
    defparam i1_2_lut_adj_137.init = 16'h8888;
    LUT4 mux_679_i12_3_lut_4_lut (.A(pending_sequence[11]), .B(accepted_sequence_spi[11]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[11])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i12_3_lut_4_lut.init = 16'hacaa;
    CCU2D feed_wr_spi_3073_add_4_5 (.A0(feed_wr_spi[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(feed_wr_spi[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24768), .COUT(n24769), .S0(n52), .S1(n51));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073_add_4_5.INIT0 = 16'hfaaa;
    defparam feed_wr_spi_3073_add_4_5.INIT1 = 16'hfaaa;
    defparam feed_wr_spi_3073_add_4_5.INJECT1_0 = "NO";
    defparam feed_wr_spi_3073_add_4_5.INJECT1_1 = "NO";
    FD1P3AX spi_update_flags__i1 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_21), 
            .CK(spi1_sck_c), .Q(spi_rgb_start[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_update_flags__i1.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i12 (.D(spi_frame_sequence[12]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX spi_extension_length_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_extension_length[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i0.GSR = "ENABLED";
    MUX21 i1587163_i1 (.D0(n27645), .D1(n27699), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[6]));
    FD1P3AX spi_frame_sequence_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i0.GSR = "ENABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i0.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(rgb_values[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i0.GSR = "DISABLED";
    FD1P3AX mod_level_spi_i0_i0 (.D(spi_compact_data[0]), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .Q(mod_level_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mod_level_spi_i0_i0.GSR = "DISABLED";
    FD1P3IX mic_window_count__i0 (.D(n10807), .SP(pll_clk_enable_117), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_window_count[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_window_count__i0.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i11 (.D(spi_frame_sequence[11]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i11.GSR = "DISABLED";
    LUT4 i15408_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_2)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15408_2_lut_3_lut.init = 16'h0202;
    FD1S3IX mod_settle__i0 (.D(n17475), .CK(pll_clk), .CD(n15546), .Q(mod_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mod_settle__i0.GSR = "DISABLED";
    INV i17216 (.A(spi1_sck_c), .Z(spi1_sck_N_161));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(86[24:32])
    FD1P3AX accepted_sequence_spi_i0_i10 (.D(spi_frame_sequence[10]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i10.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i0 (.D(pcm_rd_ptr_spi[0]), .CK(pll_clk), .Q(pcm_rd_sync1[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync1_i0.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i0 (.D(pcm_rd_sync1[0]), .CK(pll_clk), .Q(pcm_rd_sync2[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync2_i0.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i0 (.D(pcm_wr_ptr[0]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync1_spi_i0.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i0 (.D(pcm_wr_sync1_spi[0]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync2_spi_i0.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i0 (.D(pcm_rd_ptr_spi_5__N_1626[0]), .SP(spi1_sck_c_enable_157), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_rd_ptr_spi_i0.GSR = "DISABLED";
    FD1S3AX phase_frac_i0 (.D(phase_frac_sum[0]), .CK(pll_clk), .Q(phase_frac[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i0.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i9 (.D(spi_frame_sequence[9]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i8 (.D(spi_frame_sequence[8]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i8.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i0 (.D(feed_q[0]), .SP(pll_clk_enable_150), 
            .CD(n13685), .CK(pll_clk), .Q(pending_mod_level[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_mod_level__i0.GSR = "DISABLED";
    PFUMX i16586 (.BLUT(n28152), .ALUT(n10751), .C0(mic_warm), .Z(n28153));
    LUT4 i15387_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_4)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15387_2_lut_3_lut.init = 16'h0404;
    FD1P3AX accepted_sequence_spi_i0_i7 (.D(spi_frame_sequence[7]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i7.GSR = "DISABLED";
    LUT4 i15508_2_lut (.A(pll_clk_enable_602), .B(mic_run_state_1__N_2410[1]), 
         .Z(n10879)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15508_2_lut.init = 16'h1111;
    LUT4 i9911_2_lut (.A(mic_acc_busy_N_2292), .B(mic_done_N_2656), .Z(n20734)) /* synthesis lut_function=((B)+!A) */ ;
    defparam i9911_2_lut.init = 16'hdddd;
    FD1P3AX accepted_sequence_spi_i0_i6 (.D(spi_frame_sequence[6]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i5 (.D(spi_frame_sequence[5]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i5.GSR = "DISABLED";
    LUT4 i15374_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_5)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15374_2_lut_3_lut.init = 16'h4040;
    FD1S3AX spi_bit_count_3072__i0 (.D(n3_adj_3173), .CK(spi1_sck_c), .Q(spi_bit_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[34:54])
    defparam spi_bit_count_3072__i0.GSR = "ENABLED";
    LUT4 i6954_2_lut (.A(spi1_sck_c_enable_161), .B(expected_next_8__N_1406[3]), 
         .Z(n17802)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i6954_2_lut.init = 16'h2222;
    FD1P3AX accepted_sequence_spi_i0_i4 (.D(spi_frame_sequence[4]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i3 (.D(spi_frame_sequence[3]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i2 (.D(spi_frame_sequence[2]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i1 (.D(spi_frame_sequence[1]), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_sequence_spi_i0_i1.GSR = "DISABLED";
    FD1P3AX accepted_update_flags_spi_i0_i0 (.D(spi_rgb_start[7]), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .Q(accepted_update_flags_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_update_flags_spi_i0_i0.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_138 (.A(spi_rgb_captured[0]), .B(spi_rgb_captured[1]), 
         .Z(n16500)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_138.init = 16'h8888;
    LUT4 mic_cfg_count_6__I_0_i14_3_lut (.A(n12_adj_3237), .B(mic_done_N_2657[6]), 
         .C(mic_cfg_count[6]), .Z(mic_done_N_2656)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_cfg_count_6__I_0_i14_3_lut.init = 16'h8e8e;
    LUT4 mic_cfg_count_6__I_0_i12_3_lut (.A(n10_adj_3238), .B(mic_done_N_2657[5]), 
         .C(mic_cfg_count[5]), .Z(n12_adj_3237)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_cfg_count_6__I_0_i12_3_lut.init = 16'h8e8e;
    LUT4 i9_4_lut (.A(mic_gate_wait[0]), .B(mic_gate_wait[1]), .C(mic_gate_wait[6]), 
         .D(mic_gate_wait[4]), .Z(n25)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(964[21:43])
    defparam i9_4_lut.init = 16'hfffe;
    CCU2D pcm_decim_count_3081_add_4_3 (.A0(pcm_decim_count[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(pcm_decim_count[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24763), .COUT(n24764), .S0(n44_adj_3248), 
          .S1(n43_adj_3249));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081_add_4_3.INIT0 = 16'hfaaa;
    defparam pcm_decim_count_3081_add_4_3.INIT1 = 16'hfaaa;
    defparam pcm_decim_count_3081_add_4_3.INJECT1_0 = "NO";
    defparam pcm_decim_count_3081_add_4_3.INJECT1_1 = "NO";
    LUT4 i15218_3_lut (.A(n29109), .B(status_bit_index[3]), .C(status_bit_index[4]), 
         .Z(n26129)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(448[69:94])
    defparam i15218_3_lut.init = 16'h0808;
    LUT4 i11_4_lut (.A(mic_gate_wait[10]), .B(mic_gate_wait[2]), .C(mic_gate_wait[12]), 
         .D(mic_gate_wait[7]), .Z(n27)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(964[21:43])
    defparam i11_4_lut.init = 16'hfffe;
    VLO i1 (.Z(GND_net));
    TSALL TSALL_INST (.TSALL(GND_net));
    CCU2D add_2537_9 (.A0(\mic_acc_q[1] [6]), .B0(n20759), .C0(n4570), 
          .D0(GND_net), .A1(\mic_acc_q[1] [7]), .B1(n20759), .C1(n4570), 
          .D1(GND_net), .CIN(n24867), .S0(n5752), .S1(n5751));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam add_2537_9.INIT0 = 16'h7848;
    defparam add_2537_9.INIT1 = 16'h7848;
    defparam add_2537_9.INJECT1_0 = "NO";
    defparam add_2537_9.INJECT1_1 = "NO";
    CCU2D add_2537_7 (.A0(\mic_acc_q[1] [4]), .B0(n20759), .C0(n4570), 
          .D0(GND_net), .A1(\mic_acc_q[1] [5]), .B1(n20759), .C1(n4570), 
          .D1(GND_net), .CIN(n24866), .COUT(n24867), .S0(n5754), .S1(n5753));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam add_2537_7.INIT0 = 16'h7848;
    defparam add_2537_7.INIT1 = 16'h7848;
    defparam add_2537_7.INJECT1_0 = "NO";
    defparam add_2537_7.INJECT1_1 = "NO";
    LUT4 i9984_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_7)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i9984_2_lut_3_lut.init = 16'h8080;
    FD1P3AX mic_cfg_start_i0_i0 (.D(mic_cfg_hold_spi[8]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i0.GSR = "DISABLED";
    OB us_tx_pad_69 (.I(us_tx_c_69), .O(us_tx[69]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    CCU2D pcm_decim_count_3081_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_decim_count[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24763), .S1(n45_adj_3247));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081_add_4_1.INIT0 = 16'hF000;
    defparam pcm_decim_count_3081_add_4_1.INIT1 = 16'h0555;
    defparam pcm_decim_count_3081_add_4_1.INJECT1_0 = "NO";
    defparam pcm_decim_count_3081_add_4_1.INJECT1_1 = "NO";
    FD1P3IX mic_bc_valid_1318 (.D(mic_bc_valid_N_2628), .SP(pll_clk_enable_11), 
            .CD(mic_run_state_1__N_2410[1]), .CK(pll_clk), .Q(mic_bc_valid)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_bc_valid_1318.GSR = "DISABLED";
    FD1S3AX feed_wr_spi_3073__i9 (.D(n46_adj_3269), .CK(spi1_sck_c), .Q(feed_wr_spi[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073__i9.GSR = "DISABLED";
    LUT4 ws2812_toggle_spi_I_0_2_lut (.A(ws2812_toggle_spi), .B(invalid_frame_spi_N_2556), 
         .Z(ws2812_toggle_spi_N_2565)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(568[30] 582[24])
    defparam ws2812_toggle_spi_I_0_2_lut.init = 16'h6666;
    FD1S3AX feed_wr_spi_3073__i8 (.D(n47_adj_3268), .CK(spi1_sck_c), .Q(feed_wr_spi[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073__i8.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_139 (.A(mic_cfg_toggle_spi), .B(n16180), .Z(mic_cfg_toggle_spi_N_2691)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_139.init = 16'h9999;
    FD1S3AX time_divider_3075__i0 (.D(n35), .CK(pll_clk), .Q(time_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(649[25:44])
    defparam time_divider_3075__i0.GSR = "DISABLED";
    FD1S3AX feed_wr_spi_3073__i7 (.D(n48_adj_3267), .CK(spi1_sck_c), .Q(feed_wr_spi[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073__i7.GSR = "DISABLED";
    LUT4 i10_4_lut (.A(mic_gate_wait[8]), .B(mic_gate_wait[3]), .C(mic_gate_wait[13]), 
         .D(mic_gate_wait[5]), .Z(n26)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(964[21:43])
    defparam i10_4_lut.init = 16'hfffe;
    LUT4 i6_4_lut (.A(n53_adj_3205), .B(n27417), .C(status_bit_index[4]), 
         .D(status_bit_index[3]), .Z(n62)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(88[24:33])
    defparam i6_4_lut.init = 16'hcac0;
    PFUMX mux_1618_i1 (.BLUT(pcm_audio_miso_N_2783[0]), .ALUT(n26091), .C0(n21389), 
          .Z(pcm_audio_miso));
    OB us_tx_pad_77 (.I(us_tx_c_77), .O(us_tx[77]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_78 (.I(us_tx_c_78), .O(us_tx[78]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    CCU2D add_2537_5 (.A0(\mic_acc_q[1] [2]), .B0(n20759), .C0(n4570), 
          .D0(GND_net), .A1(\mic_acc_q[1] [3]), .B1(n20759), .C1(n4570), 
          .D1(GND_net), .CIN(n24865), .COUT(n24866), .S0(n5756), .S1(n5755));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam add_2537_5.INIT0 = 16'h7848;
    defparam add_2537_5.INIT1 = 16'h7848;
    defparam add_2537_5.INJECT1_0 = "NO";
    defparam add_2537_5.INJECT1_1 = "NO";
    LUT4 i15364_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_3)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i15364_2_lut_3_lut_4_lut.init = 16'h0008;
    LUT4 mic_cfg_count_6__I_0_i10_3_lut (.A(n8_adj_3239), .B(mic_done_N_2657[4]), 
         .C(mic_cfg_count[4]), .Z(n10_adj_3238)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_cfg_count_6__I_0_i10_3_lut.init = 16'h8e8e;
    OB us_tx_pad_79 (.I(us_tx_c_79), .O(us_tx[79]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    FD1S3AX feed_wr_spi_3073__i6 (.D(n49_adj_3266), .CK(spi1_sck_c), .Q(feed_wr_spi[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073__i6.GSR = "DISABLED";
    CCU2D add_2510_9 (.A0(\mic_acc_i[2] [6]), .B0(n79), .C0(n4599), .D0(GND_net), 
          .A1(\mic_acc_i[2] [7]), .B1(n79), .C1(n4599), .D1(GND_net), 
          .CIN(n24761), .S0(n5727), .S1(n5726));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(858[17:43])
    defparam add_2510_9.INIT0 = 16'hd212;
    defparam add_2510_9.INIT1 = 16'hd212;
    defparam add_2510_9.INJECT1_0 = "NO";
    defparam add_2510_9.INJECT1_1 = "NO";
    LUT4 i15469_4_lut (.A(n17689), .B(n20734), .C(mic_run_state_1__N_2410[1]), 
         .D(pll_clk_enable_602), .Z(pll_clk_enable_510)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i15469_4_lut.init = 16'hfffb;
    FD1S3AX feed_wr_spi_3073__i5 (.D(n50_adj_3265), .CK(spi1_sck_c), .Q(feed_wr_spi[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073__i5.GSR = "DISABLED";
    FD1P3AX status_bit_index_i0_i0 (.D(status_next_index[0]), .SP(spi1_sck_N_161_enable_9), 
            .CK(spi1_sck_N_161), .Q(status_bit_index[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(454[14:90])
    defparam status_bit_index_i0_i0.GSR = "ENABLED";
    FD1S3IX link_idle__i0 (.D(n11_adj_3303), .CK(pll_clk), .CD(cs_fall), 
            .Q(link_idle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam link_idle__i0.GSR = "DISABLED";
    LUT4 i12_4_lut (.A(mic_gate_wait[11]), .B(mic_gate_wait[9]), .C(mic_gate_wait[14]), 
         .D(mic_gate_wait[15]), .Z(n28_adj_3194)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(964[21:43])
    defparam i12_4_lut.init = 16'hfffe;
    LUT4 i1_4_lut_adj_140 (.A(spi_bit_count[0]), .B(n25885), .C(n20745), 
         .D(n24995), .Z(n17021)) /* synthesis lut_function=((((D)+!C)+!B)+!A) */ ;
    defparam i1_4_lut_adj_140.init = 16'hff7f;
    CCU2D add_2537_3 (.A0(\mic_acc_q[1] [0]), .B0(n20759), .C0(n4570), 
          .D0(GND_net), .A1(\mic_acc_q[1] [1]), .B1(n20759), .C1(n4570), 
          .D1(GND_net), .CIN(n24864), .COUT(n24865), .S0(n5758), .S1(n5757));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam add_2537_3.INIT0 = 16'h7484;
    defparam add_2537_3.INIT1 = 16'h7848;
    defparam add_2537_3.INJECT1_0 = "NO";
    defparam add_2537_3.INJECT1_1 = "NO";
    LUT4 i13825_3_lut (.A(mic_sample_count[3]), .B(mic_sample_count[2]), 
         .C(n24602), .Z(n22)) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(879[37:60])
    defparam i13825_3_lut.init = 16'h6a6a;
    FD1S3AX feed_wr_spi_3073__i4 (.D(n51), .CK(spi1_sck_c), .Q(feed_wr_spi[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073__i4.GSR = "DISABLED";
    LUT4 i7_4_lut (.A(n13_adj_3191), .B(n14_adj_3197), .C(spi_command[1]), 
         .D(spi_extension_length[5]), .Z(n16829)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(561[34:64])
    defparam i7_4_lut.init = 16'hfffe;
    FD1P3IX mic_gate_index__i0 (.D(mic_done_N_2657[0]), .SP(pll_clk_enable_202), 
            .CD(n13689), .CK(pll_clk), .Q(mic_gate_index[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_index__i0.GSR = "DISABLED";
    FD1P3IX mic_phase__i0 (.D(n5689), .SP(pll_clk_enable_205), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_phase[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_phase__i0.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i1 (.D(n5774), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i1.GSR = "DISABLED";
    FD1S3IX mic_acc_last_1350 (.D(mic_acc_last_N_2669), .CK(pll_clk), .CD(mic_acc_busy_N_2292), 
            .Q(mic_acc_last)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_last_1350.GSR = "DISABLED";
    LUT4 i15515_4_lut (.A(n20909), .B(n11_adj_3236), .C(spi_extension_length[5]), 
         .D(n21455), .Z(n25040)) /* synthesis lut_function=(!(A+(B+(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(550[30] 582[24])
    defparam i15515_4_lut.init = 16'h0111;
    FD1P3IX mod_fast_1296 (.D(mod_level_spi[1]), .SP(pll_clk_enable_15), 
            .CD(n10742), .CK(pll_clk), .Q(mod_fast)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mod_fast_1296.GSR = "DISABLED";
    CCU2D add_2510_7 (.A0(\mic_acc_i[2] [4]), .B0(n79), .C0(n4599), .D0(GND_net), 
          .A1(\mic_acc_i[2] [5]), .B1(n79), .C1(n4599), .D1(GND_net), 
          .CIN(n24760), .COUT(n24761), .S0(n5729), .S1(n5728));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(858[17:43])
    defparam add_2510_7.INIT0 = 16'hd212;
    defparam add_2510_7.INIT1 = 16'hd212;
    defparam add_2510_7.INJECT1_0 = "NO";
    defparam add_2510_7.INJECT1_1 = "NO";
    LUT4 i6_4_lut_adj_141 (.A(n17018), .B(n25898), .C(n25895), .D(n25904), 
         .Z(n14_adj_3197)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(561[34:64])
    defparam i6_4_lut_adj_141.init = 16'hfffe;
    CCU2D add_2510_5 (.A0(\mic_acc_i[2] [2]), .B0(n79), .C0(n4599), .D0(GND_net), 
          .A1(\mic_acc_i[2] [3]), .B1(n79), .C1(n4599), .D1(GND_net), 
          .CIN(n24759), .COUT(n24760), .S0(n5731), .S1(n5730));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(858[17:43])
    defparam add_2510_5.INIT0 = 16'hd212;
    defparam add_2510_5.INIT1 = 16'hd212;
    defparam add_2510_5.INJECT1_0 = "NO";
    defparam add_2510_5.INJECT1_1 = "NO";
    PFUMX mic_acc_last_I_94 (.BLUT(mic_acc_last_N_2674), .ALUT(mic_acc_last_N_2682), 
          .C0(mic_gate_active_N_2304), .Z(mic_acc_last_N_2673));
    LUT4 i2_4_lut (.A(spi_extension_length[2]), .B(spi_command[0]), .C(spi_extension_length[1]), 
         .D(n16829), .Z(n16180)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;
    defparam i2_4_lut.init = 16'hffdf;
    FD1S3AX feed_wr_spi_3073__i3 (.D(n52), .CK(spi1_sck_c), .Q(feed_wr_spi[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073__i3.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_142 (.A(mod_toggle_spi), .B(n31), .Z(mod_toggle_spi_N_2510)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_142.init = 16'h9999;
    FD1S3IX frame_settle__i0 (.D(n17477), .CK(pll_clk), .CD(n15263), .Q(frame_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam frame_settle__i0.GSR = "DISABLED";
    FD1P3AX status_hold__i1 (.D(feed_fill[8]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i1.GSR = "DISABLED";
    FD1S3AX feed_wr_spi_3073__i2 (.D(n53), .CK(spi1_sck_c), .Q(feed_wr_spi[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073__i2.GSR = "DISABLED";
    FD1S3AX feed_wr_spi_3073__i1 (.D(n54), .CK(spi1_sck_c), .Q(feed_wr_spi[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073__i1.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i0 (.D(n165), .CK(pll_clk), .Q(fpga_time[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i0.GSR = "DISABLED";
    FD1P3JX mic_run_state_i0 (.D(mic_run_state_1__N_2410[0]), .SP(pll_clk_enable_97), 
            .PD(mic_done_N_2650), .CK(pll_clk), .Q(mic_run_state[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_run_state_i0.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_143 (.A(mic_warm), .B(mic_ultrasonic), .Z(n126)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_adj_143.init = 16'h4444;
    LUT4 i1_2_lut_adj_144 (.A(spi_payload_byte), .B(n16974), .Z(spi1_sck_c_enable_55)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_144.init = 16'h8888;
    MUX21 i15961108_i1 (.D0(n27747), .D1(n27711), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[3]));
    LUT4 i1_2_lut_4_lut (.A(ws2812_frame_done), .B(ws2812_frames_left[0]), 
         .C(ws2812_frames_left[1]), .D(ws2812_enable_N_2588), .Z(pll_clk_enable_99)) /* synthesis lut_function=(A (B+(C+(D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(671[13:68])
    defparam i1_2_lut_4_lut.init = 16'hffa8;
    LUT4 i15343_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_7)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i15343_2_lut_3_lut_4_lut.init = 16'h0800;
    FD1P3AX accepted_sequence_i0 (.D(accepted_sequence_15__N_1001[0]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i0.GSR = "DISABLED";
    PFUMX i17030 (.BLUT(n28407), .ALUT(n28890), .C0(pcm_spi_sample_15__N_1381[4]), 
          .Z(n28891));
    LUT4 i4594_4_lut (.A(ev_sum[6]), .B(staging_q[6]), .C(staging_rd_page_N_2728), 
         .D(ev_in_clear), .Z(n15433)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i4594_4_lut.init = 16'hcac0;
    LUT4 i4588_4_lut (.A(ev_sum[3]), .B(staging_q[3]), .C(staging_rd_page_N_2728), 
         .D(ev_in_clear), .Z(n15427)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i4588_4_lut.init = 16'hcac0;
    FD1P3IX mic_cfg_count_i4 (.D(mic_cfg_hold_spi[4]), .SP(pll_clk_enable_602), 
            .CD(n17763), .CK(pll_clk), .Q(mic_cfg_count[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_count_i4.GSR = "DISABLED";
    FD1S3AX phase_step_s1_1252 (.D(phase_frac_sum[24]), .CK(pll_clk), .Q(phase_step_s1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_step_s1_1252.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_145 (.A(spi_channel_page), .B(spi_phase_only_N_2792), 
         .Z(spi_channel_page_N_2499)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i1_2_lut_adj_145.init = 16'h6666;
    CCU2D add_2537_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20759), .B1(n4570), .C1(GND_net), .D1(GND_net), .COUT(n24864));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam add_2537_1.INIT0 = 16'hF000;
    defparam add_2537_1.INIT1 = 16'h7777;
    defparam add_2537_1.INJECT1_0 = "NO";
    defparam add_2537_1.INJECT1_1 = "NO";
    FD1S3AX phase_step_s2_1253 (.D(phase_step_s1), .CK(pll_clk), .Q(phase_step_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_step_s2_1253.GSR = "DISABLED";
    FD1S3AX phase_step_s3_1256 (.D(phase_step_s2), .CK(pll_clk), .Q(phase_step_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_step_s3_1256.GSR = "DISABLED";
    FD1S3AX wrap_s3_1257 (.D(wrap_s2), .CK(pll_clk), .Q(wrap_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam wrap_s3_1257.GSR = "DISABLED";
    FD1S3AX swap_now_s3_1258 (.D(active_bank_N_900), .CK(pll_clk), .Q(swap_now_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam swap_now_s3_1258.GSR = "DISABLED";
    FD1S3AX active_bank_1259 (.D(active_bank_N_899), .CK(pll_clk), .Q(active_bank)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam active_bank_1259.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i0 (.D(global_phase_s2[0]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(run_addr_s3[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam run_addr_s3_i0.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i0 (.D(feed_wr_spi[0]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam feed_wr_i0_i0.GSR = "DISABLED";
    FD1S3AX phase_step_s4_1264 (.D(phase_step_s3), .CK(pll_clk), .Q(phase_step_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_step_s4_1264.GSR = "DISABLED";
    FD1S3AX wrap_s4_1265 (.D(wrap_s3), .CK(pll_clk), .Q(wrap_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam wrap_s4_1265.GSR = "DISABLED";
    FD1S3AX wrap_s5_1266 (.D(wrap_s4), .CK(pll_clk), .Q(wrap_s5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam wrap_s5_1266.GSR = "DISABLED";
    FD1S3AX link_tick_d_1270 (.D(fpga_time[10]), .CK(pll_clk), .Q(link_tick_d)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam link_tick_d_1270.GSR = "DISABLED";
    FD1S3IX ws2812_frames_left_i0 (.D(n3369), .CK(pll_clk), .CD(ws2812_enable_N_2588), 
            .Q(ws2812_frames_left[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ws2812_frames_left_i0.GSR = "DISABLED";
    FD1S3AX frame_toggle_meta_1273 (.D(frame_toggle_spi), .CK(pll_clk), 
            .Q(frame_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam frame_toggle_meta_1273.GSR = "DISABLED";
    FD1S3AX frame_toggle_sync_1274 (.D(frame_toggle_meta), .CK(pll_clk), 
            .Q(frame_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam frame_toggle_sync_1274.GSR = "DISABLED";
    FD1S3AX stop_toggle_meta_1275 (.D(stop_toggle_spi), .CK(pll_clk), .Q(stop_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam stop_toggle_meta_1275.GSR = "DISABLED";
    FD1S3AX stop_toggle_sync_1276 (.D(stop_toggle_meta), .CK(pll_clk), .Q(stop_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam stop_toggle_sync_1276.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_meta_1277 (.D(ws2812_toggle_spi), .CK(pll_clk), 
            .Q(ws2812_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ws2812_toggle_meta_1277.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_sync_1278 (.D(ws2812_toggle_meta), .CK(pll_clk), 
            .Q(ws2812_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ws2812_toggle_sync_1278.GSR = "DISABLED";
    FD1S3AX invalid_frame_meta_1279 (.D(invalid_frame_spi), .CK(pll_clk), 
            .Q(invalid_frame_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam invalid_frame_meta_1279.GSR = "DISABLED";
    FD1S3AX invalid_frame_sync_1280 (.D(invalid_frame_meta), .CK(pll_clk), 
            .Q(invalid_frame_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam invalid_frame_sync_1280.GSR = "DISABLED";
    FD1S3AX mod_toggle_meta_1281 (.D(mod_toggle_spi), .CK(pll_clk), .Q(mod_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mod_toggle_meta_1281.GSR = "DISABLED";
    FD1S3AX mod_toggle_sync_1282 (.D(mod_toggle_meta), .CK(pll_clk), .Q(mod_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mod_toggle_sync_1282.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i0 (.D(n15109), .SP(pll_clk_enable_297), .CK(pll_clk), 
            .Q(rmw_slot[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rmw_slot_i0_i0.GSR = "DISABLED";
    FD1P3AX ws2812_enable_1284 (.D(ws2812_enable_N_2588), .SP(pll_clk_enable_21), 
            .CK(pll_clk), .Q(ws2812_enable)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ws2812_enable_1284.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i0 (.D(accepted_sequence_spi[0]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i0.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_146 (.A(mic_phase[0]), .B(mic_phase[1]), .Z(n22794)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_adj_146.init = 16'heeee;
    FD1P3IX mic_cfg_count_i3 (.D(mic_cfg_hold_spi[3]), .SP(pll_clk_enable_602), 
            .CD(n17763), .CK(pll_clk), .Q(mic_cfg_count[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_count_i3.GSR = "DISABLED";
    FD1P3AX ev_fall_1302 (.D(ev_fall_N_2693), .SP(pll_clk_enable_24), .CK(pll_clk), 
            .Q(ev_fall)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_fall_1302.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i0 (.D(mic_data_0_c), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i0.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_147 (.A(n10659), .B(n30), .Z(n16974)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i1_2_lut_adj_147.init = 16'h2222;
    FD1P3AX mic_shift_0_r__i1 (.D(mic_data_0_c), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i1.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i1 (.D(mic_data_1_c), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i0.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i1 (.D(\mic_acc_i[3] [0]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[3] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i1.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i1 (.D(\mic_acc_q[3] [0]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[3] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i1.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i0 (.D(n17726), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i0.GSR = "DISABLED";
    FD1S3IX mic_cfg_settle_i0 (.D(mic_cfg_settle_3__N_2142[0]), .CK(pll_clk), 
            .CD(mic_cfg_settle_3__N_2146), .Q(mic_cfg_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_settle_i0.GSR = "DISABLED";
    PFUMX i11749 (.BLUT(n22510), .ALUT(n26160), .C0(mic_lo_n_next_6__N_648[4]), 
          .Z(n22516));
    FD1S3AX mic_cfg_toggle_meta_1332 (.D(mic_cfg_toggle_spi), .CK(pll_clk), 
            .Q(mic_cfg_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_toggle_meta_1332.GSR = "DISABLED";
    FD1S3AX mic_cfg_toggle_sync_1333 (.D(mic_cfg_toggle_meta), .CK(pll_clk), 
            .Q(mic_cfg_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_toggle_sync_1333.GSR = "DISABLED";
    LUT4 i24_4_lut (.A(n13_adj_3168), .B(n10751), .C(mic_warm), .D(mic_ultrasonic), 
         .Z(n17_adj_3165)) /* synthesis lut_function=(!(A (B (C+!(D))+!B !(C+(D)))+!A (B+!(C)))) */ ;
    defparam i24_4_lut.init = 16'h3a30;
    FD1P3AX mic_cfg_width_i0 (.D(mic_cfg_width_w[0]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_width[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_width_i0.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i0 (.D(mic_acc_word[0]), .SP(mic_ram_we_N_2683), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_addr_i0_i0.GSR = "DISABLED";
    FD1P3AX mic_done_1341 (.D(mic_done_N_2650), .SP(pll_clk_enable_34), 
            .CK(pll_clk), .Q(mic_done)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_done_1341.GSR = "DISABLED";
    FD1S3AX mic_acc_state_i0 (.D(mic_acc_state_2__N_750[0]), .CK(pll_clk), 
            .Q(mic_acc_state[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_state_i0.GSR = "DISABLED";
    LUT4 mic_cfg_count_6__I_0_i8_3_lut (.A(n6_adj_3240), .B(mic_done_N_2657[3]), 
         .C(mic_cfg_count[3]), .Z(n8_adj_3239)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_cfg_count_6__I_0_i8_3_lut.init = 16'h8e8e;
    FD1S3AX mic_ram_we_1351 (.D(mic_ram_we_N_2683), .CK(pll_clk), .Q(mic_ram_we)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_we_1351.GSR = "DISABLED";
    FD1P3AX build_level_eff_i0_i0 (.D(n11017), .SP(pll_clk_enable_72), .CK(pll_clk), 
            .Q(build_level_eff[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_eff_i0_i0.GSR = "DISABLED";
    FD1S3AX us_tx_i1 (.D(us_tx_next[0]), .CK(pll_clk), .Q(us_tx_c_0)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i1.GSR = "DISABLED";
    FD1P3AX build_level_i0_i0 (.D(pending_mod_level[0]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(build_level[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_i0_i0.GSR = "DISABLED";
    CCU2D add_2510_3 (.A0(\mic_acc_i[2] [0]), .B0(n79), .C0(n4599), .D0(GND_net), 
          .A1(\mic_acc_i[2] [1]), .B1(n79), .C1(n4599), .D1(GND_net), 
          .CIN(n24758), .COUT(n24759), .S0(n5733), .S1(n5732));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(858[17:43])
    defparam add_2510_3.INIT0 = 16'hd121;
    defparam add_2510_3.INIT1 = 16'hd212;
    defparam add_2510_3.INJECT1_0 = "NO";
    defparam add_2510_3.INJECT1_1 = "NO";
    OB us_tx_pad_80 (.I(us_tx_c_80), .O(us_tx[80]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    FD1S3AX pll_lock_sync_i0 (.D(pll_lock), .CK(pll_clk), .Q(pll_lock_sync[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(129[12:77])
    defparam pll_lock_sync_i0.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_148 (.A(n13_adj_3180), .B(n22516), .Z(mic_lo_i_r_N_2639)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[33:52])
    defparam i1_2_lut_adj_148.init = 16'hdddd;
    FD1P3AY spi_expected_last_i0 (.D(n2), .SP(spi1_sck_c_enable_54), .CK(spi1_sck_c), 
            .Q(spi_expected_last[0])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
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
    FD1P3AY spi_expected_last_i1 (.D(n16268), .SP(spi1_sck_c_enable_54), 
            .CK(spi1_sck_c), .Q(spi_expected_last[1])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_expected_last_i1.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i2 (.D(n24212), .SP(spi1_sck_c_enable_54), 
            .CK(spi1_sck_c), .Q(spi_expected_last[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_expected_last_i2.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i3 (.D(n24211), .SP(spi1_sck_c_enable_54), 
            .CK(spi1_sck_c), .Q(spi_expected_last[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_expected_last_i3.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i4 (.D(n24210), .SP(spi1_sck_c_enable_54), 
            .CK(spi1_sck_c), .Q(spi_expected_last[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_expected_last_i4.GSR = "ENABLED";
    FD1P3AY spi_expected_last_i5 (.D(n24209), .SP(spi1_sck_c_enable_54), 
            .CK(spi1_sck_c), .Q(spi_expected_last[5])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_expected_last_i5.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i6 (.D(n24208), .SP(spi1_sck_c_enable_54), 
            .CK(spi1_sck_c), .Q(spi_expected_last[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_expected_last_i6.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i7 (.D(n24207), .SP(spi1_sck_c_enable_54), 
            .CK(spi1_sck_c), .Q(spi_expected_last[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_expected_last_i7.GSR = "ENABLED";
    FD1P3AX spi_expected_last_i8 (.D(n24206), .SP(spi1_sck_c_enable_54), 
            .CK(spi1_sck_c), .Q(spi_expected_last[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_expected_last_i8.GSR = "ENABLED";
    FD1P3AY pcm_acc_i2 (.D(n25863), .SP(pll_clk_enable_266), .CK(pll_clk), 
            .Q(pcm_acc[2])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pcm_acc_i2.GSR = "DISABLED";
    FD1P3AY pcm_acc_i3 (.D(n25861), .SP(pll_clk_enable_266), .CK(pll_clk), 
            .Q(pcm_acc[3])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pcm_acc_i3.GSR = "DISABLED";
    LUT4 i15413_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_6)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i15413_2_lut_3_lut.init = 16'h0808;
    LUT4 i15440_4_lut (.A(n16_adj_3304), .B(spi_bit_count_2__N_1425), .C(n17024), 
         .D(spi_byte_count[0]), .Z(spi1_sck_c_enable_171)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(487[21:44])
    defparam i15440_4_lut.init = 16'h0004;
    FD1P3AY pcm_acc_i9 (.D(n25862), .SP(pll_clk_enable_266), .CK(pll_clk), 
            .Q(pcm_acc[9])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pcm_acc_i9.GSR = "DISABLED";
    FD1P3IX mic_lo_n_i2 (.D(n10796), .SP(pll_clk_enable_42), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_lo_n[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_lo_n_i2.GSR = "DISABLED";
    FD1P3IX mic_lo_n_i3 (.D(n10797), .SP(pll_clk_enable_42), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_lo_n[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_lo_n_i3.GSR = "DISABLED";
    FD1P3IX mic_lo_n_i4 (.D(n10798), .SP(pll_clk_enable_42), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_lo_n[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_lo_n_i4.GSR = "DISABLED";
    FD1P3AY mic_cfg_count_i6 (.D(mic_cfg_hold_spi[6]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_count[6])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_count_i6.GSR = "DISABLED";
    LUT4 i10235_4_lut (.A(mod_settle[3]), .B(feed_rd_9__N_1712), .C(mod_settle[2]), 
         .D(n4_adj_3296), .Z(n3639)) /* synthesis lut_function=(A (B+(C+(D)))+!A (B+!(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(704[18] 714[12])
    defparam i10235_4_lut.init = 16'heeed;
    LUT4 mic_cfg_width_6__I_0_1528_i12_3_lut (.A(n10_adj_3243), .B(mic_acc_last_N_2675[5]), 
         .C(mic_cfg_width[5]), .Z(n12_adj_3242)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(980[34:73])
    defparam mic_cfg_width_6__I_0_1528_i12_3_lut.init = 16'h8e8e;
    L6MUX21 i17032 (.D0(n28891), .D1(n28484), .SD(status_bit_index[2]), 
            .Z(n28892));
    PUR PUR_INST (.PUR(VCC_net));
    defparam PUR_INST.RST_PULSE = 1;
    FD1P3AX mic_cfg_width_i1 (.D(mic_cfg_width_w[1]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_width[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_width_i1.GSR = "DISABLED";
    FD1P3AX mic_cfg_width_i2 (.D(mic_cfg_width_w[2]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_width[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_width_i2.GSR = "DISABLED";
    FD1P3AY mic_cfg_width_i3 (.D(mic_cfg_width_w[3]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_width[3])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_width_i3.GSR = "DISABLED";
    FD1P3AX mic_cfg_width_i4 (.D(mic_cfg_width_w[4]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_width[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_width_i4.GSR = "DISABLED";
    FD1P3AX mic_cfg_width_i5 (.D(mic_cfg_width_w[5]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_width[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_width_i5.GSR = "DISABLED";
    FD1P3AX mic_cfg_width_i6 (.D(mic_cfg_hold_spi[46]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_width[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_width_i6.GSR = "DISABLED";
    FD1S3AX mem_3313 (.D(n78_adj_3310), .CK(pll_clk), .Q(n14084));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3313.GSR = "DISABLED";
    LUT4 mux_679_i13_3_lut_4_lut (.A(pending_sequence[12]), .B(accepted_sequence_spi[12]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[12])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i13_3_lut_4_lut.init = 16'hacaa;
    LUT4 i15359_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_4)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15359_2_lut_3_lut_4_lut.init = 16'h0100;
    LUT4 i6878_3_lut (.A(n17725), .B(mic_cfg_gap[0]), .C(n11928), .Z(n17726)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6878_3_lut.init = 16'hcaca;
    LUT4 i15379_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_0)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15379_2_lut_3_lut_4_lut.init = 16'h0001;
    FD1S3AX mem_3315 (.D(n77_adj_3311), .CK(pll_clk), .Q(n14086));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3315.GSR = "DISABLED";
    umh_level_fifo level_fifo (.n14085(n14085), .spi1_sck_c(spi1_sck_c), 
            .feed_wr_spi({feed_wr_spi}), .n14097(n14097), .n14087(n14087), 
            .feed_q({feed_q}), .pll_clk(pll_clk), .rd_data_7__N_2913({rd_data_7__N_2913}), 
            .n14083(n14083), .n68({n69, n70_adj_3308, n71, n72, n73_adj_3288, 
            n74, n75_adj_3313, n76_adj_3312, n77_adj_3311, n78_adj_3310}), 
            .spi1_mosi_c_0(spi1_mosi_c_0), .\spi_rx_shift[0] (spi_rx_shift[0]), 
            .\spi_rx_shift[1] (spi_rx_shift[1]), .\spi_rx_shift[2] (spi_rx_shift[2]), 
            .\spi_rx_shift[3] (spi_rx_shift[3]), .\spi_rx_shift[4] (spi_rx_shift[4]), 
            .\spi_rx_shift[5] (spi_rx_shift[5]), .\spi_rx_shift[6] (spi_rx_shift[6]), 
            .n14106(n14106), .n14108(n14108), .n14110(n14110), .n14112(n14112), 
            .n14114(n14114), .n14116(n14116), .n14118(n14118), .n14120(n14120), 
            .feed_we(feed_we), .VCC_net(VCC_net), .GND_net(GND_net), .n14101(n14101), 
            .n14099(n14099), .n14089(n14089), .n14091(n14091), .n14093(n14093), 
            .n14095(n14095)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(385[20] 388[6])
    DPR16X4C pcm_fifo1 (.DI0(pcm_acc_next[6]), .DI1(pcm_acc_next[7]), .DI2(pcm_acc_next[8]), 
            .DI3(pcm_acc_next[9]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14158), .RAD0(pcm_spi_sample_15__N_1381[0]), .RAD1(pcm_spi_sample_15__N_1381[1]), 
            .RAD2(pcm_spi_sample_15__N_1381[2]), .RAD3(pcm_spi_sample_15__N_1381[3]), 
            .DO0(n14137), .DO1(n14138), .DO2(n14139), .DO3(n14140));
    defparam pcm_fifo1.initval = "0x0000000000000000";
    FD1P3IX rgb_hold__i0 (.D(rgb_values[0]), .SP(pll_clk_enable_540), .CD(n13683), 
            .CK(pll_clk), .Q(rgb_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i0.GSR = "DISABLED";
    LUT4 i15356_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_5)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15356_2_lut_3_lut_4_lut.init = 16'h0200;
    PFUMX mux_2607_i2 (.BLUT(n26103), .ALUT(mic_acc_state_2__N_2347[1]), 
          .C0(n11_adj_3228), .Z(mic_acc_state_2__N_750[1]));
    FD1P3AX spi_extension_length_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .Q(spi_extension_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i7.GSR = "ENABLED";
    CCU2D add_2510_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n4599), .B1(n79), .C1(GND_net), .D1(GND_net), .COUT(n24758));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(858[17:43])
    defparam add_2510_1.INIT0 = 16'hF000;
    defparam add_2510_1.INIT1 = 16'hdddd;
    defparam add_2510_1.INJECT1_0 = "NO";
    defparam add_2510_1.INJECT1_1 = "NO";
    FD1P3IX ev_wrap_1301 (.D(ev_sum[8]), .SP(ev_fall_N_2693), .CD(n13703), 
            .CK(pll_clk), .Q(ev_wrap)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_wrap_1301.GSR = "DISABLED";
    LUT4 mic_cfg_width_6__I_0_1528_i10_3_lut (.A(n8_adj_3244), .B(mic_acc_last_N_2675[4]), 
         .C(mic_cfg_width[4]), .Z(n10_adj_3243)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(980[34:73])
    defparam mic_cfg_width_6__I_0_1528_i10_3_lut.init = 16'h8e8e;
    LUT4 i15372_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_1)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15372_2_lut_3_lut_4_lut.init = 16'h0002;
    LUT4 i3575_2_lut (.A(mod_settle[1]), .B(mod_settle[0]), .Z(n4_adj_3296)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(705[27:44])
    defparam i3575_2_lut.init = 16'heeee;
    LUT4 i15329_2_lut_3_lut_4_lut (.A(ev_ch[5]), .B(ev_ch[6]), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(ev_ch_hi_9)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15329_2_lut_3_lut_4_lut.init = 16'h0400;
    CCU2D pcm_wr_sync2_spi_5__I_0_add_2_7 (.A0(pcm_wr_sync2_spi[5]), .B0(pcm_rd_ptr_spi[5]), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24754), .S0(pcm_count_spi[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[33:66])
    defparam pcm_wr_sync2_spi_5__I_0_add_2_7.INIT0 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_7.INIT1 = 16'h0000;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_7.INJECT1_0 = "NO";
    defparam pcm_wr_sync2_spi_5__I_0_add_2_7.INJECT1_1 = "NO";
    CCU2D pcm_wr_sync2_spi_5__I_0_add_2_5 (.A0(pcm_wr_sync2_spi[3]), .B0(pcm_rd_ptr_spi[3]), 
          .C0(GND_net), .D0(GND_net), .A1(pcm_wr_sync2_spi[4]), .B1(pcm_rd_ptr_spi[4]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24753), .COUT(n24754), .S0(pcm_count_spi[3]), 
          .S1(pcm_count_spi[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[33:66])
    defparam pcm_wr_sync2_spi_5__I_0_add_2_5.INIT0 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_5.INIT1 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_5.INJECT1_0 = "NO";
    defparam pcm_wr_sync2_spi_5__I_0_add_2_5.INJECT1_1 = "NO";
    OB us_tx_pad_81 (.I(us_tx_c_81), .O(us_tx[81]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_82 (.I(us_tx_c_82), .O(us_tx[82]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    OB us_tx_pad_83 (.I(us_tx_c_83), .O(us_tx[83]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(89[24:29])
    CCU2D pcm_wr_sync2_spi_5__I_0_add_2_3 (.A0(pcm_wr_sync2_spi[1]), .B0(pcm_rd_ptr_spi[1]), 
          .C0(GND_net), .D0(GND_net), .A1(pcm_wr_sync2_spi[2]), .B1(pcm_rd_ptr_spi[2]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24752), .COUT(n24753), .S0(pcm_count_spi[1]), 
          .S1(pcm_count_spi[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[33:66])
    defparam pcm_wr_sync2_spi_5__I_0_add_2_3.INIT0 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_3.INIT1 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_3.INJECT1_0 = "NO";
    defparam pcm_wr_sync2_spi_5__I_0_add_2_3.INJECT1_1 = "NO";
    LUT4 i4_4_lut (.A(spi_byte_count[1]), .B(n14), .C(n16953), .D(n6_adj_3289), 
         .Z(spi1_sck_c_enable_164)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i4_4_lut.init = 16'h8000;
    LUT4 ev_run_hold_s5_83__I_0_i83_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_82), .D(ev_run_hold_s5[82]), .Z(us_tx_next[82])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i83_3_lut_4_lut.init = 16'hbf40;
    PFUMX mux_2607_i1 (.BLUT(n26100), .ALUT(mic_acc_state_2__N_2347[0]), 
          .C0(n11_adj_3228), .Z(mic_acc_state_2__N_750[0]));
    LUT4 n28895_bdd_3_lut (.A(n28895), .B(n28892), .C(status_bit_index[1]), 
         .Z(n28896)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n28895_bdd_3_lut.init = 16'hcaca;
    LUT4 i10551_4_lut (.A(build_level_eff_7__N_1792[0]), .B(build_idle_N_2724), 
         .C(ev_rd_grant), .D(staging_rd_page_N_2729), .Z(pll_clk_enable_72)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (B (C)+!B (C (D)))) */ ;
    defparam i10551_4_lut.init = 16'hf0c8;
    FD1P3AX mic_acc_gate__i1 (.D(mic_gate_index[0]), .SP(mic_acc_busy_N_2293), 
            .CK(pll_clk), .Q(mic_acc_gate[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_gate__i1.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i84_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_83), .D(ev_run_hold_s5[83]), .Z(us_tx_next[83])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i84_3_lut_4_lut.init = 16'hbf40;
    LUT4 i6847_3_lut (.A(n17694), .B(mic_cfg_gap[1]), .C(n11928), .Z(n17695)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6847_3_lut.init = 16'hcaca;
    OBZ spi1_miso_pad (.I(spi1_miso_N_2452), .T(fpga_cs_n_c), .O(spi1_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(445[12:21])
    LUT4 mic_cfg_count_6__I_0_i6_3_lut (.A(n4_adj_3241), .B(mic_done_N_2657[2]), 
         .C(mic_cfg_count[2]), .Z(n6_adj_3240)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_cfg_count_6__I_0_i6_3_lut.init = 16'h8e8e;
    LUT4 staging_q_7__I_0_1380_i1_4_lut (.A(staging_q[0]), .B(build_level[0]), 
         .C(build_mod), .D(n15), .Z(event_level[0])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(390[31] 392[40])
    defparam staging_q_7__I_0_1380_i1_4_lut.init = 16'hca0a;
    LUT4 mic_cfg_width_6__I_0_1528_i8_3_lut (.A(n6_adj_3245), .B(mic_acc_last_N_2675[3]), 
         .C(mic_cfg_width[3]), .Z(n8_adj_3244)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(980[34:73])
    defparam mic_cfg_width_6__I_0_1528_i8_3_lut.init = 16'h8e8e;
    FD1S3JX ev_state_FSM_i1 (.D(n15115), .CK(pll_clk), .PD(n10742), .Q(build_idle_N_2724));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam ev_state_FSM_i1.GSR = "DISABLED";
    LUT4 i6846_4_lut (.A(mic_cfg_start[1]), .B(n6878), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17694)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6846_4_lut.init = 16'hcac0;
    LUT4 i6849_3_lut (.A(n17696), .B(mic_cfg_gap[2]), .C(n11928), .Z(n17697)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6849_3_lut.init = 16'hcaca;
    LUT4 mic_cfg_width_6__I_0_1528_i6_3_lut (.A(n4_adj_3246), .B(mic_acc_last_N_2675[2]), 
         .C(mic_cfg_width[2]), .Z(n6_adj_3245)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(980[34:73])
    defparam mic_cfg_width_6__I_0_1528_i6_3_lut.init = 16'h8e8e;
    LUT4 spi_channel_index_0__bdd_4_lut_16588 (.A(spi_channel_index[1]), .B(spi_channel_index[3]), 
         .C(spi_channel_index[6]), .D(spi_channel_index[2]), .Z(n28157)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam spi_channel_index_0__bdd_4_lut_16588.init = 16'h0020;
    LUT4 i9927_4_lut_4_lut (.A(n10_adj_3279), .B(spi_command[1]), .C(spi_command[0]), 
         .D(n14_adj_3258), .Z(n20751)) /* synthesis lut_function=(A+(B (C+(D))+!B ((D)+!C))) */ ;
    defparam i9927_4_lut_4_lut.init = 16'hffeb;
    LUT4 spi_phase_only_N_2792_bdd_4_lut_16821 (.A(spi_command[1]), .B(spi_command[2]), 
         .C(n14_adj_3258), .D(spi_command[0]), .Z(n28504)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam spi_phase_only_N_2792_bdd_4_lut_16821.init = 16'hfffe;
    CCU2D pcm_wr_sync2_spi_5__I_0_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_wr_sync2_spi[0]), .B1(pcm_rd_ptr_spi[0]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24752), .S1(pcm_count_spi[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[33:66])
    defparam pcm_wr_sync2_spi_5__I_0_add_2_1.INIT0 = 16'h0000;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_1.INIT1 = 16'h5999;
    defparam pcm_wr_sync2_spi_5__I_0_add_2_1.INJECT1_0 = "NO";
    defparam pcm_wr_sync2_spi_5__I_0_add_2_1.INJECT1_1 = "NO";
    LUT4 spi_channel_index_0__bdd_4_lut (.A(spi_channel_index[0]), .B(spi_channel_index[5]), 
         .C(spi_channel_index[4]), .D(n28157), .Z(n28158)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam spi_channel_index_0__bdd_4_lut.init = 16'h2000;
    LUT4 mic_cfg_count_6__I_0_i4_4_lut (.A(mic_done_N_2657[0]), .B(mic_done_N_2657[1]), 
         .C(mic_cfg_count[1]), .D(mic_cfg_count[0]), .Z(n4_adj_3241)) /* synthesis lut_function=(A (B+!(C))+!A !(B (C (D))+!B (C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_cfg_count_6__I_0_i4_4_lut.init = 16'h8ecf;
    LUT4 i6977_4_lut (.A(spi1_sck_c_enable_164), .B(n25035), .C(pcm_count_spi[5]), 
         .D(pcm_count_spi[4]), .Z(n17823)) /* synthesis lut_function=(A (B (C+(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i6977_4_lut.init = 16'ha8a0;
    DPR16X4C pcm_fifo7 (.DI0(GND_net), .DI1(GND_net), .DI2(GND_net), .DI3(GND_net), 
            .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), .WAD2(pcm_wr_ptr[2]), 
            .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), .WRE(n14159), .RAD0(pcm_spi_sample_15__N_1381[0]), 
            .RAD1(pcm_spi_sample_15__N_1381[1]), .RAD2(pcm_spi_sample_15__N_1381[2]), 
            .RAD3(pcm_spi_sample_15__N_1381[3]), .DO0(n14141), .DO1(n14142), 
            .DO2(n14143), .DO3(n14144));
    defparam pcm_fifo7.initval = "0x0000000000000000";
    LUT4 mic_cfg_width_6__I_0_1528_i4_4_lut (.A(mic_acc_last_N_2675[0]), .B(mic_acc_last_N_2675[1]), 
         .C(mic_cfg_width[1]), .D(mic_cfg_width[0]), .Z(n4_adj_3246)) /* synthesis lut_function=(A (B+!(C))+!A !(B (C (D))+!B (C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(980[34:73])
    defparam mic_cfg_width_6__I_0_1528_i4_4_lut.init = 16'h8ecf;
    LUT4 i6848_4_lut (.A(mic_cfg_start[2]), .B(n6877), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17696)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6848_4_lut.init = 16'hcac0;
    LUT4 i3_4_lut (.A(pcm_count_spi[0]), .B(pcm_count_spi[3]), .C(pcm_count_spi[2]), 
         .D(pcm_count_spi[1]), .Z(n25035)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut.init = 16'hfffe;
    PFUMX i17034 (.BLUT(n28410), .ALUT(n28893), .C0(pcm_spi_sample_15__N_1381[4]), 
          .Z(n28894));
    DPR16X4C pcm_fifo6 (.DI0(GND_net), .DI1(GND_net), .DI2(pcm_acc_next[0]), 
            .DI3(pcm_acc_next[1]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14159), .RAD0(pcm_spi_sample_15__N_1381[0]), .RAD1(pcm_spi_sample_15__N_1381[1]), 
            .RAD2(pcm_spi_sample_15__N_1381[2]), .RAD3(pcm_spi_sample_15__N_1381[3]), 
            .DO0(n14145), .DO1(n14146), .DO2(n14147), .DO3(n14148));
    defparam pcm_fifo6.initval = "0x0000000000000000";
    LUT4 i6851_3_lut (.A(n17698), .B(mic_cfg_gap[3]), .C(n11928), .Z(n17699)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6851_3_lut.init = 16'hcaca;
    LUT4 i6850_4_lut (.A(mic_cfg_start[3]), .B(n6876), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17698)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6850_4_lut.init = 16'hcac0;
    LUT4 n13_bdd_4_lut_16887 (.A(mic_lo_n_next_6__N_648[3]), .B(mic_lo_n_next_6__N_648[1]), 
         .C(mic_lo_n_next_6__N_648[2]), .D(mic_lo_n_next_6__N_648[4]), .Z(n28627)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B (C (D))))) */ ;
    defparam n13_bdd_4_lut_16887.init = 16'h01ff;
    LUT4 mux_679_i11_3_lut_4_lut (.A(pending_sequence[10]), .B(accepted_sequence_spi[10]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[10])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i11_3_lut_4_lut.init = 16'hacaa;
    LUT4 i10231_3_lut_4_lut (.A(frame_settle[1]), .B(frame_settle[0]), .C(n7_adj_3263), 
         .D(frame_settle[2]), .Z(n3424)) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B !((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(677[33:52])
    defparam i10231_3_lut_4_lut.init = 16'he010;
    LUT4 i2_2_lut (.A(mic_cfg_width[1]), .B(mic_cfg_width[6]), .Z(n9_adj_3292)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i2_2_lut.init = 16'heeee;
    LUT4 i4596_4_lut (.A(ev_sum[7]), .B(staging_q[7]), .C(staging_rd_page_N_2728), 
         .D(ev_in_clear), .Z(n15435)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i4596_4_lut.init = 16'hcac0;
    LUT4 i15339_2_lut_3_lut_4_lut (.A(ev_ch[5]), .B(ev_ch[6]), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(ev_ch_hi_8)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15339_2_lut_3_lut_4_lut.init = 16'h0004;
    LUT4 i832_2_lut (.A(mic_data_1_c), .B(mic_lo_q_r), .Z(n4628)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam i832_2_lut.init = 16'h6666;
    LUT4 i1_2_lut_adj_149 (.A(mic_cfg_width[3]), .B(mic_acc_busy_N_2302), 
         .Z(n8_adj_3293)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_149.init = 16'heeee;
    LUT4 i4_3_lut_4_lut (.A(spi_bit_count_2__N_1425), .B(fpga_cs_n_c), .C(n8_adj_3301), 
         .D(spi_byte_count[8]), .Z(spi_payload_byte)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i4_3_lut_4_lut.init = 16'h0020;
    LUT4 mux_1049_i1_4_lut (.A(n4_adj_3179), .B(n10879), .C(n10893), .D(mic_acc_state[0]), 
         .Z(mic_acc_state_2__N_2347[0])) /* synthesis lut_function=(A (B (C (D)))+!A (B ((D)+!C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(962[9] 982[12])
    defparam mux_1049_i1_4_lut.init = 16'hc505;
    LUT4 i3_2_lut (.A(mic_cfg_width[2]), .B(mic_cfg_width[4]), .Z(n10_adj_3291)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i3_2_lut.init = 16'heeee;
    LUT4 i6853_3_lut (.A(n17700), .B(mic_cfg_gap[4]), .C(n11928), .Z(n17701)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6853_3_lut.init = 16'hcaca;
    LUT4 status_next_bit_N_2770_2__bdd_4_lut_16313_4_lut (.A(status_next_index[2]), 
         .B(n27591), .C(n27603), .D(status_next_index[3]), .Z(n27612)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;
    defparam status_next_bit_N_2770_2__bdd_4_lut_16313_4_lut.init = 16'h55d8;
    LUT4 i1_4_lut_4_lut_then_1_lut (.A(mic_ultrasonic_N_1914), .Z(n28495)) /* synthesis lut_function=(A) */ ;
    defparam i1_4_lut_4_lut_then_1_lut.init = 16'haaaa;
    LUT4 spi_phase_only_N_2792_bdd_4_lut (.A(spi_phase_only_N_2792), .B(n21487), 
         .C(spi_command[3]), .D(n28504), .Z(n21503)) /* synthesis lut_function=(A (B+(C+(D)))+!A (B)) */ ;
    defparam spi_phase_only_N_2792_bdd_4_lut.init = 16'heeec;
    LUT4 n17015_bdd_4_lut (.A(n17015), .B(spi_command[1]), .C(spi_command[0]), 
         .D(n16995), .Z(n21415)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+!(C+(D)))) */ ;
    defparam n17015_bdd_4_lut.init = 16'hbbb8;
    LUT4 mux_679_i14_3_lut_4_lut (.A(pending_sequence[13]), .B(accepted_sequence_spi[13]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[13])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i14_3_lut_4_lut.init = 16'hacaa;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_17 (.A0(mic_cfg_hold_spi[39]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24859), .S0(mic_cfg_gap_w[15]), .S1(mic_cfg_gap_w[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_17.INIT0 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_17.INIT1 = 16'hffff;
    defparam mic_cfg_hold_spi_39__I_0_add_2_17.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_17.INJECT1_1 = "NO";
    DPR16X4C pcm_fifo5 (.DI0(pcm_acc_next[2]), .DI1(pcm_acc_next[3]), .DI2(pcm_acc_next[4]), 
            .DI3(pcm_acc_next[5]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14159), .RAD0(pcm_spi_sample_15__N_1381[0]), .RAD1(pcm_spi_sample_15__N_1381[1]), 
            .RAD2(pcm_spi_sample_15__N_1381[2]), .RAD3(pcm_spi_sample_15__N_1381[3]), 
            .DO0(n14149), .DO1(n14150), .DO2(n14151), .DO3(n14152));
    defparam pcm_fifo5.initval = "0x0000000000000000";
    LUT4 i6852_4_lut (.A(mic_cfg_start[4]), .B(n6875), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17700)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6852_4_lut.init = 16'hcac0;
    FD1P3IX mic_cfg_gap_i0 (.D(mic_cfg_gap_w[0]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i0.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i1 (.D(n5741), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i1.GSR = "DISABLED";
    LUT4 n27804_bdd_4_lut_then_3_lut (.A(n14155), .B(n14156), .C(status_bit_index[0]), 
         .Z(n28393)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27804_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 ev_run_hold_s5_83__I_0_i68_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_67), .D(ev_run_hold_s5[67]), .Z(us_tx_next[67])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i68_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_4_lut_adj_150 (.A(pll_clk_enable_602), .B(mic_run_state_1__N_2410[1]), 
         .C(mic_acc_busy_N_2292), .D(mic_done_N_2656), .Z(pll_clk_enable_34)) /* synthesis lut_function=(A+(B+(C (D)))) */ ;
    defparam i1_2_lut_4_lut_adj_150.init = 16'hfeee;
    LUT4 i6855_3_lut (.A(n17702), .B(mic_cfg_gap[5]), .C(n11928), .Z(n17703)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6855_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut (.A(spi_command[0]), .B(spi_command[1]), .C(n16995), 
         .Z(n16996)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[61:83])
    defparam i1_2_lut_3_lut.init = 16'hfefe;
    LUT4 n27414_bdd_4_lut (.A(n27414), .B(n28451), .C(n28454), .D(status_bit_index[3]), 
         .Z(n27417)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27414_bdd_4_lut.init = 16'haad8;
    LUT4 i3_4_lut_adj_151 (.A(n5689), .B(n100), .C(mic_ultrasonic), .D(n107), 
         .Z(n22773)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i3_4_lut_adj_151.init = 16'h8000;
    FD1P3AX spi_compact_data_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_compact_data[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_compact_data_i0_i1.GSR = "DISABLED";
    LUT4 i15417_2_lut (.A(mic_warm), .B(mic_phase[0]), .Z(n5689)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15417_2_lut.init = 16'h1111;
    LUT4 run_addr_s3_8__I_0_i6_3_lut_4_lut (.A(run_addr_s3[5]), .B(rmw_slot[5]), 
         .C(staging_rd_page_N_2729), .D(phase_step_s3), .Z(event_rd_addr[5])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i6_3_lut_4_lut.init = 16'haaca;
    LUT4 i4894_2_lut (.A(spi_byte_count[0]), .B(n13550), .Z(spi1_sck_c_enable_68)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i4894_2_lut.init = 16'h2222;
    LUT4 i1_3_lut_4_lut (.A(spi_bit_count[0]), .B(n20745), .C(n21415), 
         .D(n28010), .Z(compact_end)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i1_3_lut_4_lut.init = 16'h0008;
    LUT4 run_addr_s3_8__I_0_i5_3_lut_4_lut (.A(run_addr_s3[4]), .B(rmw_slot[4]), 
         .C(staging_rd_page_N_2729), .D(phase_step_s3), .Z(event_rd_addr[4])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i5_3_lut_4_lut.init = 16'haaca;
    LUT4 i6854_4_lut (.A(mic_cfg_start[5]), .B(n6874), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17702)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6854_4_lut.init = 16'hcac0;
    FD1S3AX feed_rd_3077__i0 (.D(n78_adj_3310), .CK(pll_clk), .Q(feed_rd[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077__i0.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_152 (.A(mic_run_state_1__N_2410[1]), .B(pll_clk_enable_104), 
         .Z(n22498)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_152.init = 16'heeee;
    LUT4 i15349_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_6)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15349_2_lut_3_lut_4_lut.init = 16'h0400;
    LUT4 i3375_2_lut (.A(link_idle[1]), .B(link_idle[0]), .Z(n3219)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(656[26:42])
    defparam i3375_2_lut.init = 16'h6666;
    LUT4 i6857_3_lut (.A(n17704), .B(mic_cfg_gap[6]), .C(n11928), .Z(n17705)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6857_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut_adj_153 (.A(mic_lo_n_next_6__N_648[0]), .B(mic_lo_n_next_6__N_648[2]), 
         .C(mic_lo_n_next_6__N_648[1]), .Z(n130)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[17:30])
    defparam i1_2_lut_3_lut_adj_153.init = 16'hfefe;
    PFUMX i4597 (.BLUT(n15435), .ALUT(n26174), .C0(ev_fall_N_2693), .Z(n15436));
    LUT4 i1_2_lut_3_lut_adj_154 (.A(spi_command[0]), .B(spi_command[1]), 
         .C(n16995), .Z(spi_phase_only_N_2792)) /* synthesis lut_function=(((C)+!B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(151[34:64])
    defparam i1_2_lut_3_lut_adj_154.init = 16'hf7f7;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_15 (.A0(mic_cfg_hold_spi[37]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_cfg_hold_spi[38]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24858), .COUT(n24859), .S0(mic_cfg_gap_w[13]), 
          .S1(mic_cfg_gap_w[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_15.INIT0 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_15.INIT1 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_15.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_15.INJECT1_1 = "NO";
    LUT4 i2465_1_lut (.A(mic_gate_active), .Z(mic_gate_active_N_2304)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i2465_1_lut.init = 16'h5555;
    LUT4 i4863_2_lut (.A(spi_byte_count[0]), .B(n13560), .Z(spi1_sck_c_enable_83)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i4863_2_lut.init = 16'h2222;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_13 (.A0(mic_cfg_hold_spi[35]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_cfg_hold_spi[36]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24857), .COUT(n24858), .S0(mic_cfg_gap_w[11]), 
          .S1(mic_cfg_gap_w[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_13.INIT0 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_13.INIT1 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_13.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_13.INJECT1_1 = "NO";
    LUT4 n27804_bdd_4_lut_else_3_lut (.A(n14139), .B(n14140), .C(status_bit_index[0]), 
         .Z(n28392)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27804_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 run_addr_s3_8__I_0_i4_3_lut_4_lut (.A(run_addr_s3[3]), .B(rmw_slot[3]), 
         .C(staging_rd_page_N_2729), .D(phase_step_s3), .Z(event_rd_addr[3])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i4_3_lut_4_lut.init = 16'haaca;
    LUT4 run_addr_s3_8__I_0_i3_3_lut_4_lut (.A(run_addr_s3[2]), .B(rmw_slot[2]), 
         .C(staging_rd_page_N_2729), .D(phase_step_s3), .Z(event_rd_addr[2])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i3_3_lut_4_lut.init = 16'haaca;
    LUT4 n10833_bdd_3_lut_4_lut (.A(mic_warm), .B(mic_ultrasonic), .C(mic_run_state_1__N_2410[1]), 
         .D(n28153), .Z(pll_clk_enable_117)) /* synthesis lut_function=(A (C+(D))+!A (B (C+(D))+!B (C))) */ ;
    defparam n10833_bdd_3_lut_4_lut.init = 16'hfef0;
    LUT4 mic_cfg_count_0__bdd_3_lut (.A(mic_cfg_count[4]), .B(status_bit_index[1]), 
         .C(mic_cfg_count[6]), .Z(n29104)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam mic_cfg_count_0__bdd_3_lut.init = 16'hb8b8;
    LUT4 i6856_4_lut (.A(mic_cfg_start[6]), .B(n6873), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17704)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6856_4_lut.init = 16'hcac0;
    DPR16X4C pcm_fifo4 (.DI0(GND_net), .DI1(GND_net), .DI2(GND_net), .DI3(GND_net), 
            .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), .WAD2(pcm_wr_ptr[2]), 
            .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), .WRE(n14158), .RAD0(pcm_spi_sample_15__N_1381[0]), 
            .RAD1(pcm_spi_sample_15__N_1381[1]), .RAD2(pcm_spi_sample_15__N_1381[2]), 
            .RAD3(pcm_spi_sample_15__N_1381[3]), .DO0(n14125), .DO1(n14126), 
            .DO2(n14127), .DO3(n14128));
    defparam pcm_fifo4.initval = "0x0000000000000000";
    LUT4 i1_2_lut_adj_155 (.A(ws2812_enable_N_2606), .B(ws2812_settle_3__N_1717), 
         .Z(pll_clk_enable_250)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_155.init = 16'heeee;
    LUT4 ws2812_settle_1__bdd_2_lut (.A(ws2812_settle[1]), .B(n28569), .Z(n10728)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam ws2812_settle_1__bdd_2_lut.init = 16'h4444;
    LUT4 i15266_4_lut (.A(n26145), .B(status_next_index[0]), .C(n26146), 
         .D(status_next_bit_N_2770[2]), .Z(n26137)) /* synthesis lut_function=(A (B (C+!(D)))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam i15266_4_lut.init = 16'hc088;
    LUT4 n27792_bdd_4_lut_then_3_lut (.A(n14141), .B(n14142), .C(status_bit_index[0]), 
         .Z(n28396)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27792_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 i6859_3_lut (.A(n17706), .B(mic_cfg_gap[7]), .C(n11928), .Z(n17707)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6859_3_lut.init = 16'hcaca;
    DPR16X4C pcm_fifo3 (.DI0(GND_net), .DI1(GND_net), .DI2(pcm_acc_next[0]), 
            .DI3(pcm_acc_next[1]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14158), .RAD0(pcm_spi_sample_15__N_1381[0]), .RAD1(pcm_spi_sample_15__N_1381[1]), 
            .RAD2(pcm_spi_sample_15__N_1381[2]), .RAD3(pcm_spi_sample_15__N_1381[3]), 
            .DO0(n14129), .DO1(n14130), .DO2(n14131), .DO3(n14132));
    defparam pcm_fifo3.initval = "0x0000000000000000";
    PFUMX i4595 (.BLUT(n15433), .ALUT(n26172), .C0(ev_fall_N_2693), .Z(n15434));
    DPR16X4C pcm_fifo2 (.DI0(pcm_acc_next[2]), .DI1(pcm_acc_next[3]), .DI2(pcm_acc_next[4]), 
            .DI3(pcm_acc_next[5]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14158), .RAD0(pcm_spi_sample_15__N_1381[0]), .RAD1(pcm_spi_sample_15__N_1381[1]), 
            .RAD2(pcm_spi_sample_15__N_1381[2]), .RAD3(pcm_spi_sample_15__N_1381[3]), 
            .DO0(n14133), .DO1(n14134), .DO2(n14135), .DO3(n14136));
    defparam pcm_fifo2.initval = "0x0000000000000000";
    LUT4 i2_2_lut_adj_156 (.A(mic_phase[1]), .B(mic_phase[2]), .Z(n6)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i2_2_lut_adj_156.init = 16'hbbbb;
    LUT4 run_addr_s3_8__I_0_i2_3_lut_4_lut (.A(run_addr_s3[1]), .B(rmw_slot[1]), 
         .C(staging_rd_page_N_2729), .D(phase_step_s3), .Z(event_rd_addr[1])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i2_3_lut_4_lut.init = 16'haaca;
    LUT4 i1_2_lut_4_lut_adj_157 (.A(n13_adj_3191), .B(n17018), .C(spi_command[3]), 
         .D(spi_command[2]), .Z(n17015)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(174[38:60])
    defparam i1_2_lut_4_lut_adj_157.init = 16'hfeff;
    LUT4 i1_3_lut (.A(mic_phase[2]), .B(n17307), .C(n25901), .Z(pll_clk_enable_586)) /* synthesis lut_function=(A (B (C))+!A (B)) */ ;
    defparam i1_3_lut.init = 16'hc4c4;
    LUT4 run_addr_s3_8__I_0_i1_3_lut_4_lut (.A(run_addr_s3[0]), .B(rmw_slot[0]), 
         .C(staging_rd_page_N_2729), .D(phase_step_s3), .Z(event_rd_addr[0])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i1_3_lut_4_lut.init = 16'haaca;
    LUT4 n27660_bdd_4_lut (.A(n27660), .B(\mic_win_q[2] [0]), .C(\mic_win_i[2] [0]), 
         .D(mic_acc_word[0]), .Z(n27663)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27660_bdd_4_lut.init = 16'haad8;
    LUT4 active_bank_I_0_1_lut (.A(active_bank), .Z(active_bank_N_898)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(333[34:46])
    defparam active_bank_I_0_1_lut.init = 16'h5555;
    LUT4 i2_3_lut_4_lut (.A(n4_adj_3264), .B(mic_warm), .C(mic_phase[0]), 
         .D(mic_phase[3]), .Z(pll_clk_enable_370)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i2_3_lut_4_lut.init = 16'h0200;
    LUT4 i6858_4_lut (.A(mic_cfg_start[7]), .B(n6872), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17706)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6858_4_lut.init = 16'hcac0;
    LUT4 i2_3_lut_4_lut_adj_158 (.A(mic_warm), .B(mic_phase[0]), .C(n4_adj_3264), 
         .D(mic_phase[3]), .Z(pll_clk_enable_342)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i2_3_lut_4_lut_adj_158.init = 16'h0010;
    LUT4 i1_3_lut_3_lut_4_lut (.A(spi_command[0]), .B(spi_command[1]), .C(n13_adj_3167), 
         .D(n13_adj_3191), .Z(n25844)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i1_3_lut_3_lut_4_lut.init = 16'h0040;
    LUT4 i6861_3_lut (.A(n17708), .B(mic_cfg_gap[8]), .C(n11928), .Z(n17709)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6861_3_lut.init = 16'hcaca;
    LUT4 n27792_bdd_4_lut_else_3_lut (.A(n14125), .B(n14126), .C(status_bit_index[0]), 
         .Z(n28395)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27792_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 i15263_3_lut (.A(n15211), .B(status_next_bit_N_2770[2]), .C(status_next_index[0]), 
         .Z(n26136)) /* synthesis lut_function=(A (B)+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam i15263_3_lut.init = 16'hc8c8;
    LUT4 mic_cfg_count_1__bdd_3_lut (.A(mic_cfg_count[1]), .B(mic_cfg_count[3]), 
         .C(status_bit_index[1]), .Z(n29106)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam mic_cfg_count_1__bdd_3_lut.init = 16'hacac;
    LUT4 i2828_2_lut_3_lut (.A(build_idle_N_2724), .B(frame_req), .C(swap_pending), 
         .Z(n13249)) /* synthesis lut_function=(A ((C)+!B)) */ ;
    defparam i2828_2_lut_3_lut.init = 16'ha2a2;
    LUT4 i1_2_lut_3_lut_adj_159 (.A(mic_warm), .B(mic_ultrasonic_N_1914), 
         .C(n28), .Z(mic_warm_N_1918)) /* synthesis lut_function=(!((B (C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_3_lut_adj_159.init = 16'h2a2a;
    LUT4 spi_byte_count_6__bdd_4_lut_17056 (.A(spi_byte_count[6]), .B(spi_command[2]), 
         .C(spi_command[1]), .D(n28933), .Z(n28934)) /* synthesis lut_function=(!(A (B+!(C))+!A (B+!(C (D))))) */ ;
    defparam spi_byte_count_6__bdd_4_lut_17056.init = 16'h3020;
    LUT4 i2_4_lut_adj_160 (.A(n22498), .B(n19280), .C(pll_clk_enable_266), 
         .D(n25867), .Z(n17307)) /* synthesis lut_function=(A+(B (C+(D))+!B (C))) */ ;
    defparam i2_4_lut_adj_160.init = 16'hfefa;
    LUT4 i4104_2_lut_3_lut (.A(phase_step_s2), .B(wrap_s2), .C(swap_pending), 
         .Z(pll_clk_enable_290)) /* synthesis lut_function=(A+(B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(633[18] 638[12])
    defparam i4104_2_lut_3_lut.init = 16'heaea;
    LUT4 i15399_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_10)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15399_2_lut_3_lut_4_lut.init = 16'h0040;
    LUT4 i6860_4_lut (.A(mic_cfg_start[8]), .B(n6871), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17708)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6860_4_lut.init = 16'hcac0;
    LUT4 mic_cfg_count_1__bdd_2_lut (.A(mic_cfg_count[5]), .B(status_bit_index[1]), 
         .Z(n29107)) /* synthesis lut_function=(A (B)) */ ;
    defparam mic_cfg_count_1__bdd_2_lut.init = 16'h8888;
    LUT4 i2_3_lut_4_lut_adj_161 (.A(mic_run_state[0]), .B(mic_run_state[1]), 
         .C(mic_acc_busy), .D(mic_bc_valid), .Z(mic_acc_gate_6__N_2283)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(273[37:60])
    defparam i2_3_lut_4_lut_adj_161.init = 16'h0400;
    LUT4 i15369_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(ev_ch_hi_2)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[45:73])
    defparam i15369_2_lut_3_lut_4_lut.init = 16'h0004;
    LUT4 i6863_3_lut (.A(n17710), .B(mic_cfg_gap[9]), .C(n11928), .Z(n17711)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6863_3_lut.init = 16'hcaca;
    LUT4 ev_run_hold_s5_83__I_0_i42_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_41), .D(ev_run_hold_s5[41]), .Z(us_tx_next[41])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i42_3_lut_4_lut.init = 16'hbf40;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_11 (.A0(mic_cfg_hold_spi[33]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_cfg_hold_spi[34]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24856), .COUT(n24857), .S0(mic_cfg_gap_w[9]), 
          .S1(mic_cfg_gap_w[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_11.INIT0 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_11.INIT1 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_11.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_11.INJECT1_1 = "NO";
    LUT4 n9_bdd_2_lut (.A(mic_ultrasonic), .B(mic_warm), .Z(n29123)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam n9_bdd_2_lut.init = 16'h1111;
    LUT4 i6862_4_lut (.A(mic_cfg_start[9]), .B(n6870), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17710)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6862_4_lut.init = 16'hcac0;
    LUT4 active_bank_I_0_1370_2_lut_3_lut (.A(active_bank), .B(wrap_s2), 
         .C(swap_pending), .Z(active_bank_N_899)) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(633[18] 638[12])
    defparam active_bank_I_0_1370_2_lut_3_lut.init = 16'h6a6a;
    DPR16X4C pcm_fifo0 (.DI0(pcm_acc_next[6]), .DI1(pcm_acc_next[7]), .DI2(pcm_acc_next[8]), 
            .DI3(pcm_acc_next[9]), .WAD0(pcm_wr_ptr[0]), .WAD1(pcm_wr_ptr[1]), 
            .WAD2(pcm_wr_ptr[2]), .WAD3(pcm_wr_ptr[3]), .WCK(pll_clk), 
            .WRE(n14159), .RAD0(pcm_spi_sample_15__N_1381[0]), .RAD1(pcm_spi_sample_15__N_1381[1]), 
            .RAD2(pcm_spi_sample_15__N_1381[2]), .RAD3(pcm_spi_sample_15__N_1381[3]), 
            .DO0(n14153), .DO1(n14154), .DO2(n14155), .DO3(n14156));
    defparam pcm_fifo0.initval = "0x0000000000000000";
    FD1S3AX mem (.D(spi_rx_shift[6]), .CK(spi1_sck_c), .Q(n14121));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem.GSR = "DISABLED";
    FD1S3AX mem_3341 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(n14119));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3341.GSR = "DISABLED";
    FD1S3AX mem_3340 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(n14117));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3340.GSR = "DISABLED";
    FD1S3AX mem_3339 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(n14115));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3339.GSR = "DISABLED";
    FD1S3AX mem_3338 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(n14113));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3338.GSR = "DISABLED";
    FD1S3AX mem_3337 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(n14111));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3337.GSR = "DISABLED";
    FD1S3AX mem_3336 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(n14109));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3336.GSR = "DISABLED";
    FD1S3AX mem_3335 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(n14107));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mem_3335.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_adj_162 (.A(swap_pending), .B(build_idle_N_2724), 
         .C(frame_req), .Z(fifo_credit_wire[0])) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i1_2_lut_3_lut_adj_162.init = 16'h0404;
    LUT4 i4590_4_lut (.A(ev_sum[4]), .B(staging_q[4]), .C(staging_rd_page_N_2728), 
         .D(ev_in_clear), .Z(n15429)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i4590_4_lut.init = 16'hcac0;
    LUT4 i1630_1_lut_3_lut (.A(swap_pending), .B(build_idle_N_2724), .C(frame_req), 
         .Z(fifo_depth_wire[0])) /* synthesis lut_function=(A+((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[37:63])
    defparam i1630_1_lut_3_lut.init = 16'hfbfb;
    PFUMX spi1_miso_I_28_i63 (.BLUT(n62), .ALUT(n26129), .C0(status_bit_index[5]), 
          .Z(spi1_miso_N_2462));
    LUT4 i1_3_lut_adj_163 (.A(mic_phase[1]), .B(mic_phase[2]), .C(mic_ultrasonic), 
         .Z(n4_adj_3264)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_3_lut_adj_163.init = 16'h4040;
    LUT4 frame_settle_3__bdd_4_lut (.A(frame_settle[3]), .B(frame_settle[1]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(n28060)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;
    defparam frame_settle_3__bdd_4_lut.init = 16'haaa9;
    LUT4 i10024_2_lut_3_lut (.A(spi_bit_count[0]), .B(spi_bit_count[2]), 
         .C(spi_bit_count[1]), .Z(spi_bit_count_2__N_1425)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i10024_2_lut_3_lut.init = 16'h8080;
    LUT4 n27678_bdd_4_lut_16719 (.A(n27678), .B(n27609), .C(n27242), .D(status_next_bit_N_2770[3]), 
         .Z(n26921)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27678_bdd_4_lut_16719.init = 16'haad8;
    LUT4 i3389_2_lut_4_lut (.A(link_idle[3]), .B(link_idle[0]), .C(link_idle[1]), 
         .D(link_idle[2]), .Z(n3217)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(656[26:42])
    defparam i3389_2_lut_4_lut.init = 16'h6aaa;
    LUT4 ev_run_hold_s5_83__I_0_i41_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_40), .D(ev_run_hold_s5[40]), .Z(us_tx_next[40])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i41_3_lut_4_lut.init = 16'hbf40;
    FD1S3AX mem_3334 (.D(feed_we), .CK(pll_clk), .Q(n14104));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3334.GSR = "DISABLED";
    PFUMX i16972 (.BLUT(n28788), .ALUT(n28787), .C0(mic_acc_state[0]), 
          .Z(n28789));
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_9 (.A0(mic_cfg_hold_spi[31]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_cfg_hold_spi[32]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24855), .COUT(n24856), .S0(mic_cfg_gap_w[7]), 
          .S1(mic_cfg_gap_w[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_9.INIT0 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_9.INIT1 = 16'h5555;
    defparam mic_cfg_hold_spi_39__I_0_add_2_9.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_9.INJECT1_1 = "NO";
    FD1S3AX mem_3332 (.D(n69), .CK(pll_clk), .Q(n14102));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3332.GSR = "DISABLED";
    FD1S3AX mem_3329 (.D(n70_adj_3308), .CK(pll_clk), .Q(n14100));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3329.GSR = "DISABLED";
    L6MUX21 i17036 (.D0(n28394), .D1(n28894), .SD(status_bit_index[2]), 
            .Z(n28895));
    FD1S3AX mem_3317 (.D(n76_adj_3312), .CK(pll_clk), .Q(n14088));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3317.GSR = "DISABLED";
    FD1S3AX mem_3319 (.D(n75_adj_3313), .CK(pll_clk), .Q(n14090));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3319.GSR = "DISABLED";
    LUT4 i6865_3_lut (.A(n17712), .B(mic_cfg_gap[10]), .C(n11928), .Z(n17713)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6865_3_lut.init = 16'hcaca;
    LUT4 mic_acc_state_0__bdd_4_lut_16578 (.A(mic_acc_state[0]), .B(n20918), 
         .C(n5_adj_3259), .D(n3), .Z(n25163)) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A (B+(C+!(D)))) */ ;
    defparam mic_acc_state_0__bdd_4_lut_16578.init = 16'hfc5d;
    LUT4 wrap_s2_I_0_1574_2_lut (.A(wrap_s2), .B(swap_pending), .Z(active_bank_N_900)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(626[13:36])
    defparam wrap_s2_I_0_1574_2_lut.init = 16'h8888;
    LUT4 ev_run_hold_s5_83__I_0_i40_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_39), .D(ev_run_hold_s5[39]), .Z(us_tx_next[39])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i40_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_3_lut_adj_164 (.A(pll_clk_enable_602), .B(mic_acc_busy_N_2292), 
         .C(mic_done_N_2656), .Z(pll_clk_enable_253)) /* synthesis lut_function=(A+(B (C))) */ ;
    defparam i1_2_lut_3_lut_adj_164.init = 16'heaea;
    LUT4 i1_2_lut_4_lut_adj_165 (.A(spi_command[7]), .B(spi_command[6]), 
         .C(spi_command[5]), .D(spi_command[4]), .Z(n14_adj_3258)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(174[38:60])
    defparam i1_2_lut_4_lut_adj_165.init = 16'hfeff;
    LUT4 i1_2_lut_adj_166 (.A(spi_command[3]), .B(spi_command[2]), .Z(n25904)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(174[38:60])
    defparam i1_2_lut_adj_166.init = 16'hbbbb;
    LUT4 ev_run_hold_s5_83__I_0_i39_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_38), .D(ev_run_hold_s5[38]), .Z(us_tx_next[38])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i39_3_lut_4_lut.init = 16'hbf40;
    PFUMX i4593 (.BLUT(n15431), .ALUT(n26170), .C0(ev_fall_N_2693), .Z(n15432));
    LUT4 mic_acc_word_2__bdd_4_lut_16323 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [0]), 
         .C(\mic_win_q[3] [0]), .D(mic_acc_word[0]), .Z(n27660)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16323.init = 16'he4aa;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_7 (.A0(mic_cfg_hold_spi[29]), .B0(mic_cfg_hold_spi[45]), 
          .C0(n16834), .D0(mic_cfg_hold_spi[46]), .A1(mic_cfg_hold_spi[30]), 
          .B1(mic_cfg_hold_spi[46]), .C1(GND_net), .D1(GND_net), .CIN(n24854), 
          .COUT(n24855), .S0(mic_cfg_gap_w[5]), .S1(mic_cfg_gap_w[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_7.INIT0 = 16'h5995;
    defparam mic_cfg_hold_spi_39__I_0_add_2_7.INIT1 = 16'h5999;
    defparam mic_cfg_hold_spi_39__I_0_add_2_7.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_7.INJECT1_1 = "NO";
    CCU2D mic_block_count_3084_add_4_17 (.A0(mic_block_count[15]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24751), .S0(n70));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084_add_4_17.INIT0 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_17.INIT1 = 16'h0000;
    defparam mic_block_count_3084_add_4_17.INJECT1_0 = "NO";
    defparam mic_block_count_3084_add_4_17.INJECT1_1 = "NO";
    FD1S3AX mem_3321 (.D(n74), .CK(pll_clk), .Q(n14092));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3321.GSR = "DISABLED";
    FD1S3AX mem_3323 (.D(n73_adj_3288), .CK(pll_clk), .Q(n14094));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3323.GSR = "DISABLED";
    FD1S3IX swap_pending_1260 (.D(swap_pending_N_2717), .CK(pll_clk), .CD(n10742), 
            .Q(swap_pending)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam swap_pending_1260.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i38_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_37), .D(ev_run_hold_s5[37]), .Z(us_tx_next[37])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i38_3_lut_4_lut.init = 16'hbf40;
    LUT4 i6864_4_lut (.A(mic_cfg_start[10]), .B(n6869), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17712)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6864_4_lut.init = 16'hcac0;
    LUT4 i1_2_lut_3_lut_adj_167 (.A(spi_command[0]), .B(spi_command[1]), 
         .C(n16995), .Z(spi_mod_feed_N_2490)) /* synthesis lut_function=(A+((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(173[32:61])
    defparam i1_2_lut_3_lut_adj_167.init = 16'hfbfb;
    LUT4 ev_run_hold_s5_83__I_0_i37_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_36), .D(ev_run_hold_s5[36]), .Z(us_tx_next[36])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i37_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27690_bdd_4_lut (.A(n27690), .B(\mic_win_q[2] [2]), .C(\mic_win_i[2] [2]), 
         .D(mic_acc_word[0]), .Z(n27693)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27690_bdd_4_lut.init = 16'haad8;
    LUT4 i2146_2_lut_3_lut (.A(mic_acc_busy_N_2302), .B(mic_gate_active), 
         .C(mic_acc_gate_6__N_2283), .Z(n10893)) /* synthesis lut_function=(!(A (B (C))+!A (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(975[22] 981[16])
    defparam i2146_2_lut_3_lut.init = 16'h2f2f;
    LUT4 i6867_3_lut (.A(n17714), .B(mic_cfg_gap[11]), .C(n11928), .Z(n17715)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6867_3_lut.init = 16'hcaca;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_5 (.A0(mic_cfg_hold_spi[27]), .B0(mic_cfg_hold_spi[43]), 
          .C0(n16834), .D0(mic_cfg_hold_spi[46]), .A1(mic_cfg_hold_spi[28]), 
          .B1(mic_cfg_hold_spi[44]), .C1(n16834), .D1(mic_cfg_hold_spi[46]), 
          .CIN(n24853), .COUT(n24854), .S0(mic_cfg_gap_w[3]), .S1(mic_cfg_gap_w[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_5.INIT0 = 16'h5995;
    defparam mic_cfg_hold_spi_39__I_0_add_2_5.INIT1 = 16'h5995;
    defparam mic_cfg_hold_spi_39__I_0_add_2_5.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_5.INJECT1_1 = "NO";
    FD1P3IX ev_ch_3078__i6 (.D(n34_adj_3204), .SP(pll_clk_enable_168), .CD(n17806), 
            .CK(pll_clk), .Q(ev_ch[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078__i6.GSR = "DISABLED";
    FD1P3IX ev_ch_3078__i5 (.D(n35_adj_3203), .SP(pll_clk_enable_168), .CD(n17806), 
            .CK(pll_clk), .Q(ev_ch[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078__i5.GSR = "DISABLED";
    FD1P3IX ev_ch_3078__i4 (.D(n36), .SP(pll_clk_enable_168), .CD(n17806), 
            .CK(pll_clk), .Q(ev_ch[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078__i4.GSR = "DISABLED";
    FD1P3IX ev_ch_3078__i3 (.D(n37), .SP(pll_clk_enable_168), .CD(n17806), 
            .CK(pll_clk), .Q(ev_ch[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078__i3.GSR = "DISABLED";
    FD1P3IX ev_ch_3078__i2 (.D(n38_adj_3202), .SP(pll_clk_enable_168), .CD(n17806), 
            .CK(pll_clk), .Q(ev_ch[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078__i2.GSR = "DISABLED";
    FD1P3IX ev_ch_3078__i1 (.D(n39_adj_3201), .SP(pll_clk_enable_168), .CD(n17806), 
            .CK(pll_clk), .Q(ev_ch[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078__i1.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3079__i7 (.D(n38_adj_3214), .SP(pll_clk_enable_169), 
            .CD(n20483), .CK(pll_clk), .Q(mic_warm_half[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079__i7.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3079__i6 (.D(n39_adj_3213), .SP(pll_clk_enable_169), 
            .CD(n20483), .CK(pll_clk), .Q(mic_warm_half[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079__i6.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3079__i5 (.D(n40_adj_3212), .SP(pll_clk_enable_169), 
            .CD(n20483), .CK(pll_clk), .Q(mic_warm_half[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079__i5.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3079__i4 (.D(n41_adj_3211), .SP(pll_clk_enable_169), 
            .CD(n20483), .CK(pll_clk), .Q(mic_warm_half[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079__i4.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3079__i3 (.D(n42_adj_3210), .SP(pll_clk_enable_169), 
            .CD(n20483), .CK(pll_clk), .Q(mic_warm_half[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079__i3.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3079__i2 (.D(n43_adj_3209), .SP(pll_clk_enable_169), 
            .CD(n20483), .CK(pll_clk), .Q(mic_warm_half[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079__i2.GSR = "DISABLED";
    FD1P3IX mic_warm_half_3079__i1 (.D(n44_adj_3208), .SP(pll_clk_enable_169), 
            .CD(n20483), .CK(pll_clk), .Q(mic_warm_half[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079__i1.GSR = "DISABLED";
    LUT4 i6866_4_lut (.A(mic_cfg_start[11]), .B(n6868), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17714)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6866_4_lut.init = 16'hcac0;
    LUT4 i6877_4_lut (.A(mic_cfg_start[0]), .B(n6879), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17725)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6877_4_lut.init = 16'hcac0;
    LUT4 ws2812_settle_3__bdd_4_lut (.A(ws2812_settle[3]), .B(ws2812_settle[1]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(n27914)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;
    defparam ws2812_settle_3__bdd_4_lut.init = 16'haaa9;
    LUT4 mic_acc_word_2__bdd_4_lut_16328 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [2]), 
         .C(\mic_win_q[3] [2]), .D(mic_acc_word[0]), .Z(n27690)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16328.init = 16'he4aa;
    FD1S3AX mem_3325 (.D(n72), .CK(pll_clk), .Q(n14096));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam mem_3325.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i36_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_35), .D(ev_run_hold_s5[35]), .Z(us_tx_next[35])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i36_3_lut_4_lut.init = 16'hbf40;
    LUT4 n16190_bdd_4_lut (.A(n16190), .B(ws2812_enable_N_2601), .C(frame_req_mod_N_2752), 
         .D(n28562), .Z(pll_clk_enable_247)) /* synthesis lut_function=(A+(B (C+(D))+!B (C))) */ ;
    defparam n16190_bdd_4_lut.init = 16'hfefa;
    FD1P3IX build_level_eff_i0_i7 (.D(event_level[7]), .SP(pll_clk_enable_72), 
            .CD(n17862), .CK(pll_clk), .Q(build_level_eff[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_eff_i0_i7.GSR = "DISABLED";
    FD1P3IX build_level_eff_i0_i6 (.D(event_level[6]), .SP(pll_clk_enable_72), 
            .CD(n17862), .CK(pll_clk), .Q(build_level_eff[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_eff_i0_i6.GSR = "DISABLED";
    FD1P3IX build_level_eff_i0_i5 (.D(event_level[5]), .SP(pll_clk_enable_72), 
            .CD(n17862), .CK(pll_clk), .Q(build_level_eff[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_eff_i0_i5.GSR = "DISABLED";
    FD1P3IX build_level_eff_i0_i4 (.D(event_level[4]), .SP(pll_clk_enable_72), 
            .CD(n17862), .CK(pll_clk), .Q(build_level_eff[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_eff_i0_i4.GSR = "DISABLED";
    FD1P3IX build_level_eff_i0_i3 (.D(event_level[3]), .SP(pll_clk_enable_72), 
            .CD(n17862), .CK(pll_clk), .Q(build_level_eff[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_eff_i0_i3.GSR = "DISABLED";
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_3 (.A0(mic_cfg_hold_spi[25]), .B0(mic_cfg_hold_spi[41]), 
          .C0(n16834), .D0(mic_cfg_hold_spi[46]), .A1(mic_cfg_hold_spi[26]), 
          .B1(mic_cfg_hold_spi[42]), .C1(n16834), .D1(mic_cfg_hold_spi[46]), 
          .CIN(n24852), .COUT(n24853), .S0(mic_cfg_gap_w[1]), .S1(mic_cfg_gap_w[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_3.INIT0 = 16'h5995;
    defparam mic_cfg_hold_spi_39__I_0_add_2_3.INIT1 = 16'h5995;
    defparam mic_cfg_hold_spi_39__I_0_add_2_3.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_3.INJECT1_1 = "NO";
    LUT4 mux_679_i10_3_lut_4_lut (.A(pending_sequence[9]), .B(accepted_sequence_spi[9]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[9])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i10_3_lut_4_lut.init = 16'hacaa;
    FD1P3IX build_level_eff_i0_i2 (.D(event_level[2]), .SP(pll_clk_enable_72), 
            .CD(n17862), .CK(pll_clk), .Q(build_level_eff[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_eff_i0_i2.GSR = "DISABLED";
    FD1P3IX build_level_eff_i0_i1 (.D(event_level[1]), .SP(pll_clk_enable_72), 
            .CD(n17862), .CK(pll_clk), .Q(build_level_eff[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_eff_i0_i1.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i35_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_34), .D(ev_run_hold_s5[34]), .Z(us_tx_next[34])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i35_3_lut_4_lut.init = 16'hbf40;
    LUT4 i6869_3_lut (.A(n17716), .B(mic_cfg_gap[12]), .C(n11928), .Z(n17717)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6869_3_lut.init = 16'hcaca;
    FD1P3IX mic_acc_word_i2 (.D(n7025), .SP(pll_clk_enable_267), .CD(n26048), 
            .CK(pll_clk), .Q(mic_acc_word[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_word_i2.GSR = "DISABLED";
    LUT4 i10152_4_lut (.A(ev_rd_data[48]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[48])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10152_4_lut.init = 16'h1222;
    LUT4 i6868_4_lut (.A(mic_cfg_start[12]), .B(n6867), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17716)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6868_4_lut.init = 16'hcac0;
    LUT4 i6871_3_lut (.A(n17718), .B(mic_cfg_gap[13]), .C(n11928), .Z(n17719)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6871_3_lut.init = 16'hcaca;
    FD1P3IX mic_acc_word_i1 (.D(n7026), .SP(pll_clk_enable_267), .CD(n26048), 
            .CK(pll_clk), .Q(mic_acc_word[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_word_i1.GSR = "DISABLED";
    LUT4 n14143_bdd_3_lut (.A(n14143), .B(n14144), .C(status_bit_index[0]), 
         .Z(n28683)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n14143_bdd_3_lut.init = 16'hacac;
    LUT4 i4424_2_lut_4_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(link_timeout), .D(stop_event), .Z(n15263)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i4424_2_lut_4_lut.init = 16'hfff6;
    LUT4 i6870_4_lut (.A(mic_cfg_start[13]), .B(n6866), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17718)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6870_4_lut.init = 16'hcac0;
    LUT4 n16190_bdd_4_lut_16850 (.A(n7_adj_3263), .B(mod_active), .C(frame_toggle_seen), 
         .D(frame_toggle_sync), .Z(n28562)) /* synthesis lut_function=(!(A+(B+!(C (D)+!C !(D))))) */ ;
    defparam n16190_bdd_4_lut_16850.init = 16'h1001;
    LUT4 i1_2_lut_adj_168 (.A(n25072), .B(n10659), .Z(spi1_sck_c_enable_54)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_168.init = 16'h4444;
    LUT4 i3257_1_lut (.A(spi_extension_length[0]), .Z(n2)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(561[34:64])
    defparam i3257_1_lut.init = 16'h5555;
    FD1P3IX mic_ram_wr_data_i0_i15 (.D(mic_ram_wr_data_15__N_2316[15]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i15.GSR = "DISABLED";
    LUT4 i6873_3_lut (.A(n17720), .B(mic_cfg_gap[14]), .C(n11928), .Z(n17721)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6873_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_adj_169 (.A(spi_extension_length[1]), .B(spi_extension_length[0]), 
         .Z(n16268)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_169.init = 16'h9999;
    LUT4 i6872_4_lut (.A(mic_cfg_start[14]), .B(n6865), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17720)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6872_4_lut.init = 16'hcac0;
    FD1P3IX mic_ram_wr_data_i0_i14 (.D(mic_ram_wr_data_15__N_2316[14]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i14.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i13 (.D(mic_ram_wr_data_15__N_2316[13]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i13.GSR = "DISABLED";
    LUT4 i6875_3_lut (.A(n17722), .B(mic_cfg_gap[15]), .C(n11928), .Z(n17723)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6875_3_lut.init = 16'hcaca;
    LUT4 mux_679_i9_3_lut_4_lut (.A(pending_sequence[8]), .B(accepted_sequence_spi[8]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[8])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i9_3_lut_4_lut.init = 16'hacaa;
    LUT4 i6874_4_lut (.A(mic_cfg_start[15]), .B(n6864), .C(n17689), .D(mic_run_state_1__N_2410[1]), 
         .Z(n17722)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i6874_4_lut.init = 16'hcac0;
    FD1P3IX mic_ram_wr_data_i0_i12 (.D(mic_ram_wr_data_15__N_2316[12]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i12.GSR = "DISABLED";
    LUT4 i15317_2_lut (.A(mic_cfg_settle[1]), .B(mic_cfg_settle[0]), .Z(n26302)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(285[17:31])
    defparam i15317_2_lut.init = 16'h9999;
    FD1P3IX mic_ram_wr_data_i0_i11 (.D(mic_ram_wr_data_15__N_2316[11]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i11.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i10 (.D(mic_ram_wr_data_15__N_2316[10]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i10.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i9 (.D(mic_ram_wr_data_15__N_2316[9]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i9.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i8 (.D(mic_ram_wr_data_15__N_2316[8]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i8.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i7 (.D(mic_ram_wr_data_15__N_2316[7]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i7.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i6 (.D(mic_ram_wr_data_15__N_2316[6]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i6.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i5 (.D(mic_ram_wr_data_15__N_2316[5]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i5.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i34_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_33), .D(ev_run_hold_s5[33]), .Z(us_tx_next[33])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i34_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_adj_170 (.A(mic_phase[3]), .B(n25788), .Z(n107)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[17:26])
    defparam i1_2_lut_adj_170.init = 16'h8888;
    LUT4 i4_4_lut_adj_171 (.A(pcm_decim_count[5]), .B(n23777), .C(pcm_decim_count[2]), 
         .D(n6_adj_3276), .Z(n25788)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;
    defparam i4_4_lut_adj_171.init = 16'h0200;
    LUT4 n27696_bdd_4_lut (.A(n27696), .B(\mic_win_q[2] [6]), .C(\mic_win_i[2] [6]), 
         .D(mic_acc_word[0]), .Z(n27699)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27696_bdd_4_lut.init = 16'haad8;
    LUT4 i1_2_lut_adj_172 (.A(pcm_decim_count[1]), .B(pcm_decim_count[0]), 
         .Z(n23777)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam i1_2_lut_adj_172.init = 16'hbbbb;
    LUT4 i1_4_lut_adj_173 (.A(pcm_decim_count[6]), .B(pcm_decim_count[4]), 
         .C(pcm_decim_count[7]), .D(pcm_decim_count[3]), .Z(n6_adj_3276)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i1_4_lut_adj_173.init = 16'h8000;
    LUT4 i15333_3_lut_4_lut (.A(spi_byte_count[0]), .B(spi_byte_count[1]), 
         .C(n8), .D(spi_byte_count[2]), .Z(spi1_sck_c_enable_89)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15333_3_lut_4_lut.init = 16'h1000;
    LUT4 i15321_4_lut (.A(mic_cfg_settle[3]), .B(mic_cfg_settle[2]), .C(mic_cfg_settle[1]), 
         .D(mic_cfg_settle[0]), .Z(n26306)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(285[17:31])
    defparam i15321_4_lut.init = 16'haaa9;
    LUT4 i1_2_lut_adj_174 (.A(mic_phase[2]), .B(mic_phase[1]), .Z(n100)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_adj_174.init = 16'h2222;
    LUT4 status_next_bit_N_2770_2__bdd_4_lut_4_lut (.A(status_next_index[2]), 
         .B(n27639), .C(n27651), .D(status_next_index[3]), .Z(n27678)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;
    defparam status_next_bit_N_2770_2__bdd_4_lut_4_lut.init = 16'h55d8;
    LUT4 ev_run_hold_s5_83__I_0_i73_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_72), .D(ev_run_hold_s5[72]), .Z(us_tx_next[72])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i73_3_lut_4_lut.init = 16'hbf40;
    FD1P3IX mic_ram_wr_data_i0_i4 (.D(mic_ram_wr_data_15__N_2316[4]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i4.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_adj_175 (.A(spi_command[2]), .B(spi_command[3]), 
         .C(n25844), .Z(n24995)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i1_2_lut_3_lut_adj_175.init = 16'h4040;
    LUT4 i10151_4_lut (.A(ev_rd_data[49]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[49])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10151_4_lut.init = 16'h1222;
    LUT4 i1_4_lut_adj_176 (.A(mic_phase[2]), .B(n10833), .C(pll_clk_enable_104), 
         .D(n25901), .Z(pll_clk_enable_42)) /* synthesis lut_function=(A (B (C))+!A (B (C+!(D)))) */ ;
    defparam i1_4_lut_adj_176.init = 16'hc0c4;
    PFUMX i4591 (.BLUT(n15429), .ALUT(n26168), .C0(ev_fall_N_2693), .Z(n15430));
    LUT4 mic_acc_word_2__bdd_4_lut_16338 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [6]), 
         .C(\mic_win_q[3] [6]), .D(mic_acc_word[0]), .Z(n27696)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16338.init = 16'he4aa;
    LUT4 i10150_4_lut (.A(ev_rd_data[50]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[50])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10150_4_lut.init = 16'h1222;
    LUT4 frame_settle_3__bdd_2_lut_3_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n28060), .Z(n28061)) /* synthesis lut_function=(A ((C)+!B)+!A (B+(C))) */ ;
    defparam frame_settle_3__bdd_2_lut_3_lut.init = 16'hf6f6;
    LUT4 n27732_bdd_4_lut_16635_then_3_lut (.A(mic_ram_rd_data[12]), .B(mic_ram_rd_data[14]), 
         .C(status_bit_index[1]), .Z(n28402)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27732_bdd_4_lut_16635_then_3_lut.init = 16'hacac;
    LUT4 n27732_bdd_4_lut_16635_else_3_lut (.A(mic_ram_rd_data[13]), .B(mic_ram_rd_data[15]), 
         .C(status_bit_index[1]), .Z(n28401)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27732_bdd_4_lut_16635_else_3_lut.init = 16'hacac;
    LUT4 i15492_3_lut_4_lut (.A(mic_run_state_1__N_2410[1]), .B(mic_warm), 
         .C(mic_ultrasonic), .D(n17_adj_3165), .Z(pll_clk_enable_172)) /* synthesis lut_function=(A+!(B (D)+!B ((D)+!C))) */ ;
    defparam i15492_3_lut_4_lut.init = 16'haafe;
    LUT4 n27726_bdd_4_lut_then_3_lut (.A(mic_ram_rd_data[8]), .B(mic_ram_rd_data[10]), 
         .C(status_bit_index[1]), .Z(n28405)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27726_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 i3_4_lut_adj_177 (.A(mic_lo_n[3]), .B(mic_lo_n[4]), .C(mic_lo_n[2]), 
         .D(n26038), .Z(n13_adj_3180)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i3_4_lut_adj_177.init = 16'hfeff;
    LUT4 n27726_bdd_4_lut_else_3_lut (.A(mic_ram_rd_data[9]), .B(mic_ram_rd_data[11]), 
         .C(status_bit_index[1]), .Z(n28404)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27726_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 i2_3_lut_4_lut_adj_178 (.A(n21440), .B(n17021), .C(n20788), .D(n21289), 
         .Z(n13560)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_178.init = 16'hfeff;
    LUT4 i15021_4_lut (.A(mic_lo_n[0]), .B(mic_lo_n[1]), .C(mic_lo_n[5]), 
         .D(mic_lo_n[6]), .Z(n26038)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i15021_4_lut.init = 16'h8000;
    LUT4 ws2812_settle_2__bdd_3_lut (.A(ws2812_settle[2]), .B(ws2812_settle[1]), 
         .C(ws2812_settle[0]), .Z(n27913)) /* synthesis lut_function=(A (B+(C))+!A !(B+(C))) */ ;
    defparam ws2812_settle_2__bdd_3_lut.init = 16'ha9a9;
    LUT4 ev_run_hold_s5_83__I_0_i33_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_32), .D(ev_run_hold_s5[32]), .Z(us_tx_next[32])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i33_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27456_bdd_4_lut (.A(n27456), .B(n27314), .C(n27310), .D(status_next_bit_N_2770[3]), 
         .Z(n27459)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27456_bdd_4_lut.init = 16'haad8;
    LUT4 i15478_2_lut_3_lut (.A(n20751), .B(n21503), .C(spi1_sck_c_enable_159), 
         .Z(spi1_sck_c_enable_162)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15478_2_lut_3_lut.init = 16'h8080;
    LUT4 i1_2_lut_3_lut_adj_179 (.A(mod_settle[2]), .B(mod_settle[1]), .C(mod_settle[0]), 
         .Z(n16282)) /* synthesis lut_function=(A (B+(C))+!A !(B+(C))) */ ;
    defparam i1_2_lut_3_lut_adj_179.init = 16'ha9a9;
    LUT4 i2_2_lut_4_lut (.A(link_idle[3]), .B(link_idle[0]), .C(link_idle[1]), 
         .D(link_idle[2]), .Z(link_timeout)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(394[25:35])
    defparam i2_2_lut_4_lut.init = 16'h8000;
    LUT4 i4259_2_lut_3_lut (.A(link_timeout), .B(stop_toggle_sync), .C(stop_toggle_seen), 
         .Z(n10742)) /* synthesis lut_function=(A+!(B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i4259_2_lut_3_lut.init = 16'hbebe;
    LUT4 ev_run_hold_s5_83__I_0_i32_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_31), .D(ev_run_hold_s5[31]), .Z(us_tx_next[31])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i32_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_3_lut_4_lut_adj_180 (.A(mic_phase[3]), .B(mic_phase[0]), .C(n6), 
         .D(mic_warm), .Z(n79)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_180.init = 16'hfffe;
    LUT4 i1_4_lut_4_lut_else_1_lut_4_lut (.A(mic_phase[1]), .B(mic_phase[2]), 
         .C(mic_phase[0]), .D(mic_ultrasonic), .Z(n28494)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B (C (D))))) */ ;
    defparam i1_4_lut_4_lut_else_1_lut_4_lut.init = 16'h01ff;
    LUT4 i2_2_lut_3_lut (.A(mic_warm), .B(mic_phase[0]), .C(n4_adj_3264), 
         .Z(pll_clk_enable_266)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;
    defparam i2_2_lut_3_lut.init = 16'h1010;
    LUT4 i1_2_lut_3_lut_adj_181 (.A(mic_ultrasonic), .B(n21403), .C(n4), 
         .Z(n19280)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_3_lut_adj_181.init = 16'h0808;
    LUT4 i1_3_lut_4_lut_adj_182 (.A(mic_warm), .B(mic_phase[3]), .C(mic_phase[0]), 
         .D(mic_phase[1]), .Z(n25901)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_3_lut_4_lut_adj_182.init = 16'hfffe;
    LUT4 i13818_2_lut_4_lut (.A(mic_sample_count[2]), .B(pcm_fifo_N_2624), 
         .C(mic_sample_count[0]), .D(mic_sample_count[1]), .Z(n23)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(879[37:60])
    defparam i13818_2_lut_4_lut.init = 16'h6aaa;
    LUT4 i1_2_lut_3_lut_adj_183 (.A(mic_ultrasonic_N_1914), .B(n28), .C(mic_warm), 
         .Z(pll_clk_enable_104)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_3_lut_adj_183.init = 16'h8080;
    LUT4 i6881_2_lut_3_lut (.A(pll_clk_enable_602), .B(mic_acc_busy_N_2292), 
         .C(mic_done_N_2656), .Z(n17809)) /* synthesis lut_function=(!((B (C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam i6881_2_lut_3_lut.init = 16'h2a2a;
    LUT4 i2_3_lut_4_lut_adj_184 (.A(n25788), .B(mic_warm), .C(pcm_fifo_N_2624), 
         .D(mic_ultrasonic), .Z(pll_clk_enable_601)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i2_3_lut_4_lut_adj_184.init = 16'h2000;
    LUT4 ev_run_hold_s5_83__I_0_i31_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_30), .D(ev_run_hold_s5[30]), .Z(us_tx_next[30])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i31_3_lut_4_lut.init = 16'hbf40;
    LUT4 mux_1626_i1_3_lut_4_lut (.A(status_bit_index[7]), .B(status_bit_index[8]), 
         .C(status_miso_bit), .D(n11_adj_3295), .Z(pcm_audio_miso_N_2783[0])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_1626_i1_3_lut_4_lut.init = 16'hfe10;
    LUT4 i1_2_lut_4_lut_adj_185 (.A(mic_warm), .B(mic_ultrasonic), .C(mic_run_state_1__N_2410[1]), 
         .D(pll_clk_enable_104), .Z(pll_clk_enable_205)) /* synthesis lut_function=(A (C+(D))+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_4_lut_adj_185.init = 16'hfff4;
    LUT4 spi_frame_sequence_5__I_0_i1_3_lut (.A(mic_acc_word[0]), .B(status_bit_index[4]), 
         .C(n5), .Z(mic_ram_rd_addr[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i1_3_lut.init = 16'hcaca;
    LUT4 spi_frame_sequence_5__I_0_i2_3_lut (.A(mic_acc_word[1]), .B(status_bit_index[5]), 
         .C(n5), .Z(mic_ram_rd_addr[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i2_3_lut.init = 16'hcaca;
    LUT4 n27720_bdd_4_lut_else_3_lut (.A(n14137), .B(n14138), .C(status_bit_index[0]), 
         .Z(n28407)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27720_bdd_4_lut_else_3_lut.init = 16'hacac;
    L6MUX21 i17190 (.D0(n29108), .D1(n29105), .SD(status_bit_index[0]), 
            .Z(n29109));
    LUT4 i1_2_lut_3_lut_adj_186 (.A(mic_phase[0]), .B(n25867), .C(n19280), 
         .Z(pll_clk_enable_495)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_3_lut_adj_186.init = 16'h8080;
    FD1P3IX mic_ram_wr_data_i0_i3 (.D(mic_ram_wr_data_15__N_2316[3]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i3.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i2 (.D(mic_ram_wr_data_15__N_2316[2]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i2.GSR = "DISABLED";
    LUT4 i10149_4_lut (.A(ev_rd_data[51]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[51])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10149_4_lut.init = 16'h1222;
    LUT4 i1_3_lut_4_lut_adj_187 (.A(n16_adj_3304), .B(n25844), .C(spi_byte_count[2]), 
         .D(spi_byte_count[3]), .Z(n6_adj_3289)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i1_3_lut_4_lut_adj_187.init = 16'h0004;
    LUT4 i2_3_lut_4_lut_adj_188 (.A(n10742), .B(mod_settle[0]), .C(n17036), 
         .D(feed_rd_9__N_1712), .Z(pll_clk_enable_108)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_188.init = 16'hfffe;
    LUT4 i15015_2_lut_3_lut (.A(spi_extension_length[1]), .B(spi_extension_length[2]), 
         .C(n16829), .Z(n26032)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i15015_2_lut_3_lut.init = 16'hfefe;
    LUT4 i2908_3_lut_4_lut (.A(mic_acc_busy_N_2292), .B(mic_done_N_2656), 
         .C(pll_clk_enable_602), .D(mic_run_state_1__N_2410[1]), .Z(n13689)) /* synthesis lut_function=(A (B)+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i2908_3_lut_4_lut.init = 16'hddd8;
    LUT4 i12_3_lut (.A(mic_acc_word[2]), .B(status_bit_index[6]), .C(n5), 
         .Z(mic_ram_rd_addr[2])) /* synthesis lut_function=(!(A (B (C))+!A (B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(259[17:29])
    defparam i12_3_lut.init = 16'h3a3a;
    FD1P3IX mic_ram_wr_data_i0_i1 (.D(mic_ram_wr_data_15__N_2316[1]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i1.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i30_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_29), .D(ev_run_hold_s5[29]), .Z(us_tx_next[29])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i30_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i29_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_28), .D(ev_run_hold_s5[28]), .Z(us_tx_next[28])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i29_3_lut_4_lut.init = 16'hbf40;
    LUT4 i3181_2_lut_3_lut (.A(mic_acc_busy_N_2292), .B(mic_done_N_2656), 
         .C(n13689), .Z(pll_clk_enable_202)) /* synthesis lut_function=(A ((C)+!B)+!A (C)) */ ;
    defparam i3181_2_lut_3_lut.init = 16'hf2f2;
    LUT4 i2_3_lut_4_lut_adj_189 (.A(n10742), .B(ws2812_enable_N_2601), .C(frame_toggle_sync), 
         .D(frame_toggle_seen), .Z(pll_clk_enable_100)) /* synthesis lut_function=(A+(B+!(C (D)+!C !(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_189.init = 16'heffe;
    LUT4 spi_frame_sequence_5__I_0_i4_3_lut (.A(mic_acc_gate[0]), .B(spi_frame_sequence[0]), 
         .C(n5), .Z(mic_ram_rd_addr[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i4_3_lut.init = 16'hcaca;
    LUT4 mux_679_i8_3_lut_4_lut (.A(pending_sequence[7]), .B(accepted_sequence_spi[7]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[7])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i8_3_lut_4_lut.init = 16'hacaa;
    LUT4 i1_2_lut_4_lut_adj_190 (.A(mic_cfg_settle[0]), .B(n17_adj_3196), 
         .C(mic_cfg_toggle_seen), .D(mic_cfg_toggle_sync), .Z(pll_clk_enable_98)) /* synthesis lut_function=(A+(B+!(C (D)+!C !(D)))) */ ;
    defparam i1_2_lut_4_lut_adj_190.init = 16'heffe;
    LUT4 spi_frame_sequence_5__I_0_i5_3_lut (.A(mic_acc_gate[1]), .B(spi_frame_sequence[1]), 
         .C(n5), .Z(mic_ram_rd_addr[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i5_3_lut.init = 16'hcaca;
    LUT4 i2_3_lut_4_lut_adj_191 (.A(fpga_time[13]), .B(fpga_time[11]), .C(fpga_time[12]), 
         .D(fpga_time[14]), .Z(n28)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam i2_3_lut_4_lut_adj_191.init = 16'h8000;
    LUT4 spi_frame_sequence_5__I_0_i6_3_lut (.A(mic_acc_gate[2]), .B(spi_frame_sequence[2]), 
         .C(n5), .Z(mic_ram_rd_addr[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i6_3_lut.init = 16'hcaca;
    LUT4 spi_frame_sequence_5__I_0_i7_3_lut (.A(mic_acc_gate[3]), .B(spi_frame_sequence[3]), 
         .C(n5), .Z(mic_ram_rd_addr[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i7_3_lut.init = 16'hcaca;
    LUT4 spi_frame_sequence_5__I_0_i8_3_lut (.A(mic_acc_gate[4]), .B(spi_frame_sequence[4]), 
         .C(n5), .Z(mic_ram_rd_addr[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i8_3_lut.init = 16'hcaca;
    LUT4 spi_frame_sequence_5__I_0_i9_3_lut (.A(mic_acc_gate[5]), .B(spi_frame_sequence[5]), 
         .C(n5), .Z(mic_ram_rd_addr[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(269[36] 270[101])
    defparam spi_frame_sequence_5__I_0_i9_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut_adj_192 (.A(spi_byte_count[8]), .B(spi_byte_count[6]), 
         .C(spi_byte_count[7]), .Z(n21300)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_3_lut_adj_192.init = 16'hfefe;
    LUT4 i15395_2_lut_3_lut (.A(mic_acc_gate_6__N_2283), .B(mic_gate_active), 
         .C(mic_acc_busy_N_2302), .Z(mic_gate_active_N_2306)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(962[9] 982[12])
    defparam i15395_2_lut_3_lut.init = 16'h0202;
    FD1P3IX mic_acc_state_i2 (.D(mic_acc_last), .SP(pll_clk_enable_90), 
            .CD(n28433), .CK(pll_clk), .Q(mic_acc_state[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_state_i2.GSR = "DISABLED";
    PFUMX i17188 (.BLUT(n29107), .ALUT(n29106), .C0(status_bit_index[2]), 
          .Z(n29108));
    LUT4 mux_3341_i1_3_lut (.A(n14106), .B(n14107), .C(n14105), .Z(rd_data_7__N_2913[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3341_i1_3_lut.init = 16'hcaca;
    LUT4 i10_4_lut_adj_193 (.A(n25958), .B(n26050), .C(n25956), .D(n18_adj_3181), 
         .Z(n14105)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i10_4_lut_adj_193.init = 16'h0100;
    LUT4 i14942_4_lut (.A(n14089), .B(n14091), .C(n14090), .D(n14092), 
         .Z(n25958)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14942_4_lut.init = 16'h7bde;
    LUT4 i15033_3_lut (.A(n14097), .B(n25978), .C(n14098), .Z(n26050)) /* synthesis lut_function=(A (B+!(C))+!A (B+(C))) */ ;
    defparam i15033_3_lut.init = 16'hdede;
    LUT4 mux_679_i7_3_lut_4_lut (.A(pending_sequence[6]), .B(accepted_sequence_spi[6]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[6])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i7_3_lut_4_lut.init = 16'hacaa;
    LUT4 i1_2_lut_3_lut_adj_194 (.A(mic_acc_state[0]), .B(mic_acc_state[1]), 
         .C(mic_acc_state[2]), .Z(n4_adj_3299)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_3_lut_adj_194.init = 16'h0808;
    LUT4 i14940_4_lut (.A(n14085), .B(n14087), .C(n14086), .D(n14088), 
         .Z(n25956)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14940_4_lut.init = 16'h7bde;
    LUT4 i7_4_lut_adj_195 (.A(n14093), .B(n25962), .C(n14104), .D(n14094), 
         .Z(n18_adj_3181)) /* synthesis lut_function=(!(A (B+!(C (D)))+!A (B+((D)+!C)))) */ ;
    defparam i7_4_lut_adj_195.init = 16'h2010;
    LUT4 i1_2_lut_3_lut_adj_196 (.A(n12000), .B(n7), .C(n11998), .Z(n16266)) /* synthesis lut_function=(!(A (C)+!A (B (C)+!B !(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(504[50:75])
    defparam i1_2_lut_3_lut_adj_196.init = 16'h1e1e;
    LUT4 i14962_4_lut (.A(n14095), .B(n14101), .C(n14096), .D(n14102), 
         .Z(n25978)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14962_4_lut.init = 16'h7bde;
    LUT4 i15501_3_lut_4_lut (.A(n11_adj_3228), .B(pll_clk_enable_602), .C(mic_run_state_1__N_2410[1]), 
         .D(n10893), .Z(pll_clk_enable_90)) /* synthesis lut_function=((B+(C+!(D)))+!A) */ ;
    defparam i15501_3_lut_4_lut.init = 16'hfdff;
    LUT4 i3655_2_lut_3_lut (.A(n12000), .B(n7), .C(n11998), .Z(n8_adj_3297)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(504[50:75])
    defparam i3655_2_lut_3_lut.init = 16'hfefe;
    LUT4 mic_warm_bdd_4_lut (.A(mic_phase[0]), .B(mic_phase[3]), .C(n17), 
         .D(n6), .Z(n28152)) /* synthesis lut_function=(!(A ((C+(D))+!B)+!A ((D)+!B))) */ ;
    defparam mic_warm_bdd_4_lut.init = 16'h004c;
    LUT4 i14946_4_lut (.A(n14083), .B(n14099), .C(n14084), .D(n14100), 
         .Z(n25962)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14946_4_lut.init = 16'h7bde;
    LUT4 i1_2_lut_4_lut_adj_197 (.A(status_bit_index[8]), .B(status_bit_index[4]), 
         .C(status_bit_index[6]), .D(status_bit_index[5]), .Z(n8_adj_3229)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(429[37:63])
    defparam i1_2_lut_4_lut_adj_197.init = 16'hfffe;
    LUT4 i3440_2_lut (.A(pcm_rd_ptr_spi[0]), .B(pcm_payload_bit[4]), .Z(pcm_spi_sample_15__N_1381[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(440[18:55])
    defparam i3440_2_lut.init = 16'h6666;
    LUT4 ev_run_hold_s5_83__I_0_i65_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_64), .D(ev_run_hold_s5[64]), .Z(us_tx_next[64])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i65_3_lut_4_lut.init = 16'hbf40;
    FD1P3IX mic_gate_fill_i6 (.D(mic_acc_last_N_2675[6]), .SP(pll_clk_enable_605), 
            .CD(n17838), .CK(pll_clk), .Q(mic_gate_fill[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_fill_i6.GSR = "DISABLED";
    FD1P3IX mic_gate_fill_i5 (.D(mic_acc_last_N_2675[5]), .SP(pll_clk_enable_605), 
            .CD(n17838), .CK(pll_clk), .Q(mic_gate_fill[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_fill_i5.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i28_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_27), .D(ev_run_hold_s5[27]), .Z(us_tx_next[27])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i28_3_lut_4_lut.init = 16'hbf40;
    LUT4 i10148_4_lut (.A(ev_rd_data[52]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[52])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10148_4_lut.init = 16'h1222;
    LUT4 i3926_1_lut (.A(mic_acc_word[0]), .Z(mic_acc_word_2__N_2332[0])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1000[13:17])
    defparam i3926_1_lut.init = 16'h5555;
    LUT4 i2_3_lut_4_lut_adj_198 (.A(spi_byte_count[1]), .B(spi_byte_count[2]), 
         .C(n20788), .D(n21440), .Z(n25885)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[28:57])
    defparam i2_3_lut_4_lut_adj_198.init = 16'hfffe;
    LUT4 i15498_2_lut_3_lut (.A(pll_clk_enable_72), .B(staging_rd_page_N_2729), 
         .C(phase_step_s3), .Z(n17862)) /* synthesis lut_function=(A ((C)+!B)) */ ;
    defparam i15498_2_lut_3_lut.init = 16'ha2a2;
    FD1P3IX mic_gate_fill_i4 (.D(mic_acc_last_N_2675[4]), .SP(pll_clk_enable_605), 
            .CD(n17838), .CK(pll_clk), .Q(mic_gate_fill[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_fill_i4.GSR = "DISABLED";
    PFUMX status_next_bit_I_0_i78 (.BLUT(n26136), .ALUT(n26137), .C0(status_next_bit_N_2770[3]), 
          .Z(n78));
    LUT4 i1_3_lut_4_lut_adj_199 (.A(mic_cfg_settle[0]), .B(n17_adj_3196), 
         .C(mic_cfg_toggle_seen), .D(mic_cfg_toggle_sync), .Z(pll_clk_enable_602)) /* synthesis lut_function=(!((B+!(C (D)+!C !(D)))+!A)) */ ;
    defparam i1_3_lut_4_lut_adj_199.init = 16'h2002;
    LUT4 n27702_bdd_4_lut_16581_else_3_lut (.A(n14135), .B(n14136), .C(status_bit_index[0]), 
         .Z(n28410)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27702_bdd_4_lut_16581_else_3_lut.init = 16'hacac;
    FD1P3IX mic_gate_fill_i3 (.D(mic_acc_last_N_2675[3]), .SP(pll_clk_enable_605), 
            .CD(n17838), .CK(pll_clk), .Q(mic_gate_fill[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_fill_i3.GSR = "DISABLED";
    FD1S3AX spi_rx_shift_i7 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(spi_rx_shift[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_rx_shift_i7.GSR = "ENABLED";
    LUT4 i1_2_lut_3_lut_4_lut (.A(spi_byte_count[6]), .B(spi_byte_count[7]), 
         .C(spi_byte_count[8]), .D(spi_byte_count[5]), .Z(n17027)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(468[97:138])
    defparam i1_2_lut_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_2_lut_3_lut_adj_200 (.A(spi_byte_count[1]), .B(spi_byte_count[2]), 
         .C(spi_byte_count[3]), .Z(n17024)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[28:57])
    defparam i1_2_lut_3_lut_adj_200.init = 16'hfefe;
    FD1P3IX mic_gate_fill_i2 (.D(mic_acc_last_N_2675[2]), .SP(pll_clk_enable_605), 
            .CD(n17838), .CK(pll_clk), .Q(mic_gate_fill[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_fill_i2.GSR = "DISABLED";
    PFUMX i17186 (.BLUT(n29104), .ALUT(n29103), .C0(status_bit_index[2]), 
          .Z(n29105));
    LUT4 i15331_3_lut_4_lut (.A(n12_adj_3175), .B(n16924), .C(spi_byte_count[0]), 
         .D(spi_byte_count[1]), .Z(spi1_sck_c_enable_135)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15331_3_lut_4_lut.init = 16'h0800;
    PFUMX i16813 (.BLUT(n28494), .ALUT(n28495), .C0(mic_warm), .Z(pll_clk_enable_188));
    LUT4 i15429_2_lut_3_lut_4_lut (.A(n12_adj_3175), .B(n16924), .C(spi_byte_count[1]), 
         .D(spi_byte_count[0]), .Z(spi1_sck_c_enable_143)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15429_2_lut_3_lut_4_lut.init = 16'h0008;
    LUT4 i1_2_lut_3_lut_4_lut_adj_201 (.A(n12_adj_3175), .B(n16924), .C(spi_byte_count[1]), 
         .D(spi_byte_count[0]), .Z(spi1_sck_c_enable_151)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i1_2_lut_3_lut_4_lut_adj_201.init = 16'h0800;
    LUT4 i10147_4_lut (.A(ev_rd_data[53]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[53])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10147_4_lut.init = 16'h1222;
    LUT4 i2_3_lut (.A(n4_adj_3306), .B(pcm_payload_bit[6]), .C(pcm_rd_ptr_spi[2]), 
         .Z(pcm_spi_sample_15__N_1381[2])) /* synthesis lut_function=(A (B (C)+!B !(C))+!A !(B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(440[18:55])
    defparam i2_3_lut.init = 16'h9696;
    LUT4 i2_3_lut_adj_202 (.A(n6_adj_3314), .B(pcm_payload_bit[7]), .C(pcm_rd_ptr_spi[3]), 
         .Z(pcm_spi_sample_15__N_1381[3])) /* synthesis lut_function=(A (B (C)+!B !(C))+!A !(B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(440[18:55])
    defparam i2_3_lut_adj_202.init = 16'h9696;
    LUT4 i3348_2_lut (.A(pcm_fifo_N_2611), .B(pcm_wr_ptr[4]), .Z(n14158)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(869[25:50])
    defparam i3348_2_lut.init = 16'h2222;
    LUT4 i15434_3_lut_4_lut_4_lut (.A(n20918), .B(mic_acc_state[0]), .C(n25163), 
         .D(n10893), .Z(n26048)) /* synthesis lut_function=(!(A (C (D))+!A (B ((D)+!C)+!B (C (D))))) */ ;
    defparam i15434_3_lut_4_lut_4_lut.init = 16'h0bfb;
    LUT4 i3_4_lut_adj_203 (.A(pcm_fifo_N_2617[5]), .B(n25788), .C(mic_phase[3]), 
         .D(pll_clk_enable_266), .Z(pcm_fifo_N_2611)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(822[18] 907[12])
    defparam i3_4_lut_adj_203.init = 16'h4000;
    LUT4 n27438_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[112]), 
         .C(status_hold[113]), .D(n27438), .Z(n27441)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27438_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 n27708_bdd_4_lut (.A(n27708), .B(\mic_win_q[2] [3]), .C(\mic_win_i[2] [3]), 
         .D(mic_acc_word[0]), .Z(n27711)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27708_bdd_4_lut.init = 16'haad8;
    LUT4 i1_4_lut_adj_204 (.A(n16983), .B(n25898), .C(spi_extension_length[5]), 
         .D(n25895), .Z(n21487)) /* synthesis lut_function=(A (B+(C))+!A (B+(C (D)))) */ ;
    defparam i1_4_lut_adj_204.init = 16'hfcec;
    LUT4 i3460_3_lut (.A(pcm_rd_ptr_spi[2]), .B(pcm_payload_bit[6]), .C(n4_adj_3306), 
         .Z(n6_adj_3314)) /* synthesis lut_function=(A (B+(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(440[18:55])
    defparam i3460_3_lut.init = 16'he8e8;
    LUT4 i13429_3_lut_4_lut (.A(feed_pop), .B(feed_wr[0]), .C(frame_req_N_2710), 
         .D(n21319), .Z(n24183)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i13429_3_lut_4_lut.init = 16'haaca;
    LUT4 ev_run_hold_s5_83__I_0_i4_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_3), .D(ev_run_hold_s5[3]), .Z(us_tx_next[3])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i4_3_lut_4_lut.init = 16'hbf40;
    LUT4 i15421_2_lut_3_lut (.A(spi_command[0]), .B(spi_command[1]), .C(n17015), 
         .Z(spi_audio_read_cmd)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(174[38:60])
    defparam i15421_2_lut_3_lut.init = 16'h0404;
    FD1P3IX mic_gate_fill_i1 (.D(mic_acc_last_N_2675[1]), .SP(pll_clk_enable_605), 
            .CD(n17838), .CK(pll_clk), .Q(mic_gate_fill[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_fill_i1.GSR = "DISABLED";
    LUT4 i10146_4_lut (.A(ev_rd_data[54]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[54])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10146_4_lut.init = 16'h1222;
    LUT4 i1_2_lut_adj_205 (.A(spi_extension_length[1]), .B(spi_extension_length[2]), 
         .Z(n16983)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(561[34:64])
    defparam i1_2_lut_adj_205.init = 16'heeee;
    FD1P3IX mic_run_state_i1 (.D(mic_run_state_1__N_2410[1]), .SP(pll_clk_enable_97), 
            .CD(mic_done_N_2650), .CK(pll_clk), .Q(mic_run_state[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_run_state_i1.GSR = "DISABLED";
    FD1P3JX mic_cfg_settle_i2 (.D(mic_cfg_settle_3__N_2142[2]), .SP(pll_clk_enable_98), 
            .PD(mic_cfg_settle_3__N_2146), .CK(pll_clk), .Q(mic_cfg_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_settle_i2.GSR = "DISABLED";
    LUT4 i15461_2_lut_3_lut_3_lut_4_lut (.A(mic_acc_state[0]), .B(mic_acc_state[1]), 
         .C(mic_acc_state[2]), .D(n5_adj_3259), .Z(pll_clk_enable_268)) /* synthesis lut_function=(!(A (B (D)+!B (C (D)))+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i15461_2_lut_3_lut_3_lut_4_lut.init = 16'h02fe;
    LUT4 i10145_4_lut (.A(ev_rd_data[55]), .B(ev_in_clear), .C(ev_ch_hi_6), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[55])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10145_4_lut.init = 16'h1222;
    LUT4 i3140_2_lut (.A(ws2812_enable_N_2588), .B(n13683), .Z(pll_clk_enable_540)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i3140_2_lut.init = 16'heeee;
    FD1S3AX spi_rx_shift_i6 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(spi_rx_shift[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_rx_shift_i6.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i5 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(spi_rx_shift[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_rx_shift_i5.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i4 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(spi_rx_shift[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_rx_shift_i4.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i3 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(spi_rx_shift[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_rx_shift_i3.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i2 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(spi_rx_shift[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_rx_shift_i2.GSR = "ENABLED";
    LUT4 mic_acc_word_2__bdd_4_lut_16343 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [3]), 
         .C(\mic_win_q[3] [3]), .D(mic_acc_word[0]), .Z(n27708)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16343.init = 16'he4aa;
    LUT4 ev_run_hold_s5_83__I_0_i27_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_26), .D(ev_run_hold_s5[26]), .Z(us_tx_next[26])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i27_3_lut_4_lut.init = 16'hbf40;
    FD1P3JX ws2812_frames_left_i1 (.D(n16248), .SP(pll_clk_enable_99), .PD(ws2812_enable_N_2588), 
            .CK(pll_clk), .Q(ws2812_frames_left[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ws2812_frames_left_i1.GSR = "DISABLED";
    LUT4 i15220_2_lut_2_lut (.A(status_next_index[1]), .B(status_hold[74]), 
         .Z(n26145)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam i15220_2_lut_2_lut.init = 16'h4444;
    LUT4 i1_2_lut_3_lut_adj_206 (.A(mic_acc_busy_N_2302), .B(mic_gate_active), 
         .C(mic_acc_gate_6__N_2283), .Z(n17689)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(975[22] 981[16])
    defparam i1_2_lut_3_lut_adj_206.init = 16'h2020;
    LUT4 ev_run_hold_s5_83__I_0_i26_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_25), .D(ev_run_hold_s5[25]), .Z(us_tx_next[25])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i26_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX mic_cfg_toggle_seen_1334 (.D(mic_cfg_toggle_sync), .SP(mic_cfg_settle_3__N_2146), 
            .CK(pll_clk), .Q(mic_cfg_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_toggle_seen_1334.GSR = "DISABLED";
    LUT4 i9892_2_lut (.A(spi_channel_page), .B(spi_phase_only_N_2792), .Z(staging_wr_page)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[35:75])
    defparam i9892_2_lut.init = 16'h8888;
    CCU2D mic_cfg_hold_spi_39__I_0_add_2_1 (.A0(GND_net), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_cfg_hold_spi[24]), .B1(mic_cfg_hold_spi[40]), 
          .C1(n16834), .D1(mic_cfg_hold_spi[46]), .COUT(n24852), .S1(mic_cfg_gap_w[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[33:90])
    defparam mic_cfg_hold_spi_39__I_0_add_2_1.INIT0 = 16'h0000;
    defparam mic_cfg_hold_spi_39__I_0_add_2_1.INIT1 = 16'h599a;
    defparam mic_cfg_hold_spi_39__I_0_add_2_1.INJECT1_0 = "NO";
    defparam mic_cfg_hold_spi_39__I_0_add_2_1.INJECT1_1 = "NO";
    PFUMX i4589 (.BLUT(n15427), .ALUT(n26166), .C0(ev_fall_N_2693), .Z(n15428));
    CCU2D fpga_time_3076_add_4_33 (.A0(fpga_time[31]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24851), .S0(n134));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_33.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_33.INIT1 = 16'h0000;
    defparam fpga_time_3076_add_4_33.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_33.INJECT1_1 = "NO";
    LUT4 n14146_bdd_3_lut_17177 (.A(n14146), .B(n14145), .C(status_bit_index[0]), 
         .Z(n28680)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n14146_bdd_3_lut_17177.init = 16'hcaca;
    FD1P3AX stop_toggle_seen_1308 (.D(stop_toggle_sync), .SP(stop_event), 
            .CK(pll_clk), .Q(stop_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam stop_toggle_seen_1308.GSR = "DISABLED";
    LUT4 i13928_2_lut (.A(mic_ram_rd_data[0]), .B(mic_acc_win[0]), .Z(mic_ram_wr_data_15__N_2316[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13928_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_adj_207 (.A(n23_adj_3177), .B(n20909), .C(n11_adj_3236), 
         .Z(n25898)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_3_lut_adj_207.init = 16'hfefe;
    LUT4 ev_run_hold_s5_83__I_0_i25_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_24), .D(ev_run_hold_s5[24]), .Z(us_tx_next[24])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i25_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27714_bdd_4_lut (.A(n27714), .B(\mic_win_q[0] [7]), .C(\mic_win_i[0] [7]), 
         .D(mic_acc_word[0]), .Z(n27717)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27714_bdd_4_lut.init = 16'haad8;
    LUT4 mic_acc_word_2__bdd_4_lut_16367 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [7]), 
         .C(\mic_win_q[1] [7]), .D(mic_acc_word[0]), .Z(n27714)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16367.init = 16'he4aa;
    LUT4 i1_3_lut_adj_208 (.A(spi_extension_length[0]), .B(spi_extension_length[3]), 
         .C(spi_extension_length[4]), .Z(n25895)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(561[34:64])
    defparam i1_3_lut_adj_208.init = 16'hfefe;
    FD1P3AX mod_toggle_seen_1293 (.D(mod_toggle_sync), .SP(feed_rd_9__N_1712), 
            .CK(pll_clk), .Q(mod_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mod_toggle_seen_1293.GSR = "DISABLED";
    LUT4 i3027_2_lut_3_lut_3_lut (.A(mic_acc_state[0]), .B(mic_acc_state[1]), 
         .C(mic_acc_state[2]), .Z(n13496)) /* synthesis lut_function=(A (C)+!A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i3027_2_lut_3_lut_3_lut.init = 16'he1e1;
    FD1P3AX frame_toggle_seen_1285 (.D(frame_toggle_sync), .SP(ws2812_enable_N_2602), 
            .CK(pll_clk), .Q(frame_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam frame_toggle_seen_1285.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i24_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_23), .D(ev_run_hold_s5[23]), .Z(us_tx_next[23])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i24_3_lut_4_lut.init = 16'hbf40;
    FD1P3IX frame_settle__i3 (.D(n28061), .SP(pll_clk_enable_100), .CD(n10742), 
            .CK(pll_clk), .Q(frame_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam frame_settle__i3.GSR = "DISABLED";
    PFUMX i17183 (.BLUT(n29101), .ALUT(n29100), .C0(status_bit_index[7]), 
          .Z(spi1_miso_N_2453));
    LUT4 n27492_bdd_4_lut_16426_4_lut (.A(status_next_index[1]), .B(status_hold[36]), 
         .C(status_hold[37]), .D(n27492), .Z(n27495)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27492_bdd_4_lut_16426_4_lut.init = 16'hf588;
    LUT4 ev_run_hold_s5_83__I_0_i23_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_22), .D(ev_run_hold_s5[22]), .Z(us_tx_next[22])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i23_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27480_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[32]), 
         .C(status_hold[33]), .D(n27480), .Z(n27483)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27480_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 i4707_2_lut_3_lut (.A(mod_toggle_sync), .B(mod_toggle_seen), .C(n10742), 
         .Z(n15546)) /* synthesis lut_function=(A ((C)+!B)+!A (B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i4707_2_lut_3_lut.init = 16'hf6f6;
    FD1P3AX mic_ultrasonic_1312 (.D(n29158), .SP(pll_clk_enable_104), .CK(pll_clk), 
            .Q(mic_ultrasonic)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ultrasonic_1312.GSR = "DISABLED";
    FD1P3AX ws2812_toggle_seen_1297 (.D(ws2812_toggle_sync), .SP(ws2812_settle_3__N_1717), 
            .CK(pll_clk), .Q(ws2812_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ws2812_toggle_seen_1297.GSR = "DISABLED";
    FD1P3AX invalid_frame_spi_1238 (.D(invalid_frame_spi_N_2536), .SP(spi1_sck_c_enable_52), 
            .CK(spi1_sck_c), .Q(invalid_frame_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam invalid_frame_spi_1238.GSR = "DISABLED";
    LUT4 n28685_bdd_3_lut (.A(n28685), .B(n28682), .C(status_bit_index[1]), 
         .Z(n28686)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n28685_bdd_3_lut.init = 16'hcaca;
    LUT4 i2_3_lut_adj_209 (.A(spi_extension_length[7]), .B(spi_extension_length[9]), 
         .C(spi_extension_length[6]), .Z(n20909)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i2_3_lut_adj_209.init = 16'hfefe;
    FD1P3AX mod_toggle_spi_1236 (.D(mod_toggle_spi_N_2510), .SP(spi1_sck_c_enable_53), 
            .CK(spi1_sck_c), .Q(mod_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mod_toggle_spi_1236.GSR = "DISABLED";
    FD1P3AX expected_valid_1227 (.D(n29158), .SP(spi1_sck_c_enable_54), 
            .CK(spi1_sck_c), .Q(expected_valid)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam expected_valid_1227.GSR = "ENABLED";
    CCU2D fpga_time_3076_add_4_31 (.A0(fpga_time[29]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[30]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24850), .COUT(n24851), .S0(n136_adj_3195), 
          .S1(n135));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_31.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_31.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_31.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_31.INJECT1_1 = "NO";
    CCU2D fpga_time_3076_add_4_29 (.A0(fpga_time[27]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[28]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24849), .COUT(n24850), .S0(n138), .S1(n137));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_29.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_29.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_29.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_29.INJECT1_1 = "NO";
    LUT4 mic_acc_last_bdd_2_lut (.A(mic_acc_state[1]), .B(mic_acc_state[2]), 
         .Z(n28788)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam mic_acc_last_bdd_2_lut.init = 16'h4444;
    FD1P3AX spi_channel_page_1229 (.D(spi_channel_page_N_2499), .SP(spi1_sck_c_enable_55), 
            .CK(spi1_sck_c), .Q(spi_channel_page)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_channel_page_1229.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_171), 
            .CK(spi1_sck_c), .Q(spi_command[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_command_i0_i0.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3071__i0 (.D(n50), .CK(spi1_sck_c), .Q(spi_byte_count[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071__i0.GSR = "ENABLED";
    FD1S3AX spi_rgb_captured_3070__i0 (.D(n15_adj_3230), .CK(spi1_sck_c), 
            .Q(spi_rgb_captured[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(537[49:72])
    defparam spi_rgb_captured_3070__i0.GSR = "ENABLED";
    LUT4 i3_4_lut_adj_210 (.A(n31_adj_3309), .B(spi_rgb_start[7]), .C(n25850), 
         .D(n25879), .Z(n8_adj_3301)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[36] 155[83])
    defparam i3_4_lut_adj_210.init = 16'h4c00;
    CCU2D fpga_time_3076_add_4_27 (.A0(fpga_time[25]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[26]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24848), .COUT(n24849), .S0(n140), .S1(n139));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_27.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_27.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_27.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_27.INJECT1_1 = "NO";
    LUT4 i6_4_lut_adj_211 (.A(spi_extension_length[13]), .B(n12_adj_3271), 
         .C(spi_extension_length[8]), .D(spi_extension_length[11]), .Z(n11_adj_3236)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i6_4_lut_adj_211.init = 16'hfffe;
    LUT4 ev_run_hold_s5_83__I_0_i22_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_21), .D(ev_run_hold_s5[21]), .Z(us_tx_next[21])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i22_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i21_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_20), .D(ev_run_hold_s5[20]), .Z(us_tx_next[20])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i21_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i20_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_19), .D(ev_run_hold_s5[19]), .Z(us_tx_next[19])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i20_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_4_lut_adj_212 (.A(n8_adj_3298), .B(n25878), .C(spi_byte_count[5]), 
         .D(spi_byte_count[4]), .Z(n25879)) /* synthesis lut_function=(A (B+(C))+!A (B+(C (D)))) */ ;
    defparam i1_4_lut_adj_212.init = 16'hfcec;
    LUT4 i5_4_lut (.A(spi_extension_length[14]), .B(spi_extension_length[10]), 
         .C(spi_extension_length[12]), .D(spi_extension_length[15]), .Z(n12_adj_3271)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i5_4_lut.init = 16'hfffe;
    LUT4 i2_3_lut_4_lut_adj_213 (.A(spi_byte_count[0]), .B(n8), .C(spi_byte_count[1]), 
         .D(spi_byte_count[2]), .Z(spi1_sck_c_enable_113)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i2_3_lut_4_lut_adj_213.init = 16'h8000;
    LUT4 n27420_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[56]), 
         .C(status_hold[57]), .D(n27420), .Z(n27423)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27420_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 i1_2_lut_adj_214 (.A(spi_byte_count[6]), .B(spi_byte_count[7]), 
         .Z(n25850)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_214.init = 16'h8888;
    LUT4 mux_679_i6_3_lut_4_lut (.A(pending_sequence[5]), .B(accepted_sequence_spi[5]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[5])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i6_3_lut_4_lut.init = 16'hacaa;
    CCU2D fpga_time_3076_add_4_25 (.A0(fpga_time[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[24]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24847), .COUT(n24848), .S0(n142), .S1(n141));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_25.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_25.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_25.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_25.INJECT1_1 = "NO";
    LUT4 mic_acc_last_bdd_4_lut (.A(mic_run_state_1__N_2410[1]), .B(pll_clk_enable_602), 
         .C(mic_gate_active_N_2306), .D(n28789), .Z(pll_clk_enable_605)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam mic_acc_last_bdd_4_lut.init = 16'hfffe;
    LUT4 i13406_3_lut_4_lut (.A(pcm_rd_ptr_spi[1]), .B(pcm_wr_sync2_spi[1]), 
         .C(compact_end), .D(n16996), .Z(n24160)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(607[31:74])
    defparam i13406_3_lut_4_lut.init = 16'haaca;
    LUT4 ev_run_hold_s5_83__I_0_i19_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_18), .D(ev_run_hold_s5[18]), .Z(us_tx_next[18])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i19_3_lut_4_lut.init = 16'hbf40;
    LUT4 i13408_3_lut_4_lut (.A(pcm_rd_ptr_spi[0]), .B(pcm_wr_sync2_spi[0]), 
         .C(compact_end), .D(n16996), .Z(n24162)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(607[31:74])
    defparam i13408_3_lut_4_lut.init = 16'haaca;
    LUT4 n20772_bdd_2_lut_3_lut_4_lut (.A(spi_byte_count[6]), .B(spi_byte_count[7]), 
         .C(spi_byte_count[8]), .D(n28009), .Z(n28010)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(468[97:138])
    defparam n20772_bdd_2_lut_3_lut_4_lut.init = 16'hfffe;
    LUT4 i13402_3_lut_4_lut (.A(pcm_rd_ptr_spi[3]), .B(pcm_wr_sync2_spi[3]), 
         .C(compact_end), .D(n16996), .Z(n24156)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(607[31:74])
    defparam i13402_3_lut_4_lut.init = 16'haaca;
    LUT4 i2_3_lut_adj_215 (.A(mic_acc_word[1]), .B(mic_acc_word[2]), .C(mic_acc_word[0]), 
         .Z(n20918)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i2_3_lut_adj_215.init = 16'h8080;
    LUT4 i13404_3_lut_4_lut (.A(pcm_rd_ptr_spi[2]), .B(pcm_wr_sync2_spi[2]), 
         .C(compact_end), .D(n16996), .Z(n24158)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(607[31:74])
    defparam i13404_3_lut_4_lut.init = 16'haaca;
    LUT4 i13400_3_lut_4_lut (.A(pcm_rd_ptr_spi[4]), .B(pcm_wr_sync2_spi[4]), 
         .C(compact_end), .D(n16996), .Z(n24154)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(607[31:74])
    defparam i13400_3_lut_4_lut.init = 16'haaca;
    LUT4 ev_run_hold_s5_83__I_0_i18_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_17), .D(ev_run_hold_s5[17]), .Z(us_tx_next[17])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i18_3_lut_4_lut.init = 16'hbf40;
    PFUMX i22 (.BLUT(n26122), .ALUT(n9), .C0(spi_byte_count[5]), .Z(n11));
    LUT4 equal_2618_i4_2_lut (.A(mic_acc_state[1]), .B(mic_acc_state[2]), 
         .Z(n3)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam equal_2618_i4_2_lut.init = 16'heeee;
    LUT4 i1_2_lut_adj_216 (.A(n23802), .B(n44_adj_3257), .Z(spi1_sck_c_enable_17)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_216.init = 16'h4444;
    FD1S3AX spi_channel_index_3069__i0 (.D(n40_adj_3221), .CK(spi1_sck_c), 
            .Q(spi_channel_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069__i0.GSR = "ENABLED";
    FD1P3IX mod_settle__i3 (.D(n3639), .SP(pll_clk_enable_108), .CD(n10742), 
            .CK(pll_clk), .Q(mod_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mod_settle__i3.GSR = "DISABLED";
    FD1P3JX pcm_read_count_latched_spi_i0_i4 (.D(pcm_count_spi[4]), .SP(spi1_sck_c_enable_164), 
            .PD(n17823), .CK(spi1_sck_c), .Q(pcm_read_count_latched_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam pcm_read_count_latched_spi_i0_i4.GSR = "DISABLED";
    LUT4 i13853_2_lut_3_lut_4_lut (.A(n16924), .B(n10659), .C(n30), .D(spi_rgb_captured[0]), 
         .Z(n15_adj_3230)) /* synthesis lut_function=(A (B (C (D)+!C !(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i13853_2_lut_3_lut_4_lut.init = 16'hf708;
    FD1P3IX pcm_read_count_latched_spi_i0_i3 (.D(pcm_count_spi[3]), .SP(spi1_sck_c_enable_164), 
            .CD(n17823), .CK(spi1_sck_c), .Q(pcm_read_count_latched_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam pcm_read_count_latched_spi_i0_i3.GSR = "DISABLED";
    FD1P3IX pcm_read_count_latched_spi_i0_i2 (.D(pcm_count_spi[2]), .SP(spi1_sck_c_enable_164), 
            .CD(n17823), .CK(spi1_sck_c), .Q(pcm_read_count_latched_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam pcm_read_count_latched_spi_i0_i2.GSR = "DISABLED";
    LUT4 i4372_3_lut_3_lut (.A(status_next_index[1]), .B(status_hold[71]), 
         .C(status_hold[69]), .Z(n15211)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam i4372_3_lut_3_lut.init = 16'he4e4;
    LUT4 i13588_2_lut_3_lut (.A(mic_window_count[0]), .B(pcm_fifo_N_2624), 
         .C(pll_clk_enable_104), .Z(n24374)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(886[41:64])
    defparam i13588_2_lut_3_lut.init = 16'h0808;
    LUT4 ev_run_hold_s5_83__I_0_i17_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_16), .D(ev_run_hold_s5[16]), .Z(us_tx_next[16])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i17_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i43_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_42), .D(ev_run_hold_s5[42]), .Z(us_tx_next[42])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i43_3_lut_4_lut.init = 16'hbf40;
    CCU2D fpga_time_3076_add_4_23 (.A0(fpga_time[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24846), .COUT(n24847), .S0(n144), .S1(n143));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_23.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_23.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_23.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_23.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_adj_217 (.A(spi_byte_count[0]), .B(n8), .C(spi_byte_count[2]), 
         .Z(spi1_sck_c_enable_128)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_217.init = 16'h0808;
    FD1P3AX pcm_wr_ptr_3080__i0 (.D(n35_adj_3215), .SP(pll_clk_enable_601), 
            .CK(pll_clk), .Q(pcm_wr_ptr[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[39:56])
    defparam pcm_wr_ptr_3080__i0.GSR = "DISABLED";
    FD1P3IX pcm_read_count_latched_spi_i0_i1 (.D(pcm_count_spi[1]), .SP(spi1_sck_c_enable_164), 
            .CD(n17823), .CK(spi1_sck_c), .Q(pcm_read_count_latched_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam pcm_read_count_latched_spi_i0_i1.GSR = "DISABLED";
    LUT4 n27498_bdd_4_lut (.A(n27498), .B(\mic_win_q[0] [1]), .C(\mic_win_i[0] [1]), 
         .D(mic_acc_word[0]), .Z(n27501)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27498_bdd_4_lut.init = 16'haad8;
    LUT4 mic_acc_word_2__bdd_4_lut_16179 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [1]), 
         .C(\mic_win_q[1] [1]), .D(mic_acc_word[0]), .Z(n27498)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16179.init = 16'he4aa;
    LUT4 spi_byte_count_6__bdd_4_lut (.A(spi_command[3]), .B(spi_command[6]), 
         .C(spi_command[7]), .D(n28934), .Z(n28935)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;
    defparam spi_byte_count_6__bdd_4_lut.init = 16'h0200;
    LUT4 n27426_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[60]), 
         .C(status_hold[61]), .D(n27426), .Z(n27429)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27426_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 i1_2_lut_adj_218 (.A(mod_settle[0]), .B(n17036), .Z(frame_req_N_2710)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(704[22:40])
    defparam i1_2_lut_adj_218.init = 16'heeee;
    CCU2D fpga_time_3076_add_4_21 (.A0(fpga_time[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24845), .COUT(n24846), .S0(n146), .S1(n145));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_21.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_21.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_21.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_21.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i16_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_15), .D(ev_run_hold_s5[15]), .Z(us_tx_next[15])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i16_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27588_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[40]), 
         .C(status_hold[41]), .D(n27588), .Z(n27591)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27588_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 i4274_3_lut_4_lut (.A(ev_in_clear), .B(build_level_eff_7__N_1792[0]), 
         .C(build_idle_N_2724), .D(ev_sum[8]), .Z(n15113)) /* synthesis lut_function=(A (B (C+!(D))+!B !(D))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i4274_3_lut_4_lut.init = 16'hc0ea;
    LUT4 n27600_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[44]), 
         .C(status_hold[45]), .D(n27600), .Z(n27603)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27600_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 ev_run_hold_s5_83__I_0_i15_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_14), .D(ev_run_hold_s5[14]), .Z(us_tx_next[14])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i15_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27744_bdd_4_lut (.A(n27744), .B(\mic_win_q[0] [3]), .C(\mic_win_i[0] [3]), 
         .D(mic_acc_word[0]), .Z(n27747)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27744_bdd_4_lut.init = 16'haad8;
    CCU2D fpga_time_3076_add_4_19 (.A0(fpga_time[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24844), .COUT(n24845), .S0(n148), .S1(n147));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_19.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_19.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_19.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_19.INJECT1_1 = "NO";
    PFUMX i16805 (.BLUT(n28482), .ALUT(n28483), .C0(pcm_spi_sample_15__N_1381[4]), 
          .Z(n28484));
    LUT4 mic_acc_word_2__bdd_4_lut_16377 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [3]), 
         .C(\mic_win_q[1] [3]), .D(mic_acc_word[0]), .Z(n27744)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16377.init = 16'he4aa;
    LUT4 i9919_4_lut (.A(compact_end), .B(spi_mod_feed_N_2490), .C(n26096), 
         .D(n21415), .Z(frame_end)) /* synthesis lut_function=(A (B (C+!(D)))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(472[29] 473[90])
    defparam i9919_4_lut.init = 16'hc088;
    LUT4 ev_run_hold_s5_83__I_0_i14_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_13), .D(ev_run_hold_s5[13]), .Z(us_tx_next[13])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i14_3_lut_4_lut.init = 16'hbf40;
    FD1P3IX mic_block_count_3084__i1 (.D(n84), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i1.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i1_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_0), .D(ev_run_hold_s5[0]), .Z(us_tx_next[0])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i1_3_lut_4_lut.init = 16'hbf40;
    LUT4 n10893_bdd_4_lut (.A(mic_acc_state[1]), .B(mic_acc_state[0]), .C(n20918), 
         .D(mic_acc_state[2]), .Z(n28692)) /* synthesis lut_function=(A ((C+(D))+!B)+!A ((D)+!B)) */ ;
    defparam n10893_bdd_4_lut.init = 16'hffb3;
    LUT4 i10144_4_lut (.A(ev_rd_data[56]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[56])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10144_4_lut.init = 16'h1222;
    LUT4 i4269_4_lut (.A(ev_sum[0]), .B(staging_q[0]), .C(staging_rd_page_N_2728), 
         .D(ev_in_clear), .Z(n15108)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i4269_4_lut.init = 16'hcac0;
    LUT4 ev_run_hold_s5_83__I_0_i69_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_68), .D(ev_run_hold_s5[68]), .Z(us_tx_next[68])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i69_3_lut_4_lut.init = 16'hbf40;
    LUT4 i15437_2_lut_3_lut (.A(spi_byte_count[0]), .B(n8), .C(spi_byte_count[2]), 
         .Z(spi1_sck_c_enable_121)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15437_2_lut_3_lut.init = 16'h0404;
    LUT4 ev_run_hold_s5_83__I_0_i2_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_1), .D(ev_run_hold_s5[1]), .Z(us_tx_next[1])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i2_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i3_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_2), .D(ev_run_hold_s5[2]), .Z(us_tx_next[2])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i3_3_lut_4_lut.init = 16'hbf40;
    CCU2D fpga_time_3076_add_4_17 (.A0(fpga_time[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24843), .COUT(n24844), .S0(n150), .S1(n149));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_17.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_17.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_17.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_17.INJECT1_1 = "NO";
    LUT4 n27756_bdd_4_lut (.A(n27756), .B(\mic_win_q[0] [2]), .C(\mic_win_i[0] [2]), 
         .D(mic_acc_word[0]), .Z(n27759)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27756_bdd_4_lut.init = 16'haad8;
    LUT4 mic_acc_word_2__bdd_4_lut_16387 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [2]), 
         .C(\mic_win_q[1] [2]), .D(mic_acc_word[0]), .Z(n27756)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16387.init = 16'he4aa;
    LUT4 i3211_2_lut (.A(ev_fall), .B(ev_fall_N_2693), .Z(n13703)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i3211_2_lut.init = 16'h8888;
    FD1P3AX spi_extension_length_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_extension_length[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i8.GSR = "ENABLED";
    LUT4 i2_3_lut_adj_219 (.A(mod_settle[3]), .B(mod_settle[1]), .C(mod_settle[2]), 
         .Z(n17036)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(704[22:40])
    defparam i2_3_lut_adj_219.init = 16'hfefe;
    LUT4 n27606_bdd_4_lut_16479_4_lut (.A(status_next_index[1]), .B(status_hold[20]), 
         .C(status_hold[21]), .D(n27606), .Z(n27609)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27606_bdd_4_lut_16479_4_lut.init = 16'hf588;
    LUT4 ev_run_hold_s5_83__I_0_i44_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_43), .D(ev_run_hold_s5[43]), .Z(us_tx_next[43])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i44_3_lut_4_lut.init = 16'hbf40;
    CCU2D mic_block_count_3084_add_4_15 (.A0(mic_block_count[13]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[14]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24750), .COUT(n24751), .S0(n72_adj_3188), 
          .S1(n71_adj_3192));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084_add_4_15.INIT0 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_15.INIT1 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_15.INJECT1_0 = "NO";
    defparam mic_block_count_3084_add_4_15.INJECT1_1 = "NO";
    CCU2D fpga_time_3076_add_4_15 (.A0(fpga_time[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24842), .COUT(n24843), .S0(n152), .S1(n151));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_15.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_15.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_15.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_15.INJECT1_1 = "NO";
    LUT4 n27636_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[24]), 
         .C(status_hold[25]), .D(n27636), .Z(n27639)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27636_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 mod_toggle_sync_I_0_2_lut (.A(mod_toggle_sync), .B(mod_toggle_seen), 
         .Z(feed_rd_9__N_1712)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(701[13:47])
    defparam mod_toggle_sync_I_0_2_lut.init = 16'h6666;
    LUT4 ev_run_hold_s5_83__I_0_i13_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_12), .D(ev_run_hold_s5[12]), .Z(us_tx_next[12])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i13_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i45_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_44), .D(ev_run_hold_s5[44]), .Z(us_tx_next[44])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i45_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i46_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_45), .D(ev_run_hold_s5[45]), .Z(us_tx_next[45])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i46_3_lut_4_lut.init = 16'hbf40;
    LUT4 i13919_2_lut (.A(phase_step_s1), .B(global_phase_s2[0]), .Z(n45)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13919_2_lut.init = 16'h6666;
    LUT4 ev_run_hold_s5_83__I_0_i12_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_11), .D(ev_run_hold_s5[11]), .Z(us_tx_next[11])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i12_3_lut_4_lut.init = 16'hbf40;
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
    FD1P3AX spi_extension_length_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_extension_length[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_extension_length[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_extension_length[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_extension_length[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_extension_length[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_extension_length[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_extension_length[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_extension_length_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_frame_sequence_i0_i15.GSR = "ENABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_105), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i16.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_105), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i17.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_105), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i18.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_105), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i19.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_105), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i20.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_105), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i21.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_105), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i22.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_105), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i23.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i24 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_113), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i24.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i25 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_113), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i26 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_113), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i26.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i27 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_113), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i27.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i28 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_113), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i28.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i29 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_113), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i29.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i30 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_113), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i30.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i31 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_113), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i31.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i32 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_121), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i32.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i33 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_121), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i33.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i34 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_121), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i34.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i35 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_121), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i35.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i36 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_121), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i36.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i37 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_121), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i37.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i38 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_121), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i38.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i39 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_121), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i39.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i40 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_128), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i40.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i41 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_128), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i41.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i42 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_128), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i42.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i43 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_128), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i43.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i44 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_128), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i44.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i45 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_128), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i45.GSR = "DISABLED";
    FD1P3AX mic_cfg_hold_spi_i0_i46 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_128), 
            .CK(spi1_sck_c), .Q(mic_cfg_hold_spi[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_hold_spi_i0_i46.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(rgb_values[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i1.GSR = "DISABLED";
    FD1P3IX mic_cfg_count_i2 (.D(mic_cfg_hold_spi[2]), .SP(pll_clk_enable_602), 
            .CD(n17763), .CK(pll_clk), .Q(mic_cfg_count[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_count_i2.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(rgb_values[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i2.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(rgb_values[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i3.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(rgb_values[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i4.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(rgb_values[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i5.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(rgb_values[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i6.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(rgb_values[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i7.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(rgb_values[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i8.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(rgb_values[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i9.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(rgb_values[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i10.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(rgb_values[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i11.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(rgb_values[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i12.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(rgb_values[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i13.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(rgb_values[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i14.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .Q(rgb_values[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i15.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_151), 
            .CK(spi1_sck_c), .Q(rgb_values[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i16.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_151), 
            .CK(spi1_sck_c), .Q(rgb_values[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i17.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_151), 
            .CK(spi1_sck_c), .Q(rgb_values[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i18.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_151), 
            .CK(spi1_sck_c), .Q(rgb_values[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i19.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_151), 
            .CK(spi1_sck_c), .Q(rgb_values[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i20.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_151), 
            .CK(spi1_sck_c), .Q(rgb_values[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i21.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_151), 
            .CK(spi1_sck_c), .Q(rgb_values[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i22.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_151), 
            .CK(spi1_sck_c), .Q(rgb_values[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_i0_i23.GSR = "DISABLED";
    FD1P3AX mod_level_spi_i0_i1 (.D(spi_compact_data[1]), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .Q(mod_level_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mod_level_spi_i0_i1.GSR = "DISABLED";
    FD1P3IX mic_window_count__i1 (.D(n10808), .SP(pll_clk_enable_117), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_window_count[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_window_count__i1.GSR = "DISABLED";
    FD1P3IX mic_window_count__i2 (.D(n10809), .SP(pll_clk_enable_117), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_window_count[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_window_count__i2.GSR = "DISABLED";
    FD1P3IX mic_window_count__i3 (.D(n10810), .SP(pll_clk_enable_117), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_window_count[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_window_count__i3.GSR = "DISABLED";
    FD1P3IX mic_window_count__i4 (.D(n10811), .SP(pll_clk_enable_117), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_window_count[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_window_count__i4.GSR = "DISABLED";
    FD1P3IX mic_window_count__i5 (.D(n10812), .SP(pll_clk_enable_117), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_window_count[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_window_count__i5.GSR = "DISABLED";
    FD1P3IX mic_window_count__i6 (.D(n10813), .SP(pll_clk_enable_117), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_window_count[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_window_count__i6.GSR = "DISABLED";
    FD1P3IX mod_settle__i1 (.D(n16280), .SP(frame_req_N_2710), .CD(n15546), 
            .CK(pll_clk), .Q(mod_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mod_settle__i1.GSR = "DISABLED";
    FD1P3IX mod_settle__i2 (.D(n16282), .SP(frame_req_N_2710), .CD(n15546), 
            .CK(pll_clk), .Q(mod_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mod_settle__i2.GSR = "DISABLED";
    LUT4 n27648_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[28]), 
         .C(status_hold[29]), .D(n27648), .Z(n27651)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27648_bdd_4_lut_4_lut.init = 16'hf588;
    FD1S3AX pcm_rd_sync1_i1 (.D(pcm_rd_ptr_spi[1]), .CK(pll_clk), .Q(pcm_rd_sync1[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync1_i1.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i2 (.D(pcm_rd_ptr_spi[2]), .CK(pll_clk), .Q(pcm_rd_sync1[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync1_i2.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i3 (.D(pcm_rd_ptr_spi[3]), .CK(pll_clk), .Q(pcm_rd_sync1[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync1_i3.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i4 (.D(pcm_rd_ptr_spi[4]), .CK(pll_clk), .Q(pcm_rd_sync1[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync1_i4.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync1_i5 (.D(pcm_rd_ptr_spi[5]), .CK(pll_clk), .Q(pcm_rd_sync1[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync1_i5.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i1 (.D(pcm_rd_sync1[1]), .CK(pll_clk), .Q(pcm_rd_sync2[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync2_i1.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i2 (.D(pcm_rd_sync1[2]), .CK(pll_clk), .Q(pcm_rd_sync2[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync2_i2.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i3 (.D(pcm_rd_sync1[3]), .CK(pll_clk), .Q(pcm_rd_sync2[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync2_i3.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i4 (.D(pcm_rd_sync1[4]), .CK(pll_clk), .Q(pcm_rd_sync2[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync2_i4.GSR = "DISABLED";
    FD1S3AX pcm_rd_sync2_i5 (.D(pcm_rd_sync1[5]), .CK(pll_clk), .Q(pcm_rd_sync2[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(593[12] 596[8])
    defparam pcm_rd_sync2_i5.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i1 (.D(pcm_wr_ptr[1]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync1_spi_i1.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i2 (.D(pcm_wr_ptr[2]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync1_spi_i2.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i3 (.D(pcm_wr_ptr[3]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync1_spi_i3.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i4 (.D(pcm_wr_ptr[4]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync1_spi_i4.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync1_spi_i5 (.D(pcm_wr_ptr[5]), .CK(spi1_sck_c), .Q(pcm_wr_sync1_spi[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync1_spi_i5.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i1 (.D(pcm_wr_sync1_spi[1]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync2_spi_i1.GSR = "DISABLED";
    FD1P3IX mic_block_count_3084__i2 (.D(n83), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i2.GSR = "DISABLED";
    CCU2D mic_block_count_3084_add_4_13 (.A0(mic_block_count[11]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[12]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24749), .COUT(n24750), .S0(n74_adj_3187), 
          .S1(n73));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084_add_4_13.INIT0 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_13.INIT1 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_13.INJECT1_0 = "NO";
    defparam mic_block_count_3084_add_4_13.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3084__i3 (.D(n82), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i3.GSR = "DISABLED";
    CCU2D mic_block_count_3084_add_4_11 (.A0(mic_block_count[9]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[10]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24748), .COUT(n24749), .S0(n76), 
          .S1(n75));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084_add_4_11.INIT0 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_11.INIT1 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_11.INJECT1_0 = "NO";
    defparam mic_block_count_3084_add_4_11.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3084__i4 (.D(n81), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i4.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i2 (.D(pcm_wr_sync1_spi[2]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync2_spi_i2.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i3 (.D(pcm_wr_sync1_spi[3]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync2_spi_i3.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i4 (.D(pcm_wr_sync1_spi[4]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync2_spi_i4.GSR = "DISABLED";
    FD1S3AX pcm_wr_sync2_spi_i5 (.D(pcm_wr_sync1_spi[5]), .CK(spi1_sck_c), 
            .Q(pcm_wr_sync2_spi[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_wr_sync2_spi_i5.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i1 (.D(pcm_rd_ptr_spi_5__N_1626[1]), .SP(spi1_sck_c_enable_157), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_rd_ptr_spi_i1.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i2 (.D(pcm_rd_ptr_spi_5__N_1626[2]), .SP(spi1_sck_c_enable_157), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_rd_ptr_spi_i2.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i3 (.D(pcm_rd_ptr_spi_5__N_1626[3]), .SP(spi1_sck_c_enable_157), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_rd_ptr_spi_i3.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i4 (.D(pcm_rd_ptr_spi_5__N_1626[4]), .SP(spi1_sck_c_enable_157), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_rd_ptr_spi_i4.GSR = "DISABLED";
    FD1P3AX pcm_rd_ptr_spi_i5 (.D(pcm_rd_ptr_spi_5__N_1626[5]), .SP(spi1_sck_c_enable_157), 
            .CK(spi1_sck_c), .Q(pcm_rd_ptr_spi[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(601[12] 608[8])
    defparam pcm_rd_ptr_spi_i5.GSR = "DISABLED";
    FD1S3AX phase_frac_i1 (.D(phase_frac_sum[1]), .CK(pll_clk), .Q(phase_frac[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i1.GSR = "DISABLED";
    FD1S3AX phase_frac_i2 (.D(phase_frac_sum[2]), .CK(pll_clk), .Q(phase_frac[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i2.GSR = "DISABLED";
    FD1S3AX phase_frac_i3 (.D(phase_frac_sum[3]), .CK(pll_clk), .Q(phase_frac[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i3.GSR = "DISABLED";
    FD1S3AX phase_frac_i4 (.D(phase_frac_sum[4]), .CK(pll_clk), .Q(phase_frac[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i4.GSR = "DISABLED";
    FD1S3AX phase_frac_i5 (.D(phase_frac_sum[5]), .CK(pll_clk), .Q(phase_frac[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i5.GSR = "DISABLED";
    FD1S3AX phase_frac_i6 (.D(phase_frac_sum[6]), .CK(pll_clk), .Q(phase_frac[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i6.GSR = "DISABLED";
    FD1S3AX phase_frac_i7 (.D(phase_frac_sum[7]), .CK(pll_clk), .Q(phase_frac[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i7.GSR = "DISABLED";
    FD1S3AX phase_frac_i8 (.D(phase_frac_sum[8]), .CK(pll_clk), .Q(phase_frac[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i8.GSR = "DISABLED";
    FD1S3AX phase_frac_i9 (.D(phase_frac_sum[9]), .CK(pll_clk), .Q(phase_frac[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i9.GSR = "DISABLED";
    FD1S3AX phase_frac_i10 (.D(phase_frac_sum[10]), .CK(pll_clk), .Q(phase_frac[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i10.GSR = "DISABLED";
    FD1S3AX phase_frac_i11 (.D(phase_frac_sum[11]), .CK(pll_clk), .Q(phase_frac[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i11.GSR = "DISABLED";
    FD1S3AX phase_frac_i12 (.D(phase_frac_sum[12]), .CK(pll_clk), .Q(phase_frac[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i12.GSR = "DISABLED";
    FD1S3AX phase_frac_i13 (.D(phase_frac_sum[13]), .CK(pll_clk), .Q(phase_frac[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i13.GSR = "DISABLED";
    FD1S3AX phase_frac_i14 (.D(phase_frac_sum[14]), .CK(pll_clk), .Q(phase_frac[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i14.GSR = "DISABLED";
    FD1S3AX phase_frac_i15 (.D(phase_frac_sum[15]), .CK(pll_clk), .Q(phase_frac[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i15.GSR = "DISABLED";
    FD1S3AX phase_frac_i16 (.D(phase_frac_sum[16]), .CK(pll_clk), .Q(phase_frac[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i16.GSR = "DISABLED";
    FD1S3AX phase_frac_i17 (.D(phase_frac_sum[17]), .CK(pll_clk), .Q(phase_frac[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i17.GSR = "DISABLED";
    FD1S3AX phase_frac_i18 (.D(phase_frac_sum[18]), .CK(pll_clk), .Q(phase_frac[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i18.GSR = "DISABLED";
    FD1S3AX phase_frac_i19 (.D(phase_frac_sum[19]), .CK(pll_clk), .Q(phase_frac[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i19.GSR = "DISABLED";
    FD1S3AX phase_frac_i20 (.D(phase_frac_sum[20]), .CK(pll_clk), .Q(phase_frac[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i20.GSR = "DISABLED";
    FD1S3AX phase_frac_i21 (.D(phase_frac_sum[21]), .CK(pll_clk), .Q(phase_frac[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i21.GSR = "DISABLED";
    FD1S3AX phase_frac_i22 (.D(phase_frac_sum[22]), .CK(pll_clk), .Q(phase_frac[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i22.GSR = "DISABLED";
    FD1S3AX phase_frac_i23 (.D(phase_frac_sum[23]), .CK(pll_clk), .Q(phase_frac[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam phase_frac_i23.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i47_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_46), .D(ev_run_hold_s5[46]), .Z(us_tx_next[46])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i47_3_lut_4_lut.init = 16'hbf40;
    CCU2D mic_block_count_3084_add_4_9 (.A0(mic_block_count[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[8]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24747), .COUT(n24748), .S0(n78_adj_3186), 
          .S1(n77));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084_add_4_9.INIT0 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_9.INIT1 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_9.INJECT1_0 = "NO";
    defparam mic_block_count_3084_add_4_9.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3084__i5 (.D(n80), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i5.GSR = "DISABLED";
    CCU2D mic_block_count_3084_add_4_7 (.A0(mic_block_count[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24746), .COUT(n24747), .S0(n80), 
          .S1(n79_adj_3185));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_7.INJECT1_0 = "NO";
    defparam mic_block_count_3084_add_4_7.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3084__i6 (.D(n79_adj_3185), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i6.GSR = "DISABLED";
    CCU2D mic_block_count_3084_add_4_5 (.A0(mic_block_count[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24745), .COUT(n24746), .S0(n82), 
          .S1(n81));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_5.INJECT1_0 = "NO";
    defparam mic_block_count_3084_add_4_5.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3084__i7 (.D(n78_adj_3186), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i7.GSR = "DISABLED";
    CCU2D mic_block_count_3084_add_4_3 (.A0(mic_block_count[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_block_count[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24744), .COUT(n24745), .S0(n84), 
          .S1(n83));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_block_count_3084_add_4_3.INJECT1_0 = "NO";
    defparam mic_block_count_3084_add_4_3.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3084__i8 (.D(n77), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i8.GSR = "DISABLED";
    CCU2D mic_block_count_3084_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_block_count[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24744), .S1(n85));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084_add_4_1.INIT0 = 16'hF000;
    defparam mic_block_count_3084_add_4_1.INIT1 = 16'h0555;
    defparam mic_block_count_3084_add_4_1.INJECT1_0 = "NO";
    defparam mic_block_count_3084_add_4_1.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3084__i9 (.D(n76), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i9.GSR = "DISABLED";
    CCU2D status_bit_index_8__I_0_1373_9 (.A0(status_bit_index[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[8]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24742), .S0(status_bit_index_8__N_1387[7]), 
          .S1(status_bit_index_8__N_1387[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(427[37:60])
    defparam status_bit_index_8__I_0_1373_9.INIT0 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1373_9.INIT1 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1373_9.INJECT1_0 = "NO";
    defparam status_bit_index_8__I_0_1373_9.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3084__i10 (.D(n75), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[10])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i10.GSR = "DISABLED";
    CCU2D status_bit_index_8__I_0_1373_7 (.A0(status_bit_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24741), .COUT(n24742), .S0(status_next_index[5]), 
          .S1(status_next_index[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(427[37:60])
    defparam status_bit_index_8__I_0_1373_7.INIT0 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1373_7.INIT1 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1373_7.INJECT1_0 = "NO";
    defparam status_bit_index_8__I_0_1373_7.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3084__i11 (.D(n74_adj_3187), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[11])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i11.GSR = "DISABLED";
    CCU2D status_bit_index_8__I_0_1373_5 (.A0(status_bit_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24740), .COUT(n24741), .S0(status_next_index[3]), 
          .S1(status_next_index[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(427[37:60])
    defparam status_bit_index_8__I_0_1373_5.INIT0 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1373_5.INIT1 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1373_5.INJECT1_0 = "NO";
    defparam status_bit_index_8__I_0_1373_5.INJECT1_1 = "NO";
    FD1P3IX mic_block_count_3084__i12 (.D(n73), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[12])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i12.GSR = "DISABLED";
    FD1P3IX mic_block_count_3084__i13 (.D(n72_adj_3188), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[13])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i13.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i1 (.D(feed_q[1]), .SP(pll_clk_enable_150), 
            .CD(n13685), .CK(pll_clk), .Q(pending_mod_level[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_mod_level__i1.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i2 (.D(feed_q[2]), .SP(pll_clk_enable_150), 
            .CD(n13685), .CK(pll_clk), .Q(pending_mod_level[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_mod_level__i2.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i3 (.D(feed_q[3]), .SP(pll_clk_enable_150), 
            .CD(n13685), .CK(pll_clk), .Q(pending_mod_level[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_mod_level__i3.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i4 (.D(feed_q[4]), .SP(pll_clk_enable_150), 
            .CD(n13685), .CK(pll_clk), .Q(pending_mod_level[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_mod_level__i4.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i5 (.D(feed_q[5]), .SP(pll_clk_enable_150), 
            .CD(n13685), .CK(pll_clk), .Q(pending_mod_level[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_mod_level__i5.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i6 (.D(feed_q[6]), .SP(pll_clk_enable_150), 
            .CD(n13685), .CK(pll_clk), .Q(pending_mod_level[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_mod_level__i6.GSR = "DISABLED";
    FD1P3IX pending_mod_level__i7 (.D(feed_q[7]), .SP(pll_clk_enable_150), 
            .CD(n13685), .CK(pll_clk), .Q(pending_mod_level[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_mod_level__i7.GSR = "DISABLED";
    FD1S3AX spi_bit_count_3072__i1 (.D(n19), .CK(spi1_sck_c), .Q(spi_bit_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[34:54])
    defparam spi_bit_count_3072__i1.GSR = "ENABLED";
    LUT4 i2_3_lut_adj_220 (.A(accepted_update_flags_spi[0]), .B(frame_req_mod_N_2752), 
         .C(n16857), .Z(pll_clk_enable_5)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(704[18] 714[12])
    defparam i2_3_lut_adj_220.init = 16'hecec;
    FD1P3IX mic_block_count_3084__i14 (.D(n71_adj_3192), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[14])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i14.GSR = "DISABLED";
    FD1S3AX spi_bit_count_3072__i2 (.D(n18), .CK(spi1_sck_c), .Q(spi_bit_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[34:54])
    defparam spi_bit_count_3072__i2.GSR = "ENABLED";
    FD1P3AX accepted_update_flags_spi_i0_i1 (.D(expected_next_8__N_1406[3]), 
            .SP(spi1_sck_c_enable_161), .CK(spi1_sck_c), .Q(accepted_update_flags_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam accepted_update_flags_spi_i0_i1.GSR = "DISABLED";
    CCU2D status_bit_index_8__I_0_1373_3 (.A0(status_bit_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24739), .COUT(n24740), .S0(status_next_index[1]), 
          .S1(status_next_index[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(427[37:60])
    defparam status_bit_index_8__I_0_1373_3.INIT0 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1373_3.INIT1 = 16'h5aaa;
    defparam status_bit_index_8__I_0_1373_3.INJECT1_0 = "NO";
    defparam status_bit_index_8__I_0_1373_3.INJECT1_1 = "NO";
    LUT4 n10893_bdd_2_lut (.A(n10893), .B(n28692), .Z(n28693)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam n10893_bdd_2_lut.init = 16'h4444;
    LUT4 n27768_bdd_4_lut (.A(n27768), .B(\mic_win_q[2] [7]), .C(\mic_win_i[2] [7]), 
         .D(mic_acc_word[0]), .Z(n27771)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27768_bdd_4_lut.init = 16'haad8;
    LUT4 mic_acc_word_2__bdd_4_lut_16397 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [7]), 
         .C(\mic_win_q[3] [7]), .D(mic_acc_word[0]), .Z(n27768)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16397.init = 16'he4aa;
    FD1P3AX mic_cfg_start_i0_i1 (.D(mic_cfg_hold_spi[9]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i1.GSR = "DISABLED";
    LUT4 spi1_miso_I_28_i53_4_lut_4_lut (.A(status_bit_index[2]), .B(n20896), 
         .C(n26132), .D(n28463), .Z(n53_adj_3205)) /* synthesis lut_function=(A (D)+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(448[69:94])
    defparam spi1_miso_I_28_i53_4_lut_4_lut.init = 16'hea40;
    LUT4 i15187_2_lut_2_lut_3_lut (.A(ev_fall), .B(rmw_slot[0]), .C(build_level_eff[0]), 
         .Z(n26156)) /* synthesis lut_function=(!(A+(B (C)+!B !(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(767[21:29])
    defparam i15187_2_lut_2_lut_3_lut.init = 16'h1414;
    LUT4 i1_2_lut_3_lut_4_lut_adj_221 (.A(mic_phase[3]), .B(mic_phase[1]), 
         .C(mic_phase[2]), .D(mic_phase[0]), .Z(pcm_fifo_N_2624)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_3_lut_4_lut_adj_221.init = 16'h0020;
    LUT4 n27762_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[16]), 
         .C(status_hold[17]), .D(n27762), .Z(n27242)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27762_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 ev_run_hold_s5_83__I_0_i48_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_47), .D(ev_run_hold_s5[47]), .Z(us_tx_next[47])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i48_3_lut_4_lut.init = 16'hbf40;
    FD1P3AX mic_cfg_start_i0_i2 (.D(mic_cfg_hold_spi[10]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i3 (.D(mic_cfg_hold_spi[11]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i4 (.D(mic_cfg_hold_spi[12]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i5 (.D(mic_cfg_hold_spi[13]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i6 (.D(mic_cfg_hold_spi[14]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i7 (.D(mic_cfg_hold_spi[15]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i8 (.D(mic_cfg_hold_spi[16]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i9 (.D(mic_cfg_hold_spi[17]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i10 (.D(mic_cfg_hold_spi[18]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i11 (.D(mic_cfg_hold_spi[19]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i12 (.D(mic_cfg_hold_spi[20]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i13 (.D(mic_cfg_hold_spi[21]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i14 (.D(mic_cfg_hold_spi[22]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_cfg_start_i0_i15 (.D(mic_cfg_hold_spi[23]), .SP(pll_clk_enable_602), 
            .CK(pll_clk), .Q(mic_cfg_start[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_start_i0_i15.GSR = "DISABLED";
    FD1S3AX time_divider_3075__i1 (.D(n34), .CK(pll_clk), .Q(time_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(649[25:44])
    defparam time_divider_3075__i1.GSR = "DISABLED";
    FD1P3IX mic_block_count_3084__i15 (.D(n70), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[15])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i15.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_222 (.A(pll_clk_enable_517), .B(n10742), .Z(n16190)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_adj_222.init = 16'heeee;
    FD1S3AX time_divider_3075__i2 (.D(n33), .CK(pll_clk), .Q(time_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(649[25:44])
    defparam time_divider_3075__i2.GSR = "DISABLED";
    FD1S3AX time_divider_3075__i3 (.D(n32), .CK(pll_clk), .Q(time_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(649[25:44])
    defparam time_divider_3075__i3.GSR = "DISABLED";
    FD1S3AX time_divider_3075__i4 (.D(n31_adj_3183), .CK(pll_clk), .Q(time_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(649[25:44])
    defparam time_divider_3075__i4.GSR = "DISABLED";
    FD1S3AX time_divider_3075__i5 (.D(n30_adj_3182), .CK(pll_clk), .Q(time_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(649[25:44])
    defparam time_divider_3075__i5.GSR = "DISABLED";
    FD1P3AX status_bit_index_i0_i1 (.D(status_next_index[1]), .SP(spi1_sck_N_161_enable_9), 
            .CK(spi1_sck_N_161), .Q(status_bit_index[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(454[14:90])
    defparam status_bit_index_i0_i1.GSR = "ENABLED";
    PFUMX i99 (.BLUT(n78_adj_3190), .ALUT(n26098), .C0(spi_byte_count[7]), 
          .Z(n97));
    LUT4 n27576_bdd_4_lut_else_3_lut (.A(n14129), .B(n14130), .C(status_bit_index[0]), 
         .Z(n28428)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27576_bdd_4_lut_else_3_lut.init = 16'hacac;
    FD1P3IX ev_ch_3078__i0 (.D(n40_adj_3200), .SP(pll_clk_enable_168), .CD(n17806), 
            .CK(pll_clk), .Q(ev_ch[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078__i0.GSR = "DISABLED";
    CCU2D status_bit_index_8__I_0_1373_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24739), .S1(status_next_index[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(427[37:60])
    defparam status_bit_index_8__I_0_1373_1.INIT0 = 16'hF000;
    defparam status_bit_index_8__I_0_1373_1.INIT1 = 16'h5555;
    defparam status_bit_index_8__I_0_1373_1.INJECT1_0 = "NO";
    defparam status_bit_index_8__I_0_1373_1.INJECT1_1 = "NO";
    FD1P3IX mic_warm_half_3079__i0 (.D(n45_adj_3207), .SP(pll_clk_enable_169), 
            .CD(n20483), .CK(pll_clk), .Q(mic_warm_half[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079__i0.GSR = "DISABLED";
    CCU2D add_2479_8 (.A0(spi1_mosi_c_0), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24738), .S0(n11988));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(458[33] 461[86])
    defparam add_2479_8.INIT0 = 16'h5aaa;
    defparam add_2479_8.INIT1 = 16'h0000;
    defparam add_2479_8.INJECT1_0 = "NO";
    defparam add_2479_8.INJECT1_1 = "NO";
    FD1P3AX frame_toggle_spi_1239 (.D(frame_toggle_spi_N_2518), .SP(spi1_sck_c_enable_159), 
            .CK(spi1_sck_c), .Q(frame_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam frame_toggle_spi_1239.GSR = "DISABLED";
    CCU2D add_2479_6 (.A0(spi_extension_length[6]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_extension_length[7]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24737), .COUT(n24738), .S0(n11992), .S1(n11990));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(458[33] 461[86])
    defparam add_2479_6.INIT0 = 16'h5aaa;
    defparam add_2479_6.INIT1 = 16'h5aaa;
    defparam add_2479_6.INJECT1_0 = "NO";
    defparam add_2479_6.INJECT1_1 = "NO";
    FD1P3AX stop_toggle_spi_1242 (.D(stop_toggle_spi_N_2529), .SP(spi1_sck_c_enable_160), 
            .CK(spi1_sck_c), .Q(stop_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam stop_toggle_spi_1242.GSR = "DISABLED";
    CCU2D add_2479_4 (.A0(spi_extension_length[4]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_extension_length[5]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24736), .COUT(n24737), .S0(n11996), .S1(n11994));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(458[33] 461[86])
    defparam add_2479_4.INIT0 = 16'h5aaa;
    defparam add_2479_4.INIT1 = 16'h5aaa;
    defparam add_2479_4.INJECT1_0 = "NO";
    defparam add_2479_4.INJECT1_1 = "NO";
    LUT4 mic_acc_state_1__bdd_4_lut (.A(mic_acc_state[1]), .B(n28134), .C(mic_acc_state[0]), 
         .D(mic_acc_state[2]), .Z(n28136)) /* synthesis lut_function=(!(A (B+(D))+!A (B (C+!(D))+!B (C (D)+!C !(D))))) */ ;
    defparam mic_acc_state_1__bdd_4_lut.init = 16'h0532;
    FD1P3IX rgb_values_complete_spi_1241 (.D(n16500), .SP(spi1_sck_c_enable_161), 
            .CD(n17802), .CK(spi1_sck_c), .Q(rgb_values_complete_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam rgb_values_complete_spi_1241.GSR = "DISABLED";
    CCU2D add_2479_2 (.A0(spi_extension_length[2]), .B0(expected_next_8__N_1406[3]), 
          .C0(GND_net), .D0(GND_net), .A1(spi_extension_length[3]), .B1(expected_next_8__N_1406[3]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24736), .S1(n11998));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(458[33] 461[86])
    defparam add_2479_2.INIT0 = 16'h7000;
    defparam add_2479_2.INIT1 = 16'h5666;
    defparam add_2479_2.INJECT1_0 = "NO";
    defparam add_2479_2.INJECT1_1 = "NO";
    FD1P3AX ws2812_toggle_spi_1243 (.D(ws2812_toggle_spi_N_2565), .SP(spi1_sck_c_enable_162), 
            .CK(spi1_sck_c), .Q(ws2812_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam ws2812_toggle_spi_1243.GSR = "DISABLED";
    FD1P3AX mic_cfg_toggle_spi_1244 (.D(mic_cfg_toggle_spi_N_2691), .SP(spi1_sck_c_enable_163), 
            .CK(spi1_sck_c), .Q(mic_cfg_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam mic_cfg_toggle_spi_1244.GSR = "DISABLED";
    CCU2D mic_lo_n_6__I_0_7 (.A0(mic_lo_n[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_lo_n[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24733), .S0(mic_lo_n_next_6__N_648[5]), 
          .S1(mic_lo_n_next_6__N_648[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[62:79])
    defparam mic_lo_n_6__I_0_7.INIT0 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_7.INIT1 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_7.INJECT1_0 = "NO";
    defparam mic_lo_n_6__I_0_7.INJECT1_1 = "NO";
    FD1P3JX build_mod_1305 (.D(mod_active), .SP(pll_clk_enable_517), .PD(n17799), 
            .CK(pll_clk), .Q(build_mod)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_mod_1305.GSR = "DISABLED";
    CCU2D mic_lo_n_6__I_0_5 (.A0(mic_lo_n[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_lo_n[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24732), .COUT(n24733), .S0(mic_lo_n_next_6__N_648[3]), 
          .S1(mic_lo_n_next_6__N_648[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[62:79])
    defparam mic_lo_n_6__I_0_5.INIT0 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_5.INIT1 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_5.INJECT1_0 = "NO";
    defparam mic_lo_n_6__I_0_5.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i49_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_48), .D(ev_run_hold_s5[48]), .Z(us_tx_next[48])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i49_3_lut_4_lut.init = 16'hbf40;
    FD1P3JX mic_lo_i_r_1315 (.D(mic_lo_i_r_N_2639), .SP(pll_clk_enable_172), 
            .PD(n22498), .CK(pll_clk), .Q(mic_lo_i_r)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_lo_i_r_1315.GSR = "DISABLED";
    CCU2D mic_lo_n_6__I_0_3 (.A0(mic_lo_n[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_lo_n[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24731), .COUT(n24732), .S0(mic_lo_n_next_6__N_648[1]), 
          .S1(mic_lo_n_next_6__N_648[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[62:79])
    defparam mic_lo_n_6__I_0_3.INIT0 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_3.INIT1 = 16'h5aaa;
    defparam mic_lo_n_6__I_0_3.INJECT1_0 = "NO";
    defparam mic_lo_n_6__I_0_3.INJECT1_1 = "NO";
    FD1P3JX mic_lo_q_r_1316 (.D(mic_lo_q_next), .SP(pll_clk_enable_172), 
            .PD(n22498), .CK(pll_clk), .Q(mic_lo_q_r)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_lo_q_r_1316.GSR = "DISABLED";
    CCU2D mic_lo_n_6__I_0_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_lo_n[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24731), .S1(mic_lo_n_next_6__N_648[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[62:79])
    defparam mic_lo_n_6__I_0_1.INIT0 = 16'hF000;
    defparam mic_lo_n_6__I_0_1.INIT1 = 16'h5555;
    defparam mic_lo_n_6__I_0_1.INJECT1_0 = "NO";
    defparam mic_lo_n_6__I_0_1.INJECT1_1 = "NO";
    CCU2D add_2519_9 (.A0(\mic_acc_q[2] [6]), .B0(n79), .C0(n4628), .D0(GND_net), 
          .A1(\mic_acc_q[2] [7]), .B1(n79), .C1(n4628), .D1(GND_net), 
          .CIN(n24729), .S0(n5760), .S1(n5759));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(858[17:43])
    defparam add_2519_9.INIT0 = 16'hd212;
    defparam add_2519_9.INIT1 = 16'hd212;
    defparam add_2519_9.INJECT1_0 = "NO";
    defparam add_2519_9.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_223 (.A(mic_gate_active), .B(mic_acc_busy_N_2302), 
         .Z(n4_adj_3179)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(962[9] 982[12])
    defparam i1_2_lut_adj_223.init = 16'heeee;
    LUT4 i6801_1_lut (.A(n10893), .Z(mic_acc_busy_N_2293)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(962[9] 982[12])
    defparam i6801_1_lut.init = 16'h5555;
    LUT4 ev_run_hold_s5_83__I_0_i11_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_10), .D(ev_run_hold_s5[10]), .Z(us_tx_next[10])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i11_3_lut_4_lut.init = 16'hbf40;
    CCU2D add_2519_7 (.A0(\mic_acc_q[2] [4]), .B0(n79), .C0(n4628), .D0(GND_net), 
          .A1(\mic_acc_q[2] [5]), .B1(n79), .C1(n4628), .D1(GND_net), 
          .CIN(n24728), .COUT(n24729), .S0(n5762), .S1(n5761));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(858[17:43])
    defparam add_2519_7.INIT0 = 16'hd212;
    defparam add_2519_7.INIT1 = 16'hd212;
    defparam add_2519_7.INJECT1_0 = "NO";
    defparam add_2519_7.INJECT1_1 = "NO";
    CCU2D add_2519_5 (.A0(\mic_acc_q[2] [2]), .B0(n79), .C0(n4628), .D0(GND_net), 
          .A1(\mic_acc_q[2] [3]), .B1(n79), .C1(n4628), .D1(GND_net), 
          .CIN(n24727), .COUT(n24728), .S0(n5764), .S1(n5763));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(858[17:43])
    defparam add_2519_5.INIT0 = 16'hd212;
    defparam add_2519_5.INIT1 = 16'hd212;
    defparam add_2519_5.INJECT1_0 = "NO";
    defparam add_2519_5.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3081__i1 (.D(n44_adj_3248), .SP(pll_clk_enable_370), 
            .CD(n22773), .CK(pll_clk), .Q(pcm_decim_count[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081__i1.GSR = "DISABLED";
    CCU2D add_2519_3 (.A0(\mic_acc_q[2] [0]), .B0(n79), .C0(n4628), .D0(GND_net), 
          .A1(\mic_acc_q[2] [1]), .B1(n79), .C1(n4628), .D1(GND_net), 
          .CIN(n24726), .COUT(n24727), .S0(n5766), .S1(n5765));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(858[17:43])
    defparam add_2519_3.INIT0 = 16'hd121;
    defparam add_2519_3.INIT1 = 16'hd212;
    defparam add_2519_3.INJECT1_0 = "NO";
    defparam add_2519_3.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3081__i2 (.D(n43_adj_3249), .SP(pll_clk_enable_370), 
            .CD(n22773), .CK(pll_clk), .Q(pcm_decim_count[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081__i2.GSR = "DISABLED";
    CCU2D add_2519_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n4628), .B1(n79), .C1(GND_net), .D1(GND_net), .COUT(n24726));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(858[17:43])
    defparam add_2519_1.INIT0 = 16'hF000;
    defparam add_2519_1.INIT1 = 16'hdddd;
    defparam add_2519_1.INJECT1_0 = "NO";
    defparam add_2519_1.INJECT1_1 = "NO";
    CCU2D pcm_acc_9__I_0_11 (.A0(pcm_acc[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24722), .S0(pcm_acc_next[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_11.INIT0 = 16'h5aaa;
    defparam pcm_acc_9__I_0_11.INIT1 = 16'h0000;
    defparam pcm_acc_9__I_0_11.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_11.INJECT1_1 = "NO";
    CCU2D pcm_acc_9__I_0_9 (.A0(pcm_acc[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_acc[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24721), .COUT(n24722), .S0(pcm_acc_next[7]), 
          .S1(pcm_acc_next[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_9.INIT0 = 16'h5aaa;
    defparam pcm_acc_9__I_0_9.INIT1 = 16'h5aaa;
    defparam pcm_acc_9__I_0_9.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_9.INJECT1_1 = "NO";
    CCU2D pcm_acc_9__I_0_7 (.A0(pcm_acc[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_acc[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24720), .COUT(n24721), .S0(pcm_acc_next[5]), 
          .S1(pcm_acc_next[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_7.INIT0 = 16'h5aaa;
    defparam pcm_acc_9__I_0_7.INIT1 = 16'h5aaa;
    defparam pcm_acc_9__I_0_7.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_7.INJECT1_1 = "NO";
    CCU2D pcm_acc_9__I_0_5 (.A0(pcm_acc[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_acc[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24719), .COUT(n24720), .S0(pcm_acc_next[3]), 
          .S1(pcm_acc_next[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_5.INIT0 = 16'h5aaa;
    defparam pcm_acc_9__I_0_5.INIT1 = 16'h5aaa;
    defparam pcm_acc_9__I_0_5.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_5.INJECT1_1 = "NO";
    CCU2D pcm_acc_9__I_0_3 (.A0(pcm_acc[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_acc[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24718), .COUT(n24719), .S0(pcm_acc_next[1]), 
          .S1(pcm_acc_next[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_3.INIT0 = 16'h5aaa;
    defparam pcm_acc_9__I_0_3.INIT1 = 16'h5aaa;
    defparam pcm_acc_9__I_0_3.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_3.INJECT1_1 = "NO";
    FD1P3AX status_bit_index_i0_i2 (.D(status_next_index[2]), .SP(spi1_sck_N_161_enable_9), 
            .CK(spi1_sck_N_161), .Q(status_bit_index[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(454[14:90])
    defparam status_bit_index_i0_i2.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i3 (.D(status_next_index[3]), .SP(spi1_sck_N_161_enable_9), 
            .CK(spi1_sck_N_161), .Q(status_bit_index[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(454[14:90])
    defparam status_bit_index_i0_i3.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i4 (.D(status_next_index[4]), .SP(spi1_sck_N_161_enable_9), 
            .CK(spi1_sck_N_161), .Q(status_bit_index[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(454[14:90])
    defparam status_bit_index_i0_i4.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i5 (.D(status_next_index[5]), .SP(spi1_sck_N_161_enable_9), 
            .CK(spi1_sck_N_161), .Q(status_bit_index[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(454[14:90])
    defparam status_bit_index_i0_i5.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i6 (.D(status_next_index[6]), .SP(spi1_sck_N_161_enable_9), 
            .CK(spi1_sck_N_161), .Q(status_bit_index[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(454[14:90])
    defparam status_bit_index_i0_i6.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i7 (.D(status_bit_index_8__N_1387[7]), .SP(spi1_sck_N_161_enable_9), 
            .CK(spi1_sck_N_161), .Q(status_bit_index[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(454[14:90])
    defparam status_bit_index_i0_i7.GSR = "ENABLED";
    FD1P3AX status_bit_index_i0_i8 (.D(status_bit_index_8__N_1387[8]), .SP(spi1_sck_N_161_enable_9), 
            .CK(spi1_sck_N_161), .Q(status_bit_index[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(454[14:90])
    defparam status_bit_index_i0_i8.GSR = "ENABLED";
    FD1P3IX link_idle__i1 (.D(n3219), .SP(pll_clk_enable_196), .CD(cs_fall), 
            .CK(pll_clk), .Q(link_idle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam link_idle__i1.GSR = "DISABLED";
    FD1P3IX pcm_decim_count_3081__i3 (.D(n42_adj_3250), .SP(pll_clk_enable_370), 
            .CD(n22773), .CK(pll_clk), .Q(pcm_decim_count[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081__i3.GSR = "DISABLED";
    CCU2D pcm_acc_9__I_0_1 (.A0(mic_data_1_c), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_acc[0]), .B1(mic_data_0_c), .C1(GND_net), 
          .D1(GND_net), .COUT(n24718), .S1(pcm_acc_next[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[31:59])
    defparam pcm_acc_9__I_0_1.INIT0 = 16'h5000;
    defparam pcm_acc_9__I_0_1.INIT1 = 16'h5666;
    defparam pcm_acc_9__I_0_1.INJECT1_0 = "NO";
    defparam pcm_acc_9__I_0_1.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3081__i4 (.D(n41_adj_3251), .SP(pll_clk_enable_370), 
            .CD(n22773), .CK(pll_clk), .Q(pcm_decim_count[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081__i4.GSR = "DISABLED";
    CCU2D add_1593_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24717), 
          .S0(ev_sum[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[26:68])
    defparam add_1593_cout.INIT0 = 16'h0000;
    defparam add_1593_cout.INIT1 = 16'h0000;
    defparam add_1593_cout.INJECT1_0 = "NO";
    defparam add_1593_cout.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3081__i5 (.D(n40_adj_3252), .SP(pll_clk_enable_370), 
            .CD(n22773), .CK(pll_clk), .Q(pcm_decim_count[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081__i5.GSR = "DISABLED";
    CCU2D add_1593_8 (.A0(rmw_slot[6]), .B0(build_level_eff[6]), .C0(GND_net), 
          .D0(GND_net), .A1(rmw_slot[7]), .B1(build_level_eff[7]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24716), .COUT(n24717), .S0(ev_sum[6]), 
          .S1(ev_sum[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[26:68])
    defparam add_1593_8.INIT0 = 16'h5666;
    defparam add_1593_8.INIT1 = 16'h5666;
    defparam add_1593_8.INJECT1_0 = "NO";
    defparam add_1593_8.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3081__i6 (.D(n39_adj_3253), .SP(pll_clk_enable_370), 
            .CD(n22773), .CK(pll_clk), .Q(pcm_decim_count[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081__i6.GSR = "DISABLED";
    CCU2D fpga_time_3076_add_4_13 (.A0(fpga_time[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24841), .COUT(n24842), .S0(n154), .S1(n153));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_13.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_13.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_13.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_13.INJECT1_1 = "NO";
    CCU2D add_1593_6 (.A0(rmw_slot[4]), .B0(build_level_eff[4]), .C0(GND_net), 
          .D0(GND_net), .A1(rmw_slot[5]), .B1(build_level_eff[5]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24715), .COUT(n24716), .S0(ev_sum[4]), 
          .S1(ev_sum[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[26:68])
    defparam add_1593_6.INIT0 = 16'h5666;
    defparam add_1593_6.INIT1 = 16'h5666;
    defparam add_1593_6.INJECT1_0 = "NO";
    defparam add_1593_6.INJECT1_1 = "NO";
    FD1P3IX pcm_decim_count_3081__i7 (.D(n38_adj_3254), .SP(pll_clk_enable_370), 
            .CD(n22773), .CK(pll_clk), .Q(pcm_decim_count[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081__i7.GSR = "DISABLED";
    CCU2D add_1593_4 (.A0(rmw_slot[2]), .B0(build_level_eff[2]), .C0(GND_net), 
          .D0(GND_net), .A1(rmw_slot[3]), .B1(build_level_eff[3]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24714), .COUT(n24715), .S0(ev_sum[2]), 
          .S1(ev_sum[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[26:68])
    defparam add_1593_4.INIT0 = 16'h5666;
    defparam add_1593_4.INIT1 = 16'h5666;
    defparam add_1593_4.INJECT1_0 = "NO";
    defparam add_1593_4.INJECT1_1 = "NO";
    FD1P3JX ws2812_settle_i0_i3 (.D(n27914), .SP(pll_clk_enable_250), .PD(ws2812_settle_3__N_1717), 
            .CK(pll_clk), .Q(ws2812_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ws2812_settle_i0_i3.GSR = "DISABLED";
    CCU2D add_1593_2 (.A0(rmw_slot[0]), .B0(build_level_eff[0]), .C0(GND_net), 
          .D0(GND_net), .A1(rmw_slot[1]), .B1(build_level_eff[1]), .C1(GND_net), 
          .D1(GND_net), .COUT(n24714), .S1(ev_sum[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[26:68])
    defparam add_1593_2.INIT0 = 16'h7000;
    defparam add_1593_2.INIT1 = 16'h5666;
    defparam add_1593_2.INJECT1_0 = "NO";
    defparam add_1593_2.INJECT1_1 = "NO";
    FD1P3JX ws2812_settle_i0_i2 (.D(n27913), .SP(pll_clk_enable_250), .PD(ws2812_settle_3__N_1717), 
            .CK(pll_clk), .Q(ws2812_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ws2812_settle_i0_i2.GSR = "DISABLED";
    LUT4 ev_run_hold_s5_83__I_0_i10_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_9), .D(ev_run_hold_s5[9]), .Z(us_tx_next[9])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i10_3_lut_4_lut.init = 16'hbf40;
    FD1P3JX ws2812_settle_i0_i1 (.D(n26304), .SP(pll_clk_enable_250), .PD(ws2812_settle_3__N_1717), 
            .CK(pll_clk), .Q(ws2812_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ws2812_settle_i0_i1.GSR = "DISABLED";
    LUT4 spi1_miso_I_23_3_lut (.A(spi1_miso_N_2453), .B(pcm_audio_miso), 
         .C(spi_audio_read_cmd), .Z(spi1_miso_N_2452)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[24] 450[30])
    defparam spi1_miso_I_23_3_lut.init = 16'hcaca;
    LUT4 i2_3_lut_adj_224 (.A(n23802), .B(n44_adj_3257), .C(spi_mod_feed_N_2490), 
         .Z(n10659)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i2_3_lut_adj_224.init = 16'h8080;
    CCU2D add_1592_21 (.A0(phase_frac[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24710), .COUT(n24711), .S0(phase_frac_sum[19]), 
          .S1(phase_frac_sum[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_21.INIT0 = 16'h5555;
    defparam add_1592_21.INIT1 = 16'h5aaa;
    defparam add_1592_21.INJECT1_0 = "NO";
    defparam add_1592_21.INJECT1_1 = "NO";
    FD1P3IX mic_clock_reg_1309 (.D(mic_clk_N_2474), .SP(pll_clk_enable_188), 
            .CD(n29123), .CK(pll_clk), .Q(mic_clk_c)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_clock_reg_1309.GSR = "DISABLED";
    CCU2D add_1592_19 (.A0(phase_frac[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24709), .COUT(n24710), .S0(phase_frac_sum[17]), 
          .S1(phase_frac_sum[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_19.INIT0 = 16'h5aaa;
    defparam add_1592_19.INIT1 = 16'h5aaa;
    defparam add_1592_19.INJECT1_0 = "NO";
    defparam add_1592_19.INJECT1_1 = "NO";
    CCU2D add_1592_17 (.A0(phase_frac[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24708), .COUT(n24709), .S0(phase_frac_sum[15]), 
          .S1(phase_frac_sum[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_17.INIT0 = 16'h5555;
    defparam add_1592_17.INIT1 = 16'h5aaa;
    defparam add_1592_17.INJECT1_0 = "NO";
    defparam add_1592_17.INJECT1_1 = "NO";
    CCU2D add_1592_15 (.A0(phase_frac[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24707), .COUT(n24708), .S0(phase_frac_sum[13]), 
          .S1(phase_frac_sum[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_15.INIT0 = 16'h5555;
    defparam add_1592_15.INIT1 = 16'h5555;
    defparam add_1592_15.INJECT1_0 = "NO";
    defparam add_1592_15.INJECT1_1 = "NO";
    CCU2D fpga_time_3076_add_4_11 (.A0(fpga_time[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24840), .COUT(n24841), .S0(n156), .S1(n155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_11.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_11.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_11.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_11.INJECT1_1 = "NO";
    FD1P3IX mic_cfg_gap_i15 (.D(mic_cfg_gap_w[15]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i15.GSR = "DISABLED";
    CCU2D add_1592_13 (.A0(phase_frac[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24706), .COUT(n24707), .S0(phase_frac_sum[11]), 
          .S1(phase_frac_sum[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_13.INIT0 = 16'h5aaa;
    defparam add_1592_13.INIT1 = 16'h5555;
    defparam add_1592_13.INJECT1_0 = "NO";
    defparam add_1592_13.INJECT1_1 = "NO";
    FD1P3IX mic_cfg_gap_i14 (.D(mic_cfg_gap_w[14]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i14.GSR = "DISABLED";
    CCU2D add_1592_11 (.A0(phase_frac[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24705), .COUT(n24706), .S0(phase_frac_sum[9]), 
          .S1(phase_frac_sum[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_11.INIT0 = 16'h5aaa;
    defparam add_1592_11.INIT1 = 16'h5555;
    defparam add_1592_11.INJECT1_0 = "NO";
    defparam add_1592_11.INJECT1_1 = "NO";
    FD1P3IX mic_cfg_gap_i13 (.D(mic_cfg_gap_w[13]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i13.GSR = "DISABLED";
    FD1P3IX mic_cfg_gap_i12 (.D(mic_cfg_gap_w[12]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i12.GSR = "DISABLED";
    CCU2D fpga_time_3076_add_4_9 (.A0(fpga_time[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24839), .COUT(n24840), .S0(n158), .S1(n157));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_9.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_9.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_9.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_9.INJECT1_1 = "NO";
    LUT4 mux_679_i5_3_lut_4_lut (.A(pending_sequence[4]), .B(accepted_sequence_spi[4]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[4])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i5_3_lut_4_lut.init = 16'hacaa;
    FD1P3IX mic_cfg_gap_i11 (.D(mic_cfg_gap_w[11]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i11.GSR = "DISABLED";
    CCU2D fpga_time_3076_add_4_7 (.A0(fpga_time[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24838), .COUT(n24839), .S0(n160), .S1(n159));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_7.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_7.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_7.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_7.INJECT1_1 = "NO";
    CCU2D fpga_time_3076_add_4_5 (.A0(fpga_time[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24837), .COUT(n24838), .S0(n162), .S1(n161));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_5.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_5.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_5.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_5.INJECT1_1 = "NO";
    LUT4 mux_679_i4_3_lut_4_lut (.A(pending_sequence[3]), .B(accepted_sequence_spi[3]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[3])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i4_3_lut_4_lut.init = 16'hacaa;
    LUT4 i3_4_lut_adj_225 (.A(mod_active), .B(n25810), .C(wrap_s2), .D(n25864), 
         .Z(feed_pop)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(383[28] 384[49])
    defparam i3_4_lut_adj_225.init = 16'h8000;
    LUT4 i6995_4_lut_then_1_lut (.A(pll_clk_enable_90), .Z(n28432)) /* synthesis lut_function=(A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i6995_4_lut_then_1_lut.init = 16'haaaa;
    LUT4 i1_4_lut_adj_226 (.A(n17_adj_3274), .B(mod_fast), .C(n18_adj_3273), 
         .D(swap_pending), .Z(n25810)) /* synthesis lut_function=(A (B+!(D))+!A (B (C)+!B !((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(383[28] 384[49])
    defparam i1_4_lut_adj_226.init = 16'hc8fa;
    LUT4 n27774_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[116]), 
         .C(status_hold[117]), .D(n27774), .Z(n27777)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27774_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 i15319_2_lut (.A(ws2812_settle[0]), .B(ws2812_settle[1]), .Z(n26304)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam i15319_2_lut.init = 16'h9999;
    CCU2D fpga_time_3076_add_4_3 (.A0(fpga_time[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24836), .COUT(n24837), .S0(n164), .S1(n163));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_3.INIT0 = 16'hfaaa;
    defparam fpga_time_3076_add_4_3.INIT1 = 16'hfaaa;
    defparam fpga_time_3076_add_4_3.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_3.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i50_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_49), .D(ev_run_hold_s5[49]), .Z(us_tx_next[49])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i50_3_lut_4_lut.init = 16'hbf40;
    LUT4 i7_4_lut_adj_227 (.A(feed_wr[0]), .B(n14_adj_3277), .C(n10), 
         .D(feed_rd[0]), .Z(n17_adj_3274)) /* synthesis lut_function=(A (B+(C+!(D)))+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(384[29:49])
    defparam i7_4_lut_adj_227.init = 16'hfdfe;
    LUT4 n27798_bdd_4_lut_4_lut (.A(status_next_index[1]), .B(status_hold[48]), 
         .C(status_hold[49]), .D(n27798), .Z(n27310)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27798_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 i8_3_lut (.A(n15_adj_3275), .B(n11_adj_3281), .C(n12_adj_3280), 
         .Z(n18_adj_3273)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(384[29:49])
    defparam i8_3_lut.init = 16'hfefe;
    LUT4 i3349_2_lut (.A(pcm_fifo_N_2611), .B(pcm_wr_ptr[4]), .Z(n14159)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(869[25:50])
    defparam i3349_2_lut.init = 16'h8888;
    FD1P3IX link_idle__i2 (.D(n3218), .SP(pll_clk_enable_196), .CD(cs_fall), 
            .CK(pll_clk), .Q(link_idle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam link_idle__i2.GSR = "DISABLED";
    FD1P3IX link_idle__i3 (.D(n3217), .SP(pll_clk_enable_196), .CD(cs_fall), 
            .CK(pll_clk), .Q(link_idle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam link_idle__i3.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i1 (.D(mic_done_N_2657[1]), .SP(pll_clk_enable_202), 
            .CD(n13689), .CK(pll_clk), .Q(mic_gate_index[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_index__i1.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i2 (.D(mic_done_N_2657[2]), .SP(pll_clk_enable_202), 
            .CD(n13689), .CK(pll_clk), .Q(mic_gate_index[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_index__i2.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i3 (.D(mic_done_N_2657[3]), .SP(pll_clk_enable_202), 
            .CD(n13689), .CK(pll_clk), .Q(mic_gate_index[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_index__i3.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i4 (.D(mic_done_N_2657[4]), .SP(pll_clk_enable_202), 
            .CD(n13689), .CK(pll_clk), .Q(mic_gate_index[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_index__i4.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i5 (.D(mic_done_N_2657[5]), .SP(pll_clk_enable_202), 
            .CD(n13689), .CK(pll_clk), .Q(mic_gate_index[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_index__i5.GSR = "DISABLED";
    FD1P3IX mic_gate_index__i6 (.D(mic_done_N_2657[6]), .SP(pll_clk_enable_202), 
            .CD(n13689), .CK(pll_clk), .Q(mic_gate_index[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_index__i6.GSR = "DISABLED";
    FD1P3IX mic_phase__i1 (.D(n5688), .SP(pll_clk_enable_205), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_phase[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_phase__i1.GSR = "DISABLED";
    FD1P3IX mic_phase__i2 (.D(n5687), .SP(pll_clk_enable_205), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_phase[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_phase__i2.GSR = "DISABLED";
    FD1P3IX mic_phase__i3 (.D(n5686), .SP(pll_clk_enable_205), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(mic_phase[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_phase__i3.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i2 (.D(n5773), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i2.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i3 (.D(n5772), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i3.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i4 (.D(n5771), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i4.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i5 (.D(n5770), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i5.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i6 (.D(n5769), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i6.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i7 (.D(n5768), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i7.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i8 (.D(n5767), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[3] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i8.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i9 (.D(n5766), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i9.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i10 (.D(n5765), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i10.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i11 (.D(n5764), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i11.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i12 (.D(n5763), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i12.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i13 (.D(n5762), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i13.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i14 (.D(n5761), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i14.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i15 (.D(n5760), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i15.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i16 (.D(n5759), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[2] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i16.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i17 (.D(n5758), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i17.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i18 (.D(n5757), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i18.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i19 (.D(n5756), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i19.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i20 (.D(n5755), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i20.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i21 (.D(n5754), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i21.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i22 (.D(n5753), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i22.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i23 (.D(n5752), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i23.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i24 (.D(n5751), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[1] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i24.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i25 (.D(n5750), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i25.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i26 (.D(n5749), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i26.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i27 (.D(n5748), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i27.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i28 (.D(n5747), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i28.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i29 (.D(n5746), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i29.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i30 (.D(n5745), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i30.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i31 (.D(n5744), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i31.GSR = "DISABLED";
    FD1P3IX mic_acc_q_3___i32 (.D(n5743), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_q[0] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_q_3___i32.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i1 (.D(ev_rd_data[1]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i1.GSR = "DISABLED";
    LUT4 i4_4_lut_adj_228 (.A(feed_wr[8]), .B(feed_wr[2]), .C(feed_rd[8]), 
         .D(feed_rd[2]), .Z(n14_adj_3277)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(384[29:49])
    defparam i4_4_lut_adj_228.init = 16'h7bde;
    LUT4 i1_2_lut_3_lut_4_lut_adj_229 (.A(mic_phase[3]), .B(mic_phase[1]), 
         .C(mic_phase[2]), .D(mic_warm), .Z(n25867)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_3_lut_4_lut_adj_229.init = 16'h0020;
    CCU2D fpga_time_3076_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(n20901), .B1(n25021), .C1(fpga_time[0]), 
          .D1(GND_net), .COUT(n24836), .S1(n165));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076_add_4_1.INIT0 = 16'hF000;
    defparam fpga_time_3076_add_4_1.INIT1 = 16'h7878;
    defparam fpga_time_3076_add_4_1.INJECT1_0 = "NO";
    defparam fpga_time_3076_add_4_1.INJECT1_1 = "NO";
    LUT4 n27516_bdd_4_lut (.A(n27516), .B(\mic_win_q[0] [5]), .C(\mic_win_i[0] [5]), 
         .D(mic_acc_word[0]), .Z(n27519)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27516_bdd_4_lut.init = 16'haad8;
    LUT4 mic_acc_word_2__bdd_4_lut_16184 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [5]), 
         .C(\mic_win_q[1] [5]), .D(mic_acc_word[0]), .Z(n27516)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16184.init = 16'he4aa;
    PFUMX i16799 (.BLUT(n28473), .ALUT(n28474), .C0(pcm_spi_sample_15__N_1381[4]), 
          .Z(n28475));
    LUT4 feed_wr_9__I_0_i10_2_lut (.A(feed_wr[9]), .B(feed_rd[9]), .Z(n10)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(384[29:49])
    defparam feed_wr_9__I_0_i10_2_lut.init = 16'h6666;
    LUT4 i6932_2_lut (.A(pll_clk_enable_602), .B(mic_cfg_gap_w[16]), .Z(n17767)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i6932_2_lut.init = 16'h8888;
    LUT4 i1_2_lut_4_lut_adj_230 (.A(spi_command[5]), .B(spi_command[4]), 
         .C(spi_command[7]), .D(spi_command[6]), .Z(n13_adj_3198)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i1_2_lut_4_lut_adj_230.init = 16'h0004;
    LUT4 i10170_4_lut (.A(ev_rd_data[30]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[30])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10170_4_lut.init = 16'h1222;
    LUT4 mic_acc_last_bdd_4_lut_16974 (.A(mic_acc_last), .B(n20918), .C(mic_acc_state[1]), 
         .D(mic_acc_state[2]), .Z(n28787)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;
    defparam mic_acc_last_bdd_4_lut_16974.init = 16'h0040;
    FD1P3IX mic_cfg_gap_i10 (.D(mic_cfg_gap_w[10]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i10.GSR = "DISABLED";
    CCU2D add_1067_7 (.A0(mic_gate_index[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_index[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24834), .S0(mic_done_N_2657[5]), .S1(mic_done_N_2657[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam add_1067_7.INIT0 = 16'h5aaa;
    defparam add_1067_7.INIT1 = 16'h5aaa;
    defparam add_1067_7.INJECT1_0 = "NO";
    defparam add_1067_7.INJECT1_1 = "NO";
    FD1P3IX mic_cfg_gap_i9 (.D(mic_cfg_gap_w[9]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i9.GSR = "DISABLED";
    FD1P3IX mic_cfg_gap_i8 (.D(mic_cfg_gap_w[8]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i8.GSR = "DISABLED";
    LUT4 spi1_miso_N_2463_2__bdd_4_lut_4_lut (.A(status_bit_index[2]), .B(n28460), 
         .C(n28457), .D(status_bit_index[3]), .Z(n27528)) /* synthesis lut_function=(A (B (D))+!A (C+!(D))) */ ;
    defparam spi1_miso_N_2463_2__bdd_4_lut_4_lut.init = 16'hd855;
    FD1P3IX mic_cfg_gap_i7 (.D(mic_cfg_gap_w[7]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i7.GSR = "DISABLED";
    LUT4 i5_4_lut_adj_231 (.A(feed_wr[5]), .B(feed_wr[1]), .C(feed_rd[5]), 
         .D(feed_rd[1]), .Z(n15_adj_3275)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(384[29:49])
    defparam i5_4_lut_adj_231.init = 16'h7bde;
    FD1P3IX mic_cfg_gap_i6 (.D(mic_cfg_gap_w[6]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i6.GSR = "DISABLED";
    FD1P3IX mic_cfg_gap_i5 (.D(mic_cfg_gap_w[5]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i5.GSR = "DISABLED";
    LUT4 i1_4_lut_adj_232 (.A(feed_wr[7]), .B(feed_wr[4]), .C(feed_rd[7]), 
         .D(feed_rd[4]), .Z(n11_adj_3281)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(384[29:49])
    defparam i1_4_lut_adj_232.init = 16'h7bde;
    LUT4 i10169_4_lut (.A(ev_rd_data[31]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[31])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10169_4_lut.init = 16'h1222;
    LUT4 i2_4_lut_adj_233 (.A(feed_wr[3]), .B(feed_wr[6]), .C(feed_rd[3]), 
         .D(feed_rd[6]), .Z(n12_adj_3280)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(384[29:49])
    defparam i2_4_lut_adj_233.init = 16'h7bde;
    LUT4 n27810_bdd_4_lut_16731_4_lut (.A(status_next_index[1]), .B(status_hold[52]), 
         .C(status_hold[53]), .D(n27810), .Z(n27314)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27810_bdd_4_lut_16731_4_lut.init = 16'hf588;
    LUT4 i10143_4_lut (.A(ev_rd_data[57]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[57])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10143_4_lut.init = 16'h1222;
    FD1P3IX mic_cfg_gap_i4 (.D(mic_cfg_gap_w[4]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i4.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i2 (.D(ev_rd_data[2]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i2.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i3 (.D(ev_rd_data[3]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i3.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i4 (.D(ev_rd_data[4]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i4.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i5 (.D(ev_rd_data[5]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i5.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i6 (.D(ev_rd_data[6]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i6.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i7 (.D(ev_rd_data[7]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i7.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i8 (.D(ev_rd_data[8]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i8.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i9 (.D(ev_rd_data[9]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i9.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i10 (.D(ev_rd_data[10]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i10.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i11 (.D(ev_rd_data[11]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i11.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i12 (.D(ev_rd_data[12]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i12.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i13 (.D(ev_rd_data[13]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i13.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i14 (.D(ev_rd_data[14]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i14.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i15 (.D(ev_rd_data[15]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i15.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i16 (.D(ev_rd_data[16]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i16.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i17 (.D(ev_rd_data[17]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i17.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i18 (.D(ev_rd_data[18]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i18.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i19 (.D(ev_rd_data[19]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i19.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i20 (.D(ev_rd_data[20]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i20.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i21 (.D(ev_rd_data[21]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i21.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i22 (.D(ev_rd_data[22]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i22.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i23 (.D(ev_rd_data[23]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i23.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i24 (.D(ev_rd_data[24]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i24.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i25 (.D(ev_rd_data[25]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i25.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i26 (.D(ev_rd_data[26]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i26.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i27 (.D(ev_rd_data[27]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i27.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i28 (.D(ev_rd_data[28]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i28.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i29 (.D(ev_rd_data[29]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i29.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i30 (.D(ev_rd_data[30]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i30.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i31 (.D(ev_rd_data[31]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i31.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i32 (.D(ev_rd_data[32]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i32.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i33 (.D(ev_rd_data[33]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i33.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i34 (.D(ev_rd_data[34]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i34.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i35 (.D(ev_rd_data[35]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i35.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i36 (.D(ev_rd_data[36]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i36.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i37 (.D(ev_rd_data[37]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i37.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i38 (.D(ev_rd_data[38]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i38.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i39 (.D(ev_rd_data[39]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i39.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i40 (.D(ev_rd_data[40]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i40.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i41 (.D(ev_rd_data[41]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i41.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i42 (.D(ev_rd_data[42]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i42.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i43 (.D(ev_rd_data[43]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i43.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i44 (.D(ev_rd_data[44]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i44.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i45 (.D(ev_rd_data[45]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i45.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i46 (.D(ev_rd_data[46]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i46.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i47 (.D(ev_rd_data[47]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i47.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i48 (.D(ev_rd_data[48]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i48.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i49 (.D(ev_rd_data[49]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i49.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i50 (.D(ev_rd_data[50]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i50.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i51 (.D(ev_rd_data[51]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i51.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i52 (.D(ev_rd_data[52]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i52.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i53 (.D(ev_rd_data[53]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i53.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i54 (.D(ev_rd_data[54]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i54.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i55 (.D(ev_rd_data[55]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i55.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i56 (.D(ev_rd_data[56]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i56.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i57 (.D(ev_rd_data[57]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i57.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i58 (.D(ev_rd_data[58]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i58.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i59 (.D(ev_rd_data[59]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i59.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i60 (.D(ev_rd_data[60]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i60.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i61 (.D(ev_rd_data[61]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i61.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i62 (.D(ev_rd_data[62]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i62.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i63 (.D(ev_rd_data[63]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i63.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i64 (.D(ev_rd_data[64]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i64.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i65 (.D(ev_rd_data[65]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i65.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i66 (.D(ev_rd_data[66]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i66.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i67 (.D(ev_rd_data[67]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i67.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i68 (.D(ev_rd_data[68]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i68.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i69 (.D(ev_rd_data[69]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i69.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i70 (.D(ev_rd_data[70]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i70.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i71 (.D(ev_rd_data[71]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i71.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i72 (.D(ev_rd_data[72]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i72.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i73 (.D(ev_rd_data[73]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i73.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i74 (.D(ev_rd_data[74]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i74.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i75 (.D(ev_rd_data[75]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i75.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i76 (.D(ev_rd_data[76]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i76.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i77 (.D(ev_rd_data[77]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i77.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i78 (.D(ev_rd_data[78]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i78.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i79 (.D(ev_rd_data[79]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i79.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i80 (.D(ev_rd_data[80]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i80.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i81 (.D(ev_rd_data[81]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i81.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i82 (.D(ev_rd_data[82]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i82.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i83 (.D(ev_rd_data[83]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i83.GSR = "DISABLED";
    FD1P3IX frame_settle__i1 (.D(n3425), .SP(ws2812_enable_N_2601), .CD(n15263), 
            .CK(pll_clk), .Q(frame_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam frame_settle__i1.GSR = "DISABLED";
    LUT4 i4276_2_lut_4_lut (.A(build_idle_N_2724), .B(frame_req), .C(swap_pending), 
         .D(n16947), .Z(n15115)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i4276_2_lut_4_lut.init = 16'hffa2;
    LUT4 i10168_4_lut (.A(ev_rd_data[32]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[32])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10168_4_lut.init = 16'h1222;
    LUT4 i2_2_lut_4_lut_adj_234 (.A(ws2812_enable_N_2601), .B(frame_toggle_sync), 
         .C(frame_toggle_seen), .D(n7_adj_3263), .Z(n16857)) /* synthesis lut_function=(!((B ((D)+!C)+!B (C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(675[18] 691[12])
    defparam i2_2_lut_4_lut_adj_234.init = 16'h0082;
    FD1P3IX frame_settle__i2 (.D(n3424), .SP(ws2812_enable_N_2601), .CD(n15263), 
            .CK(pll_clk), .Q(frame_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam frame_settle__i2.GSR = "DISABLED";
    LUT4 i6995_4_lut_else_1_lut (.A(pll_clk_enable_90), .B(mic_acc_state[1]), 
         .C(mic_acc_state[0]), .D(n20918), .Z(n28431)) /* synthesis lut_function=(!((B (C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i6995_4_lut_else_1_lut.init = 16'h2aaa;
    FD1P3AX status_hold__i2 (.D(feed_fill[9]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i2.GSR = "DISABLED";
    FD1P3AX status_hold__i3 (.D(accepted_sequence[8]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i3.GSR = "DISABLED";
    FD1P3AX status_hold__i4 (.D(accepted_sequence[9]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i4.GSR = "DISABLED";
    FD1P3AX status_hold__i5 (.D(accepted_sequence[10]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i5.GSR = "DISABLED";
    FD1P3AX status_hold__i6 (.D(accepted_sequence[11]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i6.GSR = "DISABLED";
    FD1P3AX status_hold__i7 (.D(accepted_sequence[12]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i7.GSR = "DISABLED";
    FD1P3AX status_hold__i8 (.D(accepted_sequence[13]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i8.GSR = "DISABLED";
    FD1P3AX status_hold__i9 (.D(accepted_sequence[14]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i9.GSR = "DISABLED";
    FD1P3AX status_hold__i10 (.D(accepted_sequence[15]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i10.GSR = "DISABLED";
    FD1P3AX status_hold__i11 (.D(accepted_sequence[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i11.GSR = "DISABLED";
    FD1P3AX status_hold__i12 (.D(accepted_sequence[1]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i12.GSR = "DISABLED";
    FD1P3AX status_hold__i13 (.D(accepted_sequence[2]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i13.GSR = "DISABLED";
    FD1P3AX status_hold__i14 (.D(accepted_sequence[3]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i14.GSR = "DISABLED";
    FD1P3AX status_hold__i15 (.D(accepted_sequence[4]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i15.GSR = "DISABLED";
    FD1P3AX status_hold__i16 (.D(accepted_sequence[5]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i16.GSR = "DISABLED";
    FD1P3AX status_hold__i17 (.D(accepted_sequence[6]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i17.GSR = "DISABLED";
    FD1P3AX status_hold__i18 (.D(accepted_sequence[7]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i18.GSR = "DISABLED";
    FD1P3AX status_hold__i19 (.D(fpga_time[24]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i19.GSR = "DISABLED";
    FD1P3AX status_hold__i20 (.D(fpga_time[25]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i20.GSR = "DISABLED";
    FD1P3AX status_hold__i21 (.D(fpga_time[26]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i21.GSR = "DISABLED";
    FD1P3AX status_hold__i22 (.D(fpga_time[27]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i22.GSR = "DISABLED";
    FD1P3AX status_hold__i23 (.D(fpga_time[28]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i23.GSR = "DISABLED";
    FD1P3AX status_hold__i24 (.D(fpga_time[29]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i24.GSR = "DISABLED";
    FD1P3AX status_hold__i25 (.D(fpga_time[30]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i25.GSR = "DISABLED";
    FD1P3AX status_hold__i26 (.D(fpga_time[31]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i26.GSR = "DISABLED";
    FD1P3AX status_hold__i27 (.D(fpga_time[16]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i27.GSR = "DISABLED";
    FD1P3AX status_hold__i28 (.D(fpga_time[17]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i28.GSR = "DISABLED";
    FD1P3AX status_hold__i29 (.D(fpga_time[18]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i29.GSR = "DISABLED";
    FD1P3AX status_hold__i30 (.D(fpga_time[19]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i30.GSR = "DISABLED";
    FD1P3AX status_hold__i31 (.D(fpga_time[20]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i31.GSR = "DISABLED";
    FD1P3AX status_hold__i32 (.D(fpga_time[21]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i32.GSR = "DISABLED";
    FD1P3AX status_hold__i33 (.D(fpga_time[22]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i33.GSR = "DISABLED";
    FD1P3AX status_hold__i34 (.D(fpga_time[23]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i34.GSR = "DISABLED";
    FD1P3AX status_hold__i35 (.D(fpga_time[8]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i35.GSR = "DISABLED";
    FD1P3AX status_hold__i36 (.D(fpga_time[9]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i36.GSR = "DISABLED";
    FD1P3AX status_hold__i37 (.D(fpga_time[10]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i37.GSR = "DISABLED";
    FD1P3AX status_hold__i38 (.D(fpga_time[11]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i38.GSR = "DISABLED";
    FD1P3AX status_hold__i39 (.D(fpga_time[12]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i39.GSR = "DISABLED";
    FD1P3AX status_hold__i40 (.D(fpga_time[13]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i40.GSR = "DISABLED";
    FD1P3AX status_hold__i41 (.D(fpga_time[14]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i41.GSR = "DISABLED";
    FD1P3AX status_hold__i42 (.D(fpga_time[15]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i42.GSR = "DISABLED";
    FD1P3AX status_hold__i43 (.D(fpga_time[0]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i43.GSR = "DISABLED";
    FD1P3AX status_hold__i44 (.D(fpga_time[1]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i44.GSR = "DISABLED";
    FD1P3AX status_hold__i45 (.D(fpga_time[2]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i45.GSR = "DISABLED";
    FD1P3AX status_hold__i46 (.D(fpga_time[3]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i46.GSR = "DISABLED";
    FD1P3AX status_hold__i47 (.D(fpga_time[4]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i47.GSR = "DISABLED";
    FD1P3AX status_hold__i48 (.D(fpga_time[5]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i48.GSR = "DISABLED";
    FD1P3AX status_hold__i49 (.D(fpga_time[6]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i49.GSR = "DISABLED";
    FD1P3AX status_hold__i50 (.D(fpga_time[7]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i50.GSR = "DISABLED";
    FD1P3AX status_hold__i51 (.D(status_flags_wire_15__N_1365[13]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i51.GSR = "DISABLED";
    FD1P3AX status_hold__i52 (.D(mod_active), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i52.GSR = "DISABLED";
    FD1P3AX status_hold__i53 (.D(invalid_frame_sync), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i53.GSR = "DISABLED";
    FD1P3AX status_hold__i54 (.D(running), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i54.GSR = "DISABLED";
    FD1P3AX status_hold__i55 (.D(fifo_depth_wire[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i55.GSR = "DISABLED";
    FD1P3AX status_hold__i56 (.D(fifo_credit_wire[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[104])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i56.GSR = "DISABLED";
    FD1P3AX status_hold__i57 (.D(feed_fill[0]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[112])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i57.GSR = "DISABLED";
    FD1P3AX status_hold__i58 (.D(feed_fill[1]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[113])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i58.GSR = "DISABLED";
    FD1P3AX status_hold__i59 (.D(feed_fill[2]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[114])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i59.GSR = "DISABLED";
    FD1P3AX status_hold__i60 (.D(feed_fill[3]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[115])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i60.GSR = "DISABLED";
    FD1P3AX status_hold__i61 (.D(feed_fill[4]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[116])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i61.GSR = "DISABLED";
    FD1P3AX status_hold__i62 (.D(feed_fill[5]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[117])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i62.GSR = "DISABLED";
    FD1P3AX status_hold__i63 (.D(feed_fill[6]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[118])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i63.GSR = "DISABLED";
    FD1P3AX status_hold__i64 (.D(feed_fill[7]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[119])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam status_hold__i64.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i1 (.D(n164), .CK(pll_clk), .Q(fpga_time[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i1.GSR = "DISABLED";
    FD1P3JX mic_lo_n_i0 (.D(mic_lo_n_next[0]), .SP(pll_clk_enable_259), 
            .PD(n22498), .CK(pll_clk), .Q(mic_lo_n[0])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_lo_n_i0.GSR = "DISABLED";
    FD1P3JX mic_cfg_count_i0 (.D(mic_cfg_count_w_6__N_856[0]), .SP(pll_clk_enable_602), 
            .PD(n25237), .CK(pll_clk), .Q(mic_cfg_count[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_count_i0.GSR = "DISABLED";
    FD1P3JX mic_warm_1310 (.D(mic_warm_N_1918), .SP(pll_clk_enable_246), 
            .PD(n17740), .CK(pll_clk), .Q(mic_warm)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_warm_1310.GSR = "DISABLED";
    FD1S3IX ev_run_hold_s5__i0 (.D(ev_rd_data[0]), .CK(pll_clk), .CD(ev_run_hold_s5_83__N_1649), 
            .Q(ev_run_hold_s5[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ev_run_hold_s5__i0.GSR = "DISABLED";
    FD1P3IX frame_req_mod_1288 (.D(frame_req_mod_N_2752), .SP(pll_clk_enable_247), 
            .CD(n16190), .CK(pll_clk), .Q(frame_req_mod)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam frame_req_mod_1288.GSR = "DISABLED";
    FD1P3IX mic_gate_active_1347 (.D(mic_gate_active_N_2306), .SP(pll_clk_enable_248), 
            .CD(mic_acc_busy_N_2292), .CK(pll_clk), .Q(mic_gate_active)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_active_1347.GSR = "DISABLED";
    FD1P3IX pcm_read_count_latched_spi_i0_i0 (.D(pcm_count_spi[0]), .SP(spi1_sck_c_enable_164), 
            .CD(n17823), .CK(spi1_sck_c), .Q(pcm_read_count_latched_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam pcm_read_count_latched_spi_i0_i0.GSR = "DISABLED";
    FD1S3IX wrap_s2_1255 (.D(wrap_s2_N_2608), .CK(pll_clk), .CD(n17733), 
            .Q(wrap_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam wrap_s2_1255.GSR = "DISABLED";
    FD1P3JX ws2812_settle_i0_i0 (.D(n1), .SP(pll_clk_enable_250), .PD(ws2812_settle_3__N_1717), 
            .CK(pll_clk), .Q(ws2812_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam ws2812_settle_i0_i0.GSR = "DISABLED";
    FD1P3IX pcm_decim_count_3081__i0 (.D(n45_adj_3247), .SP(pll_clk_enable_370), 
            .CD(n22773), .CK(pll_clk), .Q(pcm_decim_count[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081__i0.GSR = "DISABLED";
    FD1P3IX mic_block_count_3084__i0 (.D(n85), .SP(pll_clk_enable_253), 
            .CD(n17809), .CK(pll_clk), .Q(mic_block_count[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam mic_block_count_3084__i0.GSR = "DISABLED";
    FD1P3IX mic_cfg_gap_i3 (.D(mic_cfg_gap_w[3]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i3.GSR = "DISABLED";
    CCU2D add_1067_5 (.A0(mic_gate_index[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_index[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24833), .COUT(n24834), .S0(mic_done_N_2657[3]), 
          .S1(mic_done_N_2657[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam add_1067_5.INIT0 = 16'h5aaa;
    defparam add_1067_5.INIT1 = 16'h5aaa;
    defparam add_1067_5.INJECT1_0 = "NO";
    defparam add_1067_5.INJECT1_1 = "NO";
    LUT4 i10167_4_lut (.A(ev_rd_data[33]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[33])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10167_4_lut.init = 16'h1222;
    LUT4 frame_toggle_sync_I_0_2_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .Z(ws2812_enable_N_2602)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(672[13:51])
    defparam frame_toggle_sync_I_0_2_lut.init = 16'h6666;
    FD1S3AX fpga_time_3076__i2 (.D(n163), .CK(pll_clk), .Q(fpga_time[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i2.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i3 (.D(n162), .CK(pll_clk), .Q(fpga_time[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i3.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i4 (.D(n161), .CK(pll_clk), .Q(fpga_time[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i4.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i5 (.D(n160), .CK(pll_clk), .Q(fpga_time[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i5.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i6 (.D(n159), .CK(pll_clk), .Q(fpga_time[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i6.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i7 (.D(n158), .CK(pll_clk), .Q(fpga_time[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i7.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i8 (.D(n157), .CK(pll_clk), .Q(fpga_time[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i8.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i9 (.D(n156), .CK(pll_clk), .Q(fpga_time[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i9.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i10 (.D(n155), .CK(pll_clk), .Q(fpga_time[10])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i10.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i11 (.D(n154), .CK(pll_clk), .Q(fpga_time[11])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i11.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i12 (.D(n153), .CK(pll_clk), .Q(fpga_time[12])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i12.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i13 (.D(n152), .CK(pll_clk), .Q(fpga_time[13])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i13.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i14 (.D(n151), .CK(pll_clk), .Q(fpga_time[14])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i14.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i15 (.D(n150), .CK(pll_clk), .Q(fpga_time[15])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i15.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i16 (.D(n149), .CK(pll_clk), .Q(fpga_time[16])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i16.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i17 (.D(n148), .CK(pll_clk), .Q(fpga_time[17])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i17.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i18 (.D(n147), .CK(pll_clk), .Q(fpga_time[18])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i18.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i19 (.D(n146), .CK(pll_clk), .Q(fpga_time[19])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i19.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i20 (.D(n145), .CK(pll_clk), .Q(fpga_time[20])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i20.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i21 (.D(n144), .CK(pll_clk), .Q(fpga_time[21])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i21.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i22 (.D(n143), .CK(pll_clk), .Q(fpga_time[22])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i22.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i23 (.D(n142), .CK(pll_clk), .Q(fpga_time[23])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i23.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i24 (.D(n141), .CK(pll_clk), .Q(fpga_time[24])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i24.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i25 (.D(n140), .CK(pll_clk), .Q(fpga_time[25])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i25.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i26 (.D(n139), .CK(pll_clk), .Q(fpga_time[26])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i26.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i27 (.D(n138), .CK(pll_clk), .Q(fpga_time[27])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i27.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i28 (.D(n137), .CK(pll_clk), .Q(fpga_time[28])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i28.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i29 (.D(n136_adj_3195), .CK(pll_clk), .Q(fpga_time[29])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i29.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i30 (.D(n135), .CK(pll_clk), .Q(fpga_time[30])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i30.GSR = "DISABLED";
    FD1S3AX fpga_time_3076__i31 (.D(n134), .CK(pll_clk), .Q(fpga_time[31])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam fpga_time_3076__i31.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i1 (.D(accepted_sequence_15__N_1001[1]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i1.GSR = "DISABLED";
    CCU2D add_1067_3 (.A0(mic_gate_index[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_index[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24832), .COUT(n24833), .S0(mic_done_N_2657[1]), 
          .S1(mic_done_N_2657[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam add_1067_3.INIT0 = 16'h5aaa;
    defparam add_1067_3.INIT1 = 16'h5aaa;
    defparam add_1067_3.INJECT1_0 = "NO";
    defparam add_1067_3.INJECT1_1 = "NO";
    LUT4 i2209_2_lut_3_lut (.A(frame_req), .B(swap_pending), .C(build_idle_N_2724), 
         .Z(pll_clk_enable_517)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i2209_2_lut_3_lut.init = 16'h2020;
    FD1P3IX mic_cfg_count_i1 (.D(mic_cfg_hold_spi[1]), .SP(pll_clk_enable_602), 
            .CD(n17763), .CK(pll_clk), .Q(mic_cfg_count[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_count_i1.GSR = "DISABLED";
    FD1P3JX mic_lo_n_i6 (.D(mic_lo_n_next[6]), .SP(pll_clk_enable_259), 
            .PD(n22498), .CK(pll_clk), .Q(mic_lo_n[6])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_lo_n_i6.GSR = "DISABLED";
    FD1P3JX mic_lo_n_i5 (.D(mic_lo_n_next[5]), .SP(pll_clk_enable_259), 
            .PD(n22498), .CK(pll_clk), .Q(mic_lo_n[5])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_lo_n_i5.GSR = "DISABLED";
    FD1P3JX mic_lo_n_i1 (.D(mic_lo_n_next[1]), .SP(pll_clk_enable_259), 
            .PD(n22498), .CK(pll_clk), .Q(mic_lo_n[1])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_lo_n_i1.GSR = "DISABLED";
    FD1P3IX pcm_acc_i8 (.D(pcm_acc_next[8]), .SP(pll_clk_enable_266), .CD(n22781), 
            .CK(pll_clk), .Q(pcm_acc[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pcm_acc_i8.GSR = "DISABLED";
    FD1P3IX pcm_acc_i7 (.D(pcm_acc_next[7]), .SP(pll_clk_enable_266), .CD(n22781), 
            .CK(pll_clk), .Q(pcm_acc[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pcm_acc_i7.GSR = "DISABLED";
    FD1P3IX pcm_acc_i6 (.D(pcm_acc_next[6]), .SP(pll_clk_enable_266), .CD(n22781), 
            .CK(pll_clk), .Q(pcm_acc[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pcm_acc_i6.GSR = "DISABLED";
    FD1P3IX pcm_acc_i5 (.D(pcm_acc_next[5]), .SP(pll_clk_enable_266), .CD(n22781), 
            .CK(pll_clk), .Q(pcm_acc[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pcm_acc_i5.GSR = "DISABLED";
    FD1P3IX pcm_acc_i4 (.D(pcm_acc_next[4]), .SP(pll_clk_enable_266), .CD(n22781), 
            .CK(pll_clk), .Q(pcm_acc[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pcm_acc_i4.GSR = "DISABLED";
    FD1P3IX pcm_acc_i1 (.D(pcm_acc_next[1]), .SP(pll_clk_enable_266), .CD(n22781), 
            .CK(pll_clk), .Q(pcm_acc[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pcm_acc_i1.GSR = "DISABLED";
    FD1P3IX pcm_acc_i0 (.D(pcm_acc_next[0]), .SP(pll_clk_enable_266), .CD(n22781), 
            .CK(pll_clk), .Q(pcm_acc[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pcm_acc_i0.GSR = "DISABLED";
    FD1P3IX mic_acc_word_i0 (.D(mic_acc_word_2__N_2332[0]), .SP(pll_clk_enable_267), 
            .CD(n28693), .CK(pll_clk), .Q(mic_acc_word[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_word_i0.GSR = "DISABLED";
    FD1P3IX mic_ram_wr_data_i0_i0 (.D(mic_ram_wr_data_15__N_2316[0]), .SP(pll_clk_enable_268), 
            .CD(n17845), .CK(pll_clk), .Q(mic_ram_wr_data[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_data_i0_i0.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i2 (.D(accepted_sequence_15__N_1001[2]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i3 (.D(accepted_sequence_15__N_1001[3]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i4 (.D(accepted_sequence_15__N_1001[4]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i5 (.D(accepted_sequence_15__N_1001[5]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i6 (.D(accepted_sequence_15__N_1001[6]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i7 (.D(accepted_sequence_15__N_1001[7]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i8 (.D(accepted_sequence_15__N_1001[8]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i9 (.D(accepted_sequence_15__N_1001[9]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i10 (.D(accepted_sequence_15__N_1001[10]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i11 (.D(accepted_sequence_15__N_1001[11]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i12 (.D(accepted_sequence_15__N_1001[12]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i13 (.D(accepted_sequence_15__N_1001[13]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i14 (.D(accepted_sequence_15__N_1001[14]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i15 (.D(accepted_sequence_15__N_1001[15]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(accepted_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam accepted_sequence_i15.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i1 (.D(global_phase_s2[1]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(run_addr_s3[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam run_addr_s3_i1.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i2 (.D(global_phase_s2[2]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(run_addr_s3[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam run_addr_s3_i2.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i3 (.D(global_phase_s2[3]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(run_addr_s3[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam run_addr_s3_i3.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i4 (.D(global_phase_s2[4]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(run_addr_s3[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam run_addr_s3_i4.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i5 (.D(global_phase_s2[5]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(run_addr_s3[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam run_addr_s3_i5.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i6 (.D(global_phase_s2[6]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(run_addr_s3[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam run_addr_s3_i6.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i7 (.D(global_phase_s2[7]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(run_addr_s3[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam run_addr_s3_i7.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i8 (.D(active_bank_N_899), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(run_addr_s3[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam run_addr_s3_i8.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i1 (.D(feed_wr_spi[1]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam feed_wr_i0_i1.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i2 (.D(feed_wr_spi[2]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam feed_wr_i0_i2.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i3 (.D(feed_wr_spi[3]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam feed_wr_i0_i3.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i4 (.D(feed_wr_spi[4]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam feed_wr_i0_i4.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i5 (.D(feed_wr_spi[5]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam feed_wr_i0_i5.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i6 (.D(feed_wr_spi[6]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam feed_wr_i0_i6.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i7 (.D(feed_wr_spi[7]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam feed_wr_i0_i7.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i8 (.D(feed_wr_spi[8]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam feed_wr_i0_i8.GSR = "DISABLED";
    FD1P3AX feed_wr_i0_i9 (.D(feed_wr_spi[9]), .SP(cs_rise), .CK(pll_clk), 
            .Q(feed_wr[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam feed_wr_i0_i9.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i1 (.D(n25725), .SP(pll_clk_enable_297), .CK(pll_clk), 
            .Q(rmw_slot[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rmw_slot_i0_i1.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i2 (.D(n25667), .SP(pll_clk_enable_297), .CK(pll_clk), 
            .Q(rmw_slot[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rmw_slot_i0_i2.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i3 (.D(n15428), .SP(pll_clk_enable_297), .CK(pll_clk), 
            .Q(rmw_slot[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rmw_slot_i0_i3.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i4 (.D(n15430), .SP(pll_clk_enable_297), .CK(pll_clk), 
            .Q(rmw_slot[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rmw_slot_i0_i4.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i5 (.D(n15432), .SP(pll_clk_enable_297), .CK(pll_clk), 
            .Q(rmw_slot[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rmw_slot_i0_i5.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i6 (.D(n15434), .SP(pll_clk_enable_297), .CK(pll_clk), 
            .Q(rmw_slot[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rmw_slot_i0_i6.GSR = "DISABLED";
    FD1P3AX rmw_slot_i0_i7 (.D(n15436), .SP(pll_clk_enable_297), .CK(pll_clk), 
            .Q(rmw_slot[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rmw_slot_i0_i7.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i1 (.D(accepted_sequence_spi[1]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i1.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i2 (.D(accepted_sequence_spi[2]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i2.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i3 (.D(accepted_sequence_spi[3]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i3.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i4 (.D(accepted_sequence_spi[4]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i4.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i5 (.D(accepted_sequence_spi[5]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i5.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i6 (.D(accepted_sequence_spi[6]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i6.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i7 (.D(accepted_sequence_spi[7]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i7.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i8 (.D(accepted_sequence_spi[8]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i8.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i9 (.D(accepted_sequence_spi[9]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i9.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i10 (.D(accepted_sequence_spi[10]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i10.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i11 (.D(accepted_sequence_spi[11]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i11.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i12 (.D(accepted_sequence_spi[12]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i12.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i13 (.D(accepted_sequence_spi[13]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i13.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i14 (.D(accepted_sequence_spi[14]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i14.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0_i15 (.D(accepted_sequence_spi[15]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(pending_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam pending_sequence_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i1 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i2 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i3 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i4 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i5 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i6 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i7 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i8 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i9 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i10 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i11 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i12 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i13 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i14 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i15 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_0_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_l_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i1 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i1.GSR = "DISABLED";
    LUT4 n27522_bdd_4_lut (.A(n27522), .B(\mic_win_q[2] [5]), .C(\mic_win_i[2] [5]), 
         .D(mic_acc_word[0]), .Z(n27525)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27522_bdd_4_lut.init = 16'haad8;
    FD1P3AX mic_shift_1_l_i0_i2 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i3 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i4 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i5 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i6 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i7 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i8 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i9 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i10 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i11 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i12 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i13 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i14 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i15 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_342), 
            .CK(pll_clk), .Q(mic_shift_1_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_l_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i2 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i3 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i4 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i5 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i6 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i7 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i8 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i9 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i10 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i11 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i12 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i13 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i14 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i15 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_0_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_0_r__i15.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i2 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i3 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i4 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i5 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i6 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i7 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i8 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i9 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i10 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i11 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i12 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i13 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i14 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i15 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_370), 
            .CK(pll_clk), .Q(mic_shift_1_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_shift_1_r__i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i1 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i1.GSR = "DISABLED";
    LUT4 i10166_4_lut (.A(ev_rd_data[34]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[34])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10166_4_lut.init = 16'h1222;
    FD1P3AX mic_latest_i0_i2 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i3 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i4 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i5 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i6 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i7 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i8 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i9 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i10 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i11 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i12 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i13 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i14 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i15 (.D(mic_shift_1_r[14]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i16 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i16.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i17 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i17.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i18 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i18.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i19 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i19.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i20 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i20.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i21 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i21.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i22 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i22.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i23 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i23.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i24 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i24.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i25 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i26 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i26.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i27 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i27.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i28 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i28.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i29 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i29.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i30 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i30.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i31 (.D(mic_shift_1_l[15]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i31.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i32 (.D(mic_data_0_c), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i32.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i33 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i33.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i34 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i34.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i35 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i35.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i36 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i36.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i37 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i37.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i38 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i38.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i39 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i39.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i40 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i40.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i41 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i41.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i42 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i42.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i43 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i43.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i44 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i44.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i45 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i45.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i46 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i46.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i47 (.D(mic_shift_0_r[14]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i47.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i48 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i48.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i49 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i49.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i50 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i50.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i51 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i51.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i52 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i52.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i53 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i53.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i54 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i54.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i55 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i55.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i56 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i56.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i57 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i57.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i58 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i58.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i59 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i59.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i60 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i60.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i61 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i61.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i62 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i62.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i63 (.D(mic_shift_0_l[15]), .SP(pll_clk_enable_433), 
            .CK(pll_clk), .Q(mic_latest[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_latest_i0_i63.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i2 (.D(\mic_acc_i[3] [1]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[3] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i2.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i3 (.D(\mic_acc_i[3] [2]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[3] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i3.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i4 (.D(\mic_acc_i[3] [3]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[3] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i4.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i5 (.D(\mic_acc_i[3] [4]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[3] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i5.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i6 (.D(\mic_acc_i[3] [5]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[3] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i6.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i7 (.D(\mic_acc_i[3] [6]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[3] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i7.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i8 (.D(\mic_acc_i[3] [7]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[3] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i8.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i9 (.D(\mic_acc_i[2] [0]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[2] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i9.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i10 (.D(\mic_acc_i[2] [1]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[2] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i10.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i11 (.D(\mic_acc_i[2] [2]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[2] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i11.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i12 (.D(\mic_acc_i[2] [3]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[2] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i12.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i13 (.D(\mic_acc_i[2] [4]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[2] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i13.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i14 (.D(\mic_acc_i[2] [5]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[2] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i14.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i15 (.D(\mic_acc_i[2] [6]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[2] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i15.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i16 (.D(\mic_acc_i[2] [7]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[2] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i16.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i17 (.D(\mic_acc_i[1] [0]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[1] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i17.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i18 (.D(\mic_acc_i[1] [1]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[1] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i18.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i19 (.D(\mic_acc_i[1] [2]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[1] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i19.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i20 (.D(\mic_acc_i[1] [3]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[1] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i20.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i21 (.D(\mic_acc_i[1] [4]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[1] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i21.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i22 (.D(\mic_acc_i[1] [5]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[1] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i22.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i23 (.D(\mic_acc_i[1] [6]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[1] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i23.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i24 (.D(\mic_acc_i[1] [7]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[1] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i24.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i25 (.D(\mic_acc_i[0] [0]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[0] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i25.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i26 (.D(\mic_acc_i[0] [1]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[0] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i26.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i27 (.D(\mic_acc_i[0] [2]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[0] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i27.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i28 (.D(\mic_acc_i[0] [3]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[0] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i28.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i29 (.D(\mic_acc_i[0] [4]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[0] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i29.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i30 (.D(\mic_acc_i[0] [5]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[0] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i30.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i31 (.D(\mic_acc_i[0] [6]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[0] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i31.GSR = "DISABLED";
    FD1P3AX mic_win_i_3___i32 (.D(\mic_acc_i[0] [7]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_i[0] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_i_3___i32.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i2 (.D(\mic_acc_q[3] [1]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[3] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i2.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i3 (.D(\mic_acc_q[3] [2]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[3] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i3.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i4 (.D(\mic_acc_q[3] [3]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[3] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i4.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i5 (.D(\mic_acc_q[3] [4]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[3] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i5.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i6 (.D(\mic_acc_q[3] [5]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[3] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i6.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i7 (.D(\mic_acc_q[3] [6]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[3] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i7.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i8 (.D(\mic_acc_q[3] [7]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[3] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i8.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i9 (.D(\mic_acc_q[2] [0]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[2] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i9.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i10 (.D(\mic_acc_q[2] [1]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[2] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i10.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i11 (.D(\mic_acc_q[2] [2]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[2] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i11.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i12 (.D(\mic_acc_q[2] [3]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[2] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i12.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i13 (.D(\mic_acc_q[2] [4]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[2] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i13.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i14 (.D(\mic_acc_q[2] [5]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[2] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i14.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i15 (.D(\mic_acc_q[2] [6]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[2] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i15.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i16 (.D(\mic_acc_q[2] [7]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[2] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i16.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i17 (.D(\mic_acc_q[1] [0]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[1] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i17.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i18 (.D(\mic_acc_q[1] [1]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[1] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i18.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i19 (.D(\mic_acc_q[1] [2]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[1] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i19.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i20 (.D(\mic_acc_q[1] [3]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[1] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i20.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i21 (.D(\mic_acc_q[1] [4]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[1] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i21.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i22 (.D(\mic_acc_q[1] [5]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[1] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i22.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i23 (.D(\mic_acc_q[1] [6]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[1] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i23.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i24 (.D(\mic_acc_q[1] [7]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[1] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i24.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i25 (.D(\mic_acc_q[0] [0]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[0] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i25.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i26 (.D(\mic_acc_q[0] [1]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[0] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i26.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i27 (.D(\mic_acc_q[0] [2]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[0] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i27.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i28 (.D(\mic_acc_q[0] [3]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[0] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i28.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i29 (.D(\mic_acc_q[0] [4]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[0] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i29.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i30 (.D(\mic_acc_q[0] [5]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[0] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i30.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i31 (.D(\mic_acc_q[0] [6]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[0] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i31.GSR = "DISABLED";
    FD1P3AX mic_win_q_3___i32 (.D(\mic_acc_q[0] [7]), .SP(pll_clk_enable_495), 
            .CK(pll_clk), .Q(\mic_win_q[0] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_win_q_3___i32.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i1 (.D(n17695), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i2 (.D(n17697), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i3 (.D(n17699), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i4 (.D(n17701), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i5 (.D(n17703), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i6 (.D(n17705), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i7 (.D(n17707), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i8 (.D(n17709), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i9 (.D(n17711), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i10 (.D(n17713), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i11 (.D(n17715), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i12 (.D(n17717), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i13 (.D(n17719), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i14 (.D(n17721), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_gate_wait_i0_i15 (.D(n17723), .SP(pll_clk_enable_510), .CK(pll_clk), 
            .Q(mic_gate_wait[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_wait_i0_i15.GSR = "DISABLED";
    FD1P3IX mic_cfg_settle_i1 (.D(n26302), .SP(mic_acc_busy_N_2299), .CD(mic_cfg_settle_3__N_2146), 
            .CK(pll_clk), .Q(mic_cfg_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_settle_i1.GSR = "DISABLED";
    FD1P3IX mic_cfg_settle_i3 (.D(n26306), .SP(mic_acc_busy_N_2299), .CD(mic_cfg_settle_3__N_2146), 
            .CK(pll_clk), .Q(mic_cfg_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_settle_i3.GSR = "DISABLED";
    LUT4 i15255_4_lut (.A(n26056), .B(payload_end), .C(spi_phase_only_N_2792), 
         .D(n25072), .Z(n26096)) /* synthesis lut_function=(A (B)+!A (B+!((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(473[29:90])
    defparam i15255_4_lut.init = 16'hccdc;
    FD1P3AX mic_ram_wr_addr_i0_i1 (.D(mic_acc_word[1]), .SP(mic_ram_we_N_2683), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_addr_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i2 (.D(mic_acc_word[2]), .SP(mic_ram_we_N_2683), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_addr_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i3 (.D(mic_acc_gate[0]), .SP(mic_ram_we_N_2683), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_addr_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i4 (.D(mic_acc_gate[1]), .SP(mic_ram_we_N_2683), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_addr_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i5 (.D(mic_acc_gate[2]), .SP(mic_ram_we_N_2683), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_addr_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i6 (.D(mic_acc_gate[3]), .SP(mic_ram_we_N_2683), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_addr_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i7 (.D(mic_acc_gate[4]), .SP(mic_ram_we_N_2683), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_addr_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_ram_wr_addr_i0_i8 (.D(mic_acc_gate[5]), .SP(mic_ram_we_N_2683), 
            .CK(pll_clk), .Q(mic_ram_wr_addr[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_ram_wr_addr_i0_i8.GSR = "DISABLED";
    LUT4 mic_acc_word_2__bdd_4_lut_16199 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [5]), 
         .C(\mic_win_q[3] [5]), .D(mic_acc_word[0]), .Z(n27522)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16199.init = 16'he4aa;
    FD1S3AX mic_acc_state_i1 (.D(mic_acc_state_2__N_750[1]), .CK(pll_clk), 
            .Q(mic_acc_state[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_state_i1.GSR = "DISABLED";
    LUT4 i10165_4_lut (.A(ev_rd_data[35]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[35])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10165_4_lut.init = 16'h1222;
    LUT4 ev_run_hold_s5_83__I_0_i51_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_50), .D(ev_run_hold_s5[50]), .Z(us_tx_next[50])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i51_3_lut_4_lut.init = 16'hbf40;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16161_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[34]), .C(status_hold[35]), .D(status_next_index[1]), 
         .Z(n27480)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16161_4_lut_4_lut.init = 16'h55d8;
    LUT4 n27780_bdd_4_lut (.A(n27780), .B(\mic_win_q[0] [0]), .C(\mic_win_i[0] [0]), 
         .D(mic_acc_word[0]), .Z(n27783)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27780_bdd_4_lut.init = 16'haad8;
    LUT4 i15039_4_lut (.A(spi_extension_length[6]), .B(n21455), .C(n25133), 
         .D(spi_extension_length[7]), .Z(n26056)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i15039_4_lut.init = 16'hfffe;
    LUT4 ev_run_hold_s5_83__I_0_i52_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_51), .D(ev_run_hold_s5[51]), .Z(us_tx_next[51])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i52_3_lut_4_lut.init = 16'hbf40;
    LUT4 ws2812_settle_1__bdd_4_lut (.A(ws2812_settle_3__N_1717), .B(ws2812_settle[3]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(n28569)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam ws2812_settle_1__bdd_4_lut.init = 16'h0100;
    LUT4 mic_acc_state_0__bdd_4_lut (.A(mic_acc_state[0]), .B(n20918), .C(mic_acc_last), 
         .D(mic_acc_state[1]), .Z(n28134)) /* synthesis lut_function=(A ((C+!(D))+!B)+!A (D)) */ ;
    defparam mic_acc_state_0__bdd_4_lut.init = 16'hf7aa;
    FD1S3AX us_tx_i2 (.D(us_tx_next[1]), .CK(pll_clk), .Q(us_tx_c_1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i2.GSR = "DISABLED";
    FD1S3AX us_tx_i3 (.D(us_tx_next[2]), .CK(pll_clk), .Q(us_tx_c_2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i3.GSR = "DISABLED";
    FD1S3AX us_tx_i4 (.D(us_tx_next[3]), .CK(pll_clk), .Q(us_tx_c_3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i4.GSR = "DISABLED";
    FD1S3AX us_tx_i5 (.D(us_tx_next[4]), .CK(pll_clk), .Q(us_tx_c_4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i5.GSR = "DISABLED";
    FD1S3AX us_tx_i6 (.D(us_tx_next[5]), .CK(pll_clk), .Q(us_tx_c_5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i6.GSR = "DISABLED";
    FD1S3AX us_tx_i7 (.D(us_tx_next[6]), .CK(pll_clk), .Q(us_tx_c_6)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i7.GSR = "DISABLED";
    FD1S3AX us_tx_i8 (.D(us_tx_next[7]), .CK(pll_clk), .Q(us_tx_c_7)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i8.GSR = "DISABLED";
    FD1S3AX us_tx_i9 (.D(us_tx_next[8]), .CK(pll_clk), .Q(us_tx_c_8)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i9.GSR = "DISABLED";
    FD1S3AX us_tx_i10 (.D(us_tx_next[9]), .CK(pll_clk), .Q(us_tx_c_9)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i10.GSR = "DISABLED";
    FD1S3AX us_tx_i11 (.D(us_tx_next[10]), .CK(pll_clk), .Q(us_tx_c_10)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i11.GSR = "DISABLED";
    FD1S3AX us_tx_i12 (.D(us_tx_next[11]), .CK(pll_clk), .Q(us_tx_c_11)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i12.GSR = "DISABLED";
    FD1S3AX us_tx_i13 (.D(us_tx_next[12]), .CK(pll_clk), .Q(us_tx_c_12)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i13.GSR = "DISABLED";
    FD1S3AX us_tx_i14 (.D(us_tx_next[13]), .CK(pll_clk), .Q(us_tx_c_13)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i14.GSR = "DISABLED";
    FD1S3AX us_tx_i15 (.D(us_tx_next[14]), .CK(pll_clk), .Q(us_tx_c_14)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i15.GSR = "DISABLED";
    FD1S3AX us_tx_i16 (.D(us_tx_next[15]), .CK(pll_clk), .Q(us_tx_c_15)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i16.GSR = "DISABLED";
    FD1S3AX us_tx_i17 (.D(us_tx_next[16]), .CK(pll_clk), .Q(us_tx_c_16)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i17.GSR = "DISABLED";
    FD1S3AX us_tx_i18 (.D(us_tx_next[17]), .CK(pll_clk), .Q(us_tx_c_17)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i18.GSR = "DISABLED";
    FD1S3AX us_tx_i19 (.D(us_tx_next[18]), .CK(pll_clk), .Q(us_tx_c_18)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i19.GSR = "DISABLED";
    FD1S3AX us_tx_i20 (.D(us_tx_next[19]), .CK(pll_clk), .Q(us_tx_c_19)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i20.GSR = "DISABLED";
    FD1S3AX us_tx_i21 (.D(us_tx_next[20]), .CK(pll_clk), .Q(us_tx_c_20)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i21.GSR = "DISABLED";
    FD1S3AX us_tx_i22 (.D(us_tx_next[21]), .CK(pll_clk), .Q(us_tx_c_21)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i22.GSR = "DISABLED";
    FD1S3AX us_tx_i23 (.D(us_tx_next[22]), .CK(pll_clk), .Q(us_tx_c_22)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i23.GSR = "DISABLED";
    FD1S3AX us_tx_i24 (.D(us_tx_next[23]), .CK(pll_clk), .Q(us_tx_c_23)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i24.GSR = "DISABLED";
    FD1S3AX us_tx_i25 (.D(us_tx_next[24]), .CK(pll_clk), .Q(us_tx_c_24)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i25.GSR = "DISABLED";
    FD1S3AX us_tx_i26 (.D(us_tx_next[25]), .CK(pll_clk), .Q(us_tx_c_25)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i26.GSR = "DISABLED";
    FD1S3AX us_tx_i27 (.D(us_tx_next[26]), .CK(pll_clk), .Q(us_tx_c_26)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i27.GSR = "DISABLED";
    FD1S3AX us_tx_i28 (.D(us_tx_next[27]), .CK(pll_clk), .Q(us_tx_c_27)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i28.GSR = "DISABLED";
    FD1S3AX us_tx_i29 (.D(us_tx_next[28]), .CK(pll_clk), .Q(us_tx_c_28)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i29.GSR = "DISABLED";
    FD1S3AX us_tx_i30 (.D(us_tx_next[29]), .CK(pll_clk), .Q(us_tx_c_29)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i30.GSR = "DISABLED";
    FD1S3AX us_tx_i31 (.D(us_tx_next[30]), .CK(pll_clk), .Q(us_tx_c_30)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i31.GSR = "DISABLED";
    FD1S3AX us_tx_i32 (.D(us_tx_next[31]), .CK(pll_clk), .Q(us_tx_c_31)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i32.GSR = "DISABLED";
    FD1S3AX us_tx_i33 (.D(us_tx_next[32]), .CK(pll_clk), .Q(us_tx_c_32)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i33.GSR = "DISABLED";
    FD1S3AX us_tx_i34 (.D(us_tx_next[33]), .CK(pll_clk), .Q(us_tx_c_33)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i34.GSR = "DISABLED";
    FD1S3AX us_tx_i35 (.D(us_tx_next[34]), .CK(pll_clk), .Q(us_tx_c_34)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i35.GSR = "DISABLED";
    FD1S3AX us_tx_i36 (.D(us_tx_next[35]), .CK(pll_clk), .Q(us_tx_c_35)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i36.GSR = "DISABLED";
    FD1S3AX us_tx_i37 (.D(us_tx_next[36]), .CK(pll_clk), .Q(us_tx_c_36)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i37.GSR = "DISABLED";
    FD1S3AX us_tx_i38 (.D(us_tx_next[37]), .CK(pll_clk), .Q(us_tx_c_37)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i38.GSR = "DISABLED";
    FD1S3AX us_tx_i39 (.D(us_tx_next[38]), .CK(pll_clk), .Q(us_tx_c_38)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i39.GSR = "DISABLED";
    FD1S3AX us_tx_i40 (.D(us_tx_next[39]), .CK(pll_clk), .Q(us_tx_c_39)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i40.GSR = "DISABLED";
    FD1S3AX us_tx_i41 (.D(us_tx_next[40]), .CK(pll_clk), .Q(us_tx_c_40)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i41.GSR = "DISABLED";
    FD1S3AX us_tx_i42 (.D(us_tx_next[41]), .CK(pll_clk), .Q(us_tx_c_41)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i42.GSR = "DISABLED";
    FD1S3AX us_tx_i43 (.D(us_tx_next[42]), .CK(pll_clk), .Q(us_tx_c_42)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i43.GSR = "DISABLED";
    FD1S3AX us_tx_i44 (.D(us_tx_next[43]), .CK(pll_clk), .Q(us_tx_c_43)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i44.GSR = "DISABLED";
    FD1S3AX us_tx_i45 (.D(us_tx_next[44]), .CK(pll_clk), .Q(us_tx_c_44)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i45.GSR = "DISABLED";
    FD1S3AX us_tx_i46 (.D(us_tx_next[45]), .CK(pll_clk), .Q(us_tx_c_45)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i46.GSR = "DISABLED";
    FD1S3AX us_tx_i47 (.D(us_tx_next[46]), .CK(pll_clk), .Q(us_tx_c_46)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i47.GSR = "DISABLED";
    FD1S3AX us_tx_i48 (.D(us_tx_next[47]), .CK(pll_clk), .Q(us_tx_c_47)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i48.GSR = "DISABLED";
    FD1S3AX us_tx_i49 (.D(us_tx_next[48]), .CK(pll_clk), .Q(us_tx_c_48)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i49.GSR = "DISABLED";
    FD1S3AX us_tx_i50 (.D(us_tx_next[49]), .CK(pll_clk), .Q(us_tx_c_49)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i50.GSR = "DISABLED";
    FD1S3AX us_tx_i51 (.D(us_tx_next[50]), .CK(pll_clk), .Q(us_tx_c_50)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i51.GSR = "DISABLED";
    FD1S3AX us_tx_i52 (.D(us_tx_next[51]), .CK(pll_clk), .Q(us_tx_c_51)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i52.GSR = "DISABLED";
    FD1S3AX us_tx_i53 (.D(us_tx_next[52]), .CK(pll_clk), .Q(us_tx_c_52)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i53.GSR = "DISABLED";
    FD1S3AX us_tx_i54 (.D(us_tx_next[53]), .CK(pll_clk), .Q(us_tx_c_53)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i54.GSR = "DISABLED";
    FD1S3AX us_tx_i55 (.D(us_tx_next[54]), .CK(pll_clk), .Q(us_tx_c_54)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i55.GSR = "DISABLED";
    FD1S3AX us_tx_i56 (.D(us_tx_next[55]), .CK(pll_clk), .Q(us_tx_c_55)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i56.GSR = "DISABLED";
    FD1S3AX us_tx_i57 (.D(us_tx_next[56]), .CK(pll_clk), .Q(us_tx_c_56)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i57.GSR = "DISABLED";
    FD1S3AX us_tx_i58 (.D(us_tx_next[57]), .CK(pll_clk), .Q(us_tx_c_57)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i58.GSR = "DISABLED";
    FD1S3AX us_tx_i59 (.D(us_tx_next[58]), .CK(pll_clk), .Q(us_tx_c_58)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i59.GSR = "DISABLED";
    FD1S3AX us_tx_i60 (.D(us_tx_next[59]), .CK(pll_clk), .Q(us_tx_c_59)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i60.GSR = "DISABLED";
    FD1S3AX us_tx_i61 (.D(us_tx_next[60]), .CK(pll_clk), .Q(us_tx_c_60)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i61.GSR = "DISABLED";
    FD1S3AX us_tx_i62 (.D(us_tx_next[61]), .CK(pll_clk), .Q(us_tx_c_61)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i62.GSR = "DISABLED";
    FD1S3AX us_tx_i63 (.D(us_tx_next[62]), .CK(pll_clk), .Q(us_tx_c_62)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i63.GSR = "DISABLED";
    FD1S3AX us_tx_i64 (.D(us_tx_next[63]), .CK(pll_clk), .Q(us_tx_c_63)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i64.GSR = "DISABLED";
    FD1S3AX us_tx_i65 (.D(us_tx_next[64]), .CK(pll_clk), .Q(us_tx_c_64)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i65.GSR = "DISABLED";
    FD1S3AX us_tx_i66 (.D(us_tx_next[65]), .CK(pll_clk), .Q(us_tx_c_65)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i66.GSR = "DISABLED";
    FD1S3AX us_tx_i67 (.D(us_tx_next[66]), .CK(pll_clk), .Q(us_tx_c_66)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i67.GSR = "DISABLED";
    FD1S3AX us_tx_i68 (.D(us_tx_next[67]), .CK(pll_clk), .Q(us_tx_c_67)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i68.GSR = "DISABLED";
    FD1S3AX us_tx_i69 (.D(us_tx_next[68]), .CK(pll_clk), .Q(us_tx_c_68)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i69.GSR = "DISABLED";
    FD1S3AX us_tx_i70 (.D(us_tx_next[69]), .CK(pll_clk), .Q(us_tx_c_69)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i70.GSR = "DISABLED";
    FD1S3AX us_tx_i71 (.D(us_tx_next[70]), .CK(pll_clk), .Q(us_tx_c_70)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i71.GSR = "DISABLED";
    FD1S3AX us_tx_i72 (.D(us_tx_next[71]), .CK(pll_clk), .Q(us_tx_c_71)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i72.GSR = "DISABLED";
    FD1S3AX us_tx_i73 (.D(us_tx_next[72]), .CK(pll_clk), .Q(us_tx_c_72)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i73.GSR = "DISABLED";
    FD1S3AX us_tx_i74 (.D(us_tx_next[73]), .CK(pll_clk), .Q(us_tx_c_73)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i74.GSR = "DISABLED";
    FD1S3AX us_tx_i75 (.D(us_tx_next[74]), .CK(pll_clk), .Q(us_tx_c_74)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i75.GSR = "DISABLED";
    FD1S3AX us_tx_i76 (.D(us_tx_next[75]), .CK(pll_clk), .Q(us_tx_c_75)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i76.GSR = "DISABLED";
    FD1S3AX us_tx_i77 (.D(us_tx_next[76]), .CK(pll_clk), .Q(us_tx_c_76)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i77.GSR = "DISABLED";
    FD1S3AX us_tx_i78 (.D(us_tx_next[77]), .CK(pll_clk), .Q(us_tx_c_77)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i78.GSR = "DISABLED";
    FD1S3AX us_tx_i79 (.D(us_tx_next[78]), .CK(pll_clk), .Q(us_tx_c_78)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i79.GSR = "DISABLED";
    FD1S3AX us_tx_i80 (.D(us_tx_next[79]), .CK(pll_clk), .Q(us_tx_c_79)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i80.GSR = "DISABLED";
    FD1S3AX us_tx_i81 (.D(us_tx_next[80]), .CK(pll_clk), .Q(us_tx_c_80)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i81.GSR = "DISABLED";
    FD1S3AX us_tx_i82 (.D(us_tx_next[81]), .CK(pll_clk), .Q(us_tx_c_81)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i82.GSR = "DISABLED";
    FD1S3AX us_tx_i83 (.D(us_tx_next[82]), .CK(pll_clk), .Q(us_tx_c_82)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i83.GSR = "DISABLED";
    FD1S3AX us_tx_i84 (.D(us_tx_next[83]), .CK(pll_clk), .Q(us_tx_c_83)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1032[12:52])
    defparam us_tx_i84.GSR = "DISABLED";
    FD1P3AX build_level_i0_i1 (.D(pending_mod_level[1]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(build_level[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_i0_i1.GSR = "DISABLED";
    LUT4 i10164_4_lut (.A(ev_rd_data[36]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[36])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10164_4_lut.init = 16'h1222;
    LUT4 i2_3_lut_4_lut_adj_235 (.A(n17036), .B(mod_settle[0]), .C(mod_toggle_sync), 
         .D(mod_toggle_seen), .Z(n21319)) /* synthesis lut_function=(A+!(B (C (D)+!C !(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_235.init = 16'hbffb;
    LUT4 i1_2_lut_3_lut_adj_236 (.A(mod_settle[0]), .B(n17036), .C(n21319), 
         .Z(pll_clk_enable_15)) /* synthesis lut_function=(!(A (C)+!A ((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(704[18] 714[12])
    defparam i1_2_lut_3_lut_adj_236.init = 16'h0e0e;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16392_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[18]), .C(status_hold[19]), .D(status_next_index[1]), 
         .Z(n27762)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16392_4_lut_4_lut.init = 16'h55d8;
    LUT4 i15431_2_lut_3_lut_4_lut (.A(spi_byte_count[0]), .B(n8), .C(spi_byte_count[2]), 
         .D(spi_byte_count[1]), .Z(spi1_sck_c_enable_105)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15431_2_lut_3_lut_4_lut.init = 16'h4000;
    LUT4 i13941_2_lut_3_lut_4_lut (.A(feed_wr[8]), .B(mod_settle[0]), .C(n17036), 
         .D(n21319), .Z(n24167)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i13941_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 i10163_4_lut (.A(ev_rd_data[37]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[37])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10163_4_lut.init = 16'h1222;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16151_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[114]), .C(status_hold[115]), .D(status_next_index[1]), 
         .Z(n27438)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16151_4_lut_4_lut.init = 16'h55d8;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16249_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[42]), .C(status_hold[43]), .D(status_next_index[1]), 
         .Z(n27588)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16249_4_lut_4_lut.init = 16'h55d8;
    LUT4 i1_2_lut_adj_237 (.A(build_idle_N_2724), .B(frame_req), .Z(n25864)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(383[28] 384[49])
    defparam i1_2_lut_adj_237.init = 16'h2222;
    LUT4 i10162_4_lut (.A(ev_rd_data[38]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[38])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10162_4_lut.init = 16'h1222;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16109_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[58]), .C(status_hold[59]), .D(status_next_index[1]), 
         .Z(n27420)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16109_4_lut_4_lut.init = 16'h55d8;
    LUT4 n27528_bdd_4_lut (.A(n27528), .B(n28403), .C(n28406), .D(status_bit_index[3]), 
         .Z(spi1_miso_N_2456)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27528_bdd_4_lut.init = 16'haad8;
    LUT4 i2_3_lut_4_lut_adj_238 (.A(spi_command[2]), .B(spi_command[3]), 
         .C(spi_command[1]), .D(spi_command[0]), .Z(n11_adj_3278)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(580[34:54])
    defparam i2_3_lut_4_lut_adj_238.init = 16'hfeff;
    LUT4 frame_req_I_0_1559_2_lut (.A(frame_req), .B(swap_pending), .Z(build_level_eff_7__N_1792[0])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(736[21:47])
    defparam frame_req_I_0_1559_2_lut.init = 16'h2222;
    LUT4 i13942_2_lut_3_lut_4_lut (.A(feed_wr[9]), .B(mod_settle[0]), .C(n17036), 
         .D(n21319), .Z(n24165)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i13942_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 i13803_2_lut (.A(pcm_fifo_N_2624), .B(mic_sample_count[0]), .Z(n25_adj_3256)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(879[37:60])
    defparam i13803_2_lut.init = 16'h6666;
    LUT4 i15336_2_lut (.A(spi_byte_count[0]), .B(n13560), .Z(spi1_sck_c_enable_75)) /* synthesis lut_function=(!(A+(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15336_2_lut.init = 16'h1111;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16254_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[46]), .C(status_hold[47]), .D(status_next_index[1]), 
         .Z(n27600)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16254_4_lut_4_lut.init = 16'h55d8;
    LUT4 i2_3_lut_4_lut_adj_239 (.A(spi_byte_count[3]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[5]), .D(spi_byte_count[2]), .Z(n31_adj_3309)) /* synthesis lut_function=(A (B+(C+(D)))+!A (B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i2_3_lut_4_lut_adj_239.init = 16'hfefc;
    LUT4 i10465_2_lut (.A(spi_byte_count[2]), .B(spi_byte_count[1]), .Z(n21289)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10465_2_lut.init = 16'h8888;
    LUT4 i9964_2_lut (.A(spi_byte_count[4]), .B(spi_byte_count[3]), .Z(n20788)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i9964_2_lut.init = 16'heeee;
    LUT4 i10161_4_lut (.A(ev_rd_data[39]), .B(ev_in_clear), .C(ev_ch_hi_4), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[39])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10161_4_lut.init = 16'h1222;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16278_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[22]), .C(status_hold[23]), .D(status_next_index[1]), 
         .Z(n27606)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16278_4_lut_4_lut.init = 16'h55d8;
    LUT4 i13950_2_lut_3_lut_4_lut (.A(feed_wr[6]), .B(mod_settle[0]), .C(n17036), 
         .D(n21319), .Z(n24171)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i13950_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 i13939_2_lut_3_lut_4_lut (.A(feed_wr[7]), .B(mod_settle[0]), .C(n17036), 
         .D(n21319), .Z(n24169)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i13939_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 i13946_2_lut_3_lut_4_lut (.A(feed_wr[4]), .B(mod_settle[0]), .C(n17036), 
         .D(n21319), .Z(n24175)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i13946_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 i10160_4_lut (.A(ev_rd_data[40]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[40])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10160_4_lut.init = 16'h1222;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16119_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[62]), .C(status_hold[63]), .D(status_next_index[1]), 
         .Z(n27426)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16119_4_lut_4_lut.init = 16'h55d8;
    LUT4 i13947_2_lut_3_lut_4_lut (.A(feed_wr[5]), .B(mod_settle[0]), .C(n17036), 
         .D(n21319), .Z(n24173)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i13947_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16239_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[38]), .C(status_hold[39]), .D(status_next_index[1]), 
         .Z(n27492)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16239_4_lut_4_lut.init = 16'h55d8;
    LUT4 i10159_4_lut (.A(ev_rd_data[41]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[41])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10159_4_lut.init = 16'h1222;
    LUT4 i13944_2_lut_3_lut_4_lut (.A(feed_wr[2]), .B(mod_settle[0]), .C(n17036), 
         .D(n21319), .Z(n24179)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i13944_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 i13945_2_lut_3_lut_4_lut (.A(feed_wr[3]), .B(mod_settle[0]), .C(n17036), 
         .D(n21319), .Z(n24177)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i13945_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 i10158_4_lut (.A(ev_rd_data[42]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[42])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10158_4_lut.init = 16'h1222;
    LUT4 i2_3_lut_4_lut_adj_240 (.A(spi_byte_count[0]), .B(spi_byte_count[1]), 
         .C(n8), .D(spi_byte_count[2]), .Z(spi1_sck_c_enable_97)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i2_3_lut_4_lut_adj_240.init = 16'h2000;
    LUT4 i1_2_lut_3_lut_adj_241 (.A(staging_rd_page_N_2729), .B(ev_fall_N_2693), 
         .C(staging_rd_page_N_2728), .Z(staging_rd_page)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(322[35] 323[54])
    defparam i1_2_lut_3_lut_adj_241.init = 16'hfefe;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[54]), .C(status_hold[55]), .D(status_next_index[1]), 
         .Z(n27810)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_4_lut_4_lut.init = 16'h55d8;
    LUT4 i10157_4_lut (.A(ev_rd_data[43]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[43])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10157_4_lut.init = 16'h1222;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16412_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[118]), .C(status_hold[119]), .D(status_next_index[1]), 
         .Z(n27774)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16412_4_lut_4_lut.init = 16'h55d8;
    LUT4 i10156_4_lut (.A(ev_rd_data[44]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[44])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10156_4_lut.init = 16'h1222;
    LUT4 i2021_2_lut_3_lut (.A(link_timeout), .B(stop_toggle_sync), .C(stop_toggle_seen), 
         .Z(n13683)) /* synthesis lut_function=(A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(801[18] 814[12])
    defparam i2021_2_lut_3_lut.init = 16'h8282;
    LUT4 n27540_bdd_4_lut (.A(n27540), .B(\mic_win_q[0] [4]), .C(\mic_win_i[0] [4]), 
         .D(mic_acc_word[0]), .Z(n27543)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27540_bdd_4_lut.init = 16'haad8;
    LUT4 i10155_4_lut (.A(ev_rd_data[45]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[45])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10155_4_lut.init = 16'h1222;
    LUT4 i13943_2_lut_3_lut_4_lut (.A(feed_wr[1]), .B(mod_settle[0]), .C(n17036), 
         .D(n21319), .Z(n24181)) /* synthesis lut_function=(!((B (D)+!B ((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i13943_2_lut_3_lut_4_lut.init = 16'h00a8;
    LUT4 mic_acc_word_2__bdd_4_lut_16224 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [4]), 
         .C(\mic_win_q[1] [4]), .D(mic_acc_word[0]), .Z(n27540)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16224.init = 16'he4aa;
    LUT4 i1_2_lut_3_lut_4_lut_adj_242 (.A(ev_fall_N_2693), .B(frame_req), 
         .C(swap_pending), .D(build_idle_N_2724), .Z(n17806)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_242.init = 16'h0400;
    LUT4 i10154_4_lut (.A(ev_rd_data[46]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[46])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10154_4_lut.init = 16'h1222;
    LUT4 i7_4_lut_adj_243 (.A(n13_adj_3176), .B(n11_adj_3178), .C(n13_adj_3198), 
         .D(spi_command[0]), .Z(n8)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i7_4_lut_adj_243.init = 16'h0080;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16422_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[50]), .C(status_hold[51]), .D(status_next_index[1]), 
         .Z(n27798)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16422_4_lut_4_lut.init = 16'h55d8;
    LUT4 i10153_4_lut (.A(ev_rd_data[47]), .B(ev_in_clear), .C(ev_ch_hi_5), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[47])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10153_4_lut.init = 16'h1222;
    LUT4 i1_2_lut_3_lut_4_lut_adj_244 (.A(mod_settle[0]), .B(n17036), .C(n21319), 
         .D(n10742), .Z(n13685)) /* synthesis lut_function=(A ((D)+!C)+!A (B ((D)+!C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(801[18] 814[12])
    defparam i1_2_lut_3_lut_4_lut_adj_244.init = 16'hff0e;
    LUT4 i5_4_lut_adj_245 (.A(n14), .B(spi_command[1]), .C(n12_adj_3175), 
         .D(spi_byte_count[4]), .Z(n13_adj_3176)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i5_4_lut_adj_245.init = 16'h0020;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16288_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[26]), .C(status_hold[27]), .D(status_next_index[1]), 
         .Z(n27636)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16288_4_lut_4_lut.init = 16'h55d8;
    LUT4 i10142_4_lut (.A(ev_rd_data[58]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[58])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10142_4_lut.init = 16'h1222;
    LUT4 i2_4_lut_adj_246 (.A(pcm_rd_ptr_spi[3]), .B(n4_adj_3302), .C(pcm_payload_bit[7]), 
         .D(n6_adj_3314), .Z(pcm_spi_sample_15__N_1381[4])) /* synthesis lut_function=(!(A (B (C+(D))+!B !(C+(D)))+!A (B (C (D))+!B !(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(440[18:55])
    defparam i2_4_lut_adj_246.init = 16'h366c;
    LUT4 i1_2_lut_3_lut_4_lut_adj_247 (.A(fpga_time[13]), .B(fpga_time[11]), 
         .C(mic_warm), .D(mic_ultrasonic), .Z(pll_clk_enable_246)) /* synthesis lut_function=(A (B+(C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam i1_2_lut_3_lut_4_lut_adj_247.init = 16'hfff8;
    LUT4 i1_2_lut_adj_248 (.A(pcm_payload_bit[8]), .B(pcm_rd_ptr_spi[4]), 
         .Z(n4_adj_3302)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(440[18:55])
    defparam i1_2_lut_adj_248.init = 16'h6666;
    LUT4 i1_2_lut_3_lut_4_lut_adj_249 (.A(fpga_time[13]), .B(fpga_time[11]), 
         .C(mic_warm), .D(mic_ultrasonic), .Z(n17740)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(650[35:52])
    defparam i1_2_lut_3_lut_4_lut_adj_249.init = 16'h0008;
    LUT4 i1_2_lut_3_lut_4_lut_adj_250 (.A(mic_phase[0]), .B(n25867), .C(n21403), 
         .D(n4), .Z(mic_bc_valid_N_2628)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_3_lut_4_lut_adj_250.init = 16'h0080;
    LUT4 mux_679_i3_3_lut_4_lut (.A(pending_sequence[2]), .B(accepted_sequence_spi[2]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[2])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i3_3_lut_4_lut.init = 16'hacaa;
    LUT4 i1_2_lut_3_lut_4_lut_adj_251 (.A(mic_phase[3]), .B(mic_phase[0]), 
         .C(mic_phase[1]), .D(mic_phase[2]), .Z(n13_adj_3168)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_251.init = 16'hfffe;
    LUT4 status_next_bit_N_2770_2__bdd_4_lut_16259_4_lut (.A(status_next_index[2]), 
         .B(n27423), .C(n27429), .D(status_next_index[3]), .Z(n27456)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;
    defparam status_next_bit_N_2770_2__bdd_4_lut_16259_4_lut.init = 16'h55d8;
    LUT4 i15475_2_lut_3_lut_4_lut (.A(invalid_frame_spi_N_2556), .B(n20751), 
         .C(n21503), .D(spi1_sck_c_enable_159), .Z(spi1_sck_c_enable_163)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15475_2_lut_3_lut_4_lut.init = 16'h4000;
    LUT4 i6951_2_lut_3_lut_4_lut (.A(frame_req), .B(swap_pending), .C(build_idle_N_2724), 
         .D(frame_req_mod), .Z(n17799)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i6951_2_lut_3_lut_4_lut.init = 16'h2000;
    LUT4 status_next_bit_N_2770_0__bdd_4_lut_16382_4_lut_4_lut (.A(status_next_index[0]), 
         .B(status_hold[30]), .C(status_hold[31]), .D(status_next_index[1]), 
         .Z(n27648)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_0__bdd_4_lut_16382_4_lut_4_lut.init = 16'h55d8;
    LUT4 i10507_2_lut_4_lut (.A(n25972), .B(status_next_index[3]), .C(status_next_index[2]), 
         .D(status_hold[88]), .Z(n93)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i10507_2_lut_4_lut.init = 16'h2000;
    LUT4 i2_2_lut_3_lut_4_lut (.A(spi_byte_count[5]), .B(n21300), .C(n17021), 
         .D(spi_byte_count[3]), .Z(n7_adj_3260)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_2_lut_3_lut_4_lut.init = 16'hfffe;
    LUT4 i3_4_lut_adj_252 (.A(spi_byte_count[1]), .B(n13_adj_3171), .C(spi_byte_count[2]), 
         .D(spi_byte_count[3]), .Z(n11_adj_3178)) /* synthesis lut_function=(!(A (((D)+!C)+!B)+!A ((C (D)+!C !(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i3_4_lut_adj_252.init = 16'h04c0;
    LUT4 i4_2_lut (.A(spi_command[3]), .B(spi_command[2]), .Z(n14)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i4_2_lut.init = 16'h4444;
    LUT4 spi_byte_count_6__bdd_3_lut_4_lut (.A(spi_command[5]), .B(spi_command[4]), 
         .C(spi_command[0]), .D(n28935), .Z(n16952)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam spi_byte_count_6__bdd_3_lut_4_lut.init = 16'h0400;
    LUT4 mux_679_i16_3_lut_4_lut (.A(pending_sequence[15]), .B(accepted_sequence_spi[15]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[15])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i16_3_lut_4_lut.init = 16'hacaa;
    LUT4 i10141_4_lut (.A(ev_rd_data[59]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[59])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10141_4_lut.init = 16'h1222;
    LUT4 mux_679_i2_3_lut_4_lut (.A(pending_sequence[1]), .B(accepted_sequence_spi[1]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[1])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i2_3_lut_4_lut.init = 16'hacaa;
    LUT4 mux_679_i1_3_lut_4_lut (.A(pending_sequence[0]), .B(accepted_sequence_spi[0]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[0])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i1_3_lut_4_lut.init = 16'hacaa;
    LUT4 n13_bdd_4_lut (.A(n13_adj_3180), .B(mic_lo_n_next_6__N_648[5]), 
         .C(mic_lo_n_next_6__N_648[6]), .D(n28627), .Z(mic_lo_q_next)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C)))) */ ;
    defparam n13_bdd_4_lut.init = 16'h5f57;
    LUT4 status_next_bit_I_0_i124_4_lut (.A(n26986), .B(n25972), .C(status_next_index[3]), 
         .D(status_next_index[2]), .Z(n124)) /* synthesis lut_function=(A (B (C+(D))+!B (C))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_I_0_i124_4_lut.init = 16'haca0;
    CCU2D add_1592_23 (.A0(phase_frac[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24711), .COUT(n24712), .S0(phase_frac_sum[21]), 
          .S1(phase_frac_sum[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_23.INIT0 = 16'h5555;
    defparam add_1592_23.INIT1 = 16'h5aaa;
    defparam add_1592_23.INJECT1_0 = "NO";
    defparam add_1592_23.INJECT1_1 = "NO";
    FD1P3AX build_level_i0_i2 (.D(pending_mod_level[2]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(build_level[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_i0_i2.GSR = "DISABLED";
    FD1P3AX build_level_i0_i3 (.D(pending_mod_level[3]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(build_level[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_i0_i3.GSR = "DISABLED";
    FD1P3AX build_level_i0_i4 (.D(pending_mod_level[4]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(build_level[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_i0_i4.GSR = "DISABLED";
    FD1P3AX build_level_i0_i5 (.D(pending_mod_level[5]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(build_level[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_i0_i5.GSR = "DISABLED";
    FD1P3AX build_level_i0_i6 (.D(pending_mod_level[6]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(build_level[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_i0_i6.GSR = "DISABLED";
    FD1P3AX build_level_i0_i7 (.D(pending_mod_level[7]), .SP(pll_clk_enable_517), 
            .CK(pll_clk), .Q(build_level[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam build_level_i0_i7.GSR = "DISABLED";
    FD1S3AX pll_lock_sync_i1 (.D(pll_lock_sync[0]), .CK(pll_clk), .Q(status_flags_wire_15__N_1365[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(129[12:77])
    defparam pll_lock_sync_i1.GSR = "DISABLED";
    LUT4 i10506_2_lut_4_lut (.A(n25972), .B(status_next_index[3]), .C(status_next_index[2]), 
         .D(status_hold[104]), .Z(n109)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i10506_2_lut_4_lut.init = 16'h2000;
    umh_toggle_ram84 event_ram (.pll_clk(pll_clk), .ev_we(ev_we), .VCC_net(VCC_net), 
            .GND_net(GND_net), .rmw_slot({rmw_slot}), .event_rd_addr({event_rd_addr}), 
            .active_bank_N_898(active_bank_N_898), .ev_wr_data({ev_wr_data}), 
            .ev_rd_data({ev_rd_data})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[22] 342[6])
    FD1P3IX rgb_hold__i1 (.D(rgb_values[1]), .SP(pll_clk_enable_540), .CD(n13683), 
            .CK(pll_clk), .Q(rgb_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i1.GSR = "DISABLED";
    CCU2D add_1067_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(mic_gate_index[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24832), .S1(mic_done_N_2657[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1022[17:33])
    defparam add_1067_1.INIT0 = 16'hF000;
    defparam add_1067_1.INIT1 = 16'h5555;
    defparam add_1067_1.INJECT1_0 = "NO";
    defparam add_1067_1.INJECT1_1 = "NO";
    CCU2D add_1058_7 (.A0(mic_gate_fill[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_fill[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24830), .S0(mic_acc_last_N_2675[5]), .S1(mic_acc_last_N_2675[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1010[42:62])
    defparam add_1058_7.INIT0 = 16'h5aaa;
    defparam add_1058_7.INIT1 = 16'h5aaa;
    defparam add_1058_7.INJECT1_0 = "NO";
    defparam add_1058_7.INJECT1_1 = "NO";
    PFUMX i16793 (.BLUT(n28464), .ALUT(n28465), .C0(status_bit_index[1]), 
          .Z(n28466));
    LUT4 i3_4_lut_adj_253 (.A(n12_adj_3270), .B(status_next_index[1]), .C(status_next_index[3]), 
         .D(status_next_index[2]), .Z(n25210)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i3_4_lut_adj_253.init = 16'h0002;
    CCU2D add_1592_7 (.A0(phase_frac[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24703), .COUT(n24704), .S0(phase_frac_sum[5]), 
          .S1(phase_frac_sum[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_7.INIT0 = 16'h5aaa;
    defparam add_1592_7.INIT1 = 16'h5555;
    defparam add_1592_7.INJECT1_0 = "NO";
    defparam add_1592_7.INJECT1_1 = "NO";
    LUT4 status_next_bit_I_0_i12_3_lut (.A(status_hold[14]), .B(status_hold[15]), 
         .C(status_next_index[0]), .Z(n12_adj_3270)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_I_0_i12_3_lut.init = 16'hacac;
    LUT4 equal_1621_i16_2_lut_3_lut_4_lut (.A(spi_byte_count[6]), .B(spi_byte_count[7]), 
         .C(spi_byte_count[8]), .D(n4_adj_3166), .Z(n16_adj_3304)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(468[97:138])
    defparam equal_1621_i16_2_lut_3_lut_4_lut.init = 16'hfffe;
    CCU2D add_1592_9 (.A0(phase_frac[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24704), .COUT(n24705), .S0(phase_frac_sum[7]), 
          .S1(phase_frac_sum[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_9.INIT0 = 16'h5555;
    defparam add_1592_9.INIT1 = 16'h5555;
    defparam add_1592_9.INJECT1_0 = "NO";
    defparam add_1592_9.INJECT1_1 = "NO";
    LUT4 n27468_bdd_4_lut_4_lut (.A(status_next_index[5]), .B(n25210), .C(n26921), 
         .D(n27468), .Z(n27471)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27468_bdd_4_lut_4_lut.init = 16'hf588;
    FD1P3IX rgb_hold__i2 (.D(rgb_values[2]), .SP(pll_clk_enable_540), .CD(n13683), 
            .CK(pll_clk), .Q(rgb_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i2.GSR = "DISABLED";
    FD1P3IX rgb_hold__i3 (.D(rgb_values[3]), .SP(pll_clk_enable_540), .CD(n13683), 
            .CK(pll_clk), .Q(rgb_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i3.GSR = "DISABLED";
    FD1P3IX rgb_hold__i4 (.D(rgb_values[4]), .SP(pll_clk_enable_540), .CD(n13683), 
            .CK(pll_clk), .Q(rgb_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i4.GSR = "DISABLED";
    FD1P3IX rgb_hold__i5 (.D(rgb_values[5]), .SP(pll_clk_enable_540), .CD(n13683), 
            .CK(pll_clk), .Q(rgb_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i5.GSR = "DISABLED";
    FD1P3IX rgb_hold__i6 (.D(rgb_values[6]), .SP(pll_clk_enable_540), .CD(n13683), 
            .CK(pll_clk), .Q(rgb_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i6.GSR = "DISABLED";
    FD1P3IX rgb_hold__i7 (.D(rgb_values[7]), .SP(pll_clk_enable_540), .CD(n13683), 
            .CK(pll_clk), .Q(rgb_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i7.GSR = "DISABLED";
    FD1P3IX rgb_hold__i8 (.D(rgb_values[8]), .SP(pll_clk_enable_540), .CD(n13683), 
            .CK(pll_clk), .Q(rgb_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i8.GSR = "DISABLED";
    FD1P3IX rgb_hold__i9 (.D(rgb_values[9]), .SP(pll_clk_enable_540), .CD(n13683), 
            .CK(pll_clk), .Q(rgb_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i9.GSR = "DISABLED";
    FD1P3IX rgb_hold__i10 (.D(rgb_values[10]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i10.GSR = "DISABLED";
    FD1P3IX rgb_hold__i11 (.D(rgb_values[11]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i11.GSR = "DISABLED";
    FD1P3IX rgb_hold__i12 (.D(rgb_values[12]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i12.GSR = "DISABLED";
    FD1P3IX rgb_hold__i13 (.D(rgb_values[13]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i13.GSR = "DISABLED";
    FD1P3IX rgb_hold__i14 (.D(rgb_values[14]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i14.GSR = "DISABLED";
    FD1P3IX rgb_hold__i15 (.D(rgb_values[15]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i15.GSR = "DISABLED";
    FD1P3IX rgb_hold__i16 (.D(rgb_values[16]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i16.GSR = "DISABLED";
    FD1P3IX rgb_hold__i17 (.D(rgb_values[17]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i17.GSR = "DISABLED";
    FD1P3IX rgb_hold__i18 (.D(rgb_values[18]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i18.GSR = "DISABLED";
    FD1P3IX rgb_hold__i19 (.D(rgb_values[19]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i19.GSR = "DISABLED";
    FD1P3IX rgb_hold__i20 (.D(rgb_values[20]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i20.GSR = "DISABLED";
    FD1P3IX rgb_hold__i21 (.D(rgb_values[21]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i21.GSR = "DISABLED";
    FD1P3IX rgb_hold__i22 (.D(rgb_values[22]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i22.GSR = "DISABLED";
    FD1P3IX rgb_hold__i23 (.D(rgb_values[23]), .SP(pll_clk_enable_540), 
            .CD(n13683), .CK(pll_clk), .Q(rgb_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam rgb_hold__i23.GSR = "DISABLED";
    LUT4 i2_4_lut_then_3_lut_4_lut (.A(pll_clk_enable_602), .B(mic_run_state_1__N_2410[1]), 
         .C(n13496), .D(n10893), .Z(n28441)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;
    defparam i2_4_lut_then_3_lut_4_lut.init = 16'hefff;
    CCU2D add_1058_5 (.A0(mic_gate_fill[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_fill[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24829), .COUT(n24830), .S0(mic_acc_last_N_2675[3]), 
          .S1(mic_acc_last_N_2675[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1010[42:62])
    defparam add_1058_5.INIT0 = 16'h5aaa;
    defparam add_1058_5.INIT1 = 16'h5aaa;
    defparam add_1058_5.INJECT1_0 = "NO";
    defparam add_1058_5.INJECT1_1 = "NO";
    LUT4 mic_acc_word_2__bdd_4_lut_16402 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [0]), 
         .C(\mic_win_q[1] [0]), .D(mic_acc_word[0]), .Z(n27780)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16402.init = 16'he4aa;
    LUT4 i1_2_lut_3_lut_4_lut_adj_254 (.A(spi_byte_count[5]), .B(spi_byte_count[8]), 
         .C(spi_byte_count[6]), .D(spi_byte_count[7]), .Z(n13_adj_3171)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_254.init = 16'h0002;
    CCU2D add_1058_3 (.A0(mic_gate_fill[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_fill[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24828), .COUT(n24829), .S0(mic_acc_last_N_2675[1]), 
          .S1(mic_acc_last_N_2675[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1010[42:62])
    defparam add_1058_3.INIT0 = 16'h5aaa;
    defparam add_1058_3.INIT1 = 16'h5aaa;
    defparam add_1058_3.INJECT1_0 = "NO";
    defparam add_1058_3.INJECT1_1 = "NO";
    LUT4 run_addr_s3_8__I_0_i9_3_lut_4_lut (.A(run_addr_s3[8]), .B(active_bank), 
         .C(staging_rd_page_N_2729), .D(phase_step_s3), .Z(event_rd_addr[8])) /* synthesis lut_function=(A (((D)+!C)+!B)+!A !(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i9_3_lut_4_lut.init = 16'haa3a;
    CCU2D add_1058_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(mic_gate_fill[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24828), .S1(mic_acc_last_N_2675[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1010[42:62])
    defparam add_1058_1.INIT0 = 16'hF000;
    defparam add_1058_1.INIT1 = 16'h5555;
    defparam add_1058_1.INJECT1_0 = "NO";
    defparam add_1058_1.INJECT1_1 = "NO";
    LUT4 pwr_bdd_2_lut_16551 (.A(n28007), .B(n20772), .Z(n28008)) /* synthesis lut_function=(A+!(B)) */ ;
    defparam pwr_bdd_2_lut_16551.init = 16'hbbbb;
    LUT4 n27486_bdd_4_lut_4_lut (.A(status_next_index[5]), .B(n78), .C(n93), 
         .D(n27486), .Z(n27489)) /* synthesis lut_function=(A (B (C+!(D))+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam n27486_bdd_4_lut_4_lut.init = 16'hf588;
    LUT4 ev_run_hold_s5_83__I_0_i53_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_52), .D(ev_run_hold_s5[52]), .Z(us_tx_next[52])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i53_3_lut_4_lut.init = 16'hbf40;
    CCU2D add_1056_16 (.A0(mic_ram_rd_data[14]), .B0(mic_acc_win[7]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[15]), .B1(mic_acc_win[7]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24826), .S0(mic_ram_wr_data_15__N_2316[14]), 
          .S1(mic_ram_wr_data_15__N_2316[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1003[36:68])
    defparam add_1056_16.INIT0 = 16'h5666;
    defparam add_1056_16.INIT1 = 16'h5666;
    defparam add_1056_16.INJECT1_0 = "NO";
    defparam add_1056_16.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_4_lut_adj_255 (.A(spi_byte_count[0]), .B(spi_byte_count[1]), 
         .C(spi_byte_count[2]), .D(spi_byte_count[3]), .Z(n12_adj_3300)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i1_2_lut_3_lut_4_lut_adj_255.init = 16'hfffe;
    LUT4 i1_2_lut_3_lut_4_lut_adj_256 (.A(mic_acc_busy_N_2292), .B(pll_clk_enable_602), 
         .C(mic_run_state_1__N_2410[1]), .D(mic_gate_active_N_2306), .Z(pll_clk_enable_248)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_256.init = 16'hfffe;
    CCU2D add_1056_14 (.A0(mic_ram_rd_data[12]), .B0(mic_acc_win[7]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[13]), .B1(mic_acc_win[7]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24825), .COUT(n24826), .S0(mic_ram_wr_data_15__N_2316[12]), 
          .S1(mic_ram_wr_data_15__N_2316[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1003[36:68])
    defparam add_1056_14.INIT0 = 16'h5666;
    defparam add_1056_14.INIT1 = 16'h5666;
    defparam add_1056_14.INJECT1_0 = "NO";
    defparam add_1056_14.INJECT1_1 = "NO";
    PFUMX i16791 (.BLUT(n28461), .ALUT(n28462), .C0(mic_done), .Z(n28463));
    LUT4 ev_run_hold_s5_83__I_0_i9_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_8), .D(ev_run_hold_s5[8]), .Z(us_tx_next[8])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i9_3_lut_4_lut.init = 16'hbf40;
    LUT4 run_addr_s3_8__I_0_i8_3_lut_4_lut (.A(run_addr_s3[7]), .B(rmw_slot[7]), 
         .C(staging_rd_page_N_2729), .D(phase_step_s3), .Z(event_rd_addr[7])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i8_3_lut_4_lut.init = 16'haaca;
    LUT4 status_next_bit_N_2770_4__bdd_4_lut_4_lut_4_lut (.A(status_next_index[4]), 
         .B(n109), .C(n124), .D(status_next_index[5]), .Z(n27486)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_4__bdd_4_lut_4_lut_4_lut.init = 16'h55d8;
    LUT4 i14956_2_lut (.A(status_next_index[0]), .B(status_next_index[1]), 
         .Z(n25972)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14956_2_lut.init = 16'h8888;
    LUT4 ev_run_hold_s5_83__I_0_i54_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_53), .D(ev_run_hold_s5[53]), .Z(us_tx_next[53])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i54_3_lut_4_lut.init = 16'hbf40;
    LUT4 i9_4_lut_adj_257 (.A(n25944), .B(n25932), .C(n25936), .D(n16_adj_3307), 
         .Z(payload_end)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i9_4_lut_adj_257.init = 16'h0100;
    CCU2D add_1056_12 (.A0(mic_ram_rd_data[10]), .B0(mic_acc_win[7]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[11]), .B1(mic_acc_win[7]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24824), .COUT(n24825), .S0(mic_ram_wr_data_15__N_2316[10]), 
          .S1(mic_ram_wr_data_15__N_2316[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1003[36:68])
    defparam add_1056_12.INIT0 = 16'h5666;
    defparam add_1056_12.INIT1 = 16'h5666;
    defparam add_1056_12.INJECT1_0 = "NO";
    defparam add_1056_12.INJECT1_1 = "NO";
    PFUMX i16789 (.BLUT(n28458), .ALUT(n28459), .C0(status_bit_index[0]), 
          .Z(n28460));
    LUT4 i1_2_lut_adj_258 (.A(mic_run_state_1__N_2410[1]), .B(pll_clk_enable_253), 
         .Z(pll_clk_enable_97)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_258.init = 16'heeee;
    LUT4 n20772_bdd_4_lut_16613 (.A(spi_audio_read_cmd), .B(spi_byte_count[2]), 
         .C(spi_byte_count[3]), .D(spi_byte_count[5]), .Z(n28007)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;
    defparam n20772_bdd_4_lut_16613.init = 16'hffbf;
    CCU2D add_1056_10 (.A0(mic_ram_rd_data[8]), .B0(mic_acc_win[7]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[9]), .B1(mic_acc_win[7]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24823), .COUT(n24824), .S0(mic_ram_wr_data_15__N_2316[8]), 
          .S1(mic_ram_wr_data_15__N_2316[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1003[36:68])
    defparam add_1056_10.INIT0 = 16'h5666;
    defparam add_1056_10.INIT1 = 16'h5666;
    defparam add_1056_10.INJECT1_0 = "NO";
    defparam add_1056_10.INJECT1_1 = "NO";
    LUT4 n27564_bdd_4_lut_then_3_lut (.A(mic_block_count[4]), .B(mic_block_count[6]), 
         .C(status_bit_index[1]), .Z(n28435)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27564_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 i10140_4_lut (.A(ev_rd_data[60]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[60])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10140_4_lut.init = 16'h1222;
    CCU2D add_13433_7 (.A0(n16258), .B0(n20743), .C0(n16995), .D0(spi_rgb_start[7]), 
          .A1(n12_adj_3287), .B1(n11992), .C1(n11990), .D1(n11988), 
          .CIN(n24947), .S0(n24207), .S1(n24206));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(458[33] 460[80])
    defparam add_13433_7.INIT0 = 16'ha655;
    defparam add_13433_7.INIT1 = 16'hfe01;
    defparam add_13433_7.INJECT1_0 = "NO";
    defparam add_13433_7.INJECT1_1 = "NO";
    CCU2D add_13433_5 (.A0(n13030), .B0(n11996), .C0(n8_adj_3297), .D0(n11994), 
          .A1(n16260), .B1(n20743), .C1(n16995), .D1(spi_rgb_start[7]), 
          .CIN(n24946), .COUT(n24947), .S0(n24209), .S1(n24208));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(458[33] 460[80])
    defparam add_13433_5.INIT0 = 16'ha956;
    defparam add_13433_5.INIT1 = 16'haa59;
    defparam add_13433_5.INJECT1_0 = "NO";
    defparam add_13433_5.INJECT1_1 = "NO";
    CCU2D add_1056_8 (.A0(mic_ram_rd_data[6]), .B0(mic_acc_win[6]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[7]), .B1(mic_acc_win[7]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24822), .COUT(n24823), .S0(mic_ram_wr_data_15__N_2316[6]), 
          .S1(mic_ram_wr_data_15__N_2316[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1003[36:68])
    defparam add_1056_8.INIT0 = 16'h5666;
    defparam add_1056_8.INIT1 = 16'h5666;
    defparam add_1056_8.INJECT1_0 = "NO";
    defparam add_1056_8.INJECT1_1 = "NO";
    CCU2D add_13433_3 (.A0(n16266), .B0(n20743), .C0(n16995), .D0(spi_rgb_start[7]), 
          .A1(n16995), .B1(n20743), .C1(n11996), .D1(n8_adj_3297), .CIN(n24945), 
          .COUT(n24946), .S0(n24211), .S1(n24210));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(458[33] 460[80])
    defparam add_13433_3.INIT0 = 16'haa59;
    defparam add_13433_3.INIT1 = 16'hb44b;
    defparam add_13433_3.INJECT1_0 = "NO";
    defparam add_13433_3.INJECT1_1 = "NO";
    CCU2D add_1056_6 (.A0(mic_ram_rd_data[4]), .B0(mic_acc_win[4]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[5]), .B1(mic_acc_win[5]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24821), .COUT(n24822), .S0(mic_ram_wr_data_15__N_2316[4]), 
          .S1(mic_ram_wr_data_15__N_2316[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1003[36:68])
    defparam add_1056_6.INIT0 = 16'h5666;
    defparam add_1056_6.INIT1 = 16'h5666;
    defparam add_1056_6.INJECT1_0 = "NO";
    defparam add_1056_6.INJECT1_1 = "NO";
    CCU2D add_13433_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n16995), .B1(n20743), .C1(n12000), .D1(n7), .COUT(n24945), 
          .S1(n24212));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(458[33] 460[80])
    defparam add_13433_1.INIT0 = 16'hF000;
    defparam add_13433_1.INIT1 = 16'h4bb4;
    defparam add_13433_1.INJECT1_0 = "NO";
    defparam add_13433_1.INJECT1_1 = "NO";
    LUT4 n20772_bdd_3_lut (.A(spi_audio_read_cmd), .B(n12_adj_3300), .C(spi_byte_count[5]), 
         .Z(n28006)) /* synthesis lut_function=((B+!(C))+!A) */ ;
    defparam n20772_bdd_3_lut.init = 16'hdfdf;
    CCU2D ev_ch_3078_add_4_7 (.A0(ev_ch[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_ch[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24943), .S0(n35_adj_3203), .S1(n34_adj_3204));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078_add_4_7.INIT0 = 16'hfaaa;
    defparam ev_ch_3078_add_4_7.INIT1 = 16'hfaaa;
    defparam ev_ch_3078_add_4_7.INJECT1_0 = "NO";
    defparam ev_ch_3078_add_4_7.INJECT1_1 = "NO";
    CCU2D add_1056_4 (.A0(mic_ram_rd_data[2]), .B0(mic_acc_win[2]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[3]), .B1(mic_acc_win[3]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24820), .COUT(n24821), .S0(mic_ram_wr_data_15__N_2316[2]), 
          .S1(mic_ram_wr_data_15__N_2316[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1003[36:68])
    defparam add_1056_4.INIT0 = 16'h5666;
    defparam add_1056_4.INIT1 = 16'h5666;
    defparam add_1056_4.INJECT1_0 = "NO";
    defparam add_1056_4.INJECT1_1 = "NO";
    LUT4 spi1_miso_N_2463_2__bdd_4_lut_16189_4_lut (.A(status_bit_index[2]), 
         .B(n28448), .C(n28436), .D(status_bit_index[3]), .Z(n27414)) /* synthesis lut_function=(A (B (D))+!A (C+!(D))) */ ;
    defparam spi1_miso_N_2463_2__bdd_4_lut_16189_4_lut.init = 16'hd855;
    LUT4 i3_4_lut_adj_259 (.A(spi_byte_count[1]), .B(n38), .C(spi_byte_count[3]), 
         .D(n21300), .Z(n30)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i3_4_lut_adj_259.init = 16'h0008;
    LUT4 i1_4_lut_adj_260 (.A(spi_byte_count[5]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[2]), .D(spi_byte_count[0]), .Z(n38)) /* synthesis lut_function=(!(A (B+(C))+!A (B (C)+!B !(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(492[21] 540[24])
    defparam i1_4_lut_adj_260.init = 16'h1617;
    CCU2D ev_ch_3078_add_4_5 (.A0(ev_ch[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_ch[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24942), .COUT(n24943), .S0(n37), .S1(n36));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078_add_4_5.INIT0 = 16'hfaaa;
    defparam ev_ch_3078_add_4_5.INIT1 = 16'hfaaa;
    defparam ev_ch_3078_add_4_5.INJECT1_0 = "NO";
    defparam ev_ch_3078_add_4_5.INJECT1_1 = "NO";
    FD1P3AX mic_acc_gate__i2 (.D(mic_gate_index[1]), .SP(mic_acc_busy_N_2293), 
            .CK(pll_clk), .Q(mic_acc_gate[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_gate__i2.GSR = "DISABLED";
    PFUMX i16787 (.BLUT(n28455), .ALUT(n28456), .C0(status_bit_index[0]), 
          .Z(n28457));
    LUT4 run_addr_s3_8__I_0_i7_3_lut_4_lut (.A(run_addr_s3[6]), .B(rmw_slot[6]), 
         .C(staging_rd_page_N_2729), .D(phase_step_s3), .Z(event_rd_addr[6])) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(334[33:74])
    defparam run_addr_s3_8__I_0_i7_3_lut_4_lut.init = 16'haaca;
    LUT4 ev_run_hold_s5_83__I_0_i8_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_7), .D(ev_run_hold_s5[7]), .Z(us_tx_next[7])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i8_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i7_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_6), .D(ev_run_hold_s5[6]), .Z(us_tx_next[6])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i7_3_lut_4_lut.init = 16'hbf40;
    LUT4 mux_1049_i2_4_lut (.A(mic_gate_active), .B(n10879), .C(n10893), 
         .D(mic_acc_state[1]), .Z(mic_acc_state_2__N_2347[1])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(962[9] 982[12])
    defparam mux_1049_i2_4_lut.init = 16'hca0a;
    LUT4 i3_4_lut_adj_261 (.A(spi_byte_count[0]), .B(n16_adj_3304), .C(n8_adj_3298), 
         .D(spi_byte_count[1]), .Z(n23802)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(489[30:53])
    defparam i3_4_lut_adj_261.init = 16'hfffd;
    LUT4 mux_2202_i1_3_lut_4_lut (.A(frame_req), .B(swap_pending), .C(event_level[0]), 
         .D(ev_rd_grant), .Z(n11017)) /* synthesis lut_function=(A (B (C (D))+!B (C+!(D)))+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam mux_2202_i1_3_lut_4_lut.init = 16'hf022;
    FD1P3AX mic_acc_gate__i3 (.D(mic_gate_index[2]), .SP(mic_acc_busy_N_2293), 
            .CK(pll_clk), .Q(mic_acc_gate[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_gate__i3.GSR = "DISABLED";
    FD1P3AX mic_acc_gate__i4 (.D(mic_gate_index[3]), .SP(mic_acc_busy_N_2293), 
            .CK(pll_clk), .Q(mic_acc_gate[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_gate__i4.GSR = "DISABLED";
    FD1P3AX mic_acc_gate__i5 (.D(mic_gate_index[4]), .SP(mic_acc_busy_N_2293), 
            .CK(pll_clk), .Q(mic_acc_gate[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_gate__i5.GSR = "DISABLED";
    FD1P3AX mic_acc_gate__i6 (.D(mic_gate_index[5]), .SP(mic_acc_busy_N_2293), 
            .CK(pll_clk), .Q(mic_acc_gate[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_gate__i6.GSR = "DISABLED";
    LUT4 status_next_bit_N_2770_4__bdd_4_lut_16156_4_lut_4_lut (.A(status_next_index[4]), 
         .B(n26906), .C(n27459), .D(status_next_index[5]), .Z(n27468)) /* synthesis lut_function=(!(A ((D)+!B)+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam status_next_bit_N_2770_4__bdd_4_lut_16156_4_lut_4_lut.init = 16'h55d8;
    CCU2D ev_ch_3078_add_4_3 (.A0(ev_ch[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_ch[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24941), .COUT(n24942), .S0(n39_adj_3201), .S1(n38_adj_3202));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078_add_4_3.INIT0 = 16'hfaaa;
    defparam ev_ch_3078_add_4_3.INIT1 = 16'hfaaa;
    defparam ev_ch_3078_add_4_3.INJECT1_0 = "NO";
    defparam ev_ch_3078_add_4_3.INJECT1_1 = "NO";
    CCU2D add_1056_2 (.A0(mic_ram_rd_data[0]), .B0(mic_acc_win[0]), .C0(GND_net), 
          .D0(GND_net), .A1(mic_ram_rd_data[1]), .B1(mic_acc_win[1]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24820), .S1(mic_ram_wr_data_15__N_2316[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1003[36:68])
    defparam add_1056_2.INIT0 = 16'h7000;
    defparam add_1056_2.INIT1 = 16'h5666;
    defparam add_1056_2.INJECT1_0 = "NO";
    defparam add_1056_2.INJECT1_1 = "NO";
    FD1S3IX ev_state_FSM_i2 (.D(n15113), .CK(pll_clk), .CD(n10742), .Q(ev_in_clear));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam ev_state_FSM_i2.GSR = "DISABLED";
    LUT4 n27564_bdd_4_lut_else_3_lut (.A(mic_block_count[5]), .B(mic_block_count[7]), 
         .C(status_bit_index[1]), .Z(n28434)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27564_bdd_4_lut_else_3_lut.init = 16'hacac;
    CCU2D ev_ch_3078_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_ch[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24941), .S1(n40_adj_3200));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(783[30:42])
    defparam ev_ch_3078_add_4_1.INIT0 = 16'hF000;
    defparam ev_ch_3078_add_4_1.INIT1 = 16'h0555;
    defparam ev_ch_3078_add_4_1.INJECT1_0 = "NO";
    defparam ev_ch_3078_add_4_1.INJECT1_1 = "NO";
    CCU2D time_divider_3075_add_4_7 (.A0(time_divider[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24818), .S0(n30_adj_3182));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(649[25:44])
    defparam time_divider_3075_add_4_7.INIT0 = 16'hfaaa;
    defparam time_divider_3075_add_4_7.INIT1 = 16'h0000;
    defparam time_divider_3075_add_4_7.INJECT1_0 = "NO";
    defparam time_divider_3075_add_4_7.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i6_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_5), .D(ev_run_hold_s5[5]), .Z(us_tx_next[5])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i6_3_lut_4_lut.init = 16'hbf40;
    LUT4 sub_229_inv_0_i3_1_lut (.A(status_next_index[2]), .Z(status_next_bit_N_2770[2])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam sub_229_inv_0_i3_1_lut.init = 16'h5555;
    LUT4 i2_2_lut_adj_262 (.A(spi_command[5]), .B(spi_command[4]), .Z(n13_adj_3167)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i2_2_lut_adj_262.init = 16'h4444;
    PFUMX i16785 (.BLUT(n28452), .ALUT(n28453), .C0(status_bit_index[0]), 
          .Z(n28454));
    LUT4 n27786_bdd_4_lut (.A(n27786), .B(\mic_win_q[2] [4]), .C(\mic_win_i[2] [4]), 
         .D(mic_acc_word[0]), .Z(n27789)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27786_bdd_4_lut.init = 16'haad8;
    CCU2D time_divider_3075_add_4_5 (.A0(time_divider[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24817), .COUT(n24818), .S0(n32), 
          .S1(n31_adj_3183));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(649[25:44])
    defparam time_divider_3075_add_4_5.INIT0 = 16'hfaaa;
    defparam time_divider_3075_add_4_5.INIT1 = 16'hfaaa;
    defparam time_divider_3075_add_4_5.INJECT1_0 = "NO";
    defparam time_divider_3075_add_4_5.INJECT1_1 = "NO";
    FD1S3IX ev_state_FSM_i3 (.D(n16013), .CK(pll_clk), .CD(n10742), .Q(n10092));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam ev_state_FSM_i3.GSR = "DISABLED";
    FD1S3IX ev_state_FSM_i4 (.D(n10126), .CK(pll_clk), .CD(n10742), .Q(staging_rd_page_N_2728));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam ev_state_FSM_i4.GSR = "DISABLED";
    FD1S3IX ev_state_FSM_i5 (.D(n25805), .CK(pll_clk), .CD(n10742), .Q(staging_rd_page_N_2729));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam ev_state_FSM_i5.GSR = "DISABLED";
    FD1S3IX ev_state_FSM_i6 (.D(ev_rd_grant), .CK(pll_clk), .CD(n10742), 
            .Q(ev_fall_N_2693));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam ev_state_FSM_i6.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i2 (.D(n5740), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i2.GSR = "DISABLED";
    CCU2D time_divider_3075_add_4_3 (.A0(time_divider[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24816), .COUT(n24817), .S0(n34), 
          .S1(n33));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(649[25:44])
    defparam time_divider_3075_add_4_3.INIT0 = 16'hfaaa;
    defparam time_divider_3075_add_4_3.INIT1 = 16'hfaaa;
    defparam time_divider_3075_add_4_3.INJECT1_0 = "NO";
    defparam time_divider_3075_add_4_3.INJECT1_1 = "NO";
    CCU2D mic_warm_half_3079_add_4_9 (.A0(mic_warm_half[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24940), .S0(n38_adj_3214));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079_add_4_9.INIT0 = 16'hfaaa;
    defparam mic_warm_half_3079_add_4_9.INIT1 = 16'h0000;
    defparam mic_warm_half_3079_add_4_9.INJECT1_0 = "NO";
    defparam mic_warm_half_3079_add_4_9.INJECT1_1 = "NO";
    CCU2D time_divider_3075_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(time_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24816), .S1(n35));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(649[25:44])
    defparam time_divider_3075_add_4_1.INIT0 = 16'hF000;
    defparam time_divider_3075_add_4_1.INIT1 = 16'h0555;
    defparam time_divider_3075_add_4_1.INJECT1_0 = "NO";
    defparam time_divider_3075_add_4_1.INJECT1_1 = "NO";
    CCU2D mic_warm_half_3079_add_4_7 (.A0(mic_warm_half[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_warm_half[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24939), .COUT(n24940), .S0(n40_adj_3212), 
          .S1(n39_adj_3213));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_warm_half_3079_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_warm_half_3079_add_4_7.INJECT1_0 = "NO";
    defparam mic_warm_half_3079_add_4_7.INJECT1_1 = "NO";
    LUT4 sub_229_inv_0_i4_1_lut (.A(status_next_index[3]), .Z(status_next_bit_N_2770[3])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam sub_229_inv_0_i4_1_lut.init = 16'h5555;
    PFUMX i16783 (.BLUT(n28449), .ALUT(n28450), .C0(status_bit_index[0]), 
          .Z(n28451));
    CCU2D mic_warm_half_3079_add_4_5 (.A0(mic_warm_half[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_warm_half[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24938), .COUT(n24939), .S0(n42_adj_3210), 
          .S1(n41_adj_3211));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_warm_half_3079_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_warm_half_3079_add_4_5.INJECT1_0 = "NO";
    defparam mic_warm_half_3079_add_4_5.INJECT1_1 = "NO";
    CCU2D mic_warm_half_3079_add_4_3 (.A0(mic_warm_half[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(mic_warm_half[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24937), .COUT(n24938), .S0(n44_adj_3208), 
          .S1(n43_adj_3209));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_warm_half_3079_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_warm_half_3079_add_4_3.INJECT1_0 = "NO";
    defparam mic_warm_half_3079_add_4_3.INJECT1_1 = "NO";
    FD1P3IX mic_acc_i_3___i3 (.D(n5739), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i3.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i4 (.D(n5738), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i4.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i5 (.D(n5737), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i5.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i6 (.D(n5736), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i6.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i7 (.D(n5735), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i7.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i8 (.D(n5734), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[3] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i8.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i9 (.D(n5733), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i9.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i10 (.D(n5732), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i10.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i11 (.D(n5731), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i11.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i12 (.D(n5730), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i12.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i13 (.D(n5729), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i13.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i14 (.D(n5728), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i14.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i15 (.D(n5727), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i15.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i16 (.D(n5726), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[2] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i16.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i17 (.D(n5725), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i17.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i18 (.D(n5724), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i18.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i19 (.D(n5723), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i19.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i20 (.D(n5722), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i20.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i21 (.D(n5721), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i21.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i22 (.D(n5720), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i22.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i23 (.D(n5719), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i23.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i24 (.D(n5718), .SP(pll_clk_enable_586), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[1] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i24.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i25 (.D(n5717), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i25.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i26 (.D(n5716), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i26.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i27 (.D(n5715), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i27.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i28 (.D(n5714), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i28.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i29 (.D(n5713), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i29.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i30 (.D(n5712), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i30.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i31 (.D(n5711), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i31.GSR = "DISABLED";
    FD1P3IX mic_acc_i_3___i32 (.D(n5710), .SP(pll_clk_enable_594), .CD(mic_run_state_1__N_2410[1]), 
            .CK(pll_clk), .Q(\mic_acc_i[0] [7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_i_3___i32.GSR = "DISABLED";
    LUT4 mic_acc_word_2__bdd_4_lut (.A(mic_acc_word[2]), .B(\mic_win_i[3] [4]), 
         .C(\mic_win_q[3] [4]), .D(mic_acc_word[0]), .Z(n27786)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut.init = 16'he4aa;
    LUT4 i1686_2_lut_3_lut_3_lut (.A(ev_fall), .B(ev_fall_N_2693), .C(ev_wrap), 
         .Z(n10129)) /* synthesis lut_function=(A (B (C))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(767[21:29])
    defparam i1686_2_lut_3_lut_3_lut.init = 16'hc4c4;
    PFUMX i16781 (.BLUT(n28446), .ALUT(n28447), .C0(status_bit_index[0]), 
          .Z(n28448));
    LUT4 i15_3_lut_4_lut_4_lut (.A(ev_fall), .B(pll_clk_enable_517), .C(ev_fall_N_2693), 
         .D(ev_wrap), .Z(pll_clk_enable_168)) /* synthesis lut_function=(!(A (B (C (D))+!B ((D)+!C))+!A ((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(767[21:29])
    defparam i15_3_lut_4_lut_4_lut.init = 16'h0cac;
    LUT4 ev_run_hold_s5_83__I_0_i55_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_54), .D(ev_run_hold_s5[54]), .Z(us_tx_next[54])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i55_3_lut_4_lut.init = 16'hbf40;
    LUT4 i10_4_lut_adj_263 (.A(spi_rx_shift[6]), .B(n20), .C(n16_adj_3262), 
         .D(spi_rx_shift[2]), .Z(n25133)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i10_4_lut_adj_263.init = 16'hfffe;
    LUT4 i15185_2_lut_2_lut (.A(ev_fall), .B(ev_sum[1]), .Z(n26099)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(767[21:29])
    defparam i15185_2_lut_2_lut.init = 16'h4444;
    CCU2D mic_warm_half_3079_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_warm_half[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24937), .S1(n45_adj_3207));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam mic_warm_half_3079_add_4_1.INIT0 = 16'hF000;
    defparam mic_warm_half_3079_add_4_1.INIT1 = 16'h0555;
    defparam mic_warm_half_3079_add_4_1.INJECT1_0 = "NO";
    defparam mic_warm_half_3079_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_264 (.A(n16_adj_3304), .B(spi_bit_count_2__N_1425), 
         .C(n12_adj_3300), .Z(n44_adj_3257)) /* synthesis lut_function=(A (B)+!A (B (C))) */ ;
    defparam i1_3_lut_adj_264.init = 16'hc8c8;
    LUT4 i15236_2_lut_2_lut (.A(ev_fall), .B(ev_sum[7]), .Z(n26174)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(767[21:29])
    defparam i15236_2_lut_2_lut.init = 16'h4444;
    LUT4 i5651_2_lut (.A(n16952), .B(spi_bit_count_2__N_1425), .Z(feed_we)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam i5651_2_lut.init = 16'h8888;
    LUT4 i15233_2_lut_2_lut (.A(ev_fall), .B(ev_sum[4]), .Z(n26168)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(767[21:29])
    defparam i15233_2_lut_2_lut.init = 16'h4444;
    CCU2D add_2546_9 (.A0(\mic_acc_i[3] [6]), .B0(n20759), .C0(n4599), 
          .D0(GND_net), .A1(\mic_acc_i[3] [7]), .B1(n20759), .C1(n4599), 
          .D1(GND_net), .CIN(n24814), .S0(n5735), .S1(n5734));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam add_2546_9.INIT0 = 16'h7848;
    defparam add_2546_9.INIT1 = 16'h7848;
    defparam add_2546_9.INJECT1_0 = "NO";
    defparam add_2546_9.INJECT1_1 = "NO";
    PFUMX i16777 (.BLUT(n28440), .ALUT(n28441), .C0(mic_acc_state[2]), 
          .Z(pll_clk_enable_603));
    FD1P3IX mic_cfg_gap_i2 (.D(mic_cfg_gap_w[2]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[2])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i2.GSR = "DISABLED";
    LUT4 mux_3341_i2_3_lut (.A(n14108), .B(n14109), .C(n14105), .Z(rd_data_7__N_2913[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3341_i2_3_lut.init = 16'hcaca;
    LUT4 ev_run_hold_s5_83__I_0_i74_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_73), .D(ev_run_hold_s5[73]), .Z(us_tx_next[73])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i74_3_lut_4_lut.init = 16'hbf40;
    LUT4 i12060_4_lut (.A(n13_adj_3168), .B(mic_clk_c), .C(mic_warm), 
         .D(n28), .Z(mic_clk_N_2474)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C)+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(208[17:25])
    defparam i12060_4_lut.init = 16'h0535;
    LUT4 i3_4_lut_adj_265 (.A(n20772), .B(spi_byte_count[4]), .C(n8_adj_3298), 
         .D(n17027), .Z(n25072)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[25:31])
    defparam i3_4_lut_adj_265.init = 16'hfffd;
    LUT4 ev_run_hold_s5_83__I_0_i56_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_55), .D(ev_run_hold_s5[55]), .Z(us_tx_next[55])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i56_3_lut_4_lut.init = 16'hbf40;
    LUT4 mux_3341_i3_3_lut (.A(n14110), .B(n14111), .C(n14105), .Z(rd_data_7__N_2913[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3341_i3_3_lut.init = 16'hcaca;
    LUT4 i2_3_lut_adj_266 (.A(spi_command[3]), .B(n14_adj_3258), .C(spi_command[2]), 
         .Z(n16995)) /* synthesis lut_function=((B+(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[61:83])
    defparam i2_3_lut_adj_266.init = 16'hfdfd;
    LUT4 mux_3341_i4_3_lut (.A(n14112), .B(n14113), .C(n14105), .Z(rd_data_7__N_2913[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3341_i4_3_lut.init = 16'hcaca;
    LUT4 mux_3341_i5_3_lut (.A(n14114), .B(n14115), .C(n14105), .Z(rd_data_7__N_2913[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3341_i5_3_lut.init = 16'hcaca;
    LUT4 mux_3341_i6_3_lut (.A(n14116), .B(n14117), .C(n14105), .Z(rd_data_7__N_2913[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3341_i6_3_lut.init = 16'hcaca;
    CCU2D pcm_wr_ptr_3080_add_4_7 (.A0(pcm_wr_ptr[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24936), .S0(n30_adj_3220));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[39:56])
    defparam pcm_wr_ptr_3080_add_4_7.INIT0 = 16'hfaaa;
    defparam pcm_wr_ptr_3080_add_4_7.INIT1 = 16'h0000;
    defparam pcm_wr_ptr_3080_add_4_7.INJECT1_0 = "NO";
    defparam pcm_wr_ptr_3080_add_4_7.INJECT1_1 = "NO";
    LUT4 mux_3341_i7_3_lut (.A(n14118), .B(n14119), .C(n14105), .Z(rd_data_7__N_2913[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3341_i7_3_lut.init = 16'hcaca;
    LUT4 n14151_bdd_3_lut (.A(n14151), .B(n14152), .C(status_bit_index[0]), 
         .Z(n28893)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n14151_bdd_3_lut.init = 16'hacac;
    LUT4 mux_3341_i8_3_lut (.A(n14120), .B(n14121), .C(n14105), .Z(rd_data_7__N_2913[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[41:53])
    defparam mux_3341_i8_3_lut.init = 16'hcaca;
    LUT4 mux_679_i15_3_lut_4_lut (.A(pending_sequence[14]), .B(accepted_sequence_spi[14]), 
         .C(ws2812_settle[1]), .D(n28569), .Z(accepted_sequence_15__N_1001[14])) /* synthesis lut_function=(A (B+(C+!(D)))+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(725[13] 731[16])
    defparam mux_679_i15_3_lut_4_lut.init = 16'hacaa;
    PFUMX i16773 (.BLUT(n28434), .ALUT(n28435), .C0(status_bit_index[0]), 
          .Z(n28436));
    LUT4 i6641_1_lut (.A(mic_run_state_1__N_2410[1]), .Z(mic_run_state_1__N_2410[0])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(939[9] 960[12])
    defparam i6641_1_lut.init = 16'h5555;
    FD1P3IX mic_cfg_gap_i1 (.D(mic_cfg_gap_w[1]), .SP(pll_clk_enable_602), 
            .CD(n17767), .CK(pll_clk), .Q(mic_cfg_gap[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_gap_i1.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_267 (.A(n11990), .B(n11992), .C(n12_adj_3287), .Z(n16258)) /* synthesis lut_function=(!(A (B+(C))+!A !(B+(C)))) */ ;
    defparam i1_3_lut_adj_267.init = 16'h5656;
    LUT4 i1_2_lut_adj_268 (.A(mic_cfg_settle[0]), .B(n17_adj_3196), .Z(mic_cfg_settle_3__N_2142[0])) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_adj_268.init = 16'h4444;
    CCU2D pcm_wr_ptr_3080_add_4_5 (.A0(pcm_wr_ptr[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_wr_ptr[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24935), .COUT(n24936), .S0(n32_adj_3218), 
          .S1(n31_adj_3219));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[39:56])
    defparam pcm_wr_ptr_3080_add_4_5.INIT0 = 16'hfaaa;
    defparam pcm_wr_ptr_3080_add_4_5.INIT1 = 16'hfaaa;
    defparam pcm_wr_ptr_3080_add_4_5.INJECT1_0 = "NO";
    defparam pcm_wr_ptr_3080_add_4_5.INJECT1_1 = "NO";
    CCU2D add_2546_7 (.A0(\mic_acc_i[3] [4]), .B0(n20759), .C0(n4599), 
          .D0(GND_net), .A1(\mic_acc_i[3] [5]), .B1(n20759), .C1(n4599), 
          .D1(GND_net), .CIN(n24813), .COUT(n24814), .S0(n5737), .S1(n5736));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam add_2546_7.INIT0 = 16'h7848;
    defparam add_2546_7.INIT1 = 16'h7848;
    defparam add_2546_7.INJECT1_0 = "NO";
    defparam add_2546_7.INJECT1_1 = "NO";
    FD1S3AX feed_rd_3077__i1 (.D(n77_adj_3311), .CK(pll_clk), .Q(feed_rd[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077__i1.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_269 (.A(n20788), .B(n12_adj_3272), .C(n13_adj_3171), 
         .D(n17021), .Z(n13550)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;
    defparam i1_3_lut_4_lut_adj_269.init = 16'hffbf;
    CCU2D pcm_wr_ptr_3080_add_4_3 (.A0(pcm_wr_ptr[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_wr_ptr[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24934), .COUT(n24935), .S0(n34_adj_3216), 
          .S1(n33_adj_3217));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[39:56])
    defparam pcm_wr_ptr_3080_add_4_3.INIT0 = 16'hfaaa;
    defparam pcm_wr_ptr_3080_add_4_3.INIT1 = 16'hfaaa;
    defparam pcm_wr_ptr_3080_add_4_3.INJECT1_0 = "NO";
    defparam pcm_wr_ptr_3080_add_4_3.INJECT1_1 = "NO";
    CCU2D pcm_wr_ptr_3080_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_fifo_N_2617[5]), .B1(pcm_wr_ptr[0]), .C1(GND_net), 
          .D1(GND_net), .COUT(n24934), .S1(n35_adj_3215));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[39:56])
    defparam pcm_wr_ptr_3080_add_4_1.INIT0 = 16'hF000;
    defparam pcm_wr_ptr_3080_add_4_1.INIT1 = 16'ha999;
    defparam pcm_wr_ptr_3080_add_4_1.INJECT1_0 = "NO";
    defparam pcm_wr_ptr_3080_add_4_1.INJECT1_1 = "NO";
    LUT4 i15239_4_lut (.A(mic_lo_n_next_6__N_648[5]), .B(mic_lo_n_next_6__N_648[6]), 
         .C(n130), .D(mic_lo_n_next_6__N_648[3]), .Z(n26160)) /* synthesis lut_function=(A (B)+!A (B+!(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[17:30])
    defparam i15239_4_lut.init = 16'hcddd;
    CCU2D add_2546_5 (.A0(\mic_acc_i[3] [2]), .B0(n20759), .C0(n4599), 
          .D0(GND_net), .A1(\mic_acc_i[3] [3]), .B1(n20759), .C1(n4599), 
          .D1(GND_net), .CIN(n24812), .COUT(n24813), .S0(n5739), .S1(n5738));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam add_2546_5.INIT0 = 16'h7848;
    defparam add_2546_5.INIT1 = 16'h7848;
    defparam add_2546_5.INJECT1_0 = "NO";
    defparam add_2546_5.INJECT1_1 = "NO";
    CCU2D add_2546_3 (.A0(\mic_acc_i[3] [0]), .B0(n20759), .C0(n4599), 
          .D0(GND_net), .A1(\mic_acc_i[3] [1]), .B1(n20759), .C1(n4599), 
          .D1(GND_net), .CIN(n24811), .COUT(n24812), .S0(n5741), .S1(n5740));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam add_2546_3.INIT0 = 16'h7484;
    defparam add_2546_3.INIT1 = 16'h7848;
    defparam add_2546_3.INJECT1_0 = "NO";
    defparam add_2546_3.INJECT1_1 = "NO";
    CCU2D add_2546_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20759), .B1(n4599), .C1(GND_net), .D1(GND_net), .COUT(n24811));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam add_2546_1.INIT0 = 16'hF000;
    defparam add_2546_1.INIT1 = 16'h7777;
    defparam add_2546_1.INJECT1_0 = "NO";
    defparam add_2546_1.INJECT1_1 = "NO";
    CCU2D add_850_7 (.A0(mic_window_count[5]), .B0(n13458), .C0(GND_net), 
          .D0(GND_net), .A1(mic_window_count[6]), .B1(n13458), .C1(GND_net), 
          .D1(GND_net), .CIN(n24806), .S0(n10812), .S1(n10813));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(886[41:64])
    defparam add_850_7.INIT0 = 16'h7888;
    defparam add_850_7.INIT1 = 16'h7888;
    defparam add_850_7.INJECT1_0 = "NO";
    defparam add_850_7.INJECT1_1 = "NO";
    CCU2D spi_channel_index_3069_add_4_7 (.A0(spi_channel_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24932), .S0(n35_adj_3226), 
          .S1(n34_adj_3227));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069_add_4_7.INIT0 = 16'hfaaa;
    defparam spi_channel_index_3069_add_4_7.INIT1 = 16'hfaaa;
    defparam spi_channel_index_3069_add_4_7.INJECT1_0 = "NO";
    defparam spi_channel_index_3069_add_4_7.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i57_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_56), .D(ev_run_hold_s5[56]), .Z(us_tx_next[56])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i57_3_lut_4_lut.init = 16'hbf40;
    LUT4 n14154_bdd_3_lut (.A(n14154), .B(n14153), .C(status_bit_index[0]), 
         .Z(n28890)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n14154_bdd_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_adj_270 (.A(mic_warm), .B(mic_phase[1]), .C(mic_phase[0]), 
         .Z(n5688)) /* synthesis lut_function=(!(A+(B (C)+!B !(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_3_lut_adj_270.init = 16'h1414;
    LUT4 i1_4_lut_adj_271 (.A(mic_warm), .B(mic_phase[2]), .C(mic_phase[1]), 
         .D(mic_phase[0]), .Z(n5687)) /* synthesis lut_function=(!(A+(B (C (D))+!B !(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_4_lut_adj_271.init = 16'h1444;
    LUT4 i9_4_lut_adj_272 (.A(expected_next_8__N_1406[3]), .B(n18_adj_3261), 
         .C(spi_rgb_start[7]), .D(spi_extension_length[5]), .Z(n20)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i9_4_lut_adj_272.init = 16'hfffe;
    LUT4 i1_4_lut_adj_273 (.A(mic_warm), .B(mic_phase[3]), .C(n54_adj_3170), 
         .D(mic_phase[0]), .Z(n5686)) /* synthesis lut_function=(!(A+(B (C (D))+!B !(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_4_lut_adj_273.init = 16'h1444;
    LUT4 i1_2_lut_adj_274 (.A(mic_phase[1]), .B(mic_phase[2]), .Z(n54_adj_3170)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_adj_274.init = 16'h8888;
    LUT4 i15248_2_lut (.A(mic_run_state[1]), .B(mic_run_state[0]), .Z(n26132)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(448[69:94])
    defparam i15248_2_lut.init = 16'heeee;
    LUT4 ws2812_toggle_sync_I_0_2_lut (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .Z(ws2812_settle_3__N_1717)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(716[13:53])
    defparam ws2812_toggle_sync_I_0_2_lut.init = 16'h6666;
    LUT4 i2_3_lut_adj_275 (.A(n8_adj_3297), .B(n11994), .C(n11996), .Z(n12_adj_3287)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(504[50:75])
    defparam i2_3_lut_adj_275.init = 16'hfefe;
    LUT4 i1_4_lut_adj_276 (.A(n22794), .B(n17307), .C(n4_adj_3290), .D(mic_phase[2]), 
         .Z(pll_clk_enable_594)) /* synthesis lut_function=(A (B)+!A !((C (D))+!B)) */ ;
    defparam i1_4_lut_adj_276.init = 16'h8ccc;
    PFUMX i16530 (.BLUT(n28028), .ALUT(n28027), .C0(spi_command[2]), .Z(n28029));
    LUT4 i10134_4_lut (.A(ev_rd_data[66]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[66])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10134_4_lut.init = 16'h1222;
    LUT4 i14938_2_lut (.A(spi_command[7]), .B(spi_command[6]), .Z(n13_adj_3191)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14938_2_lut.init = 16'heeee;
    LUT4 i15234_2_lut_2_lut (.A(ev_fall), .B(ev_sum[5]), .Z(n26170)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(767[21:29])
    defparam i15234_2_lut_2_lut.init = 16'h4444;
    LUT4 i1_2_lut_adj_277 (.A(mic_phase[3]), .B(mic_warm), .Z(n4_adj_3290)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_adj_277.init = 16'h2222;
    LUT4 i11743_4_lut (.A(mic_lo_n_next_6__N_648[6]), .B(mic_lo_n_next_6__N_648[3]), 
         .C(n136), .D(mic_lo_n_next_6__N_648[5]), .Z(n22510)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A !(D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[17:30])
    defparam i11743_4_lut.init = 16'haad5;
    LUT4 i9948_2_lut (.A(spi_byte_count[1]), .B(spi_byte_count[0]), .Z(n20772)) /* synthesis lut_function=(A (B)) */ ;
    defparam i9948_2_lut.init = 16'h8888;
    LUT4 i15186_2_lut_2_lut (.A(ev_fall), .B(ev_sum[2]), .Z(n26133)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(767[21:29])
    defparam i15186_2_lut_2_lut.init = 16'h4444;
    LUT4 i1_2_lut_adj_278 (.A(spi_command[5]), .B(spi_command[4]), .Z(n17018)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(174[38:60])
    defparam i1_2_lut_adj_278.init = 16'hbbbb;
    LUT4 i6917_3_lut (.A(pll_clk_enable_602), .B(n16832), .C(mic_cfg_hold_spi[6]), 
         .Z(n17763)) /* synthesis lut_function=(A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i6917_3_lut.init = 16'h8282;
    LUT4 i10133_4_lut (.A(ev_rd_data[67]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[67])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10133_4_lut.init = 16'h1222;
    LUT4 i9_4_lut_adj_279 (.A(n17_adj_3164), .B(n20772), .C(n16), .D(n11), 
         .Z(n16924)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i9_4_lut_adj_279.init = 16'h2000;
    LUT4 i13916_2_lut (.A(spi_extension_length[2]), .B(expected_next_8__N_1406[3]), 
         .Z(n12000)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13916_2_lut.init = 16'h6666;
    LUT4 i10132_4_lut (.A(ev_rd_data[68]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[68])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10132_4_lut.init = 16'h1222;
    FD1S3AX feed_rd_3077__i2 (.D(n76_adj_3312), .CK(pll_clk), .Q(feed_rd[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077__i2.GSR = "DISABLED";
    FD1S3AX feed_rd_3077__i3 (.D(n75_adj_3313), .CK(pll_clk), .Q(feed_rd[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077__i3.GSR = "DISABLED";
    FD1S3AX feed_rd_3077__i4 (.D(n74), .CK(pll_clk), .Q(feed_rd[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077__i4.GSR = "DISABLED";
    FD1S3AX feed_rd_3077__i5 (.D(n73_adj_3288), .CK(pll_clk), .Q(feed_rd[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077__i5.GSR = "DISABLED";
    FD1S3AX feed_rd_3077__i6 (.D(n72), .CK(pll_clk), .Q(feed_rd[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077__i6.GSR = "DISABLED";
    FD1S3AX feed_rd_3077__i7 (.D(n71), .CK(pll_clk), .Q(feed_rd[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077__i7.GSR = "DISABLED";
    FD1S3AX feed_rd_3077__i8 (.D(n70_adj_3308), .CK(pll_clk), .Q(feed_rd[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077__i8.GSR = "DISABLED";
    FD1S3AX feed_rd_3077__i9 (.D(n69), .CK(pll_clk), .Q(feed_rd[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077__i9.GSR = "DISABLED";
    FD1P3AX spi_command_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_171), 
            .CK(spi1_sck_c), .Q(spi_command[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_command_i0_i1.GSR = "ENABLED";
    LUT4 i7_4_lut_adj_280 (.A(spi_byte_count[4]), .B(spi_bit_count[1]), 
         .C(spi_bit_count[0]), .D(spi_byte_count[8]), .Z(n17_adj_3164)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;
    defparam i7_4_lut_adj_280.init = 16'h0040;
    CCU2D add_1592_5 (.A0(phase_frac[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24702), .COUT(n24703), .S0(phase_frac_sum[3]), 
          .S1(phase_frac_sum[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_5.INIT0 = 16'h5aaa;
    defparam add_1592_5.INIT1 = 16'h5aaa;
    defparam add_1592_5.INJECT1_0 = "NO";
    defparam add_1592_5.INJECT1_1 = "NO";
    CCU2D feed_wr_spi_3073_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(n16952), .B1(spi_bit_count_2__N_1425), .C1(feed_wr_spi[0]), 
          .D1(GND_net), .COUT(n24767), .S1(n55));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073_add_4_1.INIT0 = 16'hF000;
    defparam feed_wr_spi_3073_add_4_1.INIT1 = 16'h7878;
    defparam feed_wr_spi_3073_add_4_1.INJECT1_0 = "NO";
    defparam feed_wr_spi_3073_add_4_1.INJECT1_1 = "NO";
    CCU2D pcm_decim_count_3081_add_4_9 (.A0(pcm_decim_count[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24766), .S0(n38_adj_3254));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081_add_4_9.INIT0 = 16'hfaaa;
    defparam pcm_decim_count_3081_add_4_9.INIT1 = 16'h0000;
    defparam pcm_decim_count_3081_add_4_9.INJECT1_0 = "NO";
    defparam pcm_decim_count_3081_add_4_9.INJECT1_1 = "NO";
    CCU2D pcm_decim_count_3081_add_4_7 (.A0(pcm_decim_count[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(pcm_decim_count[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24765), .COUT(n24766), .S0(n40_adj_3252), 
          .S1(n39_adj_3253));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081_add_4_7.INIT0 = 16'hfaaa;
    defparam pcm_decim_count_3081_add_4_7.INIT1 = 16'hfaaa;
    defparam pcm_decim_count_3081_add_4_7.INJECT1_0 = "NO";
    defparam pcm_decim_count_3081_add_4_7.INJECT1_1 = "NO";
    CCU2D pcm_decim_count_3081_add_4_5 (.A0(pcm_decim_count[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(pcm_decim_count[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24764), .COUT(n24765), .S0(n42_adj_3250), 
          .S1(n41_adj_3251));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(875[40:62])
    defparam pcm_decim_count_3081_add_4_5.INIT0 = 16'hfaaa;
    defparam pcm_decim_count_3081_add_4_5.INIT1 = 16'hfaaa;
    defparam pcm_decim_count_3081_add_4_5.INJECT1_0 = "NO";
    defparam pcm_decim_count_3081_add_4_5.INJECT1_1 = "NO";
    LUT4 i10131_4_lut (.A(ev_rd_data[69]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[69])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10131_4_lut.init = 16'h1222;
    LUT4 ev_run_hold_s5_83__I_0_i67_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_66), .D(ev_run_hold_s5[66]), .Z(us_tx_next[66])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i67_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_adj_281 (.A(spi_rgb_start[7]), .B(spi_phase_only_N_2792), 
         .Z(n13030)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i1_2_lut_adj_281.init = 16'h8888;
    LUT4 i1_2_lut_adj_282 (.A(n11992), .B(n12_adj_3287), .Z(n16260)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i1_2_lut_adj_282.init = 16'h6666;
    LUT4 i15390_4_lut (.A(phase_step_s4), .B(link_timeout), .C(stop_event), 
         .D(running), .Z(ev_run_hold_s5_83__N_1649)) /* synthesis lut_function=((B+(C+!(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(644[13:69])
    defparam i15390_4_lut.init = 16'hfdff;
    LUT4 i5_2_lut (.A(spi_rx_shift[5]), .B(spi1_mosi_c_0), .Z(n16_adj_3262)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i5_2_lut.init = 16'heeee;
    LUT4 ev_run_hold_s5_83__I_0_i75_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_74), .D(ev_run_hold_s5[74]), .Z(us_tx_next[74])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i75_3_lut_4_lut.init = 16'hbf40;
    PFUMX i16518 (.BLUT(n28008), .ALUT(n28006), .C0(spi_byte_count[4]), 
          .Z(n28009));
    PFUMX i4270 (.BLUT(n15108), .ALUT(n26156), .C0(ev_fall_N_2693), .Z(n15109));
    CCU2D add_850_5 (.A0(mic_window_count[3]), .B0(n13458), .C0(GND_net), 
          .D0(GND_net), .A1(mic_window_count[4]), .B1(n13458), .C1(GND_net), 
          .D1(GND_net), .CIN(n24805), .COUT(n24806), .S0(n10810), .S1(n10811));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(886[41:64])
    defparam add_850_5.INIT0 = 16'h7888;
    defparam add_850_5.INIT1 = 16'h7888;
    defparam add_850_5.INJECT1_0 = "NO";
    defparam add_850_5.INJECT1_1 = "NO";
    LUT4 i2_4_lut_else_3_lut (.A(n10879), .B(n13496), .C(n10893), .D(n28134), 
         .Z(n28440)) /* synthesis lut_function=(!(A (B (C)+!B (C (D))))) */ ;
    defparam i2_4_lut_else_3_lut.init = 16'h5f7f;
    LUT4 i10417_4_lut (.A(mic_cfg_width[5]), .B(n9_adj_3292), .C(n8_adj_3293), 
         .D(n10_adj_3291), .Z(mic_acc_last_N_2682)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;
    defparam i10417_4_lut.init = 16'h0001;
    LUT4 i15352_2_lut (.A(spi_byte_count[0]), .B(n13550), .Z(spi1_sck_c_enable_50)) /* synthesis lut_function=(!(A+(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15352_2_lut.init = 16'h1111;
    LUT4 i10077_2_lut (.A(time_divider[2]), .B(time_divider[5]), .Z(n20901)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10077_2_lut.init = 16'h8888;
    LUT4 i15232_2_lut_2_lut (.A(ev_fall), .B(ev_sum[3]), .Z(n26166)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(767[21:29])
    defparam i15232_2_lut_2_lut.init = 16'h4444;
    LUT4 i1_2_lut_3_lut_4_lut_adj_283 (.A(feed_pop), .B(mod_settle[0]), 
         .C(n17036), .D(n21319), .Z(frame_req_mod_N_2752)) /* synthesis lut_function=(A+!(B (D)+!B ((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(704[18] 714[12])
    defparam i1_2_lut_3_lut_4_lut_adj_283.init = 16'haafe;
    LUT4 i10139_4_lut (.A(ev_rd_data[61]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[61])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10139_4_lut.init = 16'h1222;
    LUT4 n27558_bdd_4_lut_16509_else_3_lut (.A(n14127), .B(n14128), .C(status_bit_index[0]), 
         .Z(n28443)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27558_bdd_4_lut_16509_else_3_lut.init = 16'hacac;
    CCU2D add_850_3 (.A0(mic_window_count[1]), .B0(n13458), .C0(GND_net), 
          .D0(GND_net), .A1(mic_window_count[2]), .B1(n13458), .C1(GND_net), 
          .D1(GND_net), .CIN(n24804), .COUT(n24805), .S0(n10808), .S1(n10809));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(886[41:64])
    defparam add_850_3.INIT0 = 16'h7888;
    defparam add_850_3.INIT1 = 16'h7888;
    defparam add_850_3.INJECT1_0 = "NO";
    defparam add_850_3.INJECT1_1 = "NO";
    LUT4 equal_1621_i13_2_lut (.A(spi_byte_count[4]), .B(spi_byte_count[5]), 
         .Z(n4_adj_3166)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(468[97:138])
    defparam equal_1621_i13_2_lut.init = 16'heeee;
    CCU2D spi_channel_index_3069_add_4_5 (.A0(spi_channel_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24931), .COUT(n24932), .S0(n37_adj_3224), 
          .S1(n36_adj_3225));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069_add_4_5.INIT0 = 16'hfaaa;
    defparam spi_channel_index_3069_add_4_5.INIT1 = 16'hfaaa;
    defparam spi_channel_index_3069_add_4_5.INJECT1_0 = "NO";
    defparam spi_channel_index_3069_add_4_5.INJECT1_1 = "NO";
    CCU2D add_850_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n24374), .B1(n21403), .C1(n4), .D1(n13458), .COUT(n24804), 
          .S1(n10807));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(886[41:64])
    defparam add_850_1.INIT0 = 16'hF000;
    defparam add_850_1.INIT1 = 16'h59aa;
    defparam add_850_1.INJECT1_0 = "NO";
    defparam add_850_1.INJECT1_1 = "NO";
    LUT4 i9902_3_lut (.A(mic_cfg_hold_spi[40]), .B(n16834), .C(mic_cfg_hold_spi[46]), 
         .Z(mic_cfg_width_w[0])) /* synthesis lut_function=(!(A (B (C))+!A (B+(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[35] 289[104])
    defparam i9902_3_lut.init = 16'h2b2b;
    LUT4 i5_3_lut (.A(mic_cfg_hold_spi[43]), .B(n10_adj_3199), .C(mic_cfg_hold_spi[45]), 
         .Z(n16834)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[35:68])
    defparam i5_3_lut.init = 16'hfefe;
    LUT4 i10138_4_lut (.A(ev_rd_data[62]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[62])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10138_4_lut.init = 16'h1222;
    LUT4 ev_run_hold_s5_83__I_0_i76_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_75), .D(ev_run_hold_s5[75]), .Z(us_tx_next[75])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i76_3_lut_4_lut.init = 16'hbf40;
    LUT4 ev_run_hold_s5_83__I_0_i58_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_57), .D(ev_run_hold_s5[57]), .Z(us_tx_next[57])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i58_3_lut_4_lut.init = 16'hbf40;
    CCU2D spi_channel_index_3069_add_4_3 (.A0(spi_channel_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24930), .COUT(n24931), .S0(n39_adj_3222), 
          .S1(n38_adj_3223));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069_add_4_3.INIT0 = 16'hfaaa;
    defparam spi_channel_index_3069_add_4_3.INIT1 = 16'hfaaa;
    defparam spi_channel_index_3069_add_4_3.INJECT1_0 = "NO";
    defparam spi_channel_index_3069_add_4_3.INJECT1_1 = "NO";
    LUT4 i10130_4_lut (.A(ev_rd_data[70]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[70])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10130_4_lut.init = 16'h1222;
    LUT4 n27552_bdd_4_lut_then_3_lut (.A(mic_block_count[0]), .B(mic_block_count[2]), 
         .C(status_bit_index[1]), .Z(n28447)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27552_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 n27552_bdd_4_lut_else_3_lut (.A(mic_block_count[1]), .B(mic_block_count[3]), 
         .C(status_bit_index[1]), .Z(n28446)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27552_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 i6_4_lut_adj_284 (.A(fpga_cs_n_c), .B(spi_bit_count[2]), .C(expected_next_8__N_1406[3]), 
         .D(spi_byte_count[2]), .Z(n16)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;
    defparam i6_4_lut_adj_284.init = 16'h4000;
    CCU2D spi_channel_index_3069_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(n25051), .B1(n28156), .C1(spi_channel_index[0]), 
          .D1(GND_net), .COUT(n24930), .S1(n40_adj_3221));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069_add_4_1.INIT0 = 16'hF000;
    defparam spi_channel_index_3069_add_4_1.INIT1 = 16'he1e1;
    defparam spi_channel_index_3069_add_4_1.INJECT1_0 = "NO";
    defparam spi_channel_index_3069_add_4_1.INJECT1_1 = "NO";
    CCU2D spi_byte_count_3071_add_4_9 (.A0(spi_byte_count[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[8]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24928), .S0(n43_adj_3234), 
          .S1(n42_adj_3235));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071_add_4_9.INIT0 = 16'hfaaa;
    defparam spi_byte_count_3071_add_4_9.INIT1 = 16'hfaaa;
    defparam spi_byte_count_3071_add_4_9.INJECT1_0 = "NO";
    defparam spi_byte_count_3071_add_4_9.INJECT1_1 = "NO";
    LUT4 i10129_4_lut (.A(ev_rd_data[71]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[71])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10129_4_lut.init = 16'h1222;
    LUT4 i10128_4_lut (.A(ev_rd_data[72]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[72])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10128_4_lut.init = 16'h1222;
    LUT4 i10127_4_lut (.A(ev_rd_data[73]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[73])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10127_4_lut.init = 16'h1222;
    LUT4 n27546_bdd_4_lut_16739_then_3_lut (.A(mic_block_count[12]), .B(mic_block_count[14]), 
         .C(status_bit_index[1]), .Z(n28450)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27546_bdd_4_lut_16739_then_3_lut.init = 16'hacac;
    CCU2D spi_byte_count_3071_add_4_7 (.A0(spi_byte_count[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24927), .COUT(n24928), .S0(n45_adj_3232), 
          .S1(n44_adj_3233));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071_add_4_7.INIT0 = 16'hfaaa;
    defparam spi_byte_count_3071_add_4_7.INIT1 = 16'hfaaa;
    defparam spi_byte_count_3071_add_4_7.INJECT1_0 = "NO";
    defparam spi_byte_count_3071_add_4_7.INJECT1_1 = "NO";
    LUT4 n27546_bdd_4_lut_16739_else_3_lut (.A(mic_block_count[13]), .B(mic_block_count[15]), 
         .C(status_bit_index[1]), .Z(n28449)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27546_bdd_4_lut_16739_else_3_lut.init = 16'hacac;
    LUT4 i10126_4_lut (.A(ev_rd_data[74]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[74])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10126_4_lut.init = 16'h1222;
    LUT4 i10125_4_lut (.A(ev_rd_data[75]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[75])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10125_4_lut.init = 16'h1222;
    LUT4 n27534_bdd_4_lut_then_3_lut (.A(mic_block_count[8]), .B(mic_block_count[10]), 
         .C(status_bit_index[1]), .Z(n28453)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27534_bdd_4_lut_then_3_lut.init = 16'hacac;
    CCU2D add_2555_9 (.A0(\mic_acc_q[3] [6]), .B0(n20759), .C0(n4628), 
          .D0(GND_net), .A1(\mic_acc_q[3] [7]), .B1(n20759), .C1(n4628), 
          .D1(GND_net), .CIN(n24798), .S0(n5768), .S1(n5767));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam add_2555_9.INIT0 = 16'h7848;
    defparam add_2555_9.INIT1 = 16'h7848;
    defparam add_2555_9.INJECT1_0 = "NO";
    defparam add_2555_9.INJECT1_1 = "NO";
    CCU2D spi_byte_count_3071_add_4_5 (.A0(spi_byte_count[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24926), .COUT(n24927), .S0(n47), 
          .S1(n46));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071_add_4_5.INIT0 = 16'hfaaa;
    defparam spi_byte_count_3071_add_4_5.INIT1 = 16'hfaaa;
    defparam spi_byte_count_3071_add_4_5.INJECT1_0 = "NO";
    defparam spi_byte_count_3071_add_4_5.INJECT1_1 = "NO";
    LUT4 spi_byte_count_8__I_0_i11_2_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .Z(n8_adj_3298)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(468[57:94])
    defparam spi_byte_count_8__I_0_i11_2_lut.init = 16'heeee;
    LUT4 n27534_bdd_4_lut_else_3_lut (.A(mic_block_count[9]), .B(mic_block_count[11]), 
         .C(status_bit_index[1]), .Z(n28452)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27534_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 n27510_bdd_4_lut_then_3_lut (.A(mic_ram_rd_data[4]), .B(mic_ram_rd_data[6]), 
         .C(status_bit_index[1]), .Z(n28456)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27510_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 i10124_4_lut (.A(ev_rd_data[76]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[76])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10124_4_lut.init = 16'h1222;
    LUT4 n27510_bdd_4_lut_else_3_lut (.A(mic_ram_rd_data[5]), .B(mic_ram_rd_data[7]), 
         .C(status_bit_index[1]), .Z(n28455)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27510_bdd_4_lut_else_3_lut.init = 16'hacac;
    PFUMX i16771 (.BLUT(n28431), .ALUT(n28432), .C0(mic_acc_state[2]), 
          .Z(n28433));
    LUT4 reduce_or_1682_i1_3_lut (.A(n10092), .B(ev_sum[8]), .C(ev_in_clear), 
         .Z(n10126)) /* synthesis lut_function=(A+(B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam reduce_or_1682_i1_3_lut.init = 16'heaea;
    LUT4 i10123_4_lut (.A(ev_rd_data[77]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[77])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10123_4_lut.init = 16'h1222;
    CCU2D add_2555_7 (.A0(\mic_acc_q[3] [4]), .B0(n20759), .C0(n4628), 
          .D0(GND_net), .A1(\mic_acc_q[3] [5]), .B1(n20759), .C1(n4628), 
          .D1(GND_net), .CIN(n24797), .COUT(n24798), .S0(n5770), .S1(n5769));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam add_2555_7.INIT0 = 16'h7848;
    defparam add_2555_7.INIT1 = 16'h7848;
    defparam add_2555_7.INJECT1_0 = "NO";
    defparam add_2555_7.INJECT1_1 = "NO";
    LUT4 i3_4_lut_adj_285 (.A(time_divider[0]), .B(time_divider[3]), .C(time_divider[1]), 
         .D(time_divider[4]), .Z(n25021)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_4_lut_adj_285.init = 16'h8000;
    CCU2D spi_byte_count_3071_add_4_3 (.A0(spi_byte_count[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24925), .COUT(n24926), .S0(n49), 
          .S1(n48));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071_add_4_3.INIT0 = 16'hfaaa;
    defparam spi_byte_count_3071_add_4_3.INIT1 = 16'hfaaa;
    defparam spi_byte_count_3071_add_4_3.INJECT1_0 = "NO";
    defparam spi_byte_count_3071_add_4_3.INJECT1_1 = "NO";
    LUT4 i2_3_lut_4_lut_adj_286 (.A(n20788), .B(n12_adj_3272), .C(n16953), 
         .D(n21440), .Z(spi1_sck_c_enable_51)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;
    defparam i2_3_lut_4_lut_adj_286.init = 16'h0040;
    CCU2D add_2555_5 (.A0(\mic_acc_q[3] [2]), .B0(n20759), .C0(n4628), 
          .D0(GND_net), .A1(\mic_acc_q[3] [3]), .B1(n20759), .C1(n4628), 
          .D1(GND_net), .CIN(n24796), .COUT(n24797), .S0(n5772), .S1(n5771));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam add_2555_5.INIT0 = 16'h7848;
    defparam add_2555_5.INIT1 = 16'h7848;
    defparam add_2555_5.INJECT1_0 = "NO";
    defparam add_2555_5.INJECT1_1 = "NO";
    CCU2D spi_byte_count_3071_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_bit_count[0]), .B1(n20745), .C1(spi_byte_count[0]), 
          .D1(GND_net), .COUT(n24925), .S1(n50));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071_add_4_1.INIT0 = 16'hF000;
    defparam spi_byte_count_3071_add_4_1.INIT1 = 16'h7878;
    defparam spi_byte_count_3071_add_4_1.INJECT1_0 = "NO";
    defparam spi_byte_count_3071_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_287 (.A(staging_rd_page_N_2728), .B(staging_rd_page_N_2729), 
         .C(n10129), .D(phase_step_s3), .Z(n25805)) /* synthesis lut_function=(A+(B (C+(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i1_4_lut_adj_287.init = 16'hfefa;
    CCU2D sub_1025_add_2_17 (.A0(mic_gate_wait[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24924), .S0(n6864));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[38:59])
    defparam sub_1025_add_2_17.INIT0 = 16'h5555;
    defparam sub_1025_add_2_17.INIT1 = 16'h0000;
    defparam sub_1025_add_2_17.INJECT1_0 = "NO";
    defparam sub_1025_add_2_17.INJECT1_1 = "NO";
    LUT4 i10122_4_lut (.A(ev_rd_data[78]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[78])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10122_4_lut.init = 16'h1222;
    LUT4 i10121_4_lut (.A(ev_rd_data[79]), .B(ev_in_clear), .C(ev_ch_hi_9), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[79])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10121_4_lut.init = 16'h1222;
    LUT4 i4_4_lut_adj_288 (.A(mic_cfg_hold_spi[44]), .B(mic_cfg_hold_spi[42]), 
         .C(mic_cfg_hold_spi[40]), .D(mic_cfg_hold_spi[41]), .Z(n10_adj_3199)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[35:68])
    defparam i4_4_lut_adj_288.init = 16'hfffe;
    LUT4 i15235_2_lut_2_lut (.A(ev_fall), .B(ev_sum[6]), .Z(n26172)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(767[21:29])
    defparam i15235_2_lut_2_lut.init = 16'h4444;
    LUT4 spi_command_1__bdd_4_lut_16980 (.A(spi_command[1]), .B(spi_command[3]), 
         .C(n25040), .D(spi_command[0]), .Z(n28028)) /* synthesis lut_function=(A (C (D))+!A !(B (D)+!B ((D)+!C))) */ ;
    defparam spi_command_1__bdd_4_lut_16980.init = 16'ha054;
    LUT4 i10120_4_lut (.A(ev_rd_data[80]), .B(ev_in_clear), .C(ev_ch_hi_10), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[80])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10120_4_lut.init = 16'h1222;
    LUT4 i1_2_lut_4_lut_adj_289 (.A(frame_settle[3]), .B(frame_settle[1]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(n7_adj_3263)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(675[22:42])
    defparam i1_2_lut_4_lut_adj_289.init = 16'hfeff;
    LUT4 i10230_3_lut (.A(frame_settle[1]), .B(n7_adj_3263), .C(frame_settle[0]), 
         .Z(n3425)) /* synthesis lut_function=(A (B (C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(679[13] 690[16])
    defparam i10230_3_lut.init = 16'h8484;
    LUT4 i1_2_lut_adj_290 (.A(n31), .B(spi1_sck_c_enable_53), .Z(spi1_sck_c_enable_152)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i1_2_lut_adj_290.init = 16'h4444;
    LUT4 ev_run_hold_s5_83__I_0_i70_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_69), .D(ev_run_hold_s5[69]), .Z(us_tx_next[69])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i70_3_lut_4_lut.init = 16'hbf40;
    CCU2D sub_1025_add_2_15 (.A0(mic_gate_wait[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24923), .COUT(n24924), .S0(n6866), .S1(n6865));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[38:59])
    defparam sub_1025_add_2_15.INIT0 = 16'h5555;
    defparam sub_1025_add_2_15.INIT1 = 16'h5555;
    defparam sub_1025_add_2_15.INJECT1_0 = "NO";
    defparam sub_1025_add_2_15.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_291 (.A(pcm_fifo_N_2624), .B(pll_clk_enable_104), 
         .Z(n13458)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_adj_291.init = 16'h2222;
    CCU2D add_2555_3 (.A0(\mic_acc_q[3] [0]), .B0(n20759), .C0(n4628), 
          .D0(GND_net), .A1(\mic_acc_q[3] [1]), .B1(n20759), .C1(n4628), 
          .D1(GND_net), .CIN(n24795), .COUT(n24796), .S0(n5774), .S1(n5773));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam add_2555_3.INIT0 = 16'h7484;
    defparam add_2555_3.INIT1 = 16'h7848;
    defparam add_2555_3.INJECT1_0 = "NO";
    defparam add_2555_3.INJECT1_1 = "NO";
    LUT4 i10119_4_lut (.A(ev_rd_data[81]), .B(ev_in_clear), .C(ev_ch_hi_10), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[81])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10119_4_lut.init = 16'h1222;
    CCU2D add_1592_25 (.A0(phase_frac[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24712), .S0(phase_frac_sum[23]), .S1(phase_frac_sum[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_25.INIT0 = 16'h5aaa;
    defparam add_1592_25.INIT1 = 16'h0000;
    defparam add_1592_25.INJECT1_0 = "NO";
    defparam add_1592_25.INJECT1_1 = "NO";
    CCU2D add_2555_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20759), .B1(n4628), .C1(GND_net), .D1(GND_net), .COUT(n24795));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam add_2555_1.INIT0 = 16'hF000;
    defparam add_2555_1.INIT1 = 16'h7777;
    defparam add_2555_1.INJECT1_0 = "NO";
    defparam add_2555_1.INJECT1_1 = "NO";
    LUT4 i10118_4_lut (.A(ev_rd_data[82]), .B(ev_in_clear), .C(ev_ch_hi_10), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[82])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10118_4_lut.init = 16'h1222;
    LUT4 i1_2_lut_4_lut_adj_292 (.A(frame_settle[3]), .B(frame_settle[1]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(ws2812_enable_N_2601)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(675[22:42])
    defparam i1_2_lut_4_lut_adj_292.init = 16'hfffe;
    LUT4 i13913_2_lut (.A(rmw_slot[0]), .B(build_level_eff[0]), .Z(ev_sum[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13913_2_lut.init = 16'h6666;
    LUT4 mic_cfg_width_6__I_0_1528_i14_3_lut (.A(n12_adj_3242), .B(mic_acc_last_N_2675[6]), 
         .C(mic_cfg_width[6]), .Z(mic_acc_last_N_2674)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(980[34:73])
    defparam mic_cfg_width_6__I_0_1528_i14_3_lut.init = 16'h8e8e;
    LUT4 n27612_bdd_4_lut (.A(n27612), .B(n27495), .C(n27483), .D(status_next_bit_N_2770[3]), 
         .Z(n26906)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27612_bdd_4_lut.init = 16'haad8;
    LUT4 n27504_bdd_4_lut_then_3_lut (.A(mic_ram_rd_data[0]), .B(mic_ram_rd_data[2]), 
         .C(status_bit_index[1]), .Z(n28459)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27504_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 i3_4_lut_adj_293 (.A(n16841), .B(n10659), .C(n28158), .D(spi_bit_count[1]), 
         .Z(n25051)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i3_4_lut_adj_293.init = 16'hfbff;
    LUT4 i2_3_lut_adj_294 (.A(n97), .B(spi_byte_count[8]), .C(spi_rgb_start[7]), 
         .Z(n16841)) /* synthesis lut_function=(A+(B+!(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i2_3_lut_adj_294.init = 16'hefef;
    LUT4 i5_3_lut_adj_295 (.A(mic_cfg_hold_spi[3]), .B(n10_adj_3294), .C(mic_cfg_hold_spi[5]), 
         .Z(n16832)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i5_3_lut_adj_295.init = 16'hfefe;
    LUT4 i10117_4_lut (.A(ev_rd_data[83]), .B(ev_in_clear), .C(ev_ch_hi_10), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[83])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10117_4_lut.init = 16'h1222;
    LUT4 i6631_2_lut_4_lut (.A(frame_settle[3]), .B(frame_settle[1]), .C(frame_settle[2]), 
         .D(frame_settle[0]), .Z(n17477)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(675[22:42])
    defparam i6631_2_lut_4_lut.init = 16'h00fe;
    LUT4 ev_run_hold_s5_83__I_0_i77_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_76), .D(ev_run_hold_s5[76]), .Z(us_tx_next[76])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i77_3_lut_4_lut.init = 16'hbf40;
    LUT4 i13814_2_lut_3_lut (.A(pcm_fifo_N_2624), .B(mic_sample_count[0]), 
         .C(mic_sample_count[1]), .Z(n24602)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(879[37:60])
    defparam i13814_2_lut_3_lut.init = 16'h8080;
    LUT4 i13811_2_lut_3_lut (.A(pcm_fifo_N_2624), .B(mic_sample_count[0]), 
         .C(mic_sample_count[1]), .Z(n24)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(879[37:60])
    defparam i13811_2_lut_3_lut.init = 16'h7878;
    LUT4 i2_2_lut_3_lut_adj_296 (.A(ev_ch_lo_3), .B(ev_ch_hi_10), .C(n13500), 
         .Z(n16013)) /* synthesis lut_function=(!(A (B+!(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(777[25:39])
    defparam i2_2_lut_3_lut_adj_296.init = 16'h7070;
    LUT4 n27504_bdd_4_lut_else_3_lut (.A(mic_ram_rd_data[1]), .B(mic_ram_rd_data[3]), 
         .C(status_bit_index[1]), .Z(n28458)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27504_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 compact_end_I_0_2_lut (.A(compact_end), .B(n16996), .Z(audio_reset_end)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(599[28:62])
    defparam compact_end_I_0_2_lut.init = 16'h2222;
    LUT4 i7_4_lut_adj_297 (.A(spi_rx_shift[3]), .B(spi_rx_shift[0]), .C(spi_rx_shift[4]), 
         .D(spi_rx_shift[1]), .Z(n18_adj_3261)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i7_4_lut_adj_297.init = 16'hfffe;
    LUT4 i2_2_lut_3_lut_adj_298 (.A(ev_ch_lo_3), .B(ev_ch_hi_10), .C(n13500), 
         .Z(n16947)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(777[25:39])
    defparam i2_2_lut_3_lut_adj_298.init = 16'h8080;
    LUT4 i13861_3_lut_4_lut (.A(n16924), .B(n16974), .C(spi_rgb_captured[0]), 
         .D(spi_rgb_captured[1]), .Z(n14_adj_3231)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i13861_3_lut_4_lut.init = 16'h7f80;
    LUT4 i14929_4_lut (.A(spi_byte_count[1]), .B(spi_byte_count[8]), .C(spi_expected_last[1]), 
         .D(spi_expected_last[8]), .Z(n25944)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14929_4_lut.init = 16'h7bde;
    LUT4 spi1_miso_I_28_i49_then_4_lut (.A(status_bit_index[1]), .B(mic_run_state[0]), 
         .C(status_bit_index[0]), .D(mic_run_state[1]), .Z(n28462)) /* synthesis lut_function=(A (B (C+!(D))+!B (C))+!A !(B+(C+!(D)))) */ ;
    defparam spi1_miso_I_28_i49_then_4_lut.init = 16'ha1a8;
    LUT4 spi1_miso_I_28_i49_else_4_lut (.A(status_bit_index[1]), .B(mic_run_state[0]), 
         .C(status_bit_index[0]), .D(mic_run_state[1]), .Z(n28461)) /* synthesis lut_function=(!(A ((C+(D))+!B)+!A (B+(C+!(D))))) */ ;
    defparam spi1_miso_I_28_i49_else_4_lut.init = 16'h0108;
    LUT4 i9910_4_lut (.A(ev_rd_data[0]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[0])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i9910_4_lut.init = 16'h1222;
    LUT4 i1_2_lut_adj_299 (.A(pll_clk_enable_42), .B(mic_run_state_1__N_2410[1]), 
         .Z(pll_clk_enable_259)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_299.init = 16'heeee;
    LUT4 i4_2_lut_adj_300 (.A(spi_version[2]), .B(spi_version[7]), .Z(n12)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(560[34:54])
    defparam i4_2_lut_adj_300.init = 16'heeee;
    LUT4 i1_4_lut_adj_301 (.A(n20743), .B(n14_adj_3258), .C(n10_adj_3279), 
         .D(n21487), .Z(invalid_frame_spi_N_2556)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i1_4_lut_adj_301.init = 16'h0002;
    LUT4 i2_3_lut_adj_302 (.A(mic_run_state[1]), .B(mic_run_state[0]), .C(swap_now_s3), 
         .Z(mic_run_state_1__N_2410[1])) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i2_3_lut_adj_302.init = 16'h4040;
    LUT4 i14917_4_lut (.A(spi_byte_count[0]), .B(spi_byte_count[4]), .C(spi_expected_last[0]), 
         .D(spi_expected_last[4]), .Z(n25932)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14917_4_lut.init = 16'h7bde;
    CCU2D add_2501_9 (.A0(\mic_acc_q[0] [6]), .B0(n79), .C0(n4570), .D0(GND_net), 
          .A1(\mic_acc_q[0] [7]), .B1(n79), .C1(n4570), .D1(GND_net), 
          .CIN(n24790), .S0(n5744), .S1(n5743));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(857[17:43])
    defparam add_2501_9.INIT0 = 16'hd212;
    defparam add_2501_9.INIT1 = 16'hd212;
    defparam add_2501_9.INJECT1_0 = "NO";
    defparam add_2501_9.INJECT1_1 = "NO";
    CCU2D add_2501_7 (.A0(\mic_acc_q[0] [4]), .B0(n79), .C0(n4570), .D0(GND_net), 
          .A1(\mic_acc_q[0] [5]), .B1(n79), .C1(n4570), .D1(GND_net), 
          .CIN(n24789), .COUT(n24790), .S0(n5746), .S1(n5745));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(857[17:43])
    defparam add_2501_7.INIT0 = 16'hd212;
    defparam add_2501_7.INIT1 = 16'hd212;
    defparam add_2501_7.INJECT1_0 = "NO";
    defparam add_2501_7.INJECT1_1 = "NO";
    LUT4 i10199_4_lut (.A(ev_rd_data[1]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[1])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10199_4_lut.init = 16'h1222;
    CCU2D sub_1025_add_2_13 (.A0(mic_gate_wait[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24922), .COUT(n24923), .S0(n6868), .S1(n6867));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[38:59])
    defparam sub_1025_add_2_13.INIT0 = 16'h5555;
    defparam sub_1025_add_2_13.INIT1 = 16'h5555;
    defparam sub_1025_add_2_13.INJECT1_0 = "NO";
    defparam sub_1025_add_2_13.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i78_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_77), .D(ev_run_hold_s5[77]), .Z(us_tx_next[77])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i78_3_lut_4_lut.init = 16'hbf40;
    CCU2D add_2501_5 (.A0(\mic_acc_q[0] [2]), .B0(n79), .C0(n4570), .D0(GND_net), 
          .A1(\mic_acc_q[0] [3]), .B1(n79), .C1(n4570), .D1(GND_net), 
          .CIN(n24788), .COUT(n24789), .S0(n5748), .S1(n5747));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(857[17:43])
    defparam add_2501_5.INIT0 = 16'hd212;
    defparam add_2501_5.INIT1 = 16'hd212;
    defparam add_2501_5.INJECT1_0 = "NO";
    defparam add_2501_5.INJECT1_1 = "NO";
    CCU2D sub_1025_add_2_11 (.A0(mic_gate_wait[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24921), .COUT(n24922), .S0(n6870), .S1(n6869));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[38:59])
    defparam sub_1025_add_2_11.INIT0 = 16'h5555;
    defparam sub_1025_add_2_11.INIT1 = 16'h5555;
    defparam sub_1025_add_2_11.INJECT1_0 = "NO";
    defparam sub_1025_add_2_11.INJECT1_1 = "NO";
    LUT4 i14921_4_lut (.A(spi_byte_count[7]), .B(spi_byte_count[5]), .C(spi_expected_last[7]), 
         .D(spi_expected_last[5]), .Z(n25936)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14921_4_lut.init = 16'h7bde;
    LUT4 i15047_then_3_lut (.A(pcm_read_count_latched_spi[0]), .B(pcm_read_count_latched_spi[1]), 
         .C(status_bit_index[0]), .Z(n28465)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam i15047_then_3_lut.init = 16'hacac;
    CCU2D sub_1025_add_2_9 (.A0(mic_gate_wait[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24920), .COUT(n24921), .S0(n6872), .S1(n6871));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[38:59])
    defparam sub_1025_add_2_9.INIT0 = 16'h5555;
    defparam sub_1025_add_2_9.INIT1 = 16'h5555;
    defparam sub_1025_add_2_9.INJECT1_0 = "NO";
    defparam sub_1025_add_2_9.INJECT1_1 = "NO";
    CCU2D sub_1025_add_2_7 (.A0(mic_gate_wait[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24919), .COUT(n24920), .S0(n6874), .S1(n6873));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[38:59])
    defparam sub_1025_add_2_7.INIT0 = 16'h5555;
    defparam sub_1025_add_2_7.INIT1 = 16'h5555;
    defparam sub_1025_add_2_7.INJECT1_0 = "NO";
    defparam sub_1025_add_2_7.INJECT1_1 = "NO";
    LUT4 i4_4_lut_adj_303 (.A(mic_cfg_hold_spi[4]), .B(mic_cfg_hold_spi[2]), 
         .C(mic_cfg_hold_spi[0]), .D(mic_cfg_hold_spi[1]), .Z(n10_adj_3294)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut_adj_303.init = 16'hfffe;
    CCU2D add_2501_3 (.A0(\mic_acc_q[0] [0]), .B0(n79), .C0(n4570), .D0(GND_net), 
          .A1(\mic_acc_q[0] [1]), .B1(n79), .C1(n4570), .D1(GND_net), 
          .CIN(n24787), .COUT(n24788), .S0(n5750), .S1(n5749));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(857[17:43])
    defparam add_2501_3.INIT0 = 16'hd121;
    defparam add_2501_3.INIT1 = 16'hd212;
    defparam add_2501_3.INJECT1_0 = "NO";
    defparam add_2501_3.INJECT1_1 = "NO";
    CCU2D sub_1025_add_2_5 (.A0(mic_gate_wait[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24918), .COUT(n24919), .S0(n6876), .S1(n6875));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[38:59])
    defparam sub_1025_add_2_5.INIT0 = 16'h5555;
    defparam sub_1025_add_2_5.INIT1 = 16'h5555;
    defparam sub_1025_add_2_5.INJECT1_0 = "NO";
    defparam sub_1025_add_2_5.INJECT1_1 = "NO";
    LUT4 i15047_else_3_lut (.A(pcm_read_count_latched_spi[3]), .B(pcm_read_count_latched_spi[2]), 
         .C(status_bit_index[0]), .Z(n28464)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam i15047_else_3_lut.init = 16'hcaca;
    CCU2D add_2501_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n4570), .B1(n79), .C1(GND_net), .D1(GND_net), .COUT(n24787));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(857[17:43])
    defparam add_2501_1.INIT0 = 16'hF000;
    defparam add_2501_1.INIT1 = 16'hdddd;
    defparam add_2501_1.INJECT1_0 = "NO";
    defparam add_2501_1.INJECT1_1 = "NO";
    LUT4 i2_2_lut_adj_304 (.A(n21403), .B(n4), .Z(n17)) /* synthesis lut_function=((B)+!A) */ ;
    defparam i2_2_lut_adj_304.init = 16'hdddd;
    LUT4 i2_3_lut_adj_305 (.A(mic_window_count[6]), .B(mic_window_count[2]), 
         .C(mic_window_count[5]), .Z(n21403)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i2_3_lut_adj_305.init = 16'h8080;
    LUT4 i3_4_lut_adj_306 (.A(mic_window_count[3]), .B(mic_window_count[0]), 
         .C(mic_window_count[1]), .D(mic_window_count[4]), .Z(n4)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut_adj_306.init = 16'hfffe;
    LUT4 i15366_2_lut (.A(mic_acc_state[0]), .B(mic_acc_state[2]), .Z(mic_ram_we_N_2683)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i15366_2_lut.init = 16'h2222;
    LUT4 i1_2_lut_adj_307 (.A(mic_warm), .B(mic_ultrasonic), .Z(n10833)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_adj_307.init = 16'heeee;
    LUT4 i10198_4_lut (.A(ev_rd_data[2]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[2])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10198_4_lut.init = 16'h1222;
    LUT4 i1_2_lut_adj_308 (.A(mic_ultrasonic_N_1914), .B(n28), .Z(n10751)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam i1_2_lut_adj_308.init = 16'h8888;
    LUT4 i6_4_lut_adj_309 (.A(mic_warm_half[6]), .B(n12_adj_3184), .C(n8_adj_3189), 
         .D(mic_warm_half[5]), .Z(mic_ultrasonic_N_1914)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam i6_4_lut_adj_309.init = 16'h0040;
    LUT4 i5_4_lut_adj_310 (.A(mic_warm_half[2]), .B(mic_warm_half[4]), .C(mic_warm_half[3]), 
         .D(mic_warm_half[0]), .Z(n12_adj_3184)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam i5_4_lut_adj_310.init = 16'h8000;
    LUT4 i10197_4_lut (.A(ev_rd_data[3]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[3])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10197_4_lut.init = 16'h1222;
    LUT4 i10196_4_lut (.A(ev_rd_data[4]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[4])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10196_4_lut.init = 16'h1222;
    LUT4 ev_run_hold_s5_83__I_0_i79_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_78), .D(ev_run_hold_s5[78]), .Z(us_tx_next[78])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i79_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_adj_311 (.A(mic_warm_half[7]), .B(mic_warm_half[1]), .Z(n8_adj_3189)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(841[34:54])
    defparam i1_2_lut_adj_311.init = 16'h8888;
    LUT4 ev_run_hold_s5_83__I_0_i59_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_58), .D(ev_run_hold_s5[58]), .Z(us_tx_next[58])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i59_3_lut_4_lut.init = 16'hbf40;
    LUT4 cs_rise_I_0_2_lut (.A(cs_sync_d), .B(cs_sync), .Z(cs_rise)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(400[20:41])
    defparam cs_rise_I_0_2_lut.init = 16'h4444;
    LUT4 i10195_4_lut (.A(ev_rd_data[5]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[5])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10195_4_lut.init = 16'h1222;
    CCU2D sub_1025_add_2_3 (.A0(mic_gate_wait[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24917), .COUT(n24918), .S0(n6878), .S1(n6877));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[38:59])
    defparam sub_1025_add_2_3.INIT0 = 16'h5555;
    defparam sub_1025_add_2_3.INIT1 = 16'h5555;
    defparam sub_1025_add_2_3.INJECT1_0 = "NO";
    defparam sub_1025_add_2_3.INJECT1_1 = "NO";
    LUT4 i10194_4_lut (.A(ev_rd_data[6]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[6])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10194_4_lut.init = 16'h1222;
    LUT4 i1_2_lut_adj_312 (.A(n13_adj_3180), .B(mic_lo_n_next_6__N_648[0]), 
         .Z(mic_lo_n_next[0])) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[33:52])
    defparam i1_2_lut_adj_312.init = 16'h8888;
    CCU2D sub_1025_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_gate_wait[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24917), .S1(n6879));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(965[38:59])
    defparam sub_1025_add_2_1.INIT0 = 16'hF000;
    defparam sub_1025_add_2_1.INIT1 = 16'h5555;
    defparam sub_1025_add_2_1.INJECT1_0 = "NO";
    defparam sub_1025_add_2_1.INJECT1_1 = "NO";
    LUT4 i6627_4_lut (.A(rgb_values_complete_spi), .B(accepted_update_flags_spi[1]), 
         .C(n10728), .D(n16857), .Z(ws2812_enable_N_2588)) /* synthesis lut_function=(A (B (C+(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(730[17:115])
    defparam i6627_4_lut.init = 16'ha8a0;
    LUT4 i6629_2_lut (.A(mod_settle[0]), .B(n17036), .Z(n17475)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(704[18] 714[12])
    defparam i6629_2_lut.init = 16'h4444;
    LUT4 i14937_3_lut (.A(compact_end), .B(spi_audio_read_cmd), .C(n16996), 
         .Z(spi1_sck_c_enable_157)) /* synthesis lut_function=(A (B+!(C))) */ ;
    defparam i14937_3_lut.init = 16'h8a8a;
    CCU2D add_561_7 (.A0(n24154), .B0(compact_end), .C0(pcm_read_count_latched_spi[4]), 
          .D0(n16996), .A1(GND_net), .B1(audio_reset_end), .C1(pcm_wr_sync2_spi[5]), 
          .D1(pcm_rd_ptr_spi[5]), .CIN(n24782), .S0(pcm_rd_ptr_spi_5__N_1626[4]), 
          .S1(pcm_rd_ptr_spi_5__N_1626[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(607[31:74])
    defparam add_561_7.INIT0 = 16'h5a9a;
    defparam add_561_7.INIT1 = 16'h596a;
    defparam add_561_7.INJECT1_0 = "NO";
    defparam add_561_7.INJECT1_1 = "NO";
    LUT4 i10193_4_lut (.A(ev_rd_data[7]), .B(ev_in_clear), .C(ev_ch_hi_0), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[7])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10193_4_lut.init = 16'h1222;
    CCU2D sub_836_add_2_7 (.A0(pcm_wr_ptr[5]), .B0(pcm_rd_sync2[5]), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24916), .S0(pcm_fifo_N_2617[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(868[25:52])
    defparam sub_836_add_2_7.INIT0 = 16'h5999;
    defparam sub_836_add_2_7.INIT1 = 16'h0000;
    defparam sub_836_add_2_7.INJECT1_0 = "NO";
    defparam sub_836_add_2_7.INJECT1_1 = "NO";
    LUT4 i6_4_lut_adj_313 (.A(expected_valid), .B(n25948), .C(spi_byte_count[2]), 
         .D(spi_expected_last[2]), .Z(n16_adj_3307)) /* synthesis lut_function=(!((B+!(C (D)+!C !(D)))+!A)) */ ;
    defparam i6_4_lut_adj_313.init = 16'h2002;
    LUT4 i1_3_lut_4_lut_adj_314 (.A(mic_phase[2]), .B(n22794), .C(n126), 
         .D(mic_phase[3]), .Z(pll_clk_enable_6)) /* synthesis lut_function=(A (B (C)+!B (C (D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[17:26])
    defparam i1_3_lut_4_lut_adj_314.init = 16'hf0d0;
    CCU2D add_561_5 (.A0(n24158), .B0(compact_end), .C0(pcm_read_count_latched_spi[2]), 
          .D0(n16996), .A1(n24156), .B1(compact_end), .C1(pcm_read_count_latched_spi[3]), 
          .D1(n16996), .CIN(n24781), .COUT(n24782), .S0(pcm_rd_ptr_spi_5__N_1626[2]), 
          .S1(pcm_rd_ptr_spi_5__N_1626[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(607[31:74])
    defparam add_561_5.INIT0 = 16'h5a9a;
    defparam add_561_5.INIT1 = 16'h5a9a;
    defparam add_561_5.INJECT1_0 = "NO";
    defparam add_561_5.INJECT1_1 = "NO";
    FD1P3AX spi_command_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_171), 
            .CK(spi1_sck_c), .Q(spi_command[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_command_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_171), 
            .CK(spi1_sck_c), .Q(spi_command[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_command_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_171), 
            .CK(spi1_sck_c), .Q(spi_command[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_command_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_171), 
            .CK(spi1_sck_c), .Q(spi_command[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_command_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_171), 
            .CK(spi1_sck_c), .Q(spi_command[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_command_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_171), 
            .CK(spi1_sck_c), .Q(spi_command[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam spi_command_i0_i7.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3071__i1 (.D(n49), .CK(spi1_sck_c), .Q(spi_byte_count[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071__i1.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3071__i2 (.D(n48), .CK(spi1_sck_c), .Q(spi_byte_count[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071__i2.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3071__i3 (.D(n47), .CK(spi1_sck_c), .Q(spi_byte_count[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071__i3.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3071__i4 (.D(n46), .CK(spi1_sck_c), .Q(spi_byte_count[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071__i4.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3071__i5 (.D(n45_adj_3232), .CK(spi1_sck_c), 
            .Q(spi_byte_count[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071__i5.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3071__i6 (.D(n44_adj_3233), .CK(spi1_sck_c), 
            .Q(spi_byte_count[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071__i6.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3071__i7 (.D(n43_adj_3234), .CK(spi1_sck_c), 
            .Q(spi_byte_count[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071__i7.GSR = "ENABLED";
    FD1S3AX spi_byte_count_3071__i8 (.D(n42_adj_3235), .CK(spi1_sck_c), 
            .Q(spi_byte_count[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(542[35:56])
    defparam spi_byte_count_3071__i8.GSR = "ENABLED";
    FD1S3AX spi_rgb_captured_3070__i1 (.D(n14_adj_3231), .CK(spi1_sck_c), 
            .Q(spi_rgb_captured[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(537[49:72])
    defparam spi_rgb_captured_3070__i1.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3069__i1 (.D(n39_adj_3222), .CK(spi1_sck_c), 
            .Q(spi_channel_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069__i1.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3069__i2 (.D(n38_adj_3223), .CK(spi1_sck_c), 
            .Q(spi_channel_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069__i2.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3069__i3 (.D(n37_adj_3224), .CK(spi1_sck_c), 
            .Q(spi_channel_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069__i3.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3069__i4 (.D(n36_adj_3225), .CK(spi1_sck_c), 
            .Q(spi_channel_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069__i4.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3069__i5 (.D(n35_adj_3226), .CK(spi1_sck_c), 
            .Q(spi_channel_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069__i5.GSR = "ENABLED";
    FD1S3AX spi_channel_index_3069__i6 (.D(n34_adj_3227), .CK(spi1_sck_c), 
            .Q(spi_channel_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[58:82])
    defparam spi_channel_index_3069__i6.GSR = "ENABLED";
    FD1P3AX pcm_wr_ptr_3080__i1 (.D(n34_adj_3216), .SP(pll_clk_enable_601), 
            .CK(pll_clk), .Q(pcm_wr_ptr[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[39:56])
    defparam pcm_wr_ptr_3080__i1.GSR = "DISABLED";
    LUT4 i10137_4_lut (.A(ev_rd_data[63]), .B(ev_in_clear), .C(ev_ch_hi_7), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[63])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10137_4_lut.init = 16'h1222;
    LUT4 n27474_bdd_4_lut_then_3_lut (.A(n14147), .B(n14148), .C(status_bit_index[0]), 
         .Z(n28474)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27474_bdd_4_lut_then_3_lut.init = 16'hacac;
    CCU2D sub_836_add_2_5 (.A0(pcm_wr_ptr[3]), .B0(pcm_rd_sync2[3]), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_wr_ptr[4]), .B1(pcm_rd_sync2[4]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24915), .COUT(n24916));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(868[25:52])
    defparam sub_836_add_2_5.INIT0 = 16'h5999;
    defparam sub_836_add_2_5.INIT1 = 16'h5999;
    defparam sub_836_add_2_5.INJECT1_0 = "NO";
    defparam sub_836_add_2_5.INJECT1_1 = "NO";
    CCU2D sub_836_add_2_3 (.A0(pcm_wr_ptr[1]), .B0(pcm_rd_sync2[1]), .C0(GND_net), 
          .D0(GND_net), .A1(pcm_wr_ptr[2]), .B1(pcm_rd_sync2[2]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24914), .COUT(n24915));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(868[25:52])
    defparam sub_836_add_2_3.INIT0 = 16'h5999;
    defparam sub_836_add_2_3.INIT1 = 16'h5999;
    defparam sub_836_add_2_3.INJECT1_0 = "NO";
    defparam sub_836_add_2_3.INJECT1_1 = "NO";
    FD1P3AX pcm_wr_ptr_3080__i2 (.D(n33_adj_3217), .SP(pll_clk_enable_601), 
            .CK(pll_clk), .Q(pcm_wr_ptr[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[39:56])
    defparam pcm_wr_ptr_3080__i2.GSR = "DISABLED";
    FD1P3AX pcm_wr_ptr_3080__i3 (.D(n32_adj_3218), .SP(pll_clk_enable_601), 
            .CK(pll_clk), .Q(pcm_wr_ptr[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[39:56])
    defparam pcm_wr_ptr_3080__i3.GSR = "DISABLED";
    FD1P3AX pcm_wr_ptr_3080__i4 (.D(n31_adj_3219), .SP(pll_clk_enable_601), 
            .CK(pll_clk), .Q(pcm_wr_ptr[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[39:56])
    defparam pcm_wr_ptr_3080__i4.GSR = "DISABLED";
    FD1P3AX pcm_wr_ptr_3080__i5 (.D(n30_adj_3220), .SP(pll_clk_enable_601), 
            .CK(pll_clk), .Q(pcm_wr_ptr[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(870[39:56])
    defparam pcm_wr_ptr_3080__i5.GSR = "DISABLED";
    CCU2D sub_836_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(pcm_wr_ptr[0]), .B1(pcm_rd_sync2[0]), .C1(GND_net), .D1(GND_net), 
          .COUT(n24914));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(868[25:52])
    defparam sub_836_add_2_1.INIT0 = 16'h0000;
    defparam sub_836_add_2_1.INIT1 = 16'h5999;
    defparam sub_836_add_2_1.INJECT1_0 = "NO";
    defparam sub_836_add_2_1.INJECT1_1 = "NO";
    CCU2D add_561_3 (.A0(n24162), .B0(compact_end), .C0(pcm_read_count_latched_spi[0]), 
          .D0(n16996), .A1(n24160), .B1(compact_end), .C1(pcm_read_count_latched_spi[1]), 
          .D1(n16996), .CIN(n24780), .COUT(n24781), .S0(pcm_rd_ptr_spi_5__N_1626[0]), 
          .S1(pcm_rd_ptr_spi_5__N_1626[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(607[31:74])
    defparam add_561_3.INIT0 = 16'h5a9a;
    defparam add_561_3.INIT1 = 16'h5a9a;
    defparam add_561_3.INJECT1_0 = "NO";
    defparam add_561_3.INJECT1_1 = "NO";
    LUT4 i10192_4_lut (.A(ev_rd_data[8]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[8])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10192_4_lut.init = 16'h1222;
    LUT4 ev_run_hold_s5_83__I_0_i80_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_79), .D(ev_run_hold_s5[79]), .Z(us_tx_next[79])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i80_3_lut_4_lut.init = 16'hbf40;
    LUT4 n27474_bdd_4_lut_else_3_lut (.A(n14131), .B(n14132), .C(status_bit_index[0]), 
         .Z(n28473)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27474_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_adj_315 (.A(n14890), .B(spi1_sck_c_enable_53), .C(n31), 
         .Z(spi1_sck_c_enable_39)) /* synthesis lut_function=(A (B)+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i1_3_lut_adj_315.init = 16'h8c8c;
    LUT4 i15424_3_lut (.A(n10986), .B(ws2812_enable), .C(n13723), .Z(pll_clk_enable_143)) /* synthesis lut_function=(!(A (B)+!A !((C)+!B))) */ ;
    defparam i15424_3_lut.init = 16'h7373;
    LUT4 i10191_4_lut (.A(ev_rd_data[9]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[9])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10191_4_lut.init = 16'h1222;
    LUT4 i10190_4_lut (.A(ev_rd_data[10]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[10])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10190_4_lut.init = 16'h1222;
    CCU2D add_1592_3 (.A0(phase_frac[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24701), .COUT(n24702), .S0(phase_frac_sum[1]), 
          .S1(phase_frac_sum[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_3.INIT0 = 16'h5555;
    defparam add_1592_3.INIT1 = 16'h5aaa;
    defparam add_1592_3.INJECT1_0 = "NO";
    defparam add_1592_3.INJECT1_1 = "NO";
    LUT4 i15228_2_lut (.A(n31_adj_3309), .B(spi_byte_count[6]), .Z(n26098)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15228_2_lut.init = 16'h8888;
    LUT4 ev_run_hold_s5_83__I_0_i81_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_80), .D(ev_run_hold_s5[80]), .Z(us_tx_next[80])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i81_3_lut_4_lut.init = 16'hbf40;
    CCU2D feed_rd_3077_add_4_11 (.A0(n24167), .B0(frame_req_N_2710), .C0(feed_rd[8]), 
          .D0(n21319), .A1(n24165), .B1(frame_req_N_2710), .C1(feed_rd[9]), 
          .D1(n21319), .CIN(n24912), .S0(n70_adj_3308), .S1(n69));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077_add_4_11.INIT0 = 16'h5a9a;
    defparam feed_rd_3077_add_4_11.INIT1 = 16'h5a9a;
    defparam feed_rd_3077_add_4_11.INJECT1_0 = "NO";
    defparam feed_rd_3077_add_4_11.INJECT1_1 = "NO";
    CCU2D feed_rd_3077_add_4_9 (.A0(n24171), .B0(frame_req_N_2710), .C0(feed_rd[6]), 
          .D0(n21319), .A1(n24169), .B1(frame_req_N_2710), .C1(feed_rd[7]), 
          .D1(n21319), .CIN(n24911), .COUT(n24912), .S0(n72), .S1(n71));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077_add_4_9.INIT0 = 16'h5a9a;
    defparam feed_rd_3077_add_4_9.INIT1 = 16'h5a9a;
    defparam feed_rd_3077_add_4_9.INJECT1_0 = "NO";
    defparam feed_rd_3077_add_4_9.INJECT1_1 = "NO";
    FD1P3IX mic_cfg_count_i5 (.D(mic_cfg_hold_spi[5]), .SP(pll_clk_enable_602), 
            .CD(n17763), .CK(pll_clk), .Q(mic_cfg_count[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_cfg_count_i5.GSR = "DISABLED";
    L6MUX21 i16915 (.D0(n28475), .D1(n28684), .SD(status_bit_index[2]), 
            .Z(n28685));
    LUT4 i10189_4_lut (.A(ev_rd_data[11]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[11])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10189_4_lut.init = 16'h1222;
    LUT4 i10188_4_lut (.A(ev_rd_data[12]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[12])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10188_4_lut.init = 16'h1222;
    CCU2D feed_rd_3077_add_4_7 (.A0(n24175), .B0(frame_req_N_2710), .C0(feed_rd[4]), 
          .D0(n21319), .A1(n24173), .B1(frame_req_N_2710), .C1(feed_rd[5]), 
          .D1(n21319), .CIN(n24910), .COUT(n24911), .S0(n74), .S1(n73_adj_3288));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077_add_4_7.INIT0 = 16'h5a9a;
    defparam feed_rd_3077_add_4_7.INIT1 = 16'h5a9a;
    defparam feed_rd_3077_add_4_7.INJECT1_0 = "NO";
    defparam feed_rd_3077_add_4_7.INJECT1_1 = "NO";
    CCU2D add_561_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(compact_end), .B1(n16996), .C1(GND_net), .D1(GND_net), 
          .COUT(n24780));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(607[31:74])
    defparam add_561_1.INIT0 = 16'hF000;
    defparam add_561_1.INIT1 = 16'hffff;
    defparam add_561_1.INJECT1_0 = "NO";
    defparam add_561_1.INJECT1_1 = "NO";
    LUT4 i13766_2_lut (.A(reset_count[0]), .B(n13455), .Z(n24552)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam i13766_2_lut.init = 16'h2222;
    CCU2D feed_rd_3077_add_4_5 (.A0(n24179), .B0(frame_req_N_2710), .C0(feed_rd[2]), 
          .D0(n21319), .A1(n24177), .B1(frame_req_N_2710), .C1(feed_rd[3]), 
          .D1(n21319), .CIN(n24909), .COUT(n24910), .S0(n76_adj_3312), 
          .S1(n75_adj_3313));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077_add_4_5.INIT0 = 16'h5a9a;
    defparam feed_rd_3077_add_4_5.INIT1 = 16'h5a9a;
    defparam feed_rd_3077_add_4_5.INJECT1_0 = "NO";
    defparam feed_rd_3077_add_4_5.INJECT1_1 = "NO";
    LUT4 i10187_4_lut (.A(ev_rd_data[13]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[13])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10187_4_lut.init = 16'h1222;
    LUT4 i10186_4_lut (.A(ev_rd_data[14]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[14])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10186_4_lut.init = 16'h1222;
    LUT4 i820_2_lut (.A(mic_data_0_c), .B(mic_lo_i_r), .Z(n4541)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam i820_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_316 (.A(n20918), .B(mic_acc_state[0]), .C(n13496), 
         .D(n3), .Z(n11_adj_3228)) /* synthesis lut_function=(A (C)+!A (B (C+!(D))+!B (C))) */ ;
    defparam i1_3_lut_4_lut_adj_316.init = 16'hf0f4;
    LUT4 i10546_4_lut (.A(spi_byte_count[6]), .B(spi_byte_count[5]), .C(spi_byte_count[2]), 
         .D(n20788), .Z(n78_adj_3190)) /* synthesis lut_function=(!(A+(B (C+(D))))) */ ;
    defparam i10546_4_lut.init = 16'h1115;
    LUT4 i14933_4_lut (.A(spi_byte_count[3]), .B(spi_byte_count[6]), .C(spi_expected_last[3]), 
         .D(spi_expected_last[6]), .Z(n25948)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14933_4_lut.init = 16'h7bde;
    LUT4 i10185_4_lut (.A(ev_rd_data[15]), .B(ev_in_clear), .C(ev_ch_hi_1), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[15])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10185_4_lut.init = 16'h1222;
    LUT4 i139_3_lut (.A(mic_lo_n_next_6__N_648[2]), .B(mic_lo_n_next_6__N_648[0]), 
         .C(mic_lo_n_next_6__N_648[1]), .Z(n136)) /* synthesis lut_function=(A+(B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[17:30])
    defparam i139_3_lut.init = 16'heaea;
    FD1P3IX mic_acc_busy_1348 (.D(mic_acc_busy_N_2293), .SP(pll_clk_enable_603), 
            .CD(n28136), .CK(pll_clk), .Q(mic_acc_busy)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_acc_busy_1348.GSR = "DISABLED";
    LUT4 i10184_4_lut (.A(ev_rd_data[16]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[16])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10184_4_lut.init = 16'h1222;
    LUT4 i10183_4_lut (.A(ev_rd_data[17]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[17])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10183_4_lut.init = 16'h1222;
    LUT4 i28_1_lut (.A(fpga_cs_n_c), .Z(fpga_cs_n_N_2472)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i28_1_lut.init = 16'h5555;
    LUT4 i10182_4_lut (.A(ev_rd_data[18]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[18])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10182_4_lut.init = 16'h1222;
    LUT4 i10181_4_lut (.A(ev_rd_data[19]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[19])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10181_4_lut.init = 16'h1222;
    LUT4 i10180_4_lut (.A(ev_rd_data[20]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[20])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10180_4_lut.init = 16'h1222;
    LUT4 i10179_4_lut (.A(ev_rd_data[21]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[21])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10179_4_lut.init = 16'h1222;
    LUT4 i10178_4_lut (.A(ev_rd_data[22]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_6), .Z(ev_wr_data[22])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10178_4_lut.init = 16'h1222;
    CCU2D feed_rd_3077_add_4_3 (.A0(n24183), .B0(frame_req_N_2710), .C0(feed_rd[0]), 
          .D0(n21319), .A1(n24181), .B1(frame_req_N_2710), .C1(feed_rd[1]), 
          .D1(n21319), .CIN(n24908), .COUT(n24909), .S0(n78_adj_3310), 
          .S1(n77_adj_3311));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077_add_4_3.INIT0 = 16'h5a9a;
    defparam feed_rd_3077_add_4_3.INIT1 = 16'h5a9a;
    defparam feed_rd_3077_add_4_3.INJECT1_0 = "NO";
    defparam feed_rd_3077_add_4_3.INJECT1_1 = "NO";
    CCU2D feed_rd_3077_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(frame_req_N_2710), .B1(n21319), .C1(GND_net), 
          .D1(GND_net), .COUT(n24908));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(695[24:38])
    defparam feed_rd_3077_add_4_1.INIT0 = 16'hF000;
    defparam feed_rd_3077_add_4_1.INIT1 = 16'hffff;
    defparam feed_rd_3077_add_4_1.INJECT1_0 = "NO";
    defparam feed_rd_3077_add_4_1.INJECT1_1 = "NO";
    LUT4 i15183_3_lut_3_lut_3_lut (.A(mic_acc_state[2]), .B(mic_acc_state[1]), 
         .C(mic_acc_state[0]), .Z(n26100)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i15183_3_lut_3_lut_3_lut.init = 16'h0404;
    LUT4 i1_3_lut_4_lut_adj_317 (.A(mic_phase[2]), .B(n22794), .C(n107), 
         .D(pll_clk_enable_266), .Z(n22781)) /* synthesis lut_function=(A (B (D)+!B (C (D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[17:26])
    defparam i1_3_lut_4_lut_adj_317.init = 16'hfd00;
    LUT4 i10177_4_lut (.A(ev_rd_data[23]), .B(ev_in_clear), .C(ev_ch_hi_2), 
         .D(ev_ch_lo_7), .Z(ev_wr_data[23])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10177_4_lut.init = 16'h1222;
    LUT4 i15467_2_lut_3_lut (.A(mic_acc_state[0]), .B(mic_acc_state[1]), 
         .C(mic_acc_state[2]), .Z(mic_acc_busy_N_2292)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[13:17])
    defparam i15467_2_lut_3_lut.init = 16'h1010;
    LUT4 i10176_4_lut (.A(ev_rd_data[24]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[24])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10176_4_lut.init = 16'h1222;
    LUT4 i9922_2_lut (.A(spi_bit_count[2]), .B(spi_bit_count[1]), .Z(n20745)) /* synthesis lut_function=(A (B)) */ ;
    defparam i9922_2_lut.init = 16'h8888;
    LUT4 i10175_4_lut (.A(ev_rd_data[25]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[25])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10175_4_lut.init = 16'h1222;
    LUT4 spi_command_7__I_0_1465_i10_2_lut (.A(spi_command[2]), .B(spi_command[3]), 
         .Z(n10_adj_3279)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(580[34:54])
    defparam spi_command_7__I_0_1465_i10_2_lut.init = 16'heeee;
    CCU2D global_phase_s2_3074_add_4_8 (.A0(global_phase_s2[6]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[7]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24775), .S0(n39), .S1(n38_adj_3174));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074_add_4_8.INIT0 = 16'hfaaa;
    defparam global_phase_s2_3074_add_4_8.INIT1 = 16'hfaaa;
    defparam global_phase_s2_3074_add_4_8.INJECT1_0 = "NO";
    defparam global_phase_s2_3074_add_4_8.INJECT1_1 = "NO";
    LUT4 i1_2_lut_4_lut_adj_318 (.A(ws2812_settle[3]), .B(ws2812_settle[1]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(ws2812_enable_N_2606)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(721[22:43])
    defparam i1_2_lut_4_lut_adj_318.init = 16'hfffe;
    PFUMX i16909 (.BLUT(n28428), .ALUT(n28680), .C0(pcm_spi_sample_15__N_1381[4]), 
          .Z(n28681));
    LUT4 i10174_4_lut (.A(ev_rd_data[26]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_2), .Z(ev_wr_data[26])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10174_4_lut.init = 16'h1222;
    LUT4 i10136_4_lut (.A(ev_rd_data[64]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_0), .Z(ev_wr_data[64])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10136_4_lut.init = 16'h1222;
    CCU2D feed_wr_9__I_0_1385_add_2_11 (.A0(feed_wr[9]), .B0(feed_rd[9]), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24904), .S0(feed_fill[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(371[29:46])
    defparam feed_wr_9__I_0_1385_add_2_11.INIT0 = 16'h5999;
    defparam feed_wr_9__I_0_1385_add_2_11.INIT1 = 16'h0000;
    defparam feed_wr_9__I_0_1385_add_2_11.INJECT1_0 = "NO";
    defparam feed_wr_9__I_0_1385_add_2_11.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i71_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_70), .D(ev_run_hold_s5[70]), .Z(us_tx_next[70])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i71_3_lut_4_lut.init = 16'hbf40;
    CCU2D feed_wr_9__I_0_1385_add_2_9 (.A0(feed_wr[7]), .B0(feed_rd[7]), 
          .C0(GND_net), .D0(GND_net), .A1(feed_wr[8]), .B1(feed_rd[8]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24903), .COUT(n24904), .S0(feed_fill[7]), 
          .S1(feed_fill[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(371[29:46])
    defparam feed_wr_9__I_0_1385_add_2_9.INIT0 = 16'h5999;
    defparam feed_wr_9__I_0_1385_add_2_9.INIT1 = 16'h5999;
    defparam feed_wr_9__I_0_1385_add_2_9.INJECT1_0 = "NO";
    defparam feed_wr_9__I_0_1385_add_2_9.INJECT1_1 = "NO";
    LUT4 i4052_3_lut (.A(invalid_frame_spi_N_2556), .B(n21503), .C(n20751), 
         .Z(n14890)) /* synthesis lut_function=(A ((C)+!B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(557[30] 582[24])
    defparam i4052_3_lut.init = 16'hb3b3;
    CCU2D feed_wr_9__I_0_1385_add_2_7 (.A0(feed_wr[5]), .B0(feed_rd[5]), 
          .C0(GND_net), .D0(GND_net), .A1(feed_wr[6]), .B1(feed_rd[6]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24902), .COUT(n24903), .S0(feed_fill[5]), 
          .S1(feed_fill[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(371[29:46])
    defparam feed_wr_9__I_0_1385_add_2_7.INIT0 = 16'h5999;
    defparam feed_wr_9__I_0_1385_add_2_7.INIT1 = 16'h5999;
    defparam feed_wr_9__I_0_1385_add_2_7.INJECT1_0 = "NO";
    defparam feed_wr_9__I_0_1385_add_2_7.INJECT1_1 = "NO";
    LUT4 ev_run_hold_s5_83__I_0_i66_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_65), .D(ev_run_hold_s5[65]), .Z(us_tx_next[65])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i66_3_lut_4_lut.init = 16'hbf40;
    CCU2D global_phase_s2_3074_add_4_6 (.A0(global_phase_s2[4]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[5]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24774), .COUT(n24775), .S0(n41), 
          .S1(n40));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074_add_4_6.INIT0 = 16'hfaaa;
    defparam global_phase_s2_3074_add_4_6.INIT1 = 16'hfaaa;
    defparam global_phase_s2_3074_add_4_6.INJECT1_0 = "NO";
    defparam global_phase_s2_3074_add_4_6.INJECT1_1 = "NO";
    CCU2D global_phase_s2_3074_add_4_4 (.A0(global_phase_s2[2]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[3]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24773), .COUT(n24774), .S0(n43), 
          .S1(n42));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074_add_4_4.INIT0 = 16'hfaaa;
    defparam global_phase_s2_3074_add_4_4.INIT1 = 16'hfaaa;
    defparam global_phase_s2_3074_add_4_4.INJECT1_0 = "NO";
    defparam global_phase_s2_3074_add_4_4.INJECT1_1 = "NO";
    LUT4 i10173_4_lut (.A(ev_rd_data[27]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_3), .Z(ev_wr_data[27])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10173_4_lut.init = 16'h1222;
    CCU2D feed_wr_9__I_0_1385_add_2_5 (.A0(feed_wr[3]), .B0(feed_rd[3]), 
          .C0(GND_net), .D0(GND_net), .A1(feed_wr[4]), .B1(feed_rd[4]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24901), .COUT(n24902), .S0(feed_fill[3]), 
          .S1(feed_fill[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(371[29:46])
    defparam feed_wr_9__I_0_1385_add_2_5.INIT0 = 16'h5999;
    defparam feed_wr_9__I_0_1385_add_2_5.INIT1 = 16'h5999;
    defparam feed_wr_9__I_0_1385_add_2_5.INJECT1_0 = "NO";
    defparam feed_wr_9__I_0_1385_add_2_5.INJECT1_1 = "NO";
    LUT4 i10172_4_lut (.A(ev_rd_data[28]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_4), .Z(ev_wr_data[28])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10172_4_lut.init = 16'h1222;
    LUT4 i10171_4_lut (.A(ev_rd_data[29]), .B(ev_in_clear), .C(ev_ch_hi_3), 
         .D(ev_ch_lo_5), .Z(ev_wr_data[29])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10171_4_lut.init = 16'h1222;
    CCU2D feed_wr_9__I_0_1385_add_2_3 (.A0(feed_wr[1]), .B0(feed_rd[1]), 
          .C0(GND_net), .D0(GND_net), .A1(feed_wr[2]), .B1(feed_rd[2]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24900), .COUT(n24901), .S0(feed_fill[1]), 
          .S1(feed_fill[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(371[29:46])
    defparam feed_wr_9__I_0_1385_add_2_3.INIT0 = 16'h5999;
    defparam feed_wr_9__I_0_1385_add_2_3.INIT1 = 16'h5999;
    defparam feed_wr_9__I_0_1385_add_2_3.INJECT1_0 = "NO";
    defparam feed_wr_9__I_0_1385_add_2_3.INJECT1_1 = "NO";
    LUT4 i10436_3_lut (.A(wrap_s2), .B(n16947), .C(swap_pending), .Z(swap_pending_N_2717)) /* synthesis lut_function=(A (B)+!A (B+(C))) */ ;
    defparam i10436_3_lut.init = 16'hdcdc;
    CCU2D feed_wr_9__I_0_1385_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(feed_wr[0]), .B1(feed_rd[0]), .C1(GND_net), 
          .D1(GND_net), .COUT(n24900), .S1(feed_fill[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(371[29:46])
    defparam feed_wr_9__I_0_1385_add_2_1.INIT0 = 16'h0000;
    defparam feed_wr_9__I_0_1385_add_2_1.INIT1 = 16'h5999;
    defparam feed_wr_9__I_0_1385_add_2_1.INJECT1_0 = "NO";
    defparam feed_wr_9__I_0_1385_add_2_1.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_319 (.A(mic_warm), .B(n17740), .Z(pll_clk_enable_169)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_319.init = 16'heeee;
    CCU2D global_phase_s2_3074_add_4_2 (.A0(phase_step_s1), .B0(global_phase_s2[0]), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[1]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .COUT(n24773), .S1(n44));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(617[32:54])
    defparam global_phase_s2_3074_add_4_2.INIT0 = 16'h7000;
    defparam global_phase_s2_3074_add_4_2.INIT1 = 16'hfaaa;
    defparam global_phase_s2_3074_add_4_2.INJECT1_0 = "NO";
    defparam global_phase_s2_3074_add_4_2.INJECT1_1 = "NO";
    LUT4 i3382_2_lut_3_lut (.A(link_idle[0]), .B(link_idle[1]), .C(link_idle[2]), 
         .Z(n3218)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(394[25:35])
    defparam i3382_2_lut_3_lut.init = 16'h7878;
    LUT4 i15182_3_lut_3_lut_4_lut (.A(n20918), .B(mic_acc_state[0]), .C(mic_acc_state[1]), 
         .D(mic_acc_state[2]), .Z(n26103)) /* synthesis lut_function=(!(A (B (C+(D))+!B ((D)+!C))+!A ((D)+!C))) */ ;
    defparam i15182_3_lut_3_lut_4_lut.init = 16'h0078;
    LUT4 i1_2_lut_3_lut_4_lut_adj_320 (.A(wrap_s2), .B(swap_pending), .C(ws2812_settle[1]), 
         .D(n28569), .Z(pll_clk_enable_282)) /* synthesis lut_function=(A (B+!(C+!(D)))+!A !(C+!(D))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_320.init = 16'h8f88;
    LUT4 i1_3_lut_adj_321 (.A(mic_ultrasonic_N_1914), .B(n17740), .C(mic_warm), 
         .Z(n20483)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_321.init = 16'hecec;
    LUT4 i3142_2_lut (.A(feed_pop), .B(n13685), .Z(pll_clk_enable_150)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i3142_2_lut.init = 16'heeee;
    LUT4 i2_3_lut_4_lut_adj_322 (.A(pcm_rd_ptr_spi[0]), .B(pcm_payload_bit[4]), 
         .C(pcm_rd_ptr_spi[1]), .D(pcm_payload_bit[5]), .Z(pcm_spi_sample_15__N_1381[1])) /* synthesis lut_function=(A (B (C (D)+!C !(D))+!B !(C (D)+!C !(D)))+!A !(C (D)+!C !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(440[18:55])
    defparam i2_3_lut_4_lut_adj_322.init = 16'h8778;
    LUT4 i2_3_lut_adj_323 (.A(mic_cfg_hold_spi[6]), .B(pll_clk_enable_602), 
         .C(n16832), .Z(n25237)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i2_3_lut_adj_323.init = 16'h0404;
    LUT4 i3931_1_lut (.A(spi_bit_count[0]), .Z(n3_adj_3173)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(487[21:44])
    defparam i3931_1_lut.init = 16'h5555;
    LUT4 i2_3_lut_adj_324 (.A(n31), .B(spi1_sck_c_enable_53), .C(n14890), 
         .Z(spi1_sck_c_enable_161)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i2_3_lut_adj_324.init = 16'h8080;
    LUT4 i1_2_lut_adj_325 (.A(mic_warm), .B(pcm_fifo_N_2624), .Z(n20759)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_adj_325.init = 16'h4444;
    LUT4 i824_2_lut (.A(mic_data_0_c), .B(mic_lo_q_r), .Z(n4570)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam i824_2_lut.init = 16'h6666;
    LUT4 i2_3_lut_adj_326 (.A(mic_cfg_settle[2]), .B(mic_cfg_settle[3]), 
         .C(mic_cfg_settle[1]), .Z(n17_adj_3196)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_3_lut_adj_326.init = 16'hfefe;
    LUT4 i5_2_lut_adj_327 (.A(mic_cfg_toggle_seen), .B(mic_cfg_toggle_sync), 
         .Z(mic_cfg_settle_3__N_2146)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[58:77])
    defparam i5_2_lut_adj_327.init = 16'h6666;
    LUT4 spi_command_1__bdd_2_lut_16978 (.A(spi_command[1]), .B(spi_command[3]), 
         .Z(n28027)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam spi_command_1__bdd_2_lut_16978.init = 16'h2222;
    LUT4 staging_q_7__I_0_1380_i8_4_lut (.A(staging_q[7]), .B(build_level[7]), 
         .C(build_mod), .D(n15), .Z(event_level[7])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(390[31] 392[40])
    defparam staging_q_7__I_0_1380_i8_4_lut.init = 16'hca0a;
    LUT4 spi_version_5__bdd_4_lut_16532 (.A(spi_version[6]), .B(spi_version[3]), 
         .C(spi_version[4]), .D(n28029), .Z(n28030)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam spi_version_5__bdd_4_lut_16532.init = 16'h0100;
    LUT4 ev_run_hold_s5_83__I_0_i60_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_59), .D(ev_run_hold_s5[59]), .Z(us_tx_next[59])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i60_3_lut_4_lut.init = 16'hbf40;
    LUT4 i1_2_lut_adj_328 (.A(mic_cfg_hold_spi[0]), .B(mic_cfg_hold_spi[6]), 
         .Z(mic_cfg_count_w_6__N_856[0])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[35:100])
    defparam i1_2_lut_adj_328.init = 16'h2222;
    CCU2D sub_1617_add_2_7 (.A0(status_bit_index[8]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24899), .S0(pcm_payload_bit[8]));
    defparam sub_1617_add_2_7.INIT0 = 16'h5555;
    defparam sub_1617_add_2_7.INIT1 = 16'h0000;
    defparam sub_1617_add_2_7.INJECT1_0 = "NO";
    defparam sub_1617_add_2_7.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_329 (.A(n10751), .B(n10833), .C(mic_warm), .D(mic_run_state_1__N_2410[1]), 
         .Z(pll_clk_enable_11)) /* synthesis lut_function=(A (B+(D))+!A !(B (C)+!B (C+!(D)))) */ ;
    defparam i1_4_lut_adj_329.init = 16'haf8c;
    CCU2D feed_wr_spi_3073_add_4_11 (.A0(feed_wr_spi[9]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24771), .S0(n46_adj_3269));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073_add_4_11.INIT0 = 16'hfaaa;
    defparam feed_wr_spi_3073_add_4_11.INIT1 = 16'h0000;
    defparam feed_wr_spi_3073_add_4_11.INJECT1_0 = "NO";
    defparam feed_wr_spi_3073_add_4_11.INJECT1_1 = "NO";
    LUT4 i3453_3_lut_4_lut (.A(pcm_rd_ptr_spi[0]), .B(pcm_payload_bit[4]), 
         .C(pcm_payload_bit[5]), .D(pcm_rd_ptr_spi[1]), .Z(n4_adj_3306)) /* synthesis lut_function=(A (B (C+(D))+!B (C (D)))+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(440[18:55])
    defparam i3453_3_lut_4_lut.init = 16'hf880;
    LUT4 staging_q_7__I_0_1380_i7_4_lut (.A(staging_q[6]), .B(build_level[6]), 
         .C(build_mod), .D(n15), .Z(event_level[6])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(390[31] 392[40])
    defparam staging_q_7__I_0_1380_i7_4_lut.init = 16'hca0a;
    LUT4 staging_q_7__I_0_1380_i6_4_lut (.A(staging_q[5]), .B(build_level[5]), 
         .C(build_mod), .D(n15), .Z(event_level[5])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(390[31] 392[40])
    defparam staging_q_7__I_0_1380_i6_4_lut.init = 16'hca0a;
    CCU2D feed_wr_spi_3073_add_4_9 (.A0(feed_wr_spi[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(feed_wr_spi[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24770), .COUT(n24771), .S0(n48_adj_3267), 
          .S1(n47_adj_3268));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073_add_4_9.INIT0 = 16'hfaaa;
    defparam feed_wr_spi_3073_add_4_9.INIT1 = 16'hfaaa;
    defparam feed_wr_spi_3073_add_4_9.INJECT1_0 = "NO";
    defparam feed_wr_spi_3073_add_4_9.INJECT1_1 = "NO";
    LUT4 staging_q_7__I_0_1380_i5_4_lut (.A(staging_q[4]), .B(build_level[4]), 
         .C(build_mod), .D(n15), .Z(event_level[4])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(390[31] 392[40])
    defparam staging_q_7__I_0_1380_i5_4_lut.init = 16'hca0a;
    L6MUX21 i16911 (.D0(n28681), .D1(n28397), .SD(status_bit_index[2]), 
            .Z(n28682));
    LUT4 shift_right_242_i11_4_lut (.A(n20896), .B(n28466), .C(status_bit_index[2]), 
         .D(pcm_read_count_latched_spi[4]), .Z(n11_adj_3295)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(443[11:70])
    defparam shift_right_242_i11_4_lut.init = 16'hcac0;
    LUT4 i15472_4_lut (.A(n13500), .B(staging_rd_page_N_2729), .C(n13249), 
         .D(n10092), .Z(pll_clk_enable_297)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;
    defparam i15472_4_lut.init = 16'h0001;
    spi_mic_stream mic_stream_i (.sck_N_3140(sck_N_3140), .spi_mic_cs_n_c(spi_mic_cs_n_c), 
            .mic_latest({mic_latest}), .spi_mic_miso_c(spi_mic_miso_c), 
            .n29158(n29158), .GND_net(GND_net)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1043[20] 1046[6])
    LUT4 i10135_4_lut (.A(ev_rd_data[65]), .B(ev_in_clear), .C(ev_ch_hi_8), 
         .D(ev_ch_lo_1), .Z(ev_wr_data[65])) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(337[33:79])
    defparam i10135_4_lut.init = 16'h1222;
    LUT4 staging_rd_page_N_2729_I_0_2_lut (.A(staging_rd_page_N_2729), .B(phase_step_s3), 
         .Z(ev_rd_grant)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[33:70])
    defparam staging_rd_page_N_2729_I_0_2_lut.init = 16'h2222;
    LUT4 i15253_4_lut (.A(n26876), .B(n8_adj_3305), .C(pcm_read_count_latched_spi[4]), 
         .D(pcm_payload_bit[8]), .Z(n26091)) /* synthesis lut_function=(A (B (C+!(D))+!B !((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(441[27] 444[114])
    defparam i15253_4_lut.init = 16'h80a8;
    CCU2D sub_1617_add_2_5 (.A0(status_bit_index[6]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[7]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24898), .COUT(n24899), .S0(pcm_payload_bit[6]), 
          .S1(pcm_payload_bit[7]));
    defparam sub_1617_add_2_5.INIT0 = 16'h5555;
    defparam sub_1617_add_2_5.INIT1 = 16'h5aaa;
    defparam sub_1617_add_2_5.INJECT1_0 = "NO";
    defparam sub_1617_add_2_5.INJECT1_1 = "NO";
    CCU2D feed_wr_spi_3073_add_4_7 (.A0(feed_wr_spi[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(feed_wr_spi[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24769), .COUT(n24770), .S0(n50_adj_3265), 
          .S1(n49_adj_3266));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073_add_4_7.INIT0 = 16'hfaaa;
    defparam feed_wr_spi_3073_add_4_7.INIT1 = 16'hfaaa;
    defparam feed_wr_spi_3073_add_4_7.INJECT1_0 = "NO";
    defparam feed_wr_spi_3073_add_4_7.INJECT1_1 = "NO";
    LUT4 mic_acc_state_2__I_0_i5_2_lut_3_lut (.A(mic_acc_state[1]), .B(mic_acc_state[2]), 
         .C(mic_acc_state[0]), .Z(n5)) /* synthesis lut_function=((B+(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1000[13:17])
    defparam mic_acc_state_2__I_0_i5_2_lut_3_lut.init = 16'hfdfd;
    CCU2D sub_1617_add_2_3 (.A0(status_bit_index[4]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[5]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24897), .COUT(n24898), .S0(pcm_payload_bit[4]), 
          .S1(pcm_payload_bit[5]));
    defparam sub_1617_add_2_3.INIT0 = 16'h5555;
    defparam sub_1617_add_2_3.INIT1 = 16'h5555;
    defparam sub_1617_add_2_3.INJECT1_0 = "NO";
    defparam sub_1617_add_2_3.INJECT1_1 = "NO";
    CCU2D sub_1617_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[3]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24897), .S1(pcm_payload_bit[3]));
    defparam sub_1617_add_2_1.INIT0 = 16'hF000;
    defparam sub_1617_add_2_1.INIT1 = 16'h5555;
    defparam sub_1617_add_2_1.INJECT1_0 = "NO";
    defparam sub_1617_add_2_1.INJECT1_1 = "NO";
    LUT4 mic_acc_state_2__I_0_1400_i5_2_lut_2_lut_3_lut (.A(mic_acc_state[1]), 
         .B(mic_acc_state[2]), .C(mic_acc_state[0]), .Z(n5_adj_3259)) /* synthesis lut_function=((B+!(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1000[13:17])
    defparam mic_acc_state_2__I_0_1400_i5_2_lut_2_lut_3_lut.init = 16'hdfdf;
    PFUMX i16753 (.BLUT(n28404), .ALUT(n28405), .C0(status_bit_index[0]), 
          .Z(n28406));
    LUT4 ev_run_hold_s5_83__I_0_i82_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_81), .D(ev_run_hold_s5[81]), .Z(us_tx_next[81])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i82_3_lut_4_lut.init = 16'hbf40;
    LUT4 i3031_3_lut (.A(ev_wrap), .B(ev_fall_N_2693), .C(ev_fall), .Z(n13500)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(734[9] 787[16])
    defparam i3031_3_lut.init = 16'h4040;
    LUT4 i9913_4_lut (.A(status_miso_q), .B(status_bit_index[7]), .C(n12_adj_3206), 
         .D(n8_adj_3229), .Z(status_miso_bit)) /* synthesis lut_function=(A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(429[37] 430[79])
    defparam i9913_4_lut.init = 16'haaa8;
    LUT4 i4_4_lut_adj_330 (.A(n7), .B(spi_extension_length[4]), .C(spi_extension_length[2]), 
         .D(spi_extension_length[3]), .Z(n21455)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut_adj_330.init = 16'hfffe;
    LUT4 ev_run_hold_s5_83__I_0_i61_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_60), .D(ev_run_hold_s5[60]), .Z(us_tx_next[60])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i61_3_lut_4_lut.init = 16'hbf40;
    LUT4 i5_4_lut_adj_331 (.A(status_bit_index[0]), .B(status_bit_index[1]), 
         .C(status_bit_index[3]), .D(status_bit_index[2]), .Z(n12_adj_3206)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(429[37:63])
    defparam i5_4_lut_adj_331.init = 16'hfffe;
    LUT4 i15459_2_lut (.A(n10893), .B(n25163), .Z(pll_clk_enable_267)) /* synthesis lut_function=(!(A (B))) */ ;
    defparam i15459_2_lut.init = 16'h7777;
    LUT4 spi_version_5__bdd_4_lut_16986 (.A(spi_version[5]), .B(spi_version[1]), 
         .C(n12), .D(n28030), .Z(n28031)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam spi_version_5__bdd_4_lut_16986.init = 16'h0100;
    LUT4 i10610_2_lut_3_lut (.A(n16834), .B(mic_cfg_hold_spi[46]), .C(mic_cfg_hold_spi[41]), 
         .Z(mic_cfg_width_w[1])) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (C)))) */ ;
    defparam i10610_2_lut_3_lut.init = 16'h6060;
    LUT4 i1_4_lut_adj_332 (.A(ws2812_frame_done), .B(ws2812_enable_N_2588), 
         .C(ws2812_frames_left[1]), .D(ws2812_frames_left[0]), .Z(pll_clk_enable_21)) /* synthesis lut_function=(A (B+!(C+!(D)))+!A (B)) */ ;
    defparam i1_4_lut_adj_332.init = 16'hcecc;
    PFUMX i16751 (.BLUT(n28401), .ALUT(n28402), .C0(status_bit_index[0]), 
          .Z(n28403));
    LUT4 i3491_3_lut (.A(mic_acc_word[2]), .B(mic_acc_word[1]), .C(mic_acc_word[0]), 
         .Z(n7025)) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(993[37:56])
    defparam i3491_3_lut.init = 16'h6a6a;
    LUT4 i10460_2_lut_3_lut (.A(n16834), .B(mic_cfg_hold_spi[46]), .C(mic_cfg_hold_spi[42]), 
         .Z(mic_cfg_width_w[2])) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (C)))) */ ;
    defparam i10460_2_lut_3_lut.init = 16'h6060;
    LUT4 ev_run_hold_s5_83__I_0_i5_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_4), .D(ev_run_hold_s5[4]), .Z(us_tx_next[4])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i5_3_lut_4_lut.init = 16'hbf40;
    LUT4 i2_2_lut_adj_333 (.A(spi_extension_length[1]), .B(spi_extension_length[0]), 
         .Z(n7)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i2_2_lut_adj_333.init = 16'heeee;
    LUT4 spi_command_2__bdd_4_lut_16533 (.A(spi_command[2]), .B(spi_command[0]), 
         .C(spi_command[1]), .D(spi_command[3]), .Z(n28032)) /* synthesis lut_function=(!(A+(B (C+(D))+!B ((D)+!C)))) */ ;
    defparam spi_command_2__bdd_4_lut_16533.init = 16'h0014;
    LUT4 n27450_bdd_4_lut_then_3_lut (.A(n14149), .B(n14150), .C(status_bit_index[0]), 
         .Z(n28483)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27450_bdd_4_lut_then_3_lut.init = 16'hacac;
    LUT4 i2_3_lut_adj_334 (.A(status_bit_index[4]), .B(status_bit_index[6]), 
         .C(status_bit_index[5]), .Z(n34_adj_3286)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(454[14:90])
    defparam i2_3_lut_adj_334.init = 16'hfefe;
    LUT4 i1_2_lut_adj_335 (.A(ev_fall_N_2693), .B(staging_rd_page_N_2728), 
         .Z(pll_clk_enable_24)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(322[35] 323[54])
    defparam i1_2_lut_adj_335.init = 16'heeee;
    LUT4 i10072_2_lut (.A(status_bit_index[0]), .B(status_bit_index[1]), 
         .Z(n20896)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10072_2_lut.init = 16'h8888;
    LUT4 LessThan_245_i8_3_lut (.A(n6_adj_3169), .B(pcm_read_count_latched_spi[3]), 
         .C(pcm_payload_bit[7]), .Z(n8_adj_3305)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(444[10:59])
    defparam LessThan_245_i8_3_lut.init = 16'h8e8e;
    LUT4 spi_phase_only_N_2792_bdd_4_lut_16820 (.A(spi_phase_only_N_2792), 
         .B(spi_channel_page), .C(spi_bit_count[0]), .D(spi_bit_count[2]), 
         .Z(n28155)) /* synthesis lut_function=(!(A (B (C (D)))+!A (C (D)))) */ ;
    defparam spi_phase_only_N_2792_bdd_4_lut_16820.init = 16'h2fff;
    LUT4 LessThan_245_i6_3_lut (.A(n4_adj_3172), .B(pcm_read_count_latched_spi[2]), 
         .C(pcm_payload_bit[6]), .Z(n6_adj_3169)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(444[10:59])
    defparam LessThan_245_i6_3_lut.init = 16'h8e8e;
    LUT4 LessThan_245_i4_4_lut (.A(pcm_payload_bit[4]), .B(pcm_read_count_latched_spi[1]), 
         .C(pcm_payload_bit[5]), .D(pcm_read_count_latched_spi[0]), .Z(n4_adj_3172)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B ((D)+!C)+!B !(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(444[10:59])
    defparam LessThan_245_i4_4_lut.init = 16'h4d0c;
    ws2812_stream ws2812_i (.reset_count({Open_0, Open_1, Open_2, Open_3, 
            Open_4, Open_5, Open_6, Open_7, Open_8, Open_9, Open_10, 
            Open_11, Open_12, Open_13, reset_count[0]}), .pll_clk(pll_clk), 
            .pll_clk_enable_143(pll_clk_enable_143), .ws2812_enable(ws2812_enable), 
            .n10986(n10986), .rgb_hold({rgb_hold}), .ws2812_frame_done(ws2812_frame_done), 
            .GND_net(GND_net), .n13455(n13455), .n13723(n13723), .n24552(n24552), 
            .rgb_data_c(rgb_data_c)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1036[19] 1042[6])
    LUT4 i10564_4_lut (.A(status_bit_index[3]), .B(status_bit_index[8]), 
         .C(status_bit_index[7]), .D(n34_adj_3286), .Z(n21389)) /* synthesis lut_function=(A (B+(C))+!A (B+(C (D)))) */ ;
    defparam i10564_4_lut.init = 16'hfcec;
    LUT4 i10457_2_lut_3_lut (.A(n16834), .B(mic_cfg_hold_spi[46]), .C(mic_cfg_hold_spi[43]), 
         .Z(mic_cfg_width_w[3])) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (C)))) */ ;
    defparam i10457_2_lut_3_lut.init = 16'h6060;
    LUT4 i10456_2_lut_3_lut (.A(n16834), .B(mic_cfg_hold_spi[46]), .C(mic_cfg_hold_spi[44]), 
         .Z(mic_cfg_width_w[4])) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (C)))) */ ;
    defparam i10456_2_lut_3_lut.init = 16'h6060;
    LUT4 i2_3_lut_4_lut_adj_336 (.A(mic_acc_state[0]), .B(mic_acc_state[1]), 
         .C(mic_acc_state[2]), .D(mic_done_N_2656), .Z(n11928)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1017[13:17])
    defparam i2_3_lut_4_lut_adj_336.init = 16'h0010;
    LUT4 i10455_2_lut_3_lut (.A(n16834), .B(mic_cfg_hold_spi[46]), .C(mic_cfg_hold_spi[45]), 
         .Z(mic_cfg_width_w[5])) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (C)))) */ ;
    defparam i10455_2_lut_3_lut.init = 16'h6060;
    LUT4 i6885_1_lut (.A(phase_step_s1), .Z(n17733)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i6885_1_lut.init = 16'h5555;
    LUT4 i7_4_lut_adj_337 (.A(global_phase_s2[0]), .B(n14_adj_3282), .C(n10_adj_3284), 
         .D(global_phase_s2[3]), .Z(wrap_s2_N_2608)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut_adj_337.init = 16'h8000;
    LUT4 i1_2_lut_3_lut_adj_338 (.A(n13_adj_3180), .B(pll_clk_enable_104), 
         .C(mic_lo_n_next_6__N_648[2]), .Z(n10796)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(208[17:25])
    defparam i1_2_lut_3_lut_adj_338.init = 16'h2020;
    LUT4 ev_run_hold_s5_83__I_0_i62_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_61), .D(ev_run_hold_s5[61]), .Z(us_tx_next[61])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i62_3_lut_4_lut.init = 16'hbf40;
    LUT4 i6_4_lut_adj_339 (.A(global_phase_s2[1]), .B(global_phase_s2[4]), 
         .C(global_phase_s2[5]), .D(global_phase_s2[7]), .Z(n14_adj_3282)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_339.init = 16'h8000;
    LUT4 i1_2_lut_3_lut_adj_340 (.A(n13_adj_3180), .B(pll_clk_enable_104), 
         .C(mic_lo_n_next_6__N_648[3]), .Z(n10797)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(208[17:25])
    defparam i1_2_lut_3_lut_adj_340.init = 16'h2020;
    LUT4 n27570_bdd_4_lut (.A(n27570), .B(\mic_win_q[2] [1]), .C(\mic_win_i[2] [1]), 
         .D(mic_acc_word[0]), .Z(n27573)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27570_bdd_4_lut.init = 16'haad8;
    LUT4 mic_acc_word_2__bdd_4_lut_16283 (.A(mic_acc_word[2]), .B(\mic_win_i[3] [1]), 
         .C(\mic_win_q[3] [1]), .D(mic_acc_word[0]), .Z(n27570)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16283.init = 16'he4aa;
    LUT4 ev_in_clear_I_0_2_lut (.A(ev_in_clear), .B(ev_fall_N_2693), .Z(ev_we)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(335[33:67])
    defparam ev_in_clear_I_0_2_lut.init = 16'heeee;
    LUT4 i1_2_lut_3_lut_adj_341 (.A(n13_adj_3180), .B(pll_clk_enable_104), 
         .C(mic_lo_n_next_6__N_648[4]), .Z(n10798)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(208[17:25])
    defparam i1_2_lut_3_lut_adj_341.init = 16'h2020;
    LUT4 spi_phase_only_N_2792_bdd_3_lut (.A(fpga_cs_n_c), .B(n30), .C(n28155), 
         .Z(n28156)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam spi_phase_only_N_2792_bdd_3_lut.init = 16'hfefe;
    LUT4 spi_byte_count_6__bdd_4_lut_17055 (.A(spi_byte_count[8]), .B(spi_byte_count[7]), 
         .C(n17024), .D(n4_adj_3166), .Z(n28933)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam spi_byte_count_6__bdd_4_lut_17055.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_342 (.A(mic_phase[0]), .B(n100), .C(n107), 
         .D(pcm_acc_next[2]), .Z(n25863)) /* synthesis lut_function=(!(A+!(B (C+(D))))) */ ;
    defparam i1_3_lut_4_lut_adj_342.init = 16'h4440;
    LUT4 i828_2_lut (.A(mic_data_1_c), .B(mic_lo_i_r), .Z(n4599)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(864[17:43])
    defparam i828_2_lut.init = 16'h6666;
    LUT4 i3484_2_lut (.A(mic_acc_word[1]), .B(mic_acc_word[0]), .Z(n7026)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(993[37:56])
    defparam i3484_2_lut.init = 16'h6666;
    LUT4 i15346_4_lut (.A(n7_adj_3260), .B(n12_adj_3272), .C(spi_byte_count[0]), 
         .D(spi_byte_count[4]), .Z(spi1_sck_c_enable_21)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(483[18] 588[12])
    defparam i15346_4_lut.init = 16'h0400;
    CCU2D add_2528_9 (.A0(\mic_acc_i[1] [6]), .B0(n20759), .C0(n4541), 
          .D0(GND_net), .A1(\mic_acc_i[1] [7]), .B1(n20759), .C1(n4541), 
          .D1(GND_net), .CIN(n24883), .S0(n5719), .S1(n5718));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam add_2528_9.INIT0 = 16'h7848;
    defparam add_2528_9.INIT1 = 16'h7848;
    defparam add_2528_9.INJECT1_0 = "NO";
    defparam add_2528_9.INJECT1_1 = "NO";
    CCU2D add_2528_7 (.A0(\mic_acc_i[1] [4]), .B0(n20759), .C0(n4541), 
          .D0(GND_net), .A1(\mic_acc_i[1] [5]), .B1(n20759), .C1(n4541), 
          .D1(GND_net), .CIN(n24882), .COUT(n24883), .S0(n5721), .S1(n5720));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam add_2528_7.INIT0 = 16'h7848;
    defparam add_2528_7.INIT1 = 16'h7848;
    defparam add_2528_7.INJECT1_0 = "NO";
    defparam add_2528_7.INJECT1_1 = "NO";
    LUT4 n27642_bdd_4_lut (.A(n27642), .B(\mic_win_q[0] [6]), .C(\mic_win_i[0] [6]), 
         .D(mic_acc_word[0]), .Z(n27645)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27642_bdd_4_lut.init = 16'haad8;
    LUT4 status_bit_index_7__bdd_4_lut (.A(status_bit_index[8]), .B(status_miso_bit), 
         .C(spi1_miso_N_2456), .D(status_bit_index[6]), .Z(n29101)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B))) */ ;
    defparam status_bit_index_7__bdd_4_lut.init = 16'h44e4;
    LUT4 i7010_2_lut (.A(pll_clk_enable_268), .B(n5_adj_3259), .Z(n17845)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i7010_2_lut.init = 16'h8888;
    CCU2D add_2528_5 (.A0(\mic_acc_i[1] [2]), .B0(n20759), .C0(n4541), 
          .D0(GND_net), .A1(\mic_acc_i[1] [3]), .B1(n20759), .C1(n4541), 
          .D1(GND_net), .CIN(n24881), .COUT(n24882), .S0(n5723), .S1(n5722));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam add_2528_5.INIT0 = 16'h7848;
    defparam add_2528_5.INIT1 = 16'h7848;
    defparam add_2528_5.INJECT1_0 = "NO";
    defparam add_2528_5.INJECT1_1 = "NO";
    LUT4 i2_2_lut_adj_343 (.A(global_phase_s2[2]), .B(global_phase_s2[6]), 
         .Z(n10_adj_3284)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut_adj_343.init = 16'h8888;
    LUT4 n27450_bdd_4_lut_else_3_lut (.A(n14133), .B(n14134), .C(status_bit_index[0]), 
         .Z(n28482)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam n27450_bdd_4_lut_else_3_lut.init = 16'hacac;
    LUT4 i3899_1_lut (.A(ws2812_settle[0]), .Z(n1)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(723[17:38])
    defparam i3899_1_lut.init = 16'h5555;
    LUT4 equal_3055_i31_3_lut (.A(n23_adj_3177), .B(n20743), .C(n17015), 
         .Z(n31)) /* synthesis lut_function=(A+((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[25:53])
    defparam equal_3055_i31_3_lut.init = 16'hfbfb;
    LUT4 i15524_4_lut (.A(n21070), .B(n14_adj_3283), .C(n10_adj_3285), 
         .D(status_bit_index[5]), .Z(spi1_sck_N_161_enable_9)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i15524_4_lut.init = 16'h7fff;
    LUT4 i6_4_lut_adj_344 (.A(status_bit_index[0]), .B(status_bit_index[4]), 
         .C(status_bit_index[2]), .D(status_bit_index[1]), .Z(n14_adj_3283)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_344.init = 16'h8000;
    LUT4 i15510_4_lut (.A(pll_clk_enable_605), .B(mic_acc_last), .C(n4_adj_3299), 
         .D(n20918), .Z(n17838)) /* synthesis lut_function=(A (B+!(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i15510_4_lut.init = 16'h8aaa;
    CCU2D add_2528_3 (.A0(\mic_acc_i[1] [0]), .B0(n20759), .C0(n4541), 
          .D0(GND_net), .A1(\mic_acc_i[1] [1]), .B1(n20759), .C1(n4541), 
          .D1(GND_net), .CIN(n24880), .COUT(n24881), .S0(n5725), .S1(n5724));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam add_2528_3.INIT0 = 16'h7484;
    defparam add_2528_3.INIT1 = 16'h7848;
    defparam add_2528_3.INJECT1_0 = "NO";
    defparam add_2528_3.INJECT1_1 = "NO";
    LUT4 m1_lut (.Z(n29158)) /* synthesis lut_function=1, syn_instantiated=1 */ ;
    defparam m1_lut.init = 16'hffff;
    LUT4 i2_2_lut_adj_345 (.A(status_bit_index[3]), .B(status_bit_index[8]), 
         .Z(n10_adj_3285)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut_adj_345.init = 16'h8888;
    LUT4 i4_2_lut_adj_346 (.A(spi_byte_count[2]), .B(spi_byte_count[1]), 
         .Z(n12_adj_3272)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i4_2_lut_adj_346.init = 16'h4444;
    LUT4 i4_4_lut_adj_347 (.A(mic_sample_count[0]), .B(mic_sample_count[2]), 
         .C(mic_sample_count[1]), .D(n6_adj_3255), .Z(pll_clk_enable_433)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i4_4_lut_adj_347.init = 16'h8000;
    LUT4 i10613_2_lut (.A(spi_byte_count[5]), .B(n21300), .Z(n21440)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10613_2_lut.init = 16'heeee;
    LUT4 mic_acc_word_2__bdd_4_lut_16298 (.A(mic_acc_word[2]), .B(\mic_win_i[1] [6]), 
         .C(\mic_win_q[1] [6]), .D(mic_acc_word[0]), .Z(n27642)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam mic_acc_word_2__bdd_4_lut_16298.init = 16'he4aa;
    CCU2D add_2528_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n20759), .B1(n4541), .C1(GND_net), .D1(GND_net), .COUT(n24880));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(863[17:43])
    defparam add_2528_1.INIT0 = 16'hF000;
    defparam add_2528_1.INIT1 = 16'h7777;
    defparam add_2528_1.INJECT1_0 = "NO";
    defparam add_2528_1.INJECT1_1 = "NO";
    LUT4 i10246_2_lut (.A(status_bit_index[7]), .B(status_bit_index[6]), 
         .Z(n21070)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10246_2_lut.init = 16'h8888;
    LUT4 i1_2_lut_3_lut_adj_348 (.A(spi_bit_count_2__N_1425), .B(fpga_cs_n_c), 
         .C(frame_end), .Z(spi1_sck_c_enable_53)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_348.init = 16'h2020;
    LUT4 i7_4_lut_adj_349 (.A(n13), .B(spi_version[3]), .C(n12), .D(spi_version[5]), 
         .Z(n23_adj_3177)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(560[34:54])
    defparam i7_4_lut_adj_349.init = 16'hfffe;
    LUT4 cs_sync_d_I_0_1576_2_lut (.A(cs_sync_d), .B(cs_sync), .Z(cs_fall)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(399[20:41])
    defparam cs_sync_d_I_0_1576_2_lut.init = 16'h2222;
    LUT4 i4011_2_lut (.A(link_idle[0]), .B(pll_clk_enable_196), .Z(n11_adj_3303)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i4011_2_lut.init = 16'h6666;
    PFUMX i16747 (.BLUT(n28395), .ALUT(n28396), .C0(pcm_spi_sample_15__N_1381[4]), 
          .Z(n28397));
    LUT4 i4009_2_lut_3_lut (.A(ws2812_frame_done), .B(ws2812_frames_left[0]), 
         .C(ws2812_frames_left[1]), .Z(n3369)) /* synthesis lut_function=(!(A (B+!(C))+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(671[13:68])
    defparam i4009_2_lut_3_lut.init = 16'h6464;
    CCU2D add_1592_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24701), .S1(phase_frac_sum[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(195[34:66])
    defparam add_1592_1.INIT0 = 16'hF000;
    defparam add_1592_1.INIT1 = 16'h5555;
    defparam add_1592_1.INJECT1_0 = "NO";
    defparam add_1592_1.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_350 (.A(mic_sample_count[3]), .B(pll_clk_enable_370), 
         .Z(n6_adj_3255)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_350.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_351 (.A(mic_phase[0]), .B(n100), .C(n107), 
         .D(pcm_acc_next[3]), .Z(n25861)) /* synthesis lut_function=(!(A+!(B (C+(D))))) */ ;
    defparam i1_3_lut_4_lut_adj_351.init = 16'h4440;
    LUT4 i15273_2_lut (.A(status_hold[76]), .B(status_next_index[1]), .Z(n26146)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[49:80])
    defparam i15273_2_lut.init = 16'h8888;
    LUT4 i5_4_lut_adj_352 (.A(spi_version[1]), .B(spi_version[4]), .C(spi_version[6]), 
         .D(spi_version[0]), .Z(n13)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(560[34:54])
    defparam i5_4_lut_adj_352.init = 16'hfeff;
    LUT4 i1_3_lut_4_lut_adj_353 (.A(mic_phase[0]), .B(n100), .C(n107), 
         .D(pcm_acc_next[9]), .Z(n25862)) /* synthesis lut_function=(!(A+!(B (C+(D))))) */ ;
    defparam i1_3_lut_4_lut_adj_353.init = 16'h4440;
    LUT4 i15382_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_0)) /* synthesis lut_function=(!(A+(B+(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15382_2_lut_3_lut.init = 16'h0101;
    PFUMX i16745 (.BLUT(n28392), .ALUT(n28393), .C0(pcm_spi_sample_15__N_1381[4]), 
          .Z(n28394));
    LUT4 i1_2_lut_3_lut_adj_354 (.A(fpga_cs_n_c), .B(n10659), .C(n30), 
         .Z(n12_adj_3175)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(85[24:33])
    defparam i1_2_lut_3_lut_adj_354.init = 16'h0404;
    LUT4 i9920_2_lut (.A(spi_command[0]), .B(spi_command[1]), .Z(n20743)) /* synthesis lut_function=(A (B)) */ ;
    defparam i9920_2_lut.init = 16'h8888;
    PFUMX i16913 (.BLUT(n28443), .ALUT(n28683), .C0(pcm_spi_sample_15__N_1381[4]), 
          .Z(n28684));
    LUT4 i1_2_lut_adj_355 (.A(spi_byte_count[6]), .B(spi_byte_count[7]), 
         .Z(n25878)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_355.init = 16'heeee;
    LUT4 i15323_3_lut (.A(mic_cfg_settle[2]), .B(mic_cfg_settle[1]), .C(mic_cfg_settle[0]), 
         .Z(mic_cfg_settle_3__N_2142[2])) /* synthesis lut_function=(A (B+(C))+!A !(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(285[17:31])
    defparam i15323_3_lut.init = 16'ha9a9;
    LUT4 i1_2_lut_adj_356 (.A(mic_cfg_settle[0]), .B(n17_adj_3196), .Z(mic_acc_busy_N_2299)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam i1_2_lut_adj_356.init = 16'heeee;
    LUT4 i1_2_lut_3_lut_adj_357 (.A(fpga_cs_n_c), .B(n10659), .C(spi_byte_count[0]), 
         .Z(n16953)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(85[24:33])
    defparam i1_2_lut_3_lut_adj_357.init = 16'h0404;
    LUT4 status_bit_index_7__bdd_4_lut_17182 (.A(status_bit_index[8]), .B(spi1_miso_N_2456), 
         .C(spi1_miso_N_2462), .D(status_bit_index[6]), .Z(n29100)) /* synthesis lut_function=(!(A+!(B (C+(D))+!B !((D)+!C)))) */ ;
    defparam status_bit_index_7__bdd_4_lut_17182.init = 16'h4450;
    LUT4 i1_2_lut_adj_358 (.A(ws2812_frames_left[1]), .B(ws2812_frames_left[0]), 
         .Z(n16248)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_358.init = 16'h9999;
    MUX21 i15973114_i1 (.D0(n27783), .D1(n27663), .SD(mic_acc_word[1]), 
          .Z(mic_acc_win[0]));
    LUT4 i15036_4_lut (.A(spi_byte_count[3]), .B(spi_byte_count[7]), .C(spi_byte_count[6]), 
         .D(spi_rgb_start[7]), .Z(n9)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;
    defparam i15036_4_lut.init = 16'h0001;
    LUT4 i3137_4_lut (.A(link_timeout), .B(cs_fall), .C(link_tick_d), 
         .D(fpga_time[10]), .Z(pll_clk_enable_196)) /* synthesis lut_function=(A (B)+!A (B+!(C+!(D)))) */ ;
    defparam i3137_4_lut.init = 16'hcdcc;
    LUT4 i15376_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(ev_ch_lo_1)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[45:73])
    defparam i15376_2_lut_3_lut.init = 16'h1010;
    CCU2D add_2492_9 (.A0(\mic_acc_i[0] [6]), .B0(n79), .C0(n4541), .D0(GND_net), 
          .A1(\mic_acc_i[0] [7]), .B1(n79), .C1(n4541), .D1(GND_net), 
          .CIN(n24875), .S0(n5711), .S1(n5710));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(857[17:43])
    defparam add_2492_9.INIT0 = 16'hd212;
    defparam add_2492_9.INIT1 = 16'hd212;
    defparam add_2492_9.INJECT1_0 = "NO";
    defparam add_2492_9.INJECT1_1 = "NO";
    CCU2D add_2492_7 (.A0(\mic_acc_i[0] [4]), .B0(n79), .C0(n4541), .D0(GND_net), 
          .A1(\mic_acc_i[0] [5]), .B1(n79), .C1(n4541), .D1(GND_net), 
          .CIN(n24874), .COUT(n24875), .S0(n5713), .S1(n5712));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(857[17:43])
    defparam add_2492_7.INIT0 = 16'hd212;
    defparam add_2492_7.INIT1 = 16'hd212;
    defparam add_2492_7.INJECT1_0 = "NO";
    defparam add_2492_7.INJECT1_1 = "NO";
    LUT4 mic_cfg_count_0__bdd_3_lut_17185 (.A(mic_cfg_count[0]), .B(mic_cfg_count[2]), 
         .C(status_bit_index[1]), .Z(n29103)) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;
    defparam mic_cfg_count_0__bdd_3_lut_17185.init = 16'hacac;
    umh_channel_ram18 staging_ram (.\ev_sum[2] (ev_sum[2]), .staging_rd_page_N_2728(staging_rd_page_N_2728), 
            .ev_in_clear(ev_in_clear), .spi_channel_index({spi_channel_index}), 
            .staging_wr_page(staging_wr_page), .ev_ch({ev_ch}), .staging_rd_page(staging_rd_page), 
            .spi1_mosi_c_0(spi1_mosi_c_0), .\spi_rx_shift[0] (spi_rx_shift[0]), 
            .\spi_rx_shift[1] (spi_rx_shift[1]), .\spi_rx_shift[2] (spi_rx_shift[2]), 
            .\spi_rx_shift[3] (spi_rx_shift[3]), .\spi_rx_shift[4] (spi_rx_shift[4]), 
            .\spi_rx_shift[5] (spi_rx_shift[5]), .\spi_rx_shift[6] (spi_rx_shift[6]), 
            .\staging_q[0] (staging_q[0]), .\staging_q[3] (staging_q[3]), 
            .\staging_q[4] (staging_q[4]), .\staging_q[5] (staging_q[5]), 
            .\staging_q[6] (staging_q[6]), .\staging_q[7] (staging_q[7]), 
            .spi1_sck_c(spi1_sck_c), .spi_payload_byte(spi_payload_byte), 
            .VCC_net(VCC_net), .pll_clk(pll_clk), .GND_net(GND_net), .n15(n15), 
            .n26133(n26133), .ev_fall_N_2693(ev_fall_N_2693), .n25667(n25667), 
            .\ev_sum[1] (ev_sum[1]), .n26099(n26099), .n25725(n25725), 
            .\build_level[3] (build_level[3]), .build_mod(build_mod), .\event_level[3] (event_level[3]), 
            .\build_level[2] (build_level[2]), .\event_level[2] (event_level[2]), 
            .\build_level[1] (build_level[1]), .\event_level[1] (event_level[1])) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[23] 329[6])
    LUT4 i15259_4_lut (.A(n31_adj_3309), .B(spi_rgb_start[7]), .C(spi_byte_count[3]), 
         .D(n25850), .Z(n26122)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i15259_4_lut.init = 16'h8000;
    CCU2D add_2492_5 (.A0(\mic_acc_i[0] [2]), .B0(n79), .C0(n4541), .D0(GND_net), 
          .A1(\mic_acc_i[0] [3]), .B1(n79), .C1(n4541), .D1(GND_net), 
          .CIN(n24873), .COUT(n24874), .S0(n5715), .S1(n5714));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(857[17:43])
    defparam add_2492_5.INIT0 = 16'hd212;
    defparam add_2492_5.INIT1 = 16'hd212;
    defparam add_2492_5.INJECT1_0 = "NO";
    defparam add_2492_5.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_359 (.A(n13_adj_3191), .B(spi1_sck_c_enable_53), .C(n8_adj_3193), 
         .D(spi_command[4]), .Z(spi1_sck_c_enable_52)) /* synthesis lut_function=(A (B)+!A (B (C+(D)))) */ ;
    defparam i1_4_lut_adj_359.init = 16'hccc8;
    GSR GSR_INST (.GSR(fpga_cs_n_N_2472));
    LUT4 i3_3_lut (.A(spi_command[5]), .B(n11_adj_3278), .C(invalid_frame_spi_N_2536), 
         .Z(n8_adj_3193)) /* synthesis lut_function=(A+(B+!(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(580[34:54])
    defparam i3_3_lut.init = 16'hefef;
    LUT4 ev_run_hold_s5_83__I_0_i63_3_lut_4_lut (.A(wrap_s5), .B(running), 
         .C(us_tx_c_62), .D(ev_run_hold_s5[62]), .Z(us_tx_next[62])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(1030[46:67])
    defparam ev_run_hold_s5_83__I_0_i63_3_lut_4_lut.init = 16'hbf40;
    FD1P3IX mic_gate_fill_i0 (.D(mic_acc_last_N_2675[0]), .SP(pll_clk_enable_605), 
            .CD(n17838), .CK(pll_clk), .Q(mic_gate_fill[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[12] 1026[8])
    defparam mic_gate_fill_i0.GSR = "DISABLED";
    CCU2D add_2492_3 (.A0(\mic_acc_i[0] [0]), .B0(n79), .C0(n4541), .D0(GND_net), 
          .A1(\mic_acc_i[0] [1]), .B1(n79), .C1(n4541), .D1(GND_net), 
          .CIN(n24872), .COUT(n24873), .S0(n5717), .S1(n5716));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(857[17:43])
    defparam add_2492_3.INIT0 = 16'hd121;
    defparam add_2492_3.INIT1 = 16'hd212;
    defparam add_2492_3.INJECT1_0 = "NO";
    defparam add_2492_3.INJECT1_1 = "NO";
    CCU2D feed_wr_spi_3073_add_4_3 (.A0(feed_wr_spi[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(feed_wr_spi[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24767), .COUT(n24768), .S0(n54), .S1(n53));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[60:79])
    defparam feed_wr_spi_3073_add_4_3.INIT0 = 16'hfaaa;
    defparam feed_wr_spi_3073_add_4_3.INIT1 = 16'hfaaa;
    defparam feed_wr_spi_3073_add_4_3.INJECT1_0 = "NO";
    defparam feed_wr_spi_3073_add_4_3.INJECT1_1 = "NO";
    umh_mic_iq_ram mic_iq_ram (.pll_clk(pll_clk), .mic_ram_we(mic_ram_we), 
            .VCC_net(VCC_net), .GND_net(GND_net), .mic_ram_wr_addr({mic_ram_wr_addr}), 
            .mic_ram_rd_addr({mic_ram_rd_addr}), .mic_ram_wr_data({mic_ram_wr_data}), 
            .mic_ram_rd_data({mic_ram_rd_data})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(344[20] 348[6])
    CCU2D add_2492_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n4541), .B1(n79), .C1(GND_net), .D1(GND_net), .COUT(n24872));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(857[17:43])
    defparam add_2492_1.INIT0 = 16'hF000;
    defparam add_2492_1.INIT1 = 16'hdddd;
    defparam add_2492_1.INJECT1_0 = "NO";
    defparam add_2492_1.INJECT1_1 = "NO";
    LUT4 i2906_2_lut (.A(mic_acc_busy_N_2292), .B(mic_done_N_2656), .Z(mic_done_N_2650)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(984[9] 1025[16])
    defparam i2906_2_lut.init = 16'h8888;
    LUT4 i1_2_lut_adj_360 (.A(n13_adj_3180), .B(mic_lo_n_next_6__N_648[6]), 
         .Z(mic_lo_n_next[6])) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[33:52])
    defparam i1_2_lut_adj_360.init = 16'h8888;
    LUT4 i15362_4_lut (.A(n16180), .B(spi_command[0]), .C(n16960), .D(n26032), 
         .Z(invalid_frame_spi_N_2536)) /* synthesis lut_function=(!((B (C+!(D))+!B (C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(550[30] 582[24])
    defparam i15362_4_lut.init = 16'h0a02;
    
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
// Verilog Description of module umh_level_fifo
//

module umh_level_fifo (n14085, spi1_sck_c, feed_wr_spi, n14097, n14087, 
            feed_q, pll_clk, rd_data_7__N_2913, n14083, n68, spi1_mosi_c_0, 
            \spi_rx_shift[0] , \spi_rx_shift[1] , \spi_rx_shift[2] , \spi_rx_shift[3] , 
            \spi_rx_shift[4] , \spi_rx_shift[5] , \spi_rx_shift[6] , n14106, 
            n14108, n14110, n14112, n14114, n14116, n14118, n14120, 
            feed_we, VCC_net, GND_net, n14101, n14099, n14089, n14091, 
            n14093, n14095) /* synthesis syn_module_defined=1 */ ;
    output n14085;
    input spi1_sck_c;
    input [9:0]feed_wr_spi;
    output n14097;
    output n14087;
    output [7:0]feed_q;
    input pll_clk;
    input [7:0]rd_data_7__N_2913;
    output n14083;
    input [9:0]n68;
    input spi1_mosi_c_0;
    input \spi_rx_shift[0] ;
    input \spi_rx_shift[1] ;
    input \spi_rx_shift[2] ;
    input \spi_rx_shift[3] ;
    input \spi_rx_shift[4] ;
    input \spi_rx_shift[5] ;
    input \spi_rx_shift[6] ;
    output n14106;
    output n14108;
    output n14110;
    output n14112;
    output n14114;
    output n14116;
    output n14118;
    output n14120;
    input feed_we;
    input VCC_net;
    input GND_net;
    output n14101;
    output n14099;
    output n14089;
    output n14091;
    output n14093;
    output n14095;
    
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(86[24:32])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(108[10:17])
    
    FD1S3AX mem_3314 (.D(feed_wr_spi[1]), .CK(spi1_sck_c), .Q(n14085));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3314.GSR = "DISABLED";
    FD1S3AX mem_3326 (.D(feed_wr_spi[7]), .CK(spi1_sck_c), .Q(n14097));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3326.GSR = "DISABLED";
    FD1S3AX mem_3316 (.D(feed_wr_spi[2]), .CK(spi1_sck_c), .Q(n14087));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3316.GSR = "DISABLED";
    FD1S3AX rd_data_i0 (.D(rd_data_7__N_2913[0]), .CK(pll_clk), .Q(feed_q[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=385, LSE_RLINE=388 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i0.GSR = "DISABLED";
    FD1S3AX mem_3312 (.D(feed_wr_spi[0]), .CK(spi1_sck_c), .Q(n14083));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3312.GSR = "DISABLED";
    DP8KC mem0 (.DIA0(spi1_mosi_c_0), .DIA1(\spi_rx_shift[0] ), .DIA2(\spi_rx_shift[1] ), 
          .DIA3(\spi_rx_shift[2] ), .DIA4(\spi_rx_shift[3] ), .DIA5(\spi_rx_shift[4] ), 
          .DIA6(\spi_rx_shift[5] ), .DIA7(\spi_rx_shift[6] ), .DIA8(GND_net), 
          .ADA0(VCC_net), .ADA1(GND_net), .ADA2(GND_net), .ADA3(feed_wr_spi[0]), 
          .ADA4(feed_wr_spi[1]), .ADA5(feed_wr_spi[2]), .ADA6(feed_wr_spi[3]), 
          .ADA7(feed_wr_spi[4]), .ADA8(feed_wr_spi[5]), .ADA9(feed_wr_spi[6]), 
          .ADA10(feed_wr_spi[7]), .ADA11(feed_wr_spi[8]), .ADA12(feed_wr_spi[9]), 
          .CEA(VCC_net), .OCEA(VCC_net), .CLKA(spi1_sck_c), .WEA(feed_we), 
          .CSA0(GND_net), .CSA1(GND_net), .CSA2(GND_net), .RSTA(GND_net), 
          .DIB0(GND_net), .DIB1(GND_net), .DIB2(GND_net), .DIB3(GND_net), 
          .DIB4(GND_net), .DIB5(GND_net), .DIB6(GND_net), .DIB7(GND_net), 
          .DIB8(GND_net), .ADB0(VCC_net), .ADB1(GND_net), .ADB2(GND_net), 
          .ADB3(n68[0]), .ADB4(n68[1]), .ADB5(n68[2]), .ADB6(n68[3]), 
          .ADB7(n68[4]), .ADB8(n68[5]), .ADB9(n68[6]), .ADB10(n68[7]), 
          .ADB11(n68[8]), .ADB12(n68[9]), .CEB(VCC_net), .OCEB(VCC_net), 
          .CLKB(pll_clk), .WEB(GND_net), .CSB0(GND_net), .CSB1(GND_net), 
          .CSB2(GND_net), .RSTB(GND_net), .DOB0(n14106), .DOB1(n14108), 
          .DOB2(n14110), .DOB3(n14112), .DOB4(n14114), .DOB5(n14116), 
          .DOB6(n14118), .DOB7(n14120));
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
    FD1S3AX mem_3330 (.D(feed_wr_spi[9]), .CK(spi1_sck_c), .Q(n14101));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3330.GSR = "DISABLED";
    FD1S3AX mem_3328 (.D(feed_wr_spi[8]), .CK(spi1_sck_c), .Q(n14099));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3328.GSR = "DISABLED";
    FD1S3AX mem_3318 (.D(feed_wr_spi[3]), .CK(spi1_sck_c), .Q(n14089));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3318.GSR = "DISABLED";
    FD1S3AX mem_3320 (.D(feed_wr_spi[4]), .CK(spi1_sck_c), .Q(n14091));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3320.GSR = "DISABLED";
    FD1S3AX mem_3322 (.D(feed_wr_spi[5]), .CK(spi1_sck_c), .Q(n14093));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3322.GSR = "DISABLED";
    FD1S3AX mem_3324 (.D(feed_wr_spi[6]), .CK(spi1_sck_c), .Q(n14095));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(79[38:50])
    defparam mem_3324.GSR = "DISABLED";
    FD1S3AX rd_data_i1 (.D(rd_data_7__N_2913[1]), .CK(pll_clk), .Q(feed_q[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=385, LSE_RLINE=388 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i1.GSR = "DISABLED";
    FD1S3AX rd_data_i2 (.D(rd_data_7__N_2913[2]), .CK(pll_clk), .Q(feed_q[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=385, LSE_RLINE=388 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i2.GSR = "DISABLED";
    FD1S3AX rd_data_i3 (.D(rd_data_7__N_2913[3]), .CK(pll_clk), .Q(feed_q[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=385, LSE_RLINE=388 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i3.GSR = "DISABLED";
    FD1S3AX rd_data_i4 (.D(rd_data_7__N_2913[4]), .CK(pll_clk), .Q(feed_q[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=385, LSE_RLINE=388 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i4.GSR = "DISABLED";
    FD1S3AX rd_data_i5 (.D(rd_data_7__N_2913[5]), .CK(pll_clk), .Q(feed_q[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=385, LSE_RLINE=388 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i5.GSR = "DISABLED";
    FD1S3AX rd_data_i6 (.D(rd_data_7__N_2913[6]), .CK(pll_clk), .Q(feed_q[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=385, LSE_RLINE=388 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i6.GSR = "DISABLED";
    FD1S3AX rd_data_i7 (.D(rd_data_7__N_2913[7]), .CK(pll_clk), .Q(feed_q[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=385, LSE_RLINE=388 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(80[12:54])
    defparam rd_data_i7.GSR = "DISABLED";
    
endmodule
//
// Verilog Description of module umh_toggle_ram84
//

module umh_toggle_ram84 (pll_clk, ev_we, VCC_net, GND_net, rmw_slot, 
            event_rd_addr, active_bank_N_898, ev_wr_data, ev_rd_data) /* synthesis syn_module_defined=1 */ ;
    input pll_clk;
    input ev_we;
    input VCC_net;
    input GND_net;
    input [7:0]rmw_slot;
    input [8:0]event_rd_addr;
    input active_bank_N_898;
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
            .ADW6(rmw_slot[6]), .ADW7(rmw_slot[7]), .ADW8(active_bank_N_898), 
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
            .ADW6(rmw_slot[6]), .ADW7(rmw_slot[7]), .ADW8(active_bank_N_898), 
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
            .ADW6(rmw_slot[6]), .ADW7(rmw_slot[7]), .ADW8(active_bank_N_898), 
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
            .ADW8(active_bank_N_898), .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), 
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
            .ADW6(rmw_slot[6]), .ADW7(rmw_slot[7]), .ADW8(active_bank_N_898), 
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

module spi_mic_stream (sck_N_3140, spi_mic_cs_n_c, mic_latest, spi_mic_miso_c, 
            n29158, GND_net) /* synthesis syn_module_defined=1 */ ;
    input sck_N_3140;
    input spi_mic_cs_n_c;
    input [63:0]mic_latest;
    output spi_mic_miso_c;
    input n29158;
    input GND_net;
    
    wire sck_N_3140 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(10[12:16])
    wire [95:0]hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(10[12:16])
    wire [6:0]bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[11:20])
    
    wire n28500, n27021;
    wire [6:0]out_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(20[12:21])
    
    wire n28418, n28493, n27014, n28427, n28481, n27007, n28400, 
        n28439, n27000, n28487, n28421, n28492, n28491, sck_N_3140_enable_72, 
        sck_N_3140_enable_8;
    wire [6:0]bit_count_6__N_3141;
    
    wire n27981, n25785, n27974, n28501, n28502, n17879, n17877, 
        n17875, n17873, n17871, n17869, n28399, n28398, n26979, 
        n27753, n27741, n28414, n27028, n28415, n28413, n28417, 
        n28416, n28420, n28488, n28489, n28490, n28419, n28423, 
        n28422, n28485, n28486, n27738, n28424, n28426, n28472, 
        n28425, n27750, n28478, n28390, n28469, n28479, n28480, 
        n28476, n28477, n12, n28470, n28471, n28467, n28468, n27975, 
        n27977, n27978;
    wire [6:0]bit_count_6__N_3149;
    
    wire n28437, n28438, n27979, n27976, n27980, n28388;
    wire [6:0]n20;
    
    wire n24895, n24894, n24893, n28389;
    
    LUT4 n27408_bdd_4_lut_else_3_lut (.A(hold[85]), .B(bit_count[1]), .C(hold[87]), 
         .Z(n28500)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27408_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    MUX21 i1590781_i1 (.D0(n28418), .D1(n28493), .SD(out_index[2]), .Z(n27021));
    MUX21 i1590178_i1 (.D0(n28427), .D1(n28481), .SD(out_index[2]), .Z(n27014));
    MUX21 i1589575_i1 (.D0(n28400), .D1(n28439), .SD(out_index[2]), .Z(n27007));
    MUX21 i1588972_i1 (.D0(n28487), .D1(n28421), .SD(out_index[2]), .Z(n27000));
    LUT4 n27432_bdd_4_lut_then_3_lut (.A(hold[76]), .B(bit_count[1]), .C(hold[78]), 
         .Z(n28492)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27432_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27432_bdd_4_lut_else_3_lut (.A(hold[77]), .B(bit_count[1]), .C(hold[79]), 
         .Z(n28491)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27432_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    FD1P3DX hold__i1 (.D(mic_latest[0]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i1.GSR = "DISABLED";
    FD1P3DX bit_count_i0 (.D(bit_count_6__N_3141[0]), .SP(sck_N_3140_enable_8), 
            .CK(sck_N_3140), .CD(spi_mic_cs_n_c), .Q(bit_count[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i0.GSR = "DISABLED";
    LUT4 i1_3_lut (.A(sck_N_3140_enable_72), .B(spi_mic_cs_n_c), .C(n27981), 
         .Z(spi_mic_miso_c)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[24:36])
    defparam i1_3_lut.init = 16'h1010;
    LUT4 n27021_bdd_4_lut_16499_4_lut (.A(bit_count[0]), .B(n25785), .C(n27000), 
         .D(bit_count[3]), .Z(n27974)) /* synthesis lut_function=(A (C (D))+!A (B (C+!(D))+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(20[24:41])
    defparam n27021_bdd_4_lut_16499_4_lut.init = 16'hf044;
    PFUMX i16817 (.BLUT(n28500), .ALUT(n28501), .C0(bit_count[0]), .Z(n28502));
    LUT4 n27408_bdd_4_lut_then_3_lut (.A(hold[84]), .B(bit_count[1]), .C(hold[86]), 
         .Z(n28501)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27408_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    FD1P3DX bit_count_i6 (.D(n17879), .SP(sck_N_3140_enable_8), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i6.GSR = "DISABLED";
    FD1P3DX bit_count_i5 (.D(n17877), .SP(sck_N_3140_enable_8), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i5.GSR = "DISABLED";
    FD1P3DX bit_count_i4 (.D(n17875), .SP(sck_N_3140_enable_8), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i4.GSR = "DISABLED";
    FD1P3DX bit_count_i3 (.D(n17873), .SP(sck_N_3140_enable_8), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i3.GSR = "DISABLED";
    FD1P3DX bit_count_i2 (.D(n17871), .SP(sck_N_3140_enable_8), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i2.GSR = "DISABLED";
    FD1P3DX bit_count_i1 (.D(n17869), .SP(sck_N_3140_enable_8), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam bit_count_i1.GSR = "DISABLED";
    LUT4 n27594_bdd_4_lut_then_3_lut (.A(hold[48]), .B(bit_count[1]), .C(hold[50]), 
         .Z(n28399)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27594_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27594_bdd_4_lut_else_3_lut (.A(hold[49]), .B(bit_count[1]), .C(hold[51]), 
         .Z(n28398)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27594_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    MUX21 i15955105_i1 (.D0(n27753), .D1(n27741), .SD(out_index[2]), .Z(n26979));
    PFUMX i16811 (.BLUT(n28491), .ALUT(n28492), .C0(bit_count[0]), .Z(n28493));
    LUT4 n27684_bdd_4_lut_then_3_lut (.A(hold[80]), .B(bit_count[1]), .C(hold[82]), 
         .Z(n28414)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27684_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    MUX21 i1591384_i1 (.D0(n28415), .D1(n28502), .SD(out_index[2]), .Z(n27028));
    LUT4 n27684_bdd_4_lut_else_3_lut (.A(hold[81]), .B(bit_count[1]), .C(hold[83]), 
         .Z(n28413)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27684_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 n27672_bdd_4_lut_then_3_lut (.A(hold[72]), .B(bit_count[1]), .C(hold[74]), 
         .Z(n28417)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27672_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27672_bdd_4_lut_else_3_lut (.A(hold[73]), .B(bit_count[1]), .C(hold[75]), 
         .Z(n28416)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27672_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 n27582_bdd_4_lut_then_3_lut (.A(hold[36]), .B(bit_count[1]), .C(hold[38]), 
         .Z(n28420)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27582_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    PFUMX i16809 (.BLUT(n28488), .ALUT(n28489), .C0(bit_count[0]), .Z(n28490));
    LUT4 n27582_bdd_4_lut_else_3_lut (.A(hold[37]), .B(bit_count[1]), .C(hold[39]), 
         .Z(n28419)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27582_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 i15065_then_3_lut (.A(hold[30]), .B(bit_count[1]), .C(hold[28]), 
         .Z(n28423)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15065_then_3_lut.init = 16'he2e2;
    LUT4 i15065_else_3_lut (.A(hold[31]), .B(bit_count[1]), .C(hold[29]), 
         .Z(n28422)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15065_else_3_lut.init = 16'he2e2;
    PFUMX i16807 (.BLUT(n28485), .ALUT(n28486), .C0(bit_count[0]), .Z(n28487));
    LUT4 n27738_bdd_4_lut (.A(n27738), .B(n28490), .C(n28424), .D(bit_count[3]), 
         .Z(n27741)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27738_bdd_4_lut.init = 16'haad8;
    LUT4 n27654_bdd_4_lut_then_3_lut (.A(hold[56]), .B(bit_count[1]), .C(hold[58]), 
         .Z(n28426)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27654_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 bit_count_4__bdd_4_lut_16372 (.A(bit_count[4]), .B(n25785), .C(n28472), 
         .D(bit_count[3]), .Z(n27738)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam bit_count_4__bdd_4_lut_16372.init = 16'he4aa;
    LUT4 n27654_bdd_4_lut_else_3_lut (.A(hold[57]), .B(bit_count[1]), .C(hold[59]), 
         .Z(n28425)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27654_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 n27750_bdd_4_lut (.A(n27750), .B(n28478), .C(n28390), .D(bit_count[3]), 
         .Z(n27753)) /* synthesis lut_function=(A (B+(D))+!A !((D)+!C)) */ ;
    defparam n27750_bdd_4_lut.init = 16'haad8;
    LUT4 bit_count_4__bdd_4_lut (.A(bit_count[4]), .B(n25785), .C(n28469), 
         .D(bit_count[3]), .Z(n27750)) /* synthesis lut_function=(A (C+!(D))+!A (B (D))) */ ;
    defparam bit_count_4__bdd_4_lut.init = 16'he4aa;
    PFUMX i16803 (.BLUT(n28479), .ALUT(n28480), .C0(bit_count[0]), .Z(n28481));
    PFUMX i16801 (.BLUT(n28476), .ALUT(n28477), .C0(bit_count[0]), .Z(n28478));
    LUT4 i15454_4_lut (.A(bit_count[2]), .B(n12), .C(bit_count[6]), .D(bit_count[1]), 
         .Z(sck_N_3140_enable_72)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(23[15:34])
    defparam i15454_4_lut.init = 16'h0001;
    LUT4 i5_4_lut (.A(bit_count[0]), .B(bit_count[5]), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n12)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(23[15:34])
    defparam i5_4_lut.init = 16'hfffe;
    PFUMX i16797 (.BLUT(n28470), .ALUT(n28471), .C0(bit_count[0]), .Z(n28472));
    PFUMX i16795 (.BLUT(n28467), .ALUT(n28468), .C0(bit_count[0]), .Z(n28469));
    LUT4 n27021_bdd_4_lut (.A(n27021), .B(bit_count[3]), .C(n25785), .D(bit_count[0]), 
         .Z(n27975)) /* synthesis lut_function=(A ((C (D))+!B)+!A (B (C (D)))) */ ;
    defparam n27021_bdd_4_lut.init = 16'he222;
    LUT4 n27028_bdd_3_lut_16622 (.A(n27014), .B(n27007), .C(bit_count[3]), 
         .Z(n27977)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n27028_bdd_3_lut_16622.init = 16'hcaca;
    LUT4 n27028_bdd_2_lut_16549 (.A(n27028), .B(bit_count[3]), .Z(n27978)) /* synthesis lut_function=(A (B)) */ ;
    defparam n27028_bdd_2_lut_16549.init = 16'h8888;
    LUT4 i15443_3_lut (.A(bit_count[5]), .B(sck_N_3140_enable_72), .C(bit_count[6]), 
         .Z(sck_N_3140_enable_8)) /* synthesis lut_function=((B+!(C))+!A) */ ;
    defparam i15443_3_lut.init = 16'hdfdf;
    LUT4 i10004_2_lut (.A(bit_count_6__N_3149[0]), .B(sck_N_3140_enable_72), 
         .Z(bit_count_6__N_3141[0])) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(33[14] 35[8])
    defparam i10004_2_lut.init = 16'heeee;
    FD1P3DX hold__i2 (.D(mic_latest[1]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i2.GSR = "DISABLED";
    FD1P3DX hold__i3 (.D(mic_latest[2]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i3.GSR = "DISABLED";
    FD1P3DX hold__i4 (.D(mic_latest[3]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i4.GSR = "DISABLED";
    FD1P3DX hold__i5 (.D(mic_latest[4]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i5.GSR = "DISABLED";
    FD1P3DX hold__i6 (.D(mic_latest[5]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i6.GSR = "DISABLED";
    FD1P3DX hold__i7 (.D(mic_latest[6]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i7.GSR = "DISABLED";
    FD1P3DX hold__i8 (.D(mic_latest[7]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i8.GSR = "DISABLED";
    FD1P3DX hold__i9 (.D(mic_latest[8]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i9.GSR = "DISABLED";
    FD1P3DX hold__i10 (.D(mic_latest[9]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i10.GSR = "DISABLED";
    FD1P3DX hold__i11 (.D(mic_latest[10]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i11.GSR = "DISABLED";
    FD1P3DX hold__i12 (.D(mic_latest[11]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i12.GSR = "DISABLED";
    FD1P3DX hold__i13 (.D(mic_latest[12]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i13.GSR = "DISABLED";
    FD1P3DX hold__i14 (.D(mic_latest[13]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i14.GSR = "DISABLED";
    FD1P3DX hold__i15 (.D(mic_latest[14]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i15.GSR = "DISABLED";
    FD1P3DX hold__i16 (.D(mic_latest[15]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i16.GSR = "DISABLED";
    FD1P3DX hold__i17 (.D(mic_latest[16]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[24])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i17.GSR = "DISABLED";
    FD1P3DX hold__i18 (.D(mic_latest[17]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[25])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i18.GSR = "DISABLED";
    FD1P3DX hold__i19 (.D(mic_latest[18]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[26])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i19.GSR = "DISABLED";
    FD1P3DX hold__i20 (.D(mic_latest[19]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[27])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i20.GSR = "DISABLED";
    FD1P3DX hold__i21 (.D(mic_latest[20]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[28])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i21.GSR = "DISABLED";
    FD1P3DX hold__i22 (.D(mic_latest[21]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[29])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i22.GSR = "DISABLED";
    FD1P3DX hold__i23 (.D(mic_latest[22]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[30])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i23.GSR = "DISABLED";
    FD1P3DX hold__i24 (.D(mic_latest[23]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[31])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i24.GSR = "DISABLED";
    FD1P3DX hold__i25 (.D(mic_latest[24]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[32])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i25.GSR = "DISABLED";
    FD1P3DX hold__i26 (.D(mic_latest[25]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[33])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i26.GSR = "DISABLED";
    FD1P3DX hold__i27 (.D(mic_latest[26]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[34])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i27.GSR = "DISABLED";
    FD1P3DX hold__i28 (.D(mic_latest[27]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[35])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i28.GSR = "DISABLED";
    FD1P3DX hold__i29 (.D(mic_latest[28]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[36])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i29.GSR = "DISABLED";
    FD1P3DX hold__i30 (.D(mic_latest[29]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[37])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i30.GSR = "DISABLED";
    FD1P3DX hold__i31 (.D(mic_latest[30]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[38])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i31.GSR = "DISABLED";
    FD1P3DX hold__i32 (.D(mic_latest[31]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[39])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i32.GSR = "DISABLED";
    FD1P3DX hold__i33 (.D(mic_latest[32]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[48])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i33.GSR = "DISABLED";
    FD1P3DX hold__i34 (.D(mic_latest[33]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[49])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i34.GSR = "DISABLED";
    FD1P3DX hold__i35 (.D(mic_latest[34]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[50])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i35.GSR = "DISABLED";
    FD1P3DX hold__i36 (.D(mic_latest[35]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[51])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i36.GSR = "DISABLED";
    FD1P3DX hold__i37 (.D(mic_latest[36]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[52])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i37.GSR = "DISABLED";
    FD1P3DX hold__i38 (.D(mic_latest[37]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[53])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i38.GSR = "DISABLED";
    FD1P3DX hold__i39 (.D(mic_latest[38]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[54])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i39.GSR = "DISABLED";
    FD1P3DX hold__i40 (.D(mic_latest[39]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[55])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i40.GSR = "DISABLED";
    FD1P3DX hold__i41 (.D(mic_latest[40]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[56])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i41.GSR = "DISABLED";
    FD1P3DX hold__i42 (.D(mic_latest[41]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[57])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i42.GSR = "DISABLED";
    FD1P3DX hold__i43 (.D(mic_latest[42]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[58])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i43.GSR = "DISABLED";
    FD1P3DX hold__i44 (.D(mic_latest[43]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[59])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i44.GSR = "DISABLED";
    FD1P3DX hold__i45 (.D(mic_latest[44]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[60])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i45.GSR = "DISABLED";
    FD1P3DX hold__i46 (.D(mic_latest[45]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[61])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i46.GSR = "DISABLED";
    FD1P3DX hold__i47 (.D(mic_latest[46]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[62])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i47.GSR = "DISABLED";
    FD1P3DX hold__i48 (.D(mic_latest[47]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[63])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i48.GSR = "DISABLED";
    FD1P3DX hold__i49 (.D(n29158), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[64])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i49.GSR = "DISABLED";
    FD1P3DX hold__i50 (.D(mic_latest[48]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[72])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i50.GSR = "DISABLED";
    FD1P3DX hold__i51 (.D(mic_latest[49]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[73])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i51.GSR = "DISABLED";
    FD1P3DX hold__i52 (.D(mic_latest[50]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[74])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i52.GSR = "DISABLED";
    FD1P3DX hold__i53 (.D(mic_latest[51]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[75])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i53.GSR = "DISABLED";
    FD1P3DX hold__i54 (.D(mic_latest[52]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[76])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i54.GSR = "DISABLED";
    FD1P3DX hold__i55 (.D(mic_latest[53]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[77])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i55.GSR = "DISABLED";
    FD1P3DX hold__i56 (.D(mic_latest[54]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[78])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i56.GSR = "DISABLED";
    FD1P3DX hold__i57 (.D(mic_latest[55]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[79])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i57.GSR = "DISABLED";
    FD1P3DX hold__i58 (.D(mic_latest[56]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[80])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i58.GSR = "DISABLED";
    FD1P3DX hold__i59 (.D(mic_latest[57]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[81])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i59.GSR = "DISABLED";
    FD1P3DX hold__i60 (.D(mic_latest[58]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[82])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i60.GSR = "DISABLED";
    FD1P3DX hold__i61 (.D(mic_latest[59]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[83])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i61.GSR = "DISABLED";
    FD1P3DX hold__i62 (.D(mic_latest[60]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[84])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i62.GSR = "DISABLED";
    FD1P3DX hold__i63 (.D(mic_latest[61]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[85])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i63.GSR = "DISABLED";
    FD1P3DX hold__i64 (.D(mic_latest[62]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[86])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i64.GSR = "DISABLED";
    FD1P3DX hold__i65 (.D(mic_latest[63]), .SP(sck_N_3140_enable_72), .CK(sck_N_3140), 
            .CD(spi_mic_cs_n_c), .Q(hold[87])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=1043, LSE_RLINE=1046 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam hold__i65.GSR = "DISABLED";
    LUT4 i2_3_lut (.A(bit_count[2]), .B(hold[64]), .C(bit_count[1]), .Z(n25785)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(24[49:58])
    defparam i2_3_lut.init = 16'h8080;
    LUT4 bit_count_6__I_0_22_inv_0_i3_1_lut (.A(bit_count[2]), .Z(out_index[2])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(20[24:41])
    defparam bit_count_6__I_0_22_inv_0_i3_1_lut.init = 16'h5555;
    PFUMX i16775 (.BLUT(n28437), .ALUT(n28438), .C0(bit_count[0]), .Z(n28439));
    LUT4 n27630_bdd_4_lut_then_3_lut (.A(hold[52]), .B(bit_count[1]), .C(hold[54]), 
         .Z(n28438)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27630_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27630_bdd_4_lut_else_3_lut (.A(hold[53]), .B(bit_count[1]), .C(hold[55]), 
         .Z(n28437)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27630_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    L6MUX21 i16505 (.D0(n27979), .D1(n27976), .SD(bit_count[4]), .Z(n27980));
    PFUMX i16503 (.BLUT(n27978), .ALUT(n27977), .C0(bit_count[5]), .Z(n27979));
    PFUMX i16500 (.BLUT(n27975), .ALUT(n27974), .C0(bit_count[5]), .Z(n27976));
    LUT4 n27980_bdd_3_lut_4_lut (.A(bit_count[5]), .B(n26979), .C(bit_count[6]), 
         .D(n27980), .Z(n27981)) /* synthesis lut_function=(!(A (C+!(D))+!A !(B (C+(D))+!B !(C+!(D))))) */ ;
    defparam n27980_bdd_3_lut_4_lut.init = 16'h4f40;
    PFUMX i16767 (.BLUT(n28425), .ALUT(n28426), .C0(bit_count[0]), .Z(n28427));
    LUT4 i15050_then_3_lut (.A(hold[2]), .B(bit_count[1]), .C(hold[0]), 
         .Z(n28468)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15050_then_3_lut.init = 16'he2e2;
    LUT4 i15050_else_3_lut (.A(hold[3]), .B(bit_count[1]), .C(hold[1]), 
         .Z(n28467)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15050_else_3_lut.init = 16'he2e2;
    LUT4 i15053_then_3_lut (.A(hold[6]), .B(bit_count[1]), .C(hold[4]), 
         .Z(n28471)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15053_then_3_lut.init = 16'he2e2;
    LUT4 i15053_else_3_lut (.A(hold[7]), .B(bit_count[1]), .C(hold[5]), 
         .Z(n28470)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15053_else_3_lut.init = 16'he2e2;
    PFUMX i16765 (.BLUT(n28422), .ALUT(n28423), .C0(bit_count[0]), .Z(n28424));
    PFUMX i16763 (.BLUT(n28419), .ALUT(n28420), .C0(bit_count[0]), .Z(n28421));
    PFUMX i16761 (.BLUT(n28416), .ALUT(n28417), .C0(bit_count[0]), .Z(n28418));
    PFUMX i16759 (.BLUT(n28413), .ALUT(n28414), .C0(bit_count[0]), .Z(n28415));
    LUT4 i15056_then_3_lut (.A(hold[10]), .B(bit_count[1]), .C(hold[8]), 
         .Z(n28477)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15056_then_3_lut.init = 16'he2e2;
    LUT4 i15056_else_3_lut (.A(hold[11]), .B(bit_count[1]), .C(hold[9]), 
         .Z(n28476)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15056_else_3_lut.init = 16'he2e2;
    LUT4 i15062_else_3_lut (.A(hold[27]), .B(bit_count[1]), .C(hold[25]), 
         .Z(n28388)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15062_else_3_lut.init = 16'he2e2;
    LUT4 n27462_bdd_4_lut_then_3_lut (.A(hold[60]), .B(bit_count[1]), .C(hold[62]), 
         .Z(n28480)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27462_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 i10415_2_lut (.A(n20[6]), .B(sck_N_3140_enable_72), .Z(n17879)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10415_2_lut.init = 16'h2222;
    LUT4 i10414_2_lut (.A(n20[5]), .B(sck_N_3140_enable_72), .Z(n17877)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10414_2_lut.init = 16'h2222;
    LUT4 i10413_2_lut (.A(n20[4]), .B(sck_N_3140_enable_72), .Z(n17875)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10413_2_lut.init = 16'h2222;
    LUT4 i10412_2_lut (.A(n20[3]), .B(sck_N_3140_enable_72), .Z(n17873)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10412_2_lut.init = 16'h2222;
    LUT4 i10411_2_lut (.A(n20[2]), .B(sck_N_3140_enable_72), .Z(n17871)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10411_2_lut.init = 16'h2222;
    LUT4 n27462_bdd_4_lut_else_3_lut (.A(hold[61]), .B(bit_count[1]), .C(hold[63]), 
         .Z(n28479)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27462_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 i10408_2_lut (.A(n20[1]), .B(sck_N_3140_enable_72), .Z(n17869)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[14] 35[8])
    defparam i10408_2_lut.init = 16'h2222;
    CCU2D add_13_7 (.A0(bit_count[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24895), .S0(n20[5]), .S1(n20[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(34[22:38])
    defparam add_13_7.INIT0 = 16'h5aaa;
    defparam add_13_7.INIT1 = 16'h5aaa;
    defparam add_13_7.INJECT1_0 = "NO";
    defparam add_13_7.INJECT1_1 = "NO";
    CCU2D add_13_5 (.A0(bit_count[3]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24894), .COUT(n24895), .S0(n20[3]), .S1(n20[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(34[22:38])
    defparam add_13_5.INIT0 = 16'h5aaa;
    defparam add_13_5.INIT1 = 16'h5aaa;
    defparam add_13_5.INJECT1_0 = "NO";
    defparam add_13_5.INJECT1_1 = "NO";
    CCU2D add_13_3 (.A0(bit_count[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24893), .COUT(n24894), .S0(n20[1]), .S1(n20[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(34[22:38])
    defparam add_13_3.INIT0 = 16'h5aaa;
    defparam add_13_3.INIT1 = 16'h5aaa;
    defparam add_13_3.INJECT1_0 = "NO";
    defparam add_13_3.INJECT1_1 = "NO";
    CCU2D add_13_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24893), .S1(bit_count_6__N_3149[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(34[22:38])
    defparam add_13_1.INIT0 = 16'hF000;
    defparam add_13_1.INIT1 = 16'h5555;
    defparam add_13_1.INJECT1_0 = "NO";
    defparam add_13_1.INJECT1_1 = "NO";
    PFUMX i16749 (.BLUT(n28398), .ALUT(n28399), .C0(bit_count[0]), .Z(n28400));
    LUT4 i15062_then_3_lut (.A(hold[26]), .B(bit_count[1]), .C(hold[24]), 
         .Z(n28389)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15062_then_3_lut.init = 16'he2e2;
    LUT4 n27444_bdd_4_lut_then_3_lut (.A(hold[32]), .B(bit_count[1]), .C(hold[34]), 
         .Z(n28486)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27444_bdd_4_lut_then_3_lut.init = 16'hb8b8;
    LUT4 n27444_bdd_4_lut_else_3_lut (.A(hold[33]), .B(bit_count[1]), .C(hold[35]), 
         .Z(n28485)) /* synthesis lut_function=(A (B+(C))+!A !(B+!(C))) */ ;
    defparam n27444_bdd_4_lut_else_3_lut.init = 16'hb8b8;
    LUT4 i15059_then_3_lut (.A(hold[14]), .B(bit_count[1]), .C(hold[12]), 
         .Z(n28489)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15059_then_3_lut.init = 16'he2e2;
    PFUMX i16743 (.BLUT(n28388), .ALUT(n28389), .C0(bit_count[0]), .Z(n28390));
    LUT4 i15059_else_3_lut (.A(hold[15]), .B(bit_count[1]), .C(hold[13]), 
         .Z(n28488)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam i15059_else_3_lut.init = 16'he2e2;
    
endmodule
//
// Verilog Description of module ws2812_stream
//

module ws2812_stream (reset_count, pll_clk, pll_clk_enable_143, ws2812_enable, 
            n10986, rgb_hold, ws2812_frame_done, GND_net, n13455, 
            n13723, n24552, rgb_data_c) /* synthesis syn_module_defined=1 */ ;
    output [14:0]reset_count;
    input pll_clk;
    input pll_clk_enable_143;
    input ws2812_enable;
    output n10986;
    input [23:0]rgb_hold;
    output ws2812_frame_done;
    input GND_net;
    output n13455;
    output n13723;
    input n24552;
    output rgb_data_c;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(108[10:17])
    
    wire enable_N_2977, n10948;
    wire [3:0]n10280;
    
    wire n13, pll_clk_enable_563, n15097;
    wire [4:0]bit_number;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(21[15:25])
    wire [4:0]n1;
    
    wire n24567, n16056, n17781, data_out_N_3037, n11079, n17730, 
        pll_clk_enable_252, n29_adj_3161;
    wire [6:0]bit_cell_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[15:29])
    
    wire n16986, pll_clk_enable_249, n17827;
    wire [6:0]n33;
    wire [14:0]reset_count_c;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[16:27])
    
    wire n28100, n28101, n21, n28099;
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(22[16:30])
    
    wire n28103, n10940, n10939, n4, n28104, n10938, n10937, n10936, 
        n10935, n10934, n10933, n10932, n10931, n10930, n10929, 
        n10949, n10928, n10950, n10951, n10952, n10953, n10954, 
        n10955, n10956, n10957, n10958, n10959, n10960, n10961, 
        n10962, n10927, n25131, n16, n20, n16_adj_3162, n18, n10926, 
        n10925, n10924, n10923, n10922, n10921, n10920, n10919, 
        n10918, n10296, n15095, n24802, n24801, n24800, n10947, 
        n25, n26_adj_3163, n22, n26016, n26058, pll_clk_enable_604, 
        data_out_N_3030, n24891, n24890, n24889, n24888, n24887, 
        n24886, n24885;
    
    FD1P3IX reset_count__i0 (.D(n10948), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i0.GSR = "DISABLED";
    LUT4 i2213_2_lut_4_lut (.A(n10280[1]), .B(n13), .C(n10280[2]), .D(ws2812_enable), 
         .Z(pll_clk_enable_563)) /* synthesis lut_function=(!(A (B (C+!(D))+!B !(D))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i2213_2_lut_4_lut.init = 16'h3a00;
    FD1S3JX state_FSM_i1 (.D(n15097), .CK(pll_clk), .PD(enable_N_2977), 
            .Q(n10280[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam state_FSM_i1.GSR = "DISABLED";
    LUT4 i13776_2_lut (.A(bit_number[1]), .B(bit_number[0]), .Z(n1[1])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13776_2_lut.init = 16'h6666;
    LUT4 i13779_2_lut (.A(bit_number[1]), .B(bit_number[0]), .Z(n24567)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13779_2_lut.init = 16'h8888;
    LUT4 i15411_2_lut_3_lut (.A(ws2812_enable), .B(n10280[2]), .C(n13), 
         .Z(n16056)) /* synthesis lut_function=(((C)+!B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam i15411_2_lut_3_lut.init = 16'hf7f7;
    LUT4 i6933_2_lut_3_lut (.A(pll_clk_enable_563), .B(n10280[2]), .C(n13), 
         .Z(n17781)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam i6933_2_lut_3_lut.init = 16'h0808;
    LUT4 i13790_2_lut_3_lut_4_lut (.A(bit_number[2]), .B(bit_number[1]), 
         .C(bit_number[0]), .D(bit_number[3]), .Z(n1[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13790_2_lut_3_lut_4_lut.init = 16'h7f80;
    LUT4 i13783_2_lut_3_lut (.A(bit_number[2]), .B(bit_number[1]), .C(bit_number[0]), 
         .Z(n1[2])) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13783_2_lut_3_lut.init = 16'h6a6a;
    LUT4 i2258_2_lut_3_lut (.A(data_out_N_3037), .B(n10280[2]), .C(n13), 
         .Z(n11079)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i2258_2_lut_3_lut.init = 16'h0808;
    LUT4 i15463_2_lut_3_lut (.A(pll_clk_enable_563), .B(n10280[2]), .C(n13), 
         .Z(n17730)) /* synthesis lut_function=(A ((C)+!B)) */ ;
    defparam i15463_2_lut_3_lut.init = 16'ha2a2;
    LUT4 i15341_2_lut_4_lut (.A(data_out_N_3037), .B(n10280[2]), .C(n13), 
         .D(pll_clk_enable_563), .Z(pll_clk_enable_252)) /* synthesis lut_function=(A (B (C (D))+!B (D))+!A (D)) */ ;
    defparam i15341_2_lut_4_lut.init = 16'hf700;
    LUT4 i2841_3_lut_4_lut (.A(n29_adj_3161), .B(data_out_N_3037), .C(n10280[2]), 
         .D(n13), .Z(n10986)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A !(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i2841_3_lut_4_lut.init = 16'h5535;
    LUT4 i2_3_lut_4_lut (.A(bit_cell_count[1]), .B(bit_cell_count[0]), .C(bit_cell_count[2]), 
         .D(bit_cell_count[4]), .Z(n16986)) /* synthesis lut_function=((((D)+!C)+!B)+!A) */ ;
    defparam i2_3_lut_4_lut.init = 16'hff7f;
    FD1P3IX bit_cell_count_3087__i6 (.D(n33[6]), .SP(pll_clk_enable_249), 
            .CD(n17827), .CK(pll_clk), .Q(bit_cell_count[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087__i6.GSR = "DISABLED";
    LUT4 reset_count_11__bdd_4_lut_16877 (.A(reset_count_c[9]), .B(reset_count_c[1]), 
         .C(reset_count_c[2]), .D(n28100), .Z(n28101)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;
    defparam reset_count_11__bdd_4_lut_16877.init = 16'hfffd;
    FD1P3IX bit_cell_count_3087__i5 (.D(n33[5]), .SP(pll_clk_enable_249), 
            .CD(n17827), .CK(pll_clk), .Q(bit_cell_count[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087__i5.GSR = "DISABLED";
    FD1P3IX bit_cell_count_3087__i4 (.D(n33[4]), .SP(pll_clk_enable_249), 
            .CD(n17827), .CK(pll_clk), .Q(bit_cell_count[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087__i4.GSR = "DISABLED";
    FD1P3IX bit_cell_count_3087__i3 (.D(n33[3]), .SP(pll_clk_enable_249), 
            .CD(n17827), .CK(pll_clk), .Q(bit_cell_count[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087__i3.GSR = "DISABLED";
    FD1P3IX bit_cell_count_3087__i2 (.D(n33[2]), .SP(pll_clk_enable_249), 
            .CD(n17827), .CK(pll_clk), .Q(bit_cell_count[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087__i2.GSR = "DISABLED";
    FD1P3IX bit_cell_count_3087__i1 (.D(n33[1]), .SP(pll_clk_enable_249), 
            .CD(n17827), .CK(pll_clk), .Q(bit_cell_count[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087__i1.GSR = "DISABLED";
    LUT4 reset_count_11__bdd_4_lut_16564 (.A(reset_count_c[5]), .B(n21), 
         .C(reset_count_c[14]), .D(reset_count_c[8]), .Z(n28099)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;
    defparam reset_count_11__bdd_4_lut_16564.init = 16'hffdf;
    LUT4 reset_count_11__bdd_3_lut_16880 (.A(reset_count_c[11]), .B(reset_count_c[6]), 
         .C(n28101), .Z(n29_adj_3161)) /* synthesis lut_function=((B+(C))+!A) */ ;
    defparam reset_count_11__bdd_3_lut_16880.init = 16'hfdfd;
    LUT4 bit_cell_count_6__bdd_4_lut_16566 (.A(bit_cell_count[5]), .B(bit_cell_count[2]), 
         .C(shift_register[23]), .D(bit_cell_count[4]), .Z(n28103)) /* synthesis lut_function=(A (((D)+!C)+!B)+!A (B+(C+!(D)))) */ ;
    defparam bit_cell_count_6__bdd_4_lut_16566.init = 16'hfe7f;
    LUT4 mux_2163_i24_3_lut_4_lut (.A(rgb_hold[23]), .B(shift_register[22]), 
         .C(n10280[2]), .D(n13), .Z(n10940)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i24_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i23_3_lut_4_lut (.A(rgb_hold[22]), .B(shift_register[21]), 
         .C(n10280[2]), .D(n13), .Z(n10939)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i23_3_lut_4_lut.init = 16'haaca;
    LUT4 bit_cell_count_6__bdd_4_lut_16567 (.A(bit_cell_count[6]), .B(bit_cell_count[3]), 
         .C(n4), .D(n28103), .Z(n28104)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam bit_cell_count_6__bdd_4_lut_16567.init = 16'hfffe;
    LUT4 mux_2163_i22_3_lut_4_lut (.A(rgb_hold[21]), .B(shift_register[20]), 
         .C(n10280[2]), .D(n13), .Z(n10938)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i22_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i21_3_lut_4_lut (.A(rgb_hold[20]), .B(shift_register[19]), 
         .C(n10280[2]), .D(n13), .Z(n10937)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i21_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i20_3_lut_4_lut (.A(rgb_hold[19]), .B(shift_register[18]), 
         .C(n10280[2]), .D(n13), .Z(n10936)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i20_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i19_3_lut_4_lut (.A(rgb_hold[18]), .B(shift_register[17]), 
         .C(n10280[2]), .D(n13), .Z(n10935)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i19_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i18_3_lut_4_lut (.A(rgb_hold[17]), .B(shift_register[16]), 
         .C(n10280[2]), .D(n13), .Z(n10934)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i18_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i17_3_lut_4_lut (.A(rgb_hold[16]), .B(shift_register[15]), 
         .C(n10280[2]), .D(n13), .Z(n10933)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i17_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i16_3_lut_4_lut (.A(rgb_hold[15]), .B(shift_register[14]), 
         .C(n10280[2]), .D(n13), .Z(n10932)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i16_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i15_3_lut_4_lut (.A(rgb_hold[14]), .B(shift_register[13]), 
         .C(n10280[2]), .D(n13), .Z(n10931)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i15_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i14_3_lut_4_lut (.A(rgb_hold[13]), .B(shift_register[12]), 
         .C(n10280[2]), .D(n13), .Z(n10930)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i14_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i13_3_lut_4_lut (.A(rgb_hold[12]), .B(shift_register[11]), 
         .C(n10280[2]), .D(n13), .Z(n10929)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i13_3_lut_4_lut.init = 16'haaca;
    FD1P3IX reset_count__i1 (.D(n10949), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i1.GSR = "DISABLED";
    LUT4 mux_2163_i12_3_lut_4_lut (.A(rgb_hold[11]), .B(shift_register[10]), 
         .C(n10280[2]), .D(n13), .Z(n10928)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i12_3_lut_4_lut.init = 16'haaca;
    FD1P3IX reset_count__i2 (.D(n10950), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i2.GSR = "DISABLED";
    FD1P3IX reset_count__i3 (.D(n10951), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i3.GSR = "DISABLED";
    FD1P3IX reset_count__i4 (.D(n10952), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i4.GSR = "DISABLED";
    FD1P3IX reset_count__i5 (.D(n10953), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i5.GSR = "DISABLED";
    FD1P3IX reset_count__i6 (.D(n10954), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i6.GSR = "DISABLED";
    FD1P3IX reset_count__i7 (.D(n10955), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i7.GSR = "DISABLED";
    FD1P3IX reset_count__i8 (.D(n10956), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i8.GSR = "DISABLED";
    FD1P3IX reset_count__i9 (.D(n10957), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i9.GSR = "DISABLED";
    FD1P3IX reset_count__i10 (.D(n10958), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i10.GSR = "DISABLED";
    FD1P3IX reset_count__i11 (.D(n10959), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i11.GSR = "DISABLED";
    FD1P3IX reset_count__i12 (.D(n10960), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i12.GSR = "DISABLED";
    FD1P3IX reset_count__i13 (.D(n10961), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[13])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i13.GSR = "DISABLED";
    FD1P3IX reset_count__i14 (.D(n10962), .SP(pll_clk_enable_143), .CD(enable_N_2977), 
            .CK(pll_clk), .Q(reset_count_c[14])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam reset_count__i14.GSR = "DISABLED";
    LUT4 mux_2163_i11_3_lut_4_lut (.A(rgb_hold[10]), .B(shift_register[9]), 
         .C(n10280[2]), .D(n13), .Z(n10927)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i11_3_lut_4_lut.init = 16'haaca;
    LUT4 i4258_4_lut (.A(n10280[0]), .B(n25131), .C(n29_adj_3161), .D(n16), 
         .Z(n15097)) /* synthesis lut_function=(A (B (C+(D))+!B (C))+!A (B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i4258_4_lut.init = 16'heca0;
    LUT4 i10_4_lut (.A(bit_number[2]), .B(n20), .C(n16_adj_3162), .D(n10280[2]), 
         .Z(n25131)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i10_4_lut.init = 16'h8000;
    LUT4 i9_4_lut (.A(bit_cell_count[3]), .B(n18), .C(bit_cell_count[6]), 
         .D(bit_cell_count[4]), .Z(n20)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i9_4_lut.init = 16'h0080;
    LUT4 i5_2_lut (.A(bit_number[0]), .B(bit_cell_count[1]), .Z(n16_adj_3162)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i5_2_lut.init = 16'h8888;
    LUT4 mux_2163_i10_3_lut_4_lut (.A(rgb_hold[9]), .B(shift_register[8]), 
         .C(n10280[2]), .D(n13), .Z(n10926)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i10_3_lut_4_lut.init = 16'haaca;
    LUT4 i7_4_lut (.A(bit_cell_count[2]), .B(bit_cell_count[5]), .C(bit_number[3]), 
         .D(bit_cell_count[0]), .Z(n18)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i7_4_lut.init = 16'h0200;
    LUT4 mux_2163_i9_3_lut_4_lut (.A(rgb_hold[8]), .B(shift_register[7]), 
         .C(n10280[2]), .D(n13), .Z(n10925)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i9_3_lut_4_lut.init = 16'haaca;
    FD1P3IX bit_number_3085__i1 (.D(n1[1]), .SP(pll_clk_enable_252), .CD(n17730), 
            .CK(pll_clk), .Q(bit_number[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam bit_number_3085__i1.GSR = "DISABLED";
    FD1P3IX bit_number_3085__i2 (.D(n1[2]), .SP(pll_clk_enable_252), .CD(n17730), 
            .CK(pll_clk), .Q(bit_number[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam bit_number_3085__i2.GSR = "DISABLED";
    FD1P3IX bit_number_3085__i3 (.D(n1[3]), .SP(pll_clk_enable_252), .CD(n17730), 
            .CK(pll_clk), .Q(bit_number[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam bit_number_3085__i3.GSR = "DISABLED";
    FD1P3IX bit_number_3085__i4 (.D(n1[4]), .SP(pll_clk_enable_252), .CD(n17730), 
            .CK(pll_clk), .Q(bit_number[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam bit_number_3085__i4.GSR = "DISABLED";
    LUT4 mux_2163_i8_3_lut_4_lut (.A(rgb_hold[7]), .B(shift_register[6]), 
         .C(n10280[2]), .D(n13), .Z(n10924)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i8_3_lut_4_lut.init = 16'haaca;
    FD1P3IX shift_register_i0_i0 (.D(rgb_hold[0]), .SP(pll_clk_enable_563), 
            .CD(n17781), .CK(pll_clk), .Q(shift_register[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i0.GSR = "DISABLED";
    LUT4 mux_2163_i7_3_lut_4_lut (.A(rgb_hold[6]), .B(shift_register[5]), 
         .C(n10280[2]), .D(n13), .Z(n10923)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i7_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i6_3_lut_4_lut (.A(rgb_hold[5]), .B(shift_register[4]), 
         .C(n10280[2]), .D(n13), .Z(n10922)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i6_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i5_3_lut_4_lut (.A(rgb_hold[4]), .B(shift_register[3]), 
         .C(n10280[2]), .D(n13), .Z(n10921)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i5_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i4_3_lut_4_lut (.A(rgb_hold[3]), .B(shift_register[2]), 
         .C(n10280[2]), .D(n13), .Z(n10920)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i4_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i3_3_lut_4_lut (.A(rgb_hold[2]), .B(shift_register[1]), 
         .C(n10280[2]), .D(n13), .Z(n10919)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i3_3_lut_4_lut.init = 16'haaca;
    LUT4 mux_2163_i2_3_lut_4_lut (.A(rgb_hold[1]), .B(shift_register[0]), 
         .C(n10280[2]), .D(n13), .Z(n10918)) /* synthesis lut_function=(A (B+((D)+!C))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam mux_2163_i2_3_lut_4_lut.init = 16'haaca;
    FD1P3IX bit_cell_count_3087__i0 (.D(n33[0]), .SP(pll_clk_enable_249), 
            .CD(n17827), .CK(pll_clk), .Q(bit_cell_count[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087__i0.GSR = "DISABLED";
    FD1P3IX bit_number_3085__i0 (.D(n1[0]), .SP(pll_clk_enable_252), .CD(n17730), 
            .CK(pll_clk), .Q(bit_number[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam bit_number_3085__i0.GSR = "DISABLED";
    FD1S3IX frame_done_53 (.D(data_out_N_3037), .CK(pll_clk), .CD(n16056), 
            .Q(ws2812_frame_done)) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam frame_done_53.GSR = "DISABLED";
    FD1S3IX state_FSM_i2 (.D(n10296), .CK(pll_clk), .CD(enable_N_2977), 
            .Q(n10280[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam state_FSM_i2.GSR = "DISABLED";
    FD1S3IX state_FSM_i3 (.D(n15095), .CK(pll_clk), .CD(enable_N_2977), 
            .Q(n10280[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam state_FSM_i3.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i1 (.D(n10918), .SP(pll_clk_enable_563), .CK(pll_clk), 
            .Q(shift_register[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i1.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i2 (.D(n10919), .SP(pll_clk_enable_563), .CK(pll_clk), 
            .Q(shift_register[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i2.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i3 (.D(n10920), .SP(pll_clk_enable_563), .CK(pll_clk), 
            .Q(shift_register[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i3.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i4 (.D(n10921), .SP(pll_clk_enable_563), .CK(pll_clk), 
            .Q(shift_register[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i4.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i5 (.D(n10922), .SP(pll_clk_enable_563), .CK(pll_clk), 
            .Q(shift_register[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i5.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i6 (.D(n10923), .SP(pll_clk_enable_563), .CK(pll_clk), 
            .Q(shift_register[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i6.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i7 (.D(n10924), .SP(pll_clk_enable_563), .CK(pll_clk), 
            .Q(shift_register[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i7.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i8 (.D(n10925), .SP(pll_clk_enable_563), .CK(pll_clk), 
            .Q(shift_register[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i8.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i9 (.D(n10926), .SP(pll_clk_enable_563), .CK(pll_clk), 
            .Q(shift_register[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i9.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i10 (.D(n10927), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i10.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i11 (.D(n10928), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i11.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i12 (.D(n10929), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i12.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i13 (.D(n10930), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[13])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i13.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i14 (.D(n10931), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[14])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i14.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i15 (.D(n10932), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[15])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i15.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i16 (.D(n10933), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[16])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i16.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i17 (.D(n10934), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[17])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i17.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i18 (.D(n10935), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[18])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i18.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i19 (.D(n10936), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[19])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i19.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i20 (.D(n10937), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[20])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i20.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i21 (.D(n10938), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[21])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i21.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i22 (.D(n10939), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[22])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i22.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i23 (.D(n10940), .SP(pll_clk_enable_563), 
            .CK(pll_clk), .Q(shift_register[23])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam shift_register_i0_i23.GSR = "DISABLED";
    LUT4 i1736_2_lut (.A(n29_adj_3161), .B(n10280[0]), .Z(n10296)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i1736_2_lut.init = 16'h4444;
    LUT4 i13797_3_lut_4_lut (.A(bit_number[2]), .B(n24567), .C(bit_number[3]), 
         .D(bit_number[4]), .Z(n1[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13797_3_lut_4_lut.init = 16'h7f80;
    LUT4 i4256_4_lut (.A(n10280[2]), .B(n10280[1]), .C(n13), .D(data_out_N_3037), 
         .Z(n15095)) /* synthesis lut_function=(A (B+(C+!(D)))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i4256_4_lut.init = 16'hecee;
    CCU2D bit_cell_count_3087_add_4_7 (.A0(bit_cell_count[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(bit_cell_count[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24802), .S0(n33[5]), .S1(n33[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087_add_4_7.INIT0 = 16'hfaaa;
    defparam bit_cell_count_3087_add_4_7.INIT1 = 16'hfaaa;
    defparam bit_cell_count_3087_add_4_7.INJECT1_0 = "NO";
    defparam bit_cell_count_3087_add_4_7.INJECT1_1 = "NO";
    CCU2D bit_cell_count_3087_add_4_5 (.A0(bit_cell_count[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(bit_cell_count[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24801), .COUT(n24802), .S0(n33[3]), 
          .S1(n33[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087_add_4_5.INIT0 = 16'hfaaa;
    defparam bit_cell_count_3087_add_4_5.INIT1 = 16'hfaaa;
    defparam bit_cell_count_3087_add_4_5.INJECT1_0 = "NO";
    defparam bit_cell_count_3087_add_4_5.INJECT1_1 = "NO";
    CCU2D bit_cell_count_3087_add_4_3 (.A0(bit_cell_count[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(bit_cell_count[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24800), .COUT(n24801), .S0(n33[1]), 
          .S1(n33[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087_add_4_3.INIT0 = 16'hfaaa;
    defparam bit_cell_count_3087_add_4_3.INIT1 = 16'hfaaa;
    defparam bit_cell_count_3087_add_4_3.INJECT1_0 = "NO";
    defparam bit_cell_count_3087_add_4_3.INJECT1_1 = "NO";
    CCU2D bit_cell_count_3087_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24800), .S1(n33[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(64[43:64])
    defparam bit_cell_count_3087_add_4_1.INIT0 = 16'hF000;
    defparam bit_cell_count_3087_add_4_1.INIT1 = 16'h0555;
    defparam bit_cell_count_3087_add_4_1.INJECT1_0 = "NO";
    defparam bit_cell_count_3087_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_4_lut (.A(reset_count_c[9]), .B(n10947), .C(n25), .D(n26_adj_3163), 
         .Z(n13455)) /* synthesis lut_function=(A (B+(C (D)))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i1_4_lut.init = 16'heccc;
    LUT4 i11_3_lut (.A(reset_count_c[2]), .B(n22), .C(reset_count_c[6]), 
         .Z(n25)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i11_3_lut.init = 16'h0404;
    LUT4 i12_4_lut (.A(n26016), .B(n26058), .C(reset_count_c[4]), .D(reset_count_c[5]), 
         .Z(n26_adj_3163)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i12_4_lut.init = 16'h0100;
    LUT4 i2842_3_lut (.A(n10280[0]), .B(n13), .C(n10280[2]), .Z(n13723)) /* synthesis lut_function=(!(A (B (C))+!A (B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i2842_3_lut.init = 16'h3a3a;
    LUT4 i8_4_lut (.A(reset_count_c[14]), .B(reset_count_c[8]), .C(reset_count_c[11]), 
         .D(reset_count_c[3]), .Z(n22)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i8_4_lut.init = 16'h0020;
    LUT4 i14999_2_lut (.A(reset_count[0]), .B(reset_count_c[13]), .Z(n26016)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14999_2_lut.init = 16'heeee;
    LUT4 enable_I_0_1_lut (.A(ws2812_enable), .Z(enable_N_2977)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(27[13:20])
    defparam enable_I_0_1_lut.init = 16'h5555;
    LUT4 i15041_4_lut (.A(reset_count_c[1]), .B(reset_count_c[10]), .C(reset_count_c[7]), 
         .D(reset_count_c[12]), .Z(n26058)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;
    defparam i15041_4_lut.init = 16'hfffb;
    LUT4 i2_4_lut (.A(bit_cell_count[3]), .B(n16986), .C(bit_cell_count[6]), 
         .D(bit_cell_count[5]), .Z(n13)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;
    defparam i2_4_lut.init = 16'hffdf;
    LUT4 i15405_4_lut (.A(n28104), .B(n13), .C(n10280[2]), .D(ws2812_enable), 
         .Z(pll_clk_enable_604)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i15405_4_lut.init = 16'h7fff;
    LUT4 i15393_2_lut (.A(bit_cell_count[1]), .B(bit_cell_count[0]), .Z(n4)) /* synthesis lut_function=(!(A (B))) */ ;
    defparam i15393_2_lut.init = 16'h7777;
    LUT4 i1754_4_lut (.A(n10280[1]), .B(data_out_N_3037), .C(n10280[2]), 
         .D(n13), .Z(data_out_N_3030)) /* synthesis lut_function=(!(A (B (C)+!B (C (D)))+!A (B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i1754_4_lut.init = 16'h0a3a;
    LUT4 i8_4_lut_adj_133 (.A(reset_count_c[3]), .B(reset_count_c[13]), 
         .C(reset_count[0]), .D(reset_count_c[4]), .Z(n21)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(37[25:52])
    defparam i8_4_lut_adj_133.init = 16'hfffe;
    LUT4 i15489_4_lut (.A(bit_number[3]), .B(bit_number[2]), .C(n16), 
         .D(bit_number[0]), .Z(data_out_N_3037)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(52[29:48])
    defparam i15489_4_lut.init = 16'h4000;
    LUT4 i2163_2_lut (.A(n10280[2]), .B(n13), .Z(n10947)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i2163_2_lut.init = 16'h2222;
    LUT4 i1_2_lut (.A(bit_number[4]), .B(bit_number[1]), .Z(n16)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut.init = 16'h8888;
    CCU2D add_5_15 (.A0(reset_count_c[13]), .B0(n13455), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count_c[14]), .B1(n13455), .C1(GND_net), 
          .D1(GND_net), .CIN(n24891), .S0(n10961), .S1(n10962));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_15.INIT0 = 16'hd222;
    defparam add_5_15.INIT1 = 16'hd222;
    defparam add_5_15.INJECT1_0 = "NO";
    defparam add_5_15.INJECT1_1 = "NO";
    CCU2D add_5_13 (.A0(reset_count_c[11]), .B0(n13455), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count_c[12]), .B1(n13455), .C1(GND_net), 
          .D1(GND_net), .CIN(n24890), .COUT(n24891), .S0(n10959), .S1(n10960));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_13.INIT0 = 16'hd222;
    defparam add_5_13.INIT1 = 16'hd222;
    defparam add_5_13.INJECT1_0 = "NO";
    defparam add_5_13.INJECT1_1 = "NO";
    CCU2D add_5_11 (.A0(reset_count_c[9]), .B0(n13455), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count_c[10]), .B1(n13455), .C1(GND_net), 
          .D1(GND_net), .CIN(n24889), .COUT(n24890), .S0(n10957), .S1(n10958));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_11.INIT0 = 16'hd222;
    defparam add_5_11.INIT1 = 16'hd222;
    defparam add_5_11.INJECT1_0 = "NO";
    defparam add_5_11.INJECT1_1 = "NO";
    CCU2D add_5_9 (.A0(reset_count_c[7]), .B0(n13455), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count_c[8]), .B1(n13455), .C1(GND_net), .D1(GND_net), 
          .CIN(n24888), .COUT(n24889), .S0(n10955), .S1(n10956));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_9.INIT0 = 16'hd222;
    defparam add_5_9.INIT1 = 16'hd222;
    defparam add_5_9.INJECT1_0 = "NO";
    defparam add_5_9.INJECT1_1 = "NO";
    CCU2D add_5_7 (.A0(reset_count_c[5]), .B0(n13455), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count_c[6]), .B1(n13455), .C1(GND_net), .D1(GND_net), 
          .CIN(n24887), .COUT(n24888), .S0(n10953), .S1(n10954));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_7.INIT0 = 16'hd222;
    defparam add_5_7.INIT1 = 16'hd222;
    defparam add_5_7.INJECT1_0 = "NO";
    defparam add_5_7.INJECT1_1 = "NO";
    CCU2D add_5_5 (.A0(reset_count_c[3]), .B0(n13455), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count_c[4]), .B1(n13455), .C1(GND_net), .D1(GND_net), 
          .CIN(n24886), .COUT(n24887), .S0(n10951), .S1(n10952));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_5.INIT0 = 16'hd222;
    defparam add_5_5.INIT1 = 16'hd222;
    defparam add_5_5.INJECT1_0 = "NO";
    defparam add_5_5.INJECT1_1 = "NO";
    CCU2D add_5_3 (.A0(reset_count_c[1]), .B0(n13455), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count_c[2]), .B1(n13455), .C1(GND_net), .D1(GND_net), 
          .CIN(n24885), .COUT(n24886), .S0(n10949), .S1(n10950));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_3.INIT0 = 16'hd222;
    defparam add_5_3.INIT1 = 16'hd222;
    defparam add_5_3.INJECT1_0 = "NO";
    defparam add_5_3.INJECT1_1 = "NO";
    CCU2D add_5_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n11079), .B1(n10986), .C1(n24552), .D1(GND_net), .COUT(n24885), 
          .S1(n10948));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(38[41:60])
    defparam add_5_1.INIT0 = 16'hF000;
    defparam add_5_1.INIT1 = 16'he1e1;
    defparam add_5_1.INJECT1_0 = "NO";
    defparam add_5_1.INJECT1_1 = "NO";
    LUT4 i13774_1_lut (.A(bit_number[0]), .Z(n1[0])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(59[43:60])
    defparam i13774_1_lut.init = 16'h5555;
    LUT4 reset_count_11__bdd_4_lut_16565 (.A(reset_count_c[12]), .B(reset_count_c[10]), 
         .C(reset_count_c[7]), .D(n28099), .Z(n28100)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;
    defparam reset_count_11__bdd_4_lut_16565.init = 16'hfffb;
    LUT4 i15494_4_lut_3_lut (.A(n13), .B(n10280[2]), .C(pll_clk_enable_249), 
         .Z(n17827)) /* synthesis lut_function=(!(A (B+!(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(34[13] 77[20])
    defparam i15494_4_lut_3_lut.init = 16'h7070;
    LUT4 i1_3_lut (.A(ws2812_enable), .B(n10280[2]), .C(n10280[1]), .Z(pll_clk_enable_249)) /* synthesis lut_function=(A (B+(C))) */ ;
    defparam i1_3_lut.init = 16'ha8a8;
    OFS1P3IX data_out_reg_50 (.D(data_out_N_3030), .SP(pll_clk_enable_604), 
            .SCLK(pll_clk), .CD(enable_N_2977), .Q(rgb_data_c)) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=1036, LSE_RLINE=1042 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[12] 79[8])
    defparam data_out_reg_50.GSR = "DISABLED";
    
endmodule
//
// Verilog Description of module umh_channel_ram18
//

module umh_channel_ram18 (\ev_sum[2] , staging_rd_page_N_2728, ev_in_clear, 
            spi_channel_index, staging_wr_page, ev_ch, staging_rd_page, 
            spi1_mosi_c_0, \spi_rx_shift[0] , \spi_rx_shift[1] , \spi_rx_shift[2] , 
            \spi_rx_shift[3] , \spi_rx_shift[4] , \spi_rx_shift[5] , \spi_rx_shift[6] , 
            \staging_q[0] , \staging_q[3] , \staging_q[4] , \staging_q[5] , 
            \staging_q[6] , \staging_q[7] , spi1_sck_c, spi_payload_byte, 
            VCC_net, pll_clk, GND_net, n15, n26133, ev_fall_N_2693, 
            n25667, \ev_sum[1] , n26099, n25725, \build_level[3] , 
            build_mod, \event_level[3] , \build_level[2] , \event_level[2] , 
            \build_level[1] , \event_level[1] ) /* synthesis syn_module_defined=1 */ ;
    input \ev_sum[2] ;
    input staging_rd_page_N_2728;
    input ev_in_clear;
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
    output \staging_q[0] ;
    output \staging_q[3] ;
    output \staging_q[4] ;
    output \staging_q[5] ;
    output \staging_q[6] ;
    output \staging_q[7] ;
    input spi1_sck_c;
    input spi_payload_byte;
    input VCC_net;
    input pll_clk;
    input GND_net;
    output n15;
    input n26133;
    input ev_fall_N_2693;
    output n25667;
    input \ev_sum[1] ;
    input n26099;
    output n25725;
    input \build_level[3] ;
    input build_mod;
    output \event_level[3] ;
    input \build_level[2] ;
    output \event_level[2] ;
    input \build_level[1] ;
    output \event_level[1] ;
    
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(86[24:32])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(108[10:17])
    wire [7:0]staging_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[17:26])
    
    wire n8, n14, n10, n8_adj_3160;
    
    LUT4 i22_4_lut (.A(\ev_sum[2] ), .B(staging_q[2]), .C(staging_rd_page_N_2728), 
         .D(ev_in_clear), .Z(n8)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(331[17:28])
    defparam i22_4_lut.init = 16'hcac0;
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
          .RSTB(GND_net), .DOB0(\staging_q[0] ), .DOB1(staging_q[1]), 
          .DOB2(staging_q[2]), .DOB3(\staging_q[3] ), .DOB4(\staging_q[4] ), 
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
    LUT4 i7_4_lut (.A(staging_q[1]), .B(n14), .C(n10), .D(\staging_q[6] ), 
         .Z(n15)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i7_4_lut.init = 16'hfffe;
    LUT4 i6_4_lut (.A(\staging_q[5] ), .B(\staging_q[0] ), .C(\staging_q[3] ), 
         .D(\staging_q[4] ), .Z(n14)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i6_4_lut.init = 16'hfffe;
    LUT4 i2_2_lut (.A(staging_q[2]), .B(\staging_q[7] ), .Z(n10)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i2_2_lut.init = 16'heeee;
    PFUMX i21 (.BLUT(n8), .ALUT(n26133), .C0(ev_fall_N_2693), .Z(n25667));
    LUT4 i22_4_lut_adj_131 (.A(\ev_sum[1] ), .B(staging_q[1]), .C(staging_rd_page_N_2728), 
         .D(ev_in_clear), .Z(n8_adj_3160)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(331[17:28])
    defparam i22_4_lut_adj_131.init = 16'hcac0;
    PFUMX i21_adj_132 (.BLUT(n8_adj_3160), .ALUT(n26099), .C0(ev_fall_N_2693), 
          .Z(n25725));
    LUT4 i9848_4_lut (.A(\staging_q[3] ), .B(\build_level[3] ), .C(build_mod), 
         .D(n15), .Z(\event_level[3] )) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(364[17:26])
    defparam i9848_4_lut.init = 16'hca0a;
    LUT4 i9840_4_lut (.A(staging_q[2]), .B(\build_level[2] ), .C(build_mod), 
         .D(n15), .Z(\event_level[2] )) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(364[17:26])
    defparam i9840_4_lut.init = 16'hca0a;
    LUT4 i9856_4_lut (.A(staging_q[1]), .B(\build_level[1] ), .C(build_mod), 
         .D(n15), .Z(\event_level[1] )) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(364[17:26])
    defparam i9856_4_lut.init = 16'hca0a;
    
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
