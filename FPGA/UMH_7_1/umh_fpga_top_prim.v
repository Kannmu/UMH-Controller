// Verilog netlist produced by program LSE :  version Diamond (64-bit) 3.13.0.56.2
// Netlist written on Fri Sep 11 12:39:55 2026
//
// Verilog Description of module umh_fpga_top
//

module umh_fpga_top (fpga_clk_8m, fpga_cs_n, spi1_sck, spi1_mosi, spi1_miso, 
            us_tx, rgb_data, mic_clk, mic_data_0, mic_data_1, spi_mic_cs_n, 
            spi_mic_sck, spi_mic_miso) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(59[8:20])
    input fpga_clk_8m;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(60[24:35])
    input fpga_cs_n;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:33])
    input spi1_sck;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    input spi1_mosi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(63[24:33])
    output spi1_miso;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:33])
    output [83:0]us_tx;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    output rgb_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(66[24:32])
    output mic_clk;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(67[24:31])
    input mic_data_0;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(68[24:34])
    input mic_data_1;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(69[24:34])
    input spi_mic_cs_n;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(70[24:36])
    input spi_mic_sck;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(71[24:35])
    output spi_mic_miso;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(72[24:36])
    
    wire fpga_clk_8m_c /* synthesis is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(60[24:35])
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    wire spi_mic_sck_c /* synthesis is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(71[24:35])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[10:17])
    wire spi1_sck_N_416 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(127[17:33])
    wire sck_N_3047 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    
    wire GND_net, VCC_net, fpga_cs_n_c, spi1_mosi_c_0, spi1_miso_c, 
        us_tx_c_83, us_tx_c_82, us_tx_c_81, us_tx_c_80, us_tx_c_79, 
        us_tx_c_78, us_tx_c_77, us_tx_c_76, us_tx_c_75, us_tx_c_74, 
        us_tx_c_73, us_tx_c_72, us_tx_c_71, us_tx_c_70, us_tx_c_69, 
        us_tx_c_68, us_tx_c_67, us_tx_c_66, us_tx_c_65, us_tx_c_64, 
        us_tx_c_63, us_tx_c_62, us_tx_c_61, us_tx_c_60, us_tx_c_59, 
        us_tx_c_58, us_tx_c_57, us_tx_c_56, us_tx_c_55, us_tx_c_54, 
        us_tx_c_53, us_tx_c_52, us_tx_c_51, us_tx_c_50, us_tx_c_49, 
        us_tx_c_48, us_tx_c_47, us_tx_c_46, us_tx_c_45, us_tx_c_44, 
        us_tx_c_43, us_tx_c_42, us_tx_c_41, us_tx_c_40, us_tx_c_39, 
        us_tx_c_38, us_tx_c_37, us_tx_c_36, us_tx_c_35, us_tx_c_34, 
        us_tx_c_33, us_tx_c_32, us_tx_c_31, us_tx_c_30, us_tx_c_29, 
        us_tx_c_28, us_tx_c_27, us_tx_c_26, us_tx_c_25, us_tx_c_24, 
        us_tx_c_23, us_tx_c_22, us_tx_c_21, us_tx_c_20, us_tx_c_19, 
        us_tx_c_18, us_tx_c_17, us_tx_c_16, us_tx_c_15, us_tx_c_14, 
        us_tx_c_13, us_tx_c_12, us_tx_c_11, us_tx_c_10, us_tx_c_9, 
        us_tx_c_8, us_tx_c_7, us_tx_c_6, us_tx_c_5, us_tx_c_4, us_tx_c_3, 
        us_tx_c_2, us_tx_c_1, us_tx_c_0, rgb_data_c, mic_clk_c, mic_data_0_c, 
        mic_data_1_c, spi_mic_cs_n_c, spi_mic_miso_c, pll_locked, pll_feedback;
    wire [7:0]spi_rx_shift;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(118[17:29])
    wire [7:0]spi_command;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(118[31:42])
    wire [7:0]spi_version;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(118[44:55])
    wire [7:0]spi_phase_pending;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(118[57:74])
    wire [2:0]spi_bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(119[17:30])
    wire [15:0]spi_byte_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[17:31])
    
    wire n14026;
    wire [15:0]spi_extension_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[51:71])
    wire [31:0]spi_frame_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[17:35])
    wire [31:0]spi_expected_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[37:56])
    wire [31:0]accepted_sequence_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[58:79])
    wire [6:0]spi_channel_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(122[17:34])
    wire [1:0]spi_channel_field;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(123[17:34])
    wire [87:0]spi_bitmap;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(124[17:27])
    
    wire frame_toggle_spi, stop_toggle_spi, invalid_frame_spi, ws2812_toggle_spi, 
        pll_clk_enable_314;
    wire [95:0]rgb_values;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(126[17:27])
    wire [6:0]status_bit_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(127[17:33])
    
    wire spi_write, ws2812_enable;
    wire [23:0]phase_frac;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(153[17:27])
    
    wire phase_step_s1;
    wire [7:0]global_phase_s2;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[17:32])
    
    wire phase_step_s2, wrap_s2;
    wire [8:0]run_addr_s3;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(158[17:28])
    
    wire phase_step_s3, swap_now_s3, phase_step_s4, swap_now_s4;
    wire [83:0]ev_run_hold_s5;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(163[17:31])
    
    wire phase_step_s5, swap_now_s5;
    wire [24:0]phase_frac_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[17:31])
    wire [31:0]fpga_time;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(169[17:26])
    wire [6:0]time_divider;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(170[17:29])
    wire [6:0]mic_divider;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(173[17:28])
    
    wire mic_tick;
    wire [15:0]mic_shift_0_l;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(176[17:30])
    wire [15:0]mic_shift_0_r;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(176[32:45])
    wire [15:0]mic_shift_1_l;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:30])
    wire [15:0]mic_shift_1_r;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[32:45])
    wire [4:0]mic_sample_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(178[17:33])
    wire [63:0]mic_latest;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[17:27])
    wire [3:0]ev_state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    wire [7:0]ev_clear_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(194[17:30])
    
    wire n10;
    wire [6:0]ev_ch;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(196[17:22])
    wire [83:0]ev_bit;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(197[17:23])
    wire [83:0]init_shadow;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(197[25:36])
    wire [7:0]build_phase;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(198[17:28])
    wire [8:0]build_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(199[17:26])
    wire [6:0]staging_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(201[17:32])
    
    wire frame_req, swap_pending, active_bank;
    wire [15:0]staging_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[17:26])
    
    wire n22404;
    wire [7:0]ev_rd_slot;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[17:27])
    wire [8:0]event_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[17:30])
    
    wire ev_we;
    wire [8:0]ev_wr_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(223[17:27])
    wire [83:0]ev_rd_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[17:27])
    wire [83:0]ev_rd_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(227[17:27])
    wire [83:0]ev_wr_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[17:27])
    
    wire frame_toggle_meta, frame_toggle_sync, frame_toggle_seen, stop_toggle_meta, 
        stop_toggle_sync, stop_toggle_seen, ws2812_toggle_meta, ws2812_toggle_sync, 
        ws2812_toggle_seen, invalid_frame_meta;
    wire [31:0]accepted_sequence_meta;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(246[17:39])
    wire [31:0]accepted_sequence_sync;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(246[41:63])
    wire [31:0]pending_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(247[17:33])
    wire [31:0]accepted_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(247[35:52])
    wire [15:0]update_flags_meta;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(248[17:34])
    wire [15:0]update_flags_sync;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(248[36:53])
    wire [3:0]frame_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(249[17:29])
    
    wire n40, n39, n38, n37, n36, n35, n34, n13152, n13158, 
        n13164, n13039, n13140, n13146, n23173, n13134, n40_adj_3162, 
        n39_adj_3163, n38_adj_3164, n37_adj_3165, n36_adj_3166, n35_adj_3167, 
        n34_adj_3168;
    wire [95:0]rgb_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(250[17:25])
    
    wire cs_meta, cs_sync, cs_sync_d, n22519;
    wire [127:0]status_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(257[18:29])
    
    wire cs_fall;
    wire [15:0]fifo_credit_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(265[17:33])
    wire [15:0]expected_next;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[17:30])
    
    wire frame_end, fpga_cs_n_N_2559;
    wire [8:0]ev_wr_addr_8__N_912;
    
    wire n22402, n22351, n22367, n22366, n22401, n22400, n22365, 
        n22426;
    wire [15:0]status_flags_wire_15__N_1385;
    wire [15:0]status_flags_wire_15__N_1401;
    wire [6:0]spi1_miso_N_2513;
    
    wire n100, n23857, n8, n23928;
    wire [15:0]expected_next_15__N_1458;
    wire [15:0]expected_next_15__N_1417;
    
    wire n22435;
    wire [31:0]frame_end_N_2614;
    wire [15:0]rd_data_15__N_2649;
    
    wire frame_end_N_2613, spi1_sck_c_enable_110, n12, n13051, n23924, 
        n38_adj_3169, n39_adj_3170, n40_adj_3171, n41, n42, n43, 
        n44, n45, n63, n142, n141, n13045;
    wire [15:0]spi_byte_count_15__N_1694;
    
    wire n24241, ws2812_toggle_spi_N_2554, n12717, n12723, stop_toggle_spi_N_2535, 
        n12729, n12735, n12741, n12747, frame_toggle_spi_N_2524, n12753, 
        n12759, n12765, n12771, n12777, invalid_frame_spi_N_2542, 
        n12783, n12789, n12795, n12801, n12807, n12813, n13122, 
        n13116, n13128, n13110, n40_adj_3172, n39_adj_3173, n38_adj_3174, 
        n37_adj_3175, n36_adj_3176, n35_adj_3177, n34_adj_3178, n13093, 
        n13087, n13033, n13099, n15, n14, n13081, n13027, n12819, 
        n12825, n12831, n12837, n12843, n13021, spi1_sck_c_enable_236, 
        wrap_s2_N_2572, n15620, spi1_sck_c_enable_35, spi1_sck_c_enable_38;
    wire [8:0]run_addr_s3_8__N_425;
    
    wire n13009, n23858, n1, spi1_sck_c_enable_182, n55, spi1_sck_c_enable_237, 
        n60, n12849, n12855, pll_clk_enable_451, pll_clk_enable_360, 
        n12861, spi1_sck_c_enable_238, n12867, n12873, pll_clk_enable_730, 
        n13015, n13208, n13176, n13214, n13182, n13188, n13220, 
        n13194, n13226, n4, pll_clk_enable_215, pll_clk_enable_5, 
        n12879, n12885, n12891, n12897, n12903, n12909, n12915, 
        n12921, n12927, n12933, n12939, n12945, n12951, n12957, 
        n12969, n12975, n12981, n12987, n15703, n12997, n13003, 
        n13075, n9817;
    wire [3:0]frame_settle_3__N_1832;
    
    wire n23874, n13069, n13063;
    wire [31:0]accepted_sequence_31__N_1122;
    
    wire swap_pending_N_2586, n48, pll_clk_enable_737, spi1_sck_c_enable_95, 
        spi1_sck_c_enable_102, pll_clk_enable_11, n4_adj_3179, n140, 
        n139;
    wire [7:0]ev_clear_addr_7__N_2237;
    
    wire ev_clear_done_N_2575;
    wire [3:0]ev_state_3__N_1940;
    
    wire n12_adj_3180;
    wire [8:0]build_sum_8__N_2053;
    wire [6:0]ev_ch_6__N_1948;
    wire [3:0]ev_state_3__N_697;
    wire [6:0]ev_ch_6__N_709;
    
    wire n7, n18, n7_adj_3181, pll_clk_enable_726, n16, n24239, 
        n23013, n23871, n23923, n14_adj_3182;
    wire [6:0]staging_rd_addr_6__N_901;
    
    wire n23094, n23074, n23070, n24237, n10188, n10187, n10186, 
        n10185, n10184, n10183, n10182, n10181, n10180, n10179, 
        n10178, n10177, n10176, n10175, n10174, n10173, n10172, 
        n10171, n10170, n10169, n10168, n10167, n10166, n10165, 
        n10164, n10163, n10162, n10161, n10160, n10159, n10158, 
        n10157, n10156, n10155, n91, n10153, n10152, n10151, n10150, 
        n10149, n10148, n10147, n10146, n10145, n10144, n10143, 
        n10142, n10141, n30, n29, n28, n27, n26, n13057, n20, 
        n19, n18_adj_3183, n23870, n23869, spi1_sck_c_enable_103, 
        n23017, n23006, n6, pll_clk_enable_22, n9, n174, pll_clk_enable_739, 
        n12_adj_3184, mic_tick_N_2573, n23441, n23439, n13200, n23437, 
        n40_adj_3185, n39_adj_3186, n38_adj_3187, n37_adj_3188, n36_adj_3189, 
        n35_adj_3190, n34_adj_3191, n6_adj_3192, n20_adj_3193, n23435, 
        n23427, n138, n137, n165, n164, n163, n162, n161, n23484, 
        n23423, n18_adj_3194, spi1_sck_c_enable_117, n16_adj_3195, n22264, 
        n23419, n22425, n136, n23417, n23413, n23411, mic_clk_N_2520, 
        n22398, n19010, n23407, n23405, n23146, spi1_sck_c_enable_63, 
        pll_clk_enable_574, pll_clk_enable_483, n4_adj_3196, n23401, 
        n8_adj_3197, n23397, n18471, n166, n10140, spi1_sck_N_416_enable_7, 
        n6_adj_3198, n5, n23393, n23389, spi1_sck_c_enable_199, n9_adj_3199, 
        n14339, n14336, spi1_sck_c_enable_230;
    wire [1:0]state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[16:30])
    
    wire spi1_sck_c_enable_29, pll_clk_enable_735, pll_clk_enable_231, 
        n160, n159, n158, n157, n156, n155, n154, n153, n152, 
        n151, n150, n149, n148, n147, n146, n145, n144, n143, 
        n135, n134;
    wire [23:0]shift_register_23__N_2910;
    
    wire n15512, pll_clk_enable_743, pll_clk_enable_437, n23379, n23377, 
        n23375, n23925, n23371, n23367, n23363, n23361, n15593, 
        n23359, n23357, n6_adj_3200, n23349, n14_adj_3201, n22397, 
        n23143, n22424, n23339, n23335, n10_adj_3202, pll_clk_enable_280, 
        pll_clk_enable_181, n14617, pll_clk_enable_474, n22496, spi1_sck_c_enable_80, 
        pll_clk_enable_639, n23319, n23315, n23134, pll_clk_enable_744, 
        pll_clk_enable_109, n26_adj_3203, n25, n24, n22423, n22422, 
        n22421, n22420, spi1_sck_c_enable_239, n22419, n23289, n22418, 
        n22396, n24221, n22417, n14467, n22394, n23283, n22393, 
        n23035, n22416, n22392, n22391, n22415, n22390, n22389, 
        n22, n23281, pll_clk_enable_608, n22414, n22413, n22412, 
        n22411, n18490, n22410, n22388, n22364, n22363, n22387, 
        n22409, n22386, n22362, n22361, n22385, n22408, n23145, 
        n22407, n22360, n22352, n22359, n22348, n22373, n22406, 
        n22372, n12963, n22358, n23711, n23710, n22355, n22353, 
        n22354, n22357, n22349, n22266, n22268, n22347, n18_adj_3204, 
        n23271, n24216, spi1_sck_c_enable_71, n20519, n18460, spi1_sck_c_enable_88, 
        n23265, n22371, n11583, n11571, n11569, n11567, n11565, 
        n11563, n11561, n11559, n11557, n11555, n11553, n11551, 
        n11549, n11547, n11545, n11543, n11541, n11539, n11537, 
        n23261, n11535, n11533, n11531, n11529, n11527, n11525, 
        n11523, n11521, n11519, n11517, n11515, n11513, n11511, 
        n11509, n11507, n11505, n11503, n11501, n11499, n11497, 
        n11495, n11493, n11491, n11489, n11487, n11485, n11483, 
        n11481, n11479, n11477, n11475, n11473, n11471, n11469, 
        n11467, n11465, n11463, n11461, n11459, n11457, n11455, 
        n11453, n11451, n11449, n11447, n11445, n11443, n11441, 
        n11439, n11437, n11435, n11433, n11431, n11429, n11427, 
        n11425, n11423, n11421, n11419, n11417, n11415, n11413, 
        n11411, n11409, n11407, n23257, n23514, n23513, n23512, 
        n11384, n23511, n23510, n23509, n23508, n23253, n23507, 
        n23506, n23505, n23504, n23503, n23502, n23501, n23500, 
        n23499, n23498, n23497, n23496, n23495, n23494, n23493, 
        n23492, n23491, n23490, n23489, n23488, n23487, pll_clk_enable_728, 
        n23486, n23485, n23856, spi1_sck_c_enable_43, n23483, n23482, 
        pll_clk_enable_16, n23481, n20277, n23480, n23479, n20287, 
        n20289, n20291, n1_adj_3205, n23932, n20303, n23931, n23239, 
        n23478, n85, n23477, n133, n23235, n23860, n23476, n23475, 
        n23922, n23225, n7_adj_3206, n23474, n23473, n23472, n23471, 
        n23854, n23223, n23221, n23470, n23469, n23929, n23468, 
        n23467, n23921, n23466, n23920, n10778, n23465, n23919, 
        n23918, n23464, n23917, pll_clk_enable_117, n23916, n23463, 
        n23915, n23914, n23927, n23913, n23912, n23462, n23461, 
        n23209, n23062, n23911, n23910, n23205, n23909, n23460, 
        n23908, n23907, n23906, n23905, n23197, n23902, n23459, 
        n23899, pll_clk_enable_25, n23458, n23457, n14_adj_3207, n23456, 
        pll_clk_enable_3, n23896, pll_clk_enable_27, n23892, n23455, 
        n23926, n14170, n23454, n23452, n23451, n23450, n23889, 
        n23888, n22579, n23448, pll_clk_enable_43, n22370, spi1_sck_c_enable_197, 
        n23177, pll_clk_enable_345, n22265, n23883, n23882, n23881, 
        n22356, n22369, n22267, n23453, n13582, n14164, pll_clk_enable_10, 
        n13591, n23877, n22346, n22350, n22368, n22405;
    
    VHI i2 (.Z(VCC_net));
    INV i15040 (.A(spi_mic_sck_c), .Z(sck_N_3047));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(71[24:35])
    LUT4 i1_3_lut_4_lut (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[8]), 
         .D(ev_bit[8]), .Z(ev_wr_data[8])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut.init = 16'h7770;
    LUT4 i14493_2_lut (.A(spi_bitmap[65]), .B(spi_bitmap[76]), .Z(n23283)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14493_2_lut.init = 16'h8888;
    FD1P3AX spi_bitmap_i0_i21 (.D(spi_bitmap[13]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i21.GSR = "ENABLED";
    EHXPLLJ fpga_pll_i (.CLKI(fpga_clk_8m_c), .CLKFB(pll_feedback), .PHASESEL0(GND_net), 
            .PHASESEL1(GND_net), .PHASEDIR(GND_net), .PHASESTEP(GND_net), 
            .LOADREG(GND_net), .STDBY(GND_net), .PLLWAKESYNC(GND_net), 
            .RST(GND_net), .RESETC(GND_net), .RESETD(GND_net), .RESETM(GND_net), 
            .ENCLKOP(GND_net), .ENCLKOS(GND_net), .ENCLKOS2(GND_net), 
            .ENCLKOS3(GND_net), .PLLCLK(GND_net), .PLLRST(GND_net), .PLLSTB(GND_net), 
            .PLLWE(GND_net), .PLLDATI0(GND_net), .PLLDATI1(GND_net), .PLLDATI2(GND_net), 
            .PLLDATI3(GND_net), .PLLDATI4(GND_net), .PLLDATI5(GND_net), 
            .PLLDATI6(GND_net), .PLLDATI7(GND_net), .PLLADDR0(GND_net), 
            .PLLADDR1(GND_net), .PLLADDR2(GND_net), .PLLADDR3(GND_net), 
            .PLLADDR4(GND_net), .CLKOP(pll_clk), .LOCK(pll_locked), .CLKINTFB(pll_feedback)) /* synthesis syn_instantiated=1 */ ;
    defparam fpga_pll_i.CLKI_DIV = 1;
    defparam fpga_pll_i.CLKFB_DIV = 16;
    defparam fpga_pll_i.CLKOP_DIV = 4;
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
    defparam fpga_pll_i.CLKOP_CPHASE = 3;
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
    FD1S3AY cs_sync_451 (.D(cs_meta), .CK(pll_clk), .Q(cs_sync)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(259[12] 263[8])
    defparam cs_sync_451.GSR = "DISABLED";
    CCU2D global_phase_s2_1256_add_4_3 (.A0(global_phase_s2[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22420), .COUT(n22421), .S0(n44), 
          .S1(n43));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256_add_4_3.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1256_add_4_3.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1256_add_4_3.INJECT1_0 = "NO";
    defparam global_phase_s2_1256_add_4_3.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_643_13 (.A0(spi_rx_shift[4]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[5]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22390), .COUT(n22391), .S0(expected_next[13]), 
          .S1(expected_next[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[33] 288[86])
    defparam expected_next_15__I_0_643_13.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_643_13.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_643_13.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_643_13.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_643_11 (.A0(spi_rx_shift[2]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[3]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22389), .COUT(n22390), .S0(expected_next[11]), 
          .S1(expected_next[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[33] 288[86])
    defparam expected_next_15__I_0_643_11.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_643_11.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_643_11.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_643_11.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_643_9 (.A0(spi_rx_shift[0]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[1]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22388), .COUT(n22389), .S0(expected_next[9]), 
          .S1(expected_next[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[33] 288[86])
    defparam expected_next_15__I_0_643_9.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_643_9.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_643_9.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_643_9.INJECT1_1 = "NO";
    CCU2D global_phase_s2_1256_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(global_phase_s2[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22420), .S1(n45));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256_add_4_1.INIT0 = 16'hF000;
    defparam global_phase_s2_1256_add_4_1.INIT1 = 16'h0555;
    defparam global_phase_s2_1256_add_4_1.INJECT1_0 = "NO";
    defparam global_phase_s2_1256_add_4_1.INJECT1_1 = "NO";
    LUT4 accepted_sequence_31__I_0_i25_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[24]), .D(pending_sequence[24]), 
         .Z(accepted_sequence_31__N_1122[24])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i25_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_4_lut_adj_61 (.A(spi_byte_count[2]), .B(n23902), .C(spi_byte_count[5]), 
         .D(spi_byte_count[4]), .Z(n14336)) /* synthesis lut_function=(!(A+((C (D)+!C !(D))+!B))) */ ;
    defparam i1_3_lut_4_lut_adj_61.init = 16'h0440;
    LUT4 i1_2_lut_3_lut (.A(spi_byte_count[0]), .B(n1), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_88)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i1_2_lut_3_lut.init = 16'h8080;
    LUT4 i1_3_lut (.A(n14467), .B(init_shadow[42]), .C(ev_bit[42]), .Z(n12963)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut.init = 16'hecec;
    LUT4 mux_673_Mux_22_i3_4_lut (.A(rgb_hold[6]), .B(n23860), .C(state[1]), 
         .D(shift_register[21]), .Z(shift_register_23__N_2910[22])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_673_Mux_22_i3_4_lut.init = 16'hca0a;
    FD1S3AY cs_sync_d_452 (.D(cs_sync), .CK(pll_clk), .Q(cs_sync_d)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(259[12] 263[8])
    defparam cs_sync_d_452.GSR = "DISABLED";
    FD1S3AX spi_rx_shift_i1 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(spi_rx_shift[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_rx_shift_i1.GSR = "ENABLED";
    LUT4 i14505_3_lut_4_lut (.A(n23896), .B(pll_clk_enable_10), .C(update_flags_sync[1]), 
         .D(pll_clk_enable_3), .Z(n20277)) /* synthesis lut_function=(A ((D)+!C)+!A (B ((D)+!C)+!B (D))) */ ;
    defparam i14505_3_lut_4_lut.init = 16'hff0e;
    LUT4 i1494_2_lut_3_lut_4_lut (.A(ev_ch[2]), .B(n23908), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(ev_ch_6__N_1948[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(518[27:39])
    defparam i1494_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i4710_2_lut_4_lut (.A(n23920), .B(n22435), .C(n23909), .D(pll_clk_enable_3), 
         .Z(n13591)) /* synthesis lut_function=(A (B (D)+!B (C+(D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i4710_2_lut_4_lut.init = 16'hff20;
    LUT4 i2_3_lut_4_lut (.A(spi_byte_count[13]), .B(n23874), .C(n55), 
         .D(n48), .Z(n23070)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i2_3_lut_4_lut.init = 16'h0010;
    LUT4 i6705_2_lut_4_lut (.A(n23920), .B(n22435), .C(n23909), .D(n23910), 
         .Z(n15512)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i6705_2_lut_4_lut.init = 16'h0020;
    LUT4 i1_2_lut_4_lut (.A(n23920), .B(n22435), .C(n23909), .D(n23910), 
         .Z(pll_clk_enable_737)) /* synthesis lut_function=(A (B (D)+!B (C+(D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i1_2_lut_4_lut.init = 16'hff20;
    LUT4 i13634_3_lut_4_lut (.A(mic_sample_count[2]), .B(n23907), .C(mic_sample_count[3]), 
         .D(mic_sample_count[4]), .Z(n26)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[37:60])
    defparam i13634_3_lut_4_lut.init = 16'h7f80;
    LUT4 i1_2_lut_3_lut_4_lut (.A(spi_bit_count[2]), .B(n23912), .C(n18471), 
         .D(n23143), .Z(n4_adj_3179)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut.init = 16'h0008;
    FD1P3AX spi_command_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_117), 
            .CK(spi1_sck_c), .Q(spi_command[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_command_i0_i0.GSR = "ENABLED";
    CCU2D fpga_time_1257_add_4_33 (.A0(fpga_time[31]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22419), .S0(n134));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_33.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_33.INIT1 = 16'h0000;
    defparam fpga_time_1257_add_4_33.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_33.INJECT1_1 = "NO";
    LUT4 accepted_sequence_31__I_0_i26_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[25]), .D(pending_sequence[25]), 
         .Z(accepted_sequence_31__N_1122[25])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i26_3_lut_4_lut.init = 16'hf960;
    LUT4 i14415_2_lut_3_lut_4_lut (.A(spi_bit_count[2]), .B(n23912), .C(n23869), 
         .D(fpga_cs_n_c), .Z(n23205)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i14415_2_lut_3_lut_4_lut.init = 16'hfff7;
    OB us_tx_pad_76 (.I(us_tx_c_76), .O(us_tx[76]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3IX ev_bit_i45 (.D(ev_bit[44]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i45.GSR = "DISABLED";
    LUT4 i14605_4_lut (.A(spi_bitmap[20]), .B(n23361), .C(n23271), .D(spi_bitmap[26]), 
         .Z(n23397)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14605_4_lut.init = 16'h8000;
    FD1P3IX ev_bit_i44 (.D(ev_bit[43]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i44.GSR = "DISABLED";
    LUT4 i14575_4_lut (.A(spi_bitmap[42]), .B(spi_bitmap[83]), .C(spi_bitmap[75]), 
         .D(spi_bitmap[10]), .Z(n23367)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14575_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_62 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[78]), 
         .D(ev_bit[78]), .Z(ev_wr_data[78])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_62.init = 16'h7770;
    OB us_tx_pad_77 (.I(us_tx_c_77), .O(us_tx[77]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_78 (.I(us_tx_c_78), .O(us_tx[78]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_79 (.I(us_tx_c_79), .O(us_tx[79]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_80 (.I(us_tx_c_80), .O(us_tx[80]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX spi_version_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_110), 
            .CK(spi1_sck_c), .Q(spi_version[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_version_i0_i0.GSR = "ENABLED";
    LUT4 i14491_2_lut (.A(spi_bitmap[32]), .B(spi_bitmap[37]), .Z(n23281)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14491_2_lut.init = 16'h8888;
    LUT4 accepted_sequence_31__I_0_i27_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[26]), .D(pending_sequence[26]), 
         .Z(accepted_sequence_31__N_1122[26])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i27_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_rep_65_4_lut (.A(spi_bit_count[2]), .B(n23912), .C(n23006), 
         .D(n14339), .Z(n23857)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i1_3_lut_rep_65_4_lut.init = 16'h0008;
    FD1P3AX spi_update_flags_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_103), 
            .CK(spi1_sck_c), .Q(expected_next_15__N_1417[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_update_flags_i0_i0.GSR = "ENABLED";
    LUT4 accepted_sequence_31__I_0_i28_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[27]), .D(pending_sequence[27]), 
         .Z(accepted_sequence_31__N_1122[27])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i28_3_lut_4_lut.init = 16'hf960;
    FD1P3AX spi_extension_length_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_102), 
            .CK(spi1_sck_c), .Q(expected_next[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_extension_length_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_95), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_63), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_phase_pending_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i20 (.D(spi_bitmap[12]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i20.GSR = "ENABLED";
    LUT4 i1_2_lut_3_lut_4_lut_adj_63 (.A(spi_bit_count[2]), .B(n23912), 
         .C(frame_end), .D(fpga_cs_n_c), .Z(spi1_sck_c_enable_38)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_63.init = 16'h0080;
    LUT4 i2_3_lut_4_lut_adj_64 (.A(frame_settle[0]), .B(n23899), .C(pll_clk_enable_3), 
         .D(pll_clk_enable_25), .Z(pll_clk_enable_483)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(437[17:37])
    defparam i2_3_lut_4_lut_adj_64.init = 16'hfffe;
    FD1P3AX spi_bitmap_i0_i19 (.D(spi_bitmap[11]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i19.GSR = "ENABLED";
    FD1S3IX frame_settle__i0 (.D(n14617), .CK(pll_clk), .CD(n11583), .Q(frame_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam frame_settle__i0.GSR = "DISABLED";
    LUT4 i2_3_lut_rep_77 (.A(n23892), .B(spi_byte_count[8]), .C(n23006), 
         .Z(n23869)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_3_lut_rep_77.init = 16'hfefe;
    FD1P3AX spi_bitmap_i0_i18 (.D(spi_bitmap[10]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i18.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i17 (.D(spi_bitmap[9]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i17.GSR = "ENABLED";
    FD1S3AX phase_frac_i0 (.D(phase_frac_sum[0]), .CK(pll_clk), .Q(phase_frac[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i0.GSR = "DISABLED";
    FD1P3AX stop_toggle_seen_520 (.D(stop_toggle_sync), .SP(pll_clk_enable_3), 
            .CK(pll_clk), .Q(stop_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam stop_toggle_seen_520.GSR = "DISABLED";
    LUT4 run_addr_s3_8__I_0_i8_3_lut (.A(run_addr_s3[7]), .B(ev_rd_slot[7]), 
         .C(n22496), .Z(event_rd_addr[7])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i8_3_lut.init = 16'hacac;
    FD1P3AX spi_byte_count_i0_i0 (.D(spi_byte_count_15__N_1694[0]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i0.GSR = "ENABLED";
    LUT4 accepted_sequence_31__I_0_i29_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[28]), .D(pending_sequence[28]), 
         .Z(accepted_sequence_31__N_1122[28])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i29_3_lut_4_lut.init = 16'hf960;
    FD1P3AX status_hold__i1 (.D(accepted_sequence[24]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i1.GSR = "DISABLED";
    LUT4 i2_2_lut_3_lut_4_lut (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .C(pll_clk_enable_3), .D(pll_clk_enable_10), .Z(pll_clk_enable_730)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam i2_2_lut_3_lut_4_lut.init = 16'hfff6;
    FD1P3IX ev_bit_i52 (.D(ev_bit[51]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i52.GSR = "DISABLED";
    LUT4 accepted_sequence_31__I_0_i30_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[29]), .D(pending_sequence[29]), 
         .Z(accepted_sequence_31__N_1122[29])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i30_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i31_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[30]), .D(pending_sequence[30]), 
         .Z(accepted_sequence_31__N_1122[30])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i31_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i32_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[31]), .D(pending_sequence[31]), 
         .Z(accepted_sequence_31__N_1122[31])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i32_3_lut_4_lut.init = 16'hf960;
    OB us_tx_pad_81 (.I(us_tx_c_81), .O(us_tx[81]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX ws2812_toggle_seen_519 (.D(ws2812_toggle_sync), .SP(pll_clk_enable_5), 
            .CK(pll_clk), .Q(ws2812_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ws2812_toggle_seen_519.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i2_3_lut_4_lut (.A(n23915), .B(n23913), .C(build_phase[1]), 
         .D(build_sum[1]), .Z(ev_rd_slot[1])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i2_3_lut_4_lut.init = 16'hf4b0;
    LUT4 stop_toggle_sync_I_0_2_lut_rep_105 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .Z(pll_clk_enable_3)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam stop_toggle_sync_I_0_2_lut_rep_105.init = 16'h6666;
    LUT4 build_phase_7__I_0_i3_3_lut_4_lut (.A(n23915), .B(n23913), .C(build_phase[2]), 
         .D(build_sum[2]), .Z(ev_rd_slot[2])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i3_3_lut_4_lut.init = 16'hf4b0;
    FD1P3AX accepted_sequence_i0 (.D(accepted_sequence_31__N_1122[0]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i0.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i6_3_lut_4_lut (.A(n23915), .B(n23913), .C(build_phase[5]), 
         .D(build_sum[5]), .Z(ev_rd_slot[5])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i6_3_lut_4_lut.init = 16'hf4b0;
    FD1S3AX phase_step_s1_482 (.D(phase_frac_sum[24]), .CK(pll_clk), .Q(phase_step_s1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_step_s1_482.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i8_3_lut_4_lut (.A(n23915), .B(n23913), .C(build_phase[7]), 
         .D(build_sum[7]), .Z(ev_rd_slot[7])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i8_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i4_3_lut_4_lut (.A(n23915), .B(n23913), .C(build_phase[3]), 
         .D(build_sum[3]), .Z(ev_rd_slot[3])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i4_3_lut_4_lut.init = 16'hf4b0;
    FD1S3AX phase_step_s2_483 (.D(phase_step_s1), .CK(pll_clk), .Q(phase_step_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_step_s2_483.GSR = "DISABLED";
    FD1S3AX phase_step_s3_486 (.D(phase_step_s2), .CK(pll_clk), .Q(phase_step_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_step_s3_486.GSR = "DISABLED";
    FD1S3AX swap_now_s3_487 (.D(pll_clk_enable_27), .CK(pll_clk), .Q(swap_now_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam swap_now_s3_487.GSR = "DISABLED";
    FD1S3AX active_bank_488 (.D(run_addr_s3_8__N_425[8]), .CK(pll_clk), 
            .Q(active_bank)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam active_bank_488.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i0 (.D(global_phase_s2[0]), .SP(pll_clk_enable_117), 
            .CK(pll_clk), .Q(run_addr_s3[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam run_addr_s3_i0.GSR = "DISABLED";
    FD1S3AX phase_step_s4_493 (.D(phase_step_s3), .CK(pll_clk), .Q(phase_step_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_step_s4_493.GSR = "DISABLED";
    FD1S3AX swap_now_s4_494 (.D(swap_now_s3), .CK(pll_clk), .Q(swap_now_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam swap_now_s4_494.GSR = "DISABLED";
    FD1S3AX phase_step_s5_495 (.D(phase_step_s4), .CK(pll_clk), .Q(phase_step_s5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_step_s5_495.GSR = "DISABLED";
    FD1S3AX swap_now_s5_496 (.D(swap_now_s4), .CK(pll_clk), .Q(swap_now_s5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam swap_now_s5_496.GSR = "DISABLED";
    FD1S3AX mem_1385 (.D(staging_rd_addr_6__N_901[4]), .CK(pll_clk), .Q(n10149));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mem_1385.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i0 (.D(mic_data_0_c), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i0.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i0 (.D(accepted_sequence_spi[0]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i0.GSR = "DISABLED";
    FD1S3AX frame_toggle_meta_501 (.D(frame_toggle_spi), .CK(pll_clk), .Q(frame_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam frame_toggle_meta_501.GSR = "DISABLED";
    FD1S3AX frame_toggle_sync_502 (.D(frame_toggle_meta), .CK(pll_clk), 
            .Q(frame_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam frame_toggle_sync_502.GSR = "DISABLED";
    FD1S3AX stop_toggle_meta_503 (.D(stop_toggle_spi), .CK(pll_clk), .Q(stop_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam stop_toggle_meta_503.GSR = "DISABLED";
    FD1S3AX stop_toggle_sync_504 (.D(stop_toggle_meta), .CK(pll_clk), .Q(stop_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam stop_toggle_sync_504.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_meta_505 (.D(ws2812_toggle_spi), .CK(pll_clk), 
            .Q(ws2812_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ws2812_toggle_meta_505.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_sync_506 (.D(ws2812_toggle_meta), .CK(pll_clk), 
            .Q(ws2812_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ws2812_toggle_sync_506.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i0 (.D(accepted_sequence_meta[0]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i0.GSR = "DISABLED";
    FD1S3AX update_flags_meta_i1 (.D(expected_next_15__N_1458[3]), .CK(pll_clk), 
            .Q(update_flags_meta[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam update_flags_meta_i1.GSR = "DISABLED";
    FD1S3AX update_flags_sync_i1 (.D(update_flags_meta[1]), .CK(pll_clk), 
            .Q(update_flags_sync[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam update_flags_sync_i1.GSR = "DISABLED";
    FD1P3IX us_tx__i1 (.D(n10778), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_0)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i1.GSR = "DISABLED";
    FD1S3AX invalid_frame_meta_511 (.D(invalid_frame_spi), .CK(pll_clk), 
            .Q(invalid_frame_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam invalid_frame_meta_511.GSR = "DISABLED";
    FD1S3AX invalid_frame_sync_512 (.D(invalid_frame_meta), .CK(pll_clk), 
            .Q(status_flags_wire_15__N_1385[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam invalid_frame_sync_512.GSR = "DISABLED";
    FD1P3IX frame_req_515 (.D(n24216), .SP(pll_clk_enable_10), .CD(n13591), 
            .CK(pll_clk), .Q(frame_req)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam frame_req_515.GSR = "DISABLED";
    FD1P3AX ev_ch_i0 (.D(ev_ch_6__N_709[0]), .SP(pll_clk_enable_11), .CK(pll_clk), 
            .Q(ev_ch[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_ch_i0.GSR = "DISABLED";
    FD1P3AX build_phase_i0 (.D(staging_q[8]), .SP(pll_clk_enable_231), .CK(pll_clk), 
            .Q(build_phase[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_phase_i0.GSR = "DISABLED";
    FD1P3AX build_sum_i0 (.D(build_sum_8__N_2053[0]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_sum[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_sum_i0.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i5_3_lut_4_lut (.A(n23915), .B(n23913), .C(build_phase[4]), 
         .D(build_sum[4]), .Z(ev_rd_slot[4])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i5_3_lut_4_lut.init = 16'hf4b0;
    FD1P3AX ev_rd_hold_i0 (.D(ev_rd_data[0]), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(ev_rd_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i0.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i0 (.D(staging_rd_addr_6__N_901[0]), .CK(pll_clk), 
            .Q(staging_rd_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam staging_rd_addr_i0.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0 (.D(accepted_sequence_sync[0]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i0.GSR = "DISABLED";
    FD1P3AX ev_clear_done_531 (.D(ev_clear_done_N_2575), .SP(pll_clk_enable_16), 
            .CK(pll_clk), .Q(ev_state_3__N_1940[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_clear_done_531.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i1 (.D(mic_data_0_c), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i1.GSR = "DISABLED";
    FD1S3AX mic_tick_534 (.D(mic_tick_N_2573), .CK(pll_clk), .Q(mic_tick)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_tick_534.GSR = "DISABLED";
    FD1S3AX mic_clock_reg_536 (.D(mic_clk_N_2520), .CK(pll_clk), .Q(mic_clk_c)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_clock_reg_536.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i1 (.D(mic_data_1_c), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i0.GSR = "DISABLED";
    FD1S3AY cs_meta_450 (.D(fpga_cs_n_c), .CK(pll_clk), .Q(cs_meta)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(259[12] 263[8])
    defparam cs_meta_450.GSR = "DISABLED";
    FD1P3IX ev_bit_i0 (.D(n24216), .SP(pll_clk_enable_345), .CD(n23910), 
            .CK(pll_clk), .Q(ev_bit[0])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i0.GSR = "DISABLED";
    FD1P3AX spi_expected_length_i0 (.D(expected_next[0]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i0.GSR = "ENABLED";
    LUT4 i1311_2_lut_3_lut_4_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(ws2812_toggle_seen), .D(ws2812_toggle_sync), .Z(pll_clk_enable_5)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D)))+!A !(B+!(C (D)+!C !(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam i1311_2_lut_3_lut_4_lut.init = 16'h6ff6;
    FD1P3AX spi_expected_length_i1 (.D(expected_next[1]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i1.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i2 (.D(expected_next[2]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[2])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i2.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i3 (.D(expected_next[3]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i3.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i4 (.D(expected_next[4]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i4.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i5 (.D(expected_next[5]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[5])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i5.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i6 (.D(expected_next[6]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i6.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i7 (.D(expected_next[7]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i7.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i8 (.D(expected_next[8]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i8.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i9 (.D(expected_next[9]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i9.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i10 (.D(expected_next[10]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i10.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i11 (.D(expected_next[11]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i11.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i12 (.D(expected_next[12]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i12.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i13 (.D(expected_next[13]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i13.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i14 (.D(expected_next[14]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i14.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i15 (.D(expected_next[15]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_expected_length_i15.GSR = "ENABLED";
    LUT4 build_phase_7__I_0_i7_3_lut_4_lut (.A(n23915), .B(n23913), .C(build_phase[6]), 
         .D(build_sum[6]), .Z(ev_rd_slot[6])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i7_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i1_3_lut_4_lut (.A(n23915), .B(n23913), .C(build_phase[0]), 
         .D(build_sum[0]), .Z(ev_rd_slot[0])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i1_3_lut_4_lut.init = 16'hf4b0;
    LUT4 i1_2_lut_3_lut_4_lut_adj_65 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(pll_locked), .D(status_flags_wire_15__N_1401[4]), .Z(n9817)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam i1_2_lut_3_lut_4_lut_adj_65.init = 16'h6fff;
    LUT4 i14569_4_lut (.A(spi_bitmap[53]), .B(spi_bitmap[74]), .C(spi_bitmap[61]), 
         .D(spi_bitmap[38]), .Z(n23361)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14569_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_66 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(wrap_s2), .D(swap_pending), .Z(swap_pending_N_2586)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam i1_3_lut_4_lut_adj_66.init = 16'h0900;
    LUT4 i14481_2_lut (.A(spi_bitmap[31]), .B(spi_bitmap[51]), .Z(n23271)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14481_2_lut.init = 16'h8888;
    LUT4 frame_toggle_sync_I_0_2_lut_rep_106 (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .Z(pll_clk_enable_25)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(432[13:51])
    defparam frame_toggle_sync_I_0_2_lut_rep_106.init = 16'h6666;
    LUT4 i3_4_lut (.A(n18460), .B(n24221), .C(phase_step_s3), .D(phase_step_s2), 
         .Z(n22496)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut.init = 16'hfffe;
    LUT4 i14579_4_lut (.A(spi_bitmap[33]), .B(spi_bitmap[49]), .C(spi_bitmap[46]), 
         .D(spi_bitmap[0]), .Z(n23371)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14579_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_67 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[9]), 
         .D(ev_bit[9]), .Z(ev_wr_data[9])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_67.init = 16'h7770;
    LUT4 i1_3_lut_adj_68 (.A(n14467), .B(init_shadow[26]), .C(ev_bit[26]), 
         .Z(n12867)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_68.init = 16'hecec;
    LUT4 i2703_2_lut_3_lut_4_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(stop_toggle_seen), .D(stop_toggle_sync), .Z(n11583)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D)))+!A !(B+!(C (D)+!C !(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(432[13:51])
    defparam i2703_2_lut_3_lut_4_lut.init = 16'h6ff6;
    LUT4 i2_3_lut_rep_86_4_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(frame_settle[0]), .D(n23899), .Z(pll_clk_enable_10)) /* synthesis lut_function=(!(A (((D)+!C)+!B)+!A (B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(432[13:51])
    defparam i2_3_lut_rep_86_4_lut.init = 16'h0090;
    LUT4 i2_3_lut_rep_107 (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .Z(n23899)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(437[17:37])
    defparam i2_3_lut_rep_107.init = 16'hfefe;
    LUT4 i1_3_lut_adj_69 (.A(n14467), .B(init_shadow[25]), .C(ev_bit[25]), 
         .Z(n12861)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_69.init = 16'hecec;
    LUT4 i1_3_lut_adj_70 (.A(n14467), .B(init_shadow[41]), .C(ev_bit[41]), 
         .Z(n12957)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_70.init = 16'hecec;
    LUT4 run_addr_s3_8__I_0_i2_3_lut (.A(run_addr_s3[1]), .B(ev_rd_slot[1]), 
         .C(n22496), .Z(event_rd_addr[1])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i2_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_adj_71 (.A(n14467), .B(init_shadow[30]), .C(ev_bit[30]), 
         .Z(n12891)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_71.init = 16'hecec;
    LUT4 i2_2_lut_4_lut (.A(n23892), .B(spi_byte_count[8]), .C(n23006), 
         .D(n23906), .Z(n7)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i2_2_lut_4_lut.init = 16'hfeff;
    LUT4 i1_3_lut_adj_72 (.A(n14467), .B(init_shadow[52]), .C(ev_bit[52]), 
         .Z(n13027)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_72.init = 16'hecec;
    LUT4 i1_3_lut_adj_73 (.A(n14467), .B(init_shadow[24]), .C(ev_bit[24]), 
         .Z(n12855)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_73.init = 16'hecec;
    LUT4 i1_2_lut_4_lut_adj_74 (.A(n23919), .B(n20519), .C(n23918), .D(stop_toggle_spi), 
         .Z(stop_toggle_spi_N_2535)) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C (D)+!C !(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i1_2_lut_4_lut_adj_74.init = 16'hfe01;
    LUT4 i3_4_lut_rep_78 (.A(n23889), .B(ev_state[0]), .C(frame_req), 
         .D(swap_pending), .Z(n23870)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i3_4_lut_rep_78.init = 16'hfffd;
    LUT4 i1_3_lut_adj_75 (.A(n14467), .B(init_shadow[23]), .C(ev_bit[23]), 
         .Z(n12849)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_75.init = 16'hecec;
    LUT4 i5734_2_lut_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .D(frame_settle[0]), .Z(n14617)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(437[17:37])
    defparam i5734_2_lut_4_lut.init = 16'h00fe;
    LUT4 i14778_2_lut_4_lut (.A(n23919), .B(n20519), .C(n23918), .D(spi1_sck_c_enable_199), 
         .Z(spi1_sck_c_enable_238)) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i14778_2_lut_4_lut.init = 16'hfe00;
    LUT4 i14508_3_lut_rep_80_4_lut (.A(ev_state[3]), .B(n23920), .C(ev_state[0]), 
         .D(n23909), .Z(pll_clk_enable_726)) /* synthesis lut_function=(!(A+!(B (C+(D))))) */ ;
    defparam i14508_3_lut_rep_80_4_lut.init = 16'h4440;
    LUT4 i32_1_lut_4_lut (.A(n23889), .B(ev_state[0]), .C(frame_req), 
         .D(swap_pending), .Z(fifo_credit_wire[0])) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i32_1_lut_4_lut.init = 16'h0002;
    LUT4 i1_3_lut_adj_76 (.A(n14467), .B(init_shadow[22]), .C(ev_bit[22]), 
         .Z(n12843)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_76.init = 16'hecec;
    LUT4 i1_3_lut_adj_77 (.A(n14467), .B(init_shadow[21]), .C(ev_bit[21]), 
         .Z(n12837)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_77.init = 16'hecec;
    LUT4 i1_2_lut_rep_94_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(pll_clk_enable_43)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(437[17:37])
    defparam i1_2_lut_rep_94_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_3_lut_adj_78 (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[4]), .Z(n4_adj_3196)) /* synthesis lut_function=(A (B+(C))+!A (C)) */ ;
    defparam i1_2_lut_3_lut_adj_78.init = 16'hf8f8;
    LUT4 i86_4_lut (.A(n23257), .B(n23439), .C(n166), .D(n23253), .Z(n174)) /* synthesis lut_function=(((C+!(D))+!B)+!A) */ ;
    defparam i86_4_lut.init = 16'hf7ff;
    CCU2D add_160_17 (.A0(spi_byte_count[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22357), .S0(spi_byte_count_15__N_1694[15]), .S1(frame_end_N_2614[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(339[35:57])
    defparam add_160_17.INIT0 = 16'h5aaa;
    defparam add_160_17.INIT1 = 16'h0000;
    defparam add_160_17.INJECT1_0 = "NO";
    defparam add_160_17.INJECT1_1 = "NO";
    LUT4 i14499_2_lut (.A(spi_bitmap[68]), .B(spi_bitmap[16]), .Z(n23289)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14499_2_lut.init = 16'h8888;
    CCU2D add_160_15 (.A0(spi_byte_count[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22356), .COUT(n22357), .S0(spi_byte_count_15__N_1694[13]), 
          .S1(spi_byte_count_15__N_1694[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(339[35:57])
    defparam add_160_15.INIT0 = 16'h5aaa;
    defparam add_160_15.INIT1 = 16'h5aaa;
    defparam add_160_15.INJECT1_0 = "NO";
    defparam add_160_15.INJECT1_1 = "NO";
    CCU2D fpga_time_1257_add_4_31 (.A0(fpga_time[29]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[30]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22418), .COUT(n22419), .S0(n136), .S1(n135));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_31.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_31.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_31.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_31.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_79 (.A(n14467), .B(init_shadow[20]), .C(ev_bit[20]), 
         .Z(n12831)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_79.init = 16'hecec;
    LUT4 i14547_4_lut (.A(spi_bitmap[43]), .B(spi_bitmap[70]), .C(spi_bitmap[21]), 
         .D(spi_bitmap[73]), .Z(n23339)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14547_4_lut.init = 16'h8000;
    CCU2D fpga_time_1257_add_4_29 (.A0(fpga_time[27]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[28]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22417), .COUT(n22418), .S0(n138), .S1(n137));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_29.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_29.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_29.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_29.INJECT1_1 = "NO";
    CCU2D fpga_time_1257_add_4_27 (.A0(fpga_time[25]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[26]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22416), .COUT(n22417), .S0(n140), .S1(n139));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_27.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_27.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_27.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_27.INJECT1_1 = "NO";
    CCU2D fpga_time_1257_add_4_25 (.A0(fpga_time[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[24]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22415), .COUT(n22416), .S0(n142), .S1(n141));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_25.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_25.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_25.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_25.INJECT1_1 = "NO";
    CCU2D add_160_13 (.A0(spi_byte_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22355), .COUT(n22356), .S0(spi_byte_count_15__N_1694[11]), 
          .S1(spi_byte_count_15__N_1694[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(339[35:57])
    defparam add_160_13.INIT0 = 16'h5aaa;
    defparam add_160_13.INIT1 = 16'h5aaa;
    defparam add_160_13.INJECT1_0 = "NO";
    defparam add_160_13.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_643_7 (.A0(expected_next_15__N_1417[7]), .B0(spi_extension_length[7]), 
          .C0(GND_net), .D0(GND_net), .A1(spi1_mosi_c_0), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22387), .COUT(n22388), .S0(expected_next[7]), 
          .S1(expected_next[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[33] 288[86])
    defparam expected_next_15__I_0_643_7.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_643_7.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_643_7.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_643_7.INJECT1_1 = "NO";
    LUT4 i14445_2_lut (.A(spi_bitmap[22]), .B(spi_bitmap[44]), .Z(n23235)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14445_2_lut.init = 16'h8888;
    CCU2D fpga_time_1257_add_4_23 (.A0(fpga_time[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22414), .COUT(n22415), .S0(n144), .S1(n143));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_23.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_23.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_23.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_23.INJECT1_1 = "NO";
    LUT4 i1_2_lut (.A(expected_next[1]), .B(expected_next[0]), .Z(n23145)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i1_2_lut.init = 16'heeee;
    CCU2D fpga_time_1257_add_4_21 (.A0(fpga_time[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22413), .COUT(n22414), .S0(n146), .S1(n145));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_21.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_21.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_21.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_21.INJECT1_1 = "NO";
    CCU2D add_646_13 (.A0(phase_frac[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22367), .COUT(n22368), .S0(phase_frac_sum[11]), 
          .S1(phase_frac_sum[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_13.INIT0 = 16'h5555;
    defparam add_646_13.INIT1 = 16'h5555;
    defparam add_646_13.INJECT1_0 = "NO";
    defparam add_646_13.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_643_5 (.A0(expected_next_15__N_1417[7]), .B0(spi_extension_length[5]), 
          .C0(GND_net), .D0(GND_net), .A1(expected_next_15__N_1417[7]), 
          .B1(spi_extension_length[6]), .C1(GND_net), .D1(GND_net), .CIN(n22386), 
          .COUT(n22387), .S0(expected_next[5]), .S1(expected_next[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[33] 288[86])
    defparam expected_next_15__I_0_643_5.INIT0 = 16'ha999;
    defparam expected_next_15__I_0_643_5.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_643_5.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_643_5.INJECT1_1 = "NO";
    CCU2D add_648_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n22349), 
          .S0(build_sum_8__N_2053[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(507[32:80])
    defparam add_648_cout.INIT0 = 16'h0000;
    defparam add_648_cout.INIT1 = 16'h0000;
    defparam add_648_cout.INJECT1_0 = "NO";
    defparam add_648_cout.INJECT1_1 = "NO";
    CCU2D fpga_time_1257_add_4_19 (.A0(fpga_time[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22412), .COUT(n22413), .S0(n148), .S1(n147));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_19.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_19.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_19.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_19.INJECT1_1 = "NO";
    FD1S3AX mem_1377 (.D(staging_rd_addr_6__N_901[0]), .CK(pll_clk), .Q(n10141));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mem_1377.GSR = "DISABLED";
    FD1P3IX ws2812_enable_518 (.D(update_flags_sync[1]), .SP(pll_clk_enable_22), 
            .CD(pll_clk_enable_3), .CK(pll_clk), .Q(ws2812_enable)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ws2812_enable_518.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_80 (.A(n14467), .B(init_shadow[51]), .C(ev_bit[51]), 
         .Z(n13021)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_80.init = 16'hecec;
    LUT4 i1_3_lut_adj_81 (.A(n14467), .B(init_shadow[19]), .C(ev_bit[19]), 
         .Z(n12825)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_81.init = 16'hecec;
    LUT4 i1501_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n23883), .C(ev_ch[5]), 
         .D(ev_ch[4]), .Z(ev_ch_6__N_1948[5])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(518[27:39])
    defparam i1501_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i1_3_lut_4_lut_adj_82 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[52]), 
         .D(ev_bit[52]), .Z(ev_wr_data[52])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_82.init = 16'h7770;
    LUT4 i3_2_lut_4_lut (.A(spi1_sck_c_enable_197), .B(n14339), .C(n23006), 
         .D(n23892), .Z(n9)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i3_2_lut_4_lut.init = 16'h0002;
    FD1P3AX rgb_values_0__475 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(rgb_values[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam rgb_values_0__475.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_83 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[39]), 
         .D(ev_bit[39]), .Z(ev_wr_data[39])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_83.init = 16'h7770;
    LUT4 i1_4_lut (.A(n23134), .B(spi_command[0]), .C(n20519), .D(spi_command[4]), 
         .Z(n14026)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[34:54])
    defparam i1_4_lut.init = 16'hfbff;
    FD1P3IX init_shadow_i50 (.D(n13015), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i50.GSR = "DISABLED";
    LUT4 i9_4_lut (.A(spi_version[4]), .B(n18), .C(n14_adj_3182), .D(spi_version[6]), 
         .Z(n23134)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[34:54])
    defparam i9_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_84 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[54]), 
         .D(ev_bit[54]), .Z(ev_wr_data[54])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_84.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_85 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[58]), 
         .D(ev_bit[58]), .Z(ev_wr_data[58])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_85.init = 16'h7770;
    LUT4 i1_3_lut_adj_86 (.A(n14467), .B(init_shadow[0]), .C(ev_bit[0]), 
         .Z(n11384)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_86.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_87 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[57]), 
         .D(ev_bit[57]), .Z(ev_wr_data[57])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_87.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_88 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[59]), 
         .D(ev_bit[59]), .Z(ev_wr_data[59])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_88.init = 16'h7770;
    LUT4 i1_2_lut_adj_89 (.A(ws2812_toggle_spi), .B(n14026), .Z(ws2812_toggle_spi_N_2554)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_89.init = 16'h9999;
    LUT4 i6_4_lut (.A(spi_version[7]), .B(spi_command[1]), .C(spi_version[0]), 
         .D(spi_version[1]), .Z(n16)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[34:54])
    defparam i6_4_lut.init = 16'hffbf;
    LUT4 i4_2_lut (.A(spi_version[5]), .B(spi_version[2]), .Z(n14_adj_3182)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[34:54])
    defparam i4_2_lut.init = 16'heeee;
    LUT4 i2_3_lut (.A(spi_command[3]), .B(spi_command[2]), .C(spi_command[5]), 
         .Z(n20519)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i2_3_lut.init = 16'hfefe;
    LUT4 i1_3_lut_4_lut_adj_90 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[60]), 
         .D(ev_bit[60]), .Z(ev_wr_data[60])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_90.init = 16'h7770;
    OB us_tx_pad_82 (.I(us_tx_c_82), .O(us_tx[82]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX rgb_values_1__474 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(rgb_values[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam rgb_values_1__474.GSR = "DISABLED";
    LUT4 i1473_2_lut (.A(ev_ch[1]), .B(ev_ch[0]), .Z(ev_ch_6__N_1948[1])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(518[27:39])
    defparam i1473_2_lut.init = 16'h6666;
    FD1S3AX mem_1390 (.D(staging_rd_addr_6__N_901[6]), .CK(pll_clk), .Q(n10153));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mem_1390.GSR = "DISABLED";
    LUT4 i14621_4_lut (.A(spi_command[1]), .B(spi_command[5]), .C(n23197), 
         .D(spi_command[6]), .Z(n23413)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14621_4_lut.init = 16'hfffe;
    FD1P3AX rgb_values_6__469 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(rgb_values[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam rgb_values_6__469.GSR = "DISABLED";
    LUT4 i14407_3_lut (.A(spi_command[7]), .B(spi_command[3]), .C(spi_command[2]), 
         .Z(n23197)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i14407_3_lut.init = 16'hfefe;
    LUT4 frame_end_I_0_4_lut (.A(n25), .B(frame_end_N_2613), .C(n23858), 
         .D(n26_adj_3203), .Z(frame_end)) /* synthesis lut_function=(A (B (C))+!A (B (C+!(D))+!B !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(290[29] 291[95])
    defparam frame_end_I_0_4_lut.init = 16'hc0c5;
    FD1P3AX frame_toggle_seen_513 (.D(frame_toggle_sync), .SP(pll_clk_enable_25), 
            .CK(pll_clk), .Q(frame_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam frame_toggle_seen_513.GSR = "DISABLED";
    LUT4 i1637_1_lut (.A(status_bit_index[2]), .Z(spi1_miso_N_2513[2])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i1637_1_lut.init = 16'h5555;
    FD1S3AX mem_1387 (.D(staging_rd_addr_6__N_901[5]), .CK(pll_clk), .Q(n10151));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mem_1387.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_91 (.A(n14467), .B(init_shadow[18]), .C(ev_bit[18]), 
         .Z(n12819)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_91.init = 16'hecec;
    OB us_tx_pad_83 (.I(us_tx_c_83), .O(us_tx[83]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    LUT4 i1_2_lut_3_lut_4_lut_adj_92 (.A(ev_state[3]), .B(ev_state[0]), 
         .C(ev_state[1]), .D(ev_state[2]), .Z(pll_clk_enable_231)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_2_lut_3_lut_4_lut_adj_92.init = 16'h0020;
    LUT4 i1_3_lut_adj_93 (.A(n14467), .B(init_shadow[17]), .C(ev_bit[17]), 
         .Z(n12813)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_93.init = 16'hecec;
    LUT4 i1_3_lut_adj_94 (.A(n14467), .B(init_shadow[16]), .C(ev_bit[16]), 
         .Z(n12807)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_94.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_95 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[61]), 
         .D(ev_bit[61]), .Z(ev_wr_data[61])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_95.init = 16'h7770;
    OB spi1_miso_pad (.I(spi1_miso_c), .O(spi1_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:33])
    FD1P3AX spi_bitmap_i0_i16 (.D(spi_bitmap[8]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i16.GSR = "ENABLED";
    LUT4 i7_4_lut (.A(spi_rx_shift[3]), .B(spi1_mosi_c_0), .C(spi_rx_shift[2]), 
         .D(spi_extension_length[7]), .Z(n18_adj_3194)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i7_4_lut.init = 16'hfffe;
    LUT4 i10091_3_lut_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(n18471), .D(spi_byte_count[7]), .Z(n100)) /* synthesis lut_function=(!(A (B (D)+!B (C (D)))+!A (C (D)))) */ ;
    defparam i10091_3_lut_4_lut.init = 16'h07ff;
    LUT4 i1_3_lut_adj_96 (.A(n14467), .B(init_shadow[15]), .C(ev_bit[15]), 
         .Z(n12801)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_96.init = 16'hecec;
    LUT4 i11_4_lut (.A(expected_next[2]), .B(n22), .C(n23145), .D(expected_next[5]), 
         .Z(n25)) /* synthesis lut_function=((B+(C+!(D)))+!A) */ ;
    defparam i11_4_lut.init = 16'hfdff;
    LUT4 i9735_4_lut (.A(frame_settle[3]), .B(pll_clk_enable_25), .C(frame_settle[2]), 
         .D(n23916), .Z(frame_settle_3__N_1832[3])) /* synthesis lut_function=(A (B+(C+(D)))+!A (B+!(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(435[18] 449[12])
    defparam i9735_4_lut.init = 16'heeed;
    FD1P3AX spi_bitmap_i0_i15 (.D(spi_bitmap[7]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i15.GSR = "ENABLED";
    FD1P3AX invalid_frame_spi_478 (.D(invalid_frame_spi_N_2542), .SP(spi1_sck_c_enable_35), 
            .CK(spi1_sck_c), .Q(invalid_frame_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam invalid_frame_spi_478.GSR = "DISABLED";
    LUT4 i14761_3_lut_4_lut_4_lut (.A(status_bit_index[4]), .B(n23927), 
         .C(status_hold[88]), .D(n23917), .Z(n23450)) /* synthesis lut_function=(A (B)+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14761_3_lut_4_lut_4_lut.init = 16'hd888;
    LUT4 run_addr_s3_8__I_0_i3_3_lut (.A(run_addr_s3[2]), .B(ev_rd_slot[2]), 
         .C(n22496), .Z(event_rd_addr[2])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i3_3_lut.init = 16'hacac;
    FD1P3AX spi_bitmap_i0_i14 (.D(spi_bitmap[6]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i14.GSR = "ENABLED";
    LUT4 i13574_1_lut (.A(spi_channel_field[0]), .Z(n15)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(331[78:102])
    defparam i13574_1_lut.init = 16'h5555;
    FD1P3AX rgb_values_7__468 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(rgb_values[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam rgb_values_7__468.GSR = "DISABLED";
    LUT4 i2_3_lut_rep_100 (.A(spi_byte_count[14]), .B(spi_byte_count[15]), 
         .C(spi_byte_count[13]), .Z(n23892)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_3_lut_rep_100.init = 16'hfefe;
    LUT4 i14721_3_lut_3_lut (.A(status_bit_index[4]), .B(n23513), .C(n23512), 
         .Z(n23514)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14721_3_lut_3_lut.init = 16'he4e4;
    LUT4 i12_4_lut (.A(expected_next[4]), .B(n24), .C(n18_adj_3204), .D(expected_next[8]), 
         .Z(n26_adj_3203)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i12_4_lut.init = 16'hfffe;
    FD1S3AX mem_1383 (.D(staging_rd_addr_6__N_901[3]), .CK(pll_clk), .Q(n10147));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mem_1383.GSR = "DISABLED";
    LUT4 run_addr_s3_8__I_0_i4_3_lut (.A(run_addr_s3[3]), .B(ev_rd_slot[3]), 
         .C(n22496), .Z(event_rd_addr[3])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i4_3_lut.init = 16'hacac;
    FD1P3AX ev_run_hold_s5_i0_i0 (.D(ev_rd_data[0]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i0.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_97 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[62]), 
         .D(ev_bit[62]), .Z(ev_wr_data[62])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_97.init = 16'h7770;
    LUT4 i14791_4_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_197), .C(frame_end), 
         .D(n23856), .Z(spi1_sck_c_enable_199)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i14791_4_lut.init = 16'h0040;
    LUT4 i8_4_lut (.A(expected_next[3]), .B(expected_next[12]), .C(expected_next[6]), 
         .D(expected_next[10]), .Z(n22)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i8_4_lut.init = 16'hfffe;
    LUT4 i2_2_lut_4_lut_adj_98 (.A(spi_byte_count[14]), .B(spi_byte_count[15]), 
         .C(spi_byte_count[13]), .D(n18490), .Z(n7_adj_3206)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_2_lut_4_lut_adj_98.init = 16'hfffe;
    FD1S3AX mem (.D(spi_phase_pending[7]), .CK(spi1_sck_c), .Q(n10188));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem.GSR = "DISABLED";
    LUT4 i10_4_lut (.A(expected_next[15]), .B(expected_next[11]), .C(expected_next[13]), 
         .D(expected_next[14]), .Z(n24)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i10_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_adj_99 (.A(n14467), .B(init_shadow[14]), .C(ev_bit[14]), 
         .Z(n12795)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_99.init = 16'hecec;
    FD1S3AX mem_1407 (.D(spi_phase_pending[6]), .CK(spi1_sck_c), .Q(n10186));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1407.GSR = "DISABLED";
    FD1S3AX mem_1406 (.D(spi_phase_pending[5]), .CK(spi1_sck_c), .Q(n10184));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1406.GSR = "DISABLED";
    FD1S3AX mem_1405 (.D(spi_phase_pending[4]), .CK(spi1_sck_c), .Q(n10182));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1405.GSR = "DISABLED";
    FD1P3IX running_490 (.D(n24216), .SP(pll_clk_enable_27), .CD(pll_clk_enable_3), 
            .CK(pll_clk), .Q(status_flags_wire_15__N_1401[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam running_490.GSR = "DISABLED";
    FD1S3AX mem_1404 (.D(spi_phase_pending[3]), .CK(spi1_sck_c), .Q(n10180));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1404.GSR = "DISABLED";
    FD1S3AX mem_1403 (.D(spi_phase_pending[2]), .CK(spi1_sck_c), .Q(n10178));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1403.GSR = "DISABLED";
    FD1S3AX mem_1402 (.D(spi_phase_pending[1]), .CK(spi1_sck_c), .Q(n10176));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1402.GSR = "DISABLED";
    FD1S3AX mem_1401 (.D(spi_phase_pending[0]), .CK(spi1_sck_c), .Q(n10174));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1401.GSR = "DISABLED";
    FD1S3AX mem_1400 (.D(spi_rx_shift[6]), .CK(spi1_sck_c), .Q(n10172));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1400.GSR = "DISABLED";
    FD1S3AX mem_1399 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(n10170));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1399.GSR = "DISABLED";
    FD1S3AX mem_1398 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(n10168));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1398.GSR = "DISABLED";
    FD1S3AX mem_1397 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(n10166));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1397.GSR = "DISABLED";
    FD1S3AX mem_1396 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(n10164));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1396.GSR = "DISABLED";
    FD1S3AX mem_1395 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(n10162));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1395.GSR = "DISABLED";
    FD1S3AX mem_1394 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(n10160));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1394.GSR = "DISABLED";
    FD1P3AX frame_toggle_spi_476 (.D(frame_toggle_spi_N_2524), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .Q(frame_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam frame_toggle_spi_476.GSR = "DISABLED";
    LUT4 n1_bdd_4_lut (.A(n1_adj_3205), .B(ev_state[0]), .C(ev_state[2]), 
         .D(ev_state[3]), .Z(n23710)) /* synthesis lut_function=(!(A (B+(C (D)+!C !(D)))+!A (B+((D)+!C)))) */ ;
    defparam n1_bdd_4_lut.init = 16'h0230;
    LUT4 i4_2_lut_adj_100 (.A(expected_next[9]), .B(expected_next[7]), .Z(n18_adj_3204)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i4_2_lut_adj_100.init = 16'heeee;
    LUT4 i4351_4_lut (.A(ev_ch[3]), .B(staging_rd_addr[3]), .C(n9_adj_3199), 
         .D(n23094), .Z(staging_rd_addr_6__N_901[3])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i4351_4_lut.init = 16'hac0c;
    LUT4 i1_3_lut_adj_101 (.A(n14467), .B(init_shadow[83]), .C(ev_bit[83]), 
         .Z(n13226)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_101.init = 16'hecec;
    LUT4 i14658_3_lut_3_lut (.A(status_bit_index[4]), .B(n23917), .C(status_hold[104]), 
         .Z(n23451)) /* synthesis lut_function=(A (B (C))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14658_3_lut_3_lut.init = 16'hc4c4;
    FD1P3AX rgb_values_3__472 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(rgb_values[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam rgb_values_3__472.GSR = "DISABLED";
    FD1P3AX rgb_values_2__473 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(rgb_values[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam rgb_values_2__473.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_102 (.A(n14467), .B(init_shadow[82]), .C(ev_bit[82]), 
         .Z(n13220)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_102.init = 16'hecec;
    FD1S3AX mem_1393 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(n10158));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1393.GSR = "DISABLED";
    LUT4 i2_3_lut_4_lut_adj_103 (.A(status_flags_wire_15__N_1401[4]), .B(pll_locked), 
         .C(pll_clk_enable_3), .D(phase_step_s5), .Z(pll_clk_enable_215)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_3_lut_4_lut_adj_103.init = 16'hfff7;
    LUT4 mux_1407_i4_3_lut (.A(n10163), .B(n10164), .C(n10156), .Z(rd_data_15__N_2649[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i4_3_lut.init = 16'hcaca;
    FD1S3AX mem_1392 (.D(spi_write), .CK(pll_clk), .Q(n10155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mem_1392.GSR = "DISABLED";
    FD1S3AX mem_1379 (.D(staging_rd_addr_6__N_901[1]), .CK(pll_clk), .Q(n10143));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mem_1379.GSR = "DISABLED";
    FD1S3AX mem_1381 (.D(staging_rd_addr_6__N_901[2]), .CK(pll_clk), .Q(n10145));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mem_1381.GSR = "DISABLED";
    FD1P3AX rgb_values_4__471 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(rgb_values[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam rgb_values_4__471.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_104 (.A(n14467), .B(init_shadow[81]), .C(ev_bit[81]), 
         .Z(n13214)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_104.init = 16'hecec;
    LUT4 i1_3_lut_adj_105 (.A(n14467), .B(init_shadow[80]), .C(ev_bit[80]), 
         .Z(n13208)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_105.init = 16'hecec;
    FD1P3AX spi_bitmap_i0_i13 (.D(spi_bitmap[5]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i13.GSR = "ENABLED";
    LUT4 i4_4_lut (.A(n7_adj_3181), .B(n23205), .C(spi_channel_field[1]), 
         .D(expected_next_15__N_1417[7]), .Z(spi_write)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;
    defparam i4_4_lut.init = 16'h0200;
    LUT4 i2_4_lut (.A(spi_channel_field[0]), .B(n19010), .C(n12), .D(n23914), 
         .Z(n7_adj_3181)) /* synthesis lut_function=(!((B+!(C+(D)))+!A)) */ ;
    defparam i2_4_lut.init = 16'h2220;
    LUT4 i1_3_lut_adj_106 (.A(n14467), .B(init_shadow[79]), .C(ev_bit[79]), 
         .Z(n13200)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_106.init = 16'hecec;
    FD1P3AX rgb_values_5__470 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(rgb_values[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam rgb_values_5__470.GSR = "DISABLED";
    LUT4 i6706_1_lut (.A(phase_step_s1), .Z(n15593)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i6706_1_lut.init = 16'h5555;
    FD1P3AX accepted_sequence_spi_i0_i0 (.D(spi_frame_sequence[0]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i0.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_107 (.A(n14467), .B(init_shadow[13]), .C(ev_bit[13]), 
         .Z(n12789)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_107.init = 16'hecec;
    FD1P3AX spi_bitmap_i0_i12 (.D(spi_bitmap[4]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i12.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_108 (.A(n14467), .B(init_shadow[78]), .C(ev_bit[78]), 
         .Z(n13194)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_108.init = 16'hecec;
    LUT4 i1_3_lut_adj_109 (.A(n14467), .B(init_shadow[77]), .C(ev_bit[77]), 
         .Z(n13188)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_109.init = 16'hecec;
    LUT4 i4347_4_lut (.A(ev_ch[1]), .B(staging_rd_addr[1]), .C(n9_adj_3199), 
         .D(n23094), .Z(staging_rd_addr_6__N_901[1])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i4347_4_lut.init = 16'hac0c;
    FD1P3AX spi_bitmap_i0_i11 (.D(spi_bitmap[3]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i11.GSR = "ENABLED";
    LUT4 i4349_4_lut (.A(ev_ch[2]), .B(staging_rd_addr[2]), .C(n9_adj_3199), 
         .D(n23094), .Z(staging_rd_addr_6__N_901[2])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i4349_4_lut.init = 16'hac0c;
    FD1P3AX spi_bitmap_i0_i10 (.D(spi_bitmap[2]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i9 (.D(spi_bitmap[1]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i8 (.D(spi_bitmap[0]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i3.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_110 (.A(n14467), .B(init_shadow[76]), .C(ev_bit[76]), 
         .Z(n13182)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_110.init = 16'hecec;
    LUT4 i7_4_lut_adj_111 (.A(global_phase_s2[0]), .B(n14_adj_3207), .C(n10), 
         .D(global_phase_s2[6]), .Z(wrap_s2_N_2572)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut_adj_111.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_112 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[63]), 
         .D(ev_bit[63]), .Z(ev_wr_data[63])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_112.init = 16'h7770;
    LUT4 i1_3_lut_adj_113 (.A(n14467), .B(init_shadow[75]), .C(ev_bit[75]), 
         .Z(n13176)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_113.init = 16'hecec;
    LUT4 i1_3_lut_adj_114 (.A(n14467), .B(init_shadow[74]), .C(ev_bit[74]), 
         .Z(n13164)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_114.init = 16'hecec;
    LUT4 i1_3_lut_adj_115 (.A(n14467), .B(init_shadow[73]), .C(ev_bit[73]), 
         .Z(n13158)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_115.init = 16'hecec;
    LUT4 i1_3_lut_adj_116 (.A(n14467), .B(init_shadow[72]), .C(ev_bit[72]), 
         .Z(n13152)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_116.init = 16'hecec;
    LUT4 i6_4_lut_adj_117 (.A(global_phase_s2[3]), .B(global_phase_s2[1]), 
         .C(global_phase_s2[5]), .D(global_phase_s2[7]), .Z(n14_adj_3207)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_117.init = 16'h8000;
    LUT4 i2_2_lut (.A(global_phase_s2[2]), .B(global_phase_s2[4]), .Z(n10)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_118 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[64]), 
         .D(ev_bit[64]), .Z(ev_wr_data[64])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_118.init = 16'h7770;
    LUT4 i1_3_lut_adj_119 (.A(n14467), .B(init_shadow[71]), .C(ev_bit[71]), 
         .Z(n13146)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_119.init = 16'hecec;
    LUT4 i1_3_lut_adj_120 (.A(n14467), .B(init_shadow[70]), .C(ev_bit[70]), 
         .Z(n13140)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_120.init = 16'hecec;
    LUT4 i1_3_lut_adj_121 (.A(n14467), .B(init_shadow[69]), .C(ev_bit[69]), 
         .Z(n13134)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_121.init = 16'hecec;
    LUT4 i1_3_lut_adj_122 (.A(n14467), .B(init_shadow[68]), .C(ev_bit[68]), 
         .Z(n13128)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_122.init = 16'hecec;
    LUT4 i1_3_lut_adj_123 (.A(n14467), .B(init_shadow[67]), .C(ev_bit[67]), 
         .Z(n13122)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_123.init = 16'hecec;
    LUT4 i1_3_lut_adj_124 (.A(n14467), .B(init_shadow[66]), .C(ev_bit[66]), 
         .Z(n13116)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_124.init = 16'hecec;
    LUT4 i1_3_lut_adj_125 (.A(n14467), .B(init_shadow[65]), .C(ev_bit[65]), 
         .Z(n13110)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_125.init = 16'hecec;
    FD1P3AX spi_bitmap_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_63), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_phase_pending_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_63), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_phase_pending_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_63), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_phase_pending_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_63), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_phase_pending_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_63), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_phase_pending_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_63), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_phase_pending_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_63), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_phase_pending_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i31 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_71), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i31.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i30 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_71), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i30.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i29 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_71), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i29.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i28 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_71), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i28.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i27 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_71), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i27.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_126 (.A(n14467), .B(init_shadow[64]), .C(ev_bit[64]), 
         .Z(n13099)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_126.init = 16'hecec;
    LUT4 i1_3_lut_adj_127 (.A(n14467), .B(init_shadow[63]), .C(ev_bit[63]), 
         .Z(n13093)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_127.init = 16'hecec;
    LUT4 i1_3_lut_adj_128 (.A(n14467), .B(init_shadow[62]), .C(ev_bit[62]), 
         .Z(n13087)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_128.init = 16'hecec;
    FD1P3AX spi_frame_sequence_i0_i26 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_71), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i26.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i25 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_71), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i25.GSR = "ENABLED";
    CCU2D add_160_5 (.A0(spi_byte_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22351), .COUT(n22352), .S0(spi_byte_count_15__N_1694[3]), 
          .S1(spi_byte_count_15__N_1694[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(339[35:57])
    defparam add_160_5.INIT0 = 16'h5aaa;
    defparam add_160_5.INIT1 = 16'h5aaa;
    defparam add_160_5.INJECT1_0 = "NO";
    defparam add_160_5.INJECT1_1 = "NO";
    FD1P3AX spi_frame_sequence_i0_i24 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_71), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i24.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_80), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i23.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_80), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i22.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_80), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i21.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_80), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i20.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1253__i0 (.D(n40_adj_3172), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253__i0.GSR = "ENABLED";
    FD1S3AX time_divider_1258__i0 (.D(n40_adj_3162), .CK(pll_clk), .Q(time_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258__i0.GSR = "DISABLED";
    FD1P3AX spi_frame_sequence_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_80), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i19.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_80), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i18.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_80), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i17.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_80), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i16.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_88), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_88), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_88), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_88), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_88), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_88), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_88), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i9.GSR = "ENABLED";
    FD1P3AX status_bit_index_1252__i0 (.D(n40), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252__i0.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_88), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_95), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_95), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_95), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i5.GSR = "ENABLED";
    LUT4 i14690_3_lut_3_lut (.A(status_bit_index[4]), .B(n23482), .C(n23481), 
         .Z(n23483)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14690_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX spi_frame_sequence_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_95), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i4.GSR = "ENABLED";
    FD1P3AX fpga_time_1257__i0 (.D(n165), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i0.GSR = "DISABLED";
    FD1P3AX spi_frame_sequence_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_95), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i3.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_129 (.A(n14467), .B(init_shadow[61]), .C(ev_bit[61]), 
         .Z(n13081)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_129.init = 16'hecec;
    FD1P3AX spi_frame_sequence_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_95), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_95), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_frame_sequence_i0_i1.GSR = "ENABLED";
    FD1P3AX global_phase_s2_1256__i0 (.D(n45), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256__i0.GSR = "DISABLED";
    FD1S3IX mic_divider_1260__i0 (.D(n40_adj_3185), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260__i0.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_130 (.A(n14467), .B(init_shadow[60]), .C(ev_bit[60]), 
         .Z(n13075)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_130.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_131 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[65]), 
         .D(ev_bit[65]), .Z(ev_wr_data[65])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_131.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_132 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[10]), 
         .D(ev_bit[10]), .Z(ev_wr_data[10])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_132.init = 16'h7770;
    LUT4 mux_1407_i5_3_lut (.A(n10165), .B(n10166), .C(n10156), .Z(rd_data_15__N_2649[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i5_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_adj_133 (.A(n14467), .B(init_shadow[12]), .C(ev_bit[12]), 
         .Z(n12783)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_133.init = 16'hecec;
    LUT4 frame_toggle_spi_I_0_2_lut (.A(frame_toggle_spi), .B(n23856), .Z(frame_toggle_spi_N_2524)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(346[30] 356[24])
    defparam frame_toggle_spi_I_0_2_lut.init = 16'h6666;
    LUT4 i13591_2_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), .Z(n19)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(360[34:54])
    defparam i13591_2_lut.init = 16'h6666;
    LUT4 i13613_2_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .Z(n29)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[37:60])
    defparam i13613_2_lut.init = 16'h6666;
    FD1S3AX spi_bit_count_1255__i0 (.D(n20), .CK(spi1_sck_c), .Q(spi_bit_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(360[34:54])
    defparam spi_bit_count_1255__i0.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_102), 
            .CK(spi1_sck_c), .Q(spi_extension_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_extension_length_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_102), 
            .CK(spi1_sck_c), .Q(spi_extension_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_extension_length_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_102), 
            .CK(spi1_sck_c), .Q(spi_extension_length[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_extension_length_i0_i5.GSR = "ENABLED";
    CCU2D add_160_3 (.A0(spi_byte_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22350), .COUT(n22351), .S0(spi_byte_count_15__N_1694[1]), 
          .S1(spi_byte_count_15__N_1694[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(339[35:57])
    defparam add_160_3.INIT0 = 16'h5aaa;
    defparam add_160_3.INIT1 = 16'h5aaa;
    defparam add_160_3.INJECT1_0 = "NO";
    defparam add_160_3.INJECT1_1 = "NO";
    FD1P3AX spi_extension_length_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_102), 
            .CK(spi1_sck_c), .Q(spi_extension_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_extension_length_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_102), 
            .CK(spi1_sck_c), .Q(spi_extension_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_extension_length_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_102), 
            .CK(spi1_sck_c), .Q(spi_extension_length[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_extension_length_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_102), 
            .CK(spi1_sck_c), .Q(expected_next[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_extension_length_i0_i1.GSR = "ENABLED";
    FD1P3AX mic_sample_count_1259__i0 (.D(n30), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_sample_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[37:60])
    defparam mic_sample_count_1259__i0.GSR = "DISABLED";
    OB us_tx_pad_75 (.I(us_tx_c_75), .O(us_tx[75]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX spi_update_flags_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_103), 
            .CK(spi1_sck_c), .Q(expected_next_15__N_1458[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_update_flags_i0_i1.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_134 (.A(n14467), .B(init_shadow[35]), .C(ev_bit[35]), 
         .Z(n12921)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_134.init = 16'hecec;
    LUT4 i1_3_lut_adj_135 (.A(n14467), .B(init_shadow[55]), .C(ev_bit[55]), 
         .Z(n13045)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_135.init = 16'hecec;
    FD1P3AX spi_version_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_110), 
            .CK(spi1_sck_c), .Q(spi_version[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_version_i0_i7.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_136 (.A(n14467), .B(init_shadow[48]), .C(ev_bit[48]), 
         .Z(n13003)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_136.init = 16'hecec;
    LUT4 i1_3_lut_adj_137 (.A(n14467), .B(init_shadow[47]), .C(ev_bit[47]), 
         .Z(n12997)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_137.init = 16'hecec;
    LUT4 i1_3_lut_adj_138 (.A(n14467), .B(init_shadow[46]), .C(ev_bit[46]), 
         .Z(n12987)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_138.init = 16'hecec;
    FD1P3AX spi_version_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_110), 
            .CK(spi1_sck_c), .Q(spi_version[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_version_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_110), 
            .CK(spi1_sck_c), .Q(spi_version[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_version_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_110), 
            .CK(spi1_sck_c), .Q(spi_version[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_version_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_110), 
            .CK(spi1_sck_c), .Q(spi_version[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_version_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_110), 
            .CK(spi1_sck_c), .Q(spi_version[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_version_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_110), 
            .CK(spi1_sck_c), .Q(spi_version[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_version_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_117), 
            .CK(spi1_sck_c), .Q(spi_command[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_command_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_117), 
            .CK(spi1_sck_c), .Q(spi_command[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_command_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_117), 
            .CK(spi1_sck_c), .Q(spi_command[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_command_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_117), 
            .CK(spi1_sck_c), .Q(spi_command[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_command_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_117), 
            .CK(spi1_sck_c), .Q(spi_command[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_command_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_117), 
            .CK(spi1_sck_c), .Q(spi_command[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_command_i0_i2.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_139 (.A(n14467), .B(init_shadow[45]), .C(ev_bit[45]), 
         .Z(n12981)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_139.init = 16'hecec;
    LUT4 i1_2_lut_rep_103 (.A(swap_pending), .B(wrap_s2), .Z(pll_clk_enable_27)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_2_lut_rep_103.init = 16'h8888;
    FD1P3AX spi_command_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_117), 
            .CK(spi1_sck_c), .Q(spi_command[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_command_i0_i1.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i7 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(spi_rx_shift[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_rx_shift_i7.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut_adj_140 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[0]), 
         .D(ev_bit[0]), .Z(ev_wr_data[0])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_140.init = 16'h7770;
    LUT4 i1955_2_lut_3_lut (.A(swap_pending), .B(wrap_s2), .C(phase_step_s2), 
         .Z(pll_clk_enable_117)) /* synthesis lut_function=(A (B+(C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1955_2_lut_3_lut.init = 16'hf8f8;
    LUT4 i1_3_lut_adj_141 (.A(n14467), .B(init_shadow[44]), .C(ev_bit[44]), 
         .Z(n12975)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_141.init = 16'hecec;
    FD1P3IX ev_bit_i43 (.D(ev_bit[42]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i43.GSR = "DISABLED";
    FD1P3IX ev_bit_i42 (.D(ev_bit[41]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i42.GSR = "DISABLED";
    FD1P3IX ev_bit_i41 (.D(ev_bit[40]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i41.GSR = "DISABLED";
    FD1P3IX ev_bit_i40 (.D(ev_bit[39]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i40.GSR = "DISABLED";
    FD1P3IX ev_bit_i39 (.D(ev_bit[38]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i39.GSR = "DISABLED";
    FD1P3IX ev_bit_i38 (.D(ev_bit[37]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i38.GSR = "DISABLED";
    FD1P3IX ev_bit_i37 (.D(ev_bit[36]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i37.GSR = "DISABLED";
    FD1P3IX ev_bit_i36 (.D(ev_bit[35]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i36.GSR = "DISABLED";
    FD1P3IX ev_bit_i35 (.D(ev_bit[34]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i35.GSR = "DISABLED";
    FD1P3IX ev_bit_i34 (.D(ev_bit[33]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i34.GSR = "DISABLED";
    FD1P3IX ev_bit_i33 (.D(ev_bit[32]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i33.GSR = "DISABLED";
    FD1P3IX ev_bit_i32 (.D(ev_bit[31]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i32.GSR = "DISABLED";
    LUT4 ws2812_toggle_sync_I_0_2_lut_rep_104 (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .Z(n23896)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam ws2812_toggle_sync_I_0_2_lut_rep_104.init = 16'h6666;
    LUT4 i1_3_lut_adj_142 (.A(n14467), .B(init_shadow[43]), .C(ev_bit[43]), 
         .Z(n12969)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_142.init = 16'hecec;
    LUT4 accepted_sequence_31__I_0_i2_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[1]), .D(pending_sequence[1]), 
         .Z(accepted_sequence_31__N_1122[1])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i2_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_2_lut_3_lut_4_lut_adj_143 (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .C(wrap_s2), .D(swap_pending), .Z(pll_clk_enable_109)) /* synthesis lut_function=(A ((C (D))+!B)+!A (B+(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam i1_2_lut_3_lut_4_lut_adj_143.init = 16'hf666;
    LUT4 i1_3_lut_adj_144 (.A(n14467), .B(init_shadow[54]), .C(ev_bit[54]), 
         .Z(n13039)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_144.init = 16'hecec;
    FD1S3AX spi_rx_shift_i6 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(spi_rx_shift[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_rx_shift_i6.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i5 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(spi_rx_shift[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_rx_shift_i5.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i4 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(spi_rx_shift[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_rx_shift_i4.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i3 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(spi_rx_shift[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_rx_shift_i3.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i2 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(spi_rx_shift[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_rx_shift_i2.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i23 (.D(spi_bitmap[15]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i23.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_145 (.A(n14467), .B(init_shadow[53]), .C(ev_bit[53]), 
         .Z(n13033)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_145.init = 16'hecec;
    LUT4 i1_3_lut_adj_146 (.A(n14467), .B(init_shadow[11]), .C(ev_bit[11]), 
         .Z(n12777)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_146.init = 16'hecec;
    LUT4 i1_4_lut_adj_147 (.A(n14026), .B(spi1_sck_c_enable_38), .C(n23856), 
         .D(n23888), .Z(spi1_sck_c_enable_230)) /* synthesis lut_function=(A (B (C))+!A (B (C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i1_4_lut_adj_147.init = 16'hc4c0;
    LUT4 i1_3_lut_4_lut_adj_148 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[1]), 
         .D(ev_bit[1]), .Z(ev_wr_data[1])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_148.init = 16'h7770;
    LUT4 accepted_sequence_31__I_0_i3_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[2]), .D(pending_sequence[2]), 
         .Z(accepted_sequence_31__N_1122[2])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i3_3_lut_4_lut.init = 16'hf960;
    PFUMX i14712 (.BLUT(n23494), .ALUT(n23495), .C0(spi1_miso_N_2513[1]), 
          .Z(n23505));
    OB us_tx_pad_74 (.I(us_tx_c_74), .O(us_tx[74]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_73 (.I(us_tx_c_73), .O(us_tx[73]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_72 (.I(us_tx_c_72), .O(us_tx[72]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_71 (.I(us_tx_c_71), .O(us_tx[71]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_70 (.I(us_tx_c_70), .O(us_tx[70]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_69 (.I(us_tx_c_69), .O(us_tx[69]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_68 (.I(us_tx_c_68), .O(us_tx[68]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_67 (.I(us_tx_c_67), .O(us_tx[67]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_66 (.I(us_tx_c_66), .O(us_tx[66]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_65 (.I(us_tx_c_65), .O(us_tx[65]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_64 (.I(us_tx_c_64), .O(us_tx[64]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_63 (.I(us_tx_c_63), .O(us_tx[63]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_62 (.I(us_tx_c_62), .O(us_tx[62]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_61 (.I(us_tx_c_61), .O(us_tx[61]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_60 (.I(us_tx_c_60), .O(us_tx[60]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_59 (.I(us_tx_c_59), .O(us_tx[59]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_58 (.I(us_tx_c_58), .O(us_tx[58]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_57 (.I(us_tx_c_57), .O(us_tx[57]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_56 (.I(us_tx_c_56), .O(us_tx[56]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_55 (.I(us_tx_c_55), .O(us_tx[55]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_54 (.I(us_tx_c_54), .O(us_tx[54]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_53 (.I(us_tx_c_53), .O(us_tx[53]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_52 (.I(us_tx_c_52), .O(us_tx[52]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_51 (.I(us_tx_c_51), .O(us_tx[51]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_50 (.I(us_tx_c_50), .O(us_tx[50]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_49 (.I(us_tx_c_49), .O(us_tx[49]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_48 (.I(us_tx_c_48), .O(us_tx[48]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_47 (.I(us_tx_c_47), .O(us_tx[47]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_46 (.I(us_tx_c_46), .O(us_tx[46]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_45 (.I(us_tx_c_45), .O(us_tx[45]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_44 (.I(us_tx_c_44), .O(us_tx[44]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_43 (.I(us_tx_c_43), .O(us_tx[43]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_42 (.I(us_tx_c_42), .O(us_tx[42]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_41 (.I(us_tx_c_41), .O(us_tx[41]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_40 (.I(us_tx_c_40), .O(us_tx[40]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_39 (.I(us_tx_c_39), .O(us_tx[39]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_38 (.I(us_tx_c_38), .O(us_tx[38]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_37 (.I(us_tx_c_37), .O(us_tx[37]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_36 (.I(us_tx_c_36), .O(us_tx[36]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_35 (.I(us_tx_c_35), .O(us_tx[35]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_34 (.I(us_tx_c_34), .O(us_tx[34]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_33 (.I(us_tx_c_33), .O(us_tx[33]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_32 (.I(us_tx_c_32), .O(us_tx[32]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_31 (.I(us_tx_c_31), .O(us_tx[31]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_30 (.I(us_tx_c_30), .O(us_tx[30]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_29 (.I(us_tx_c_29), .O(us_tx[29]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_28 (.I(us_tx_c_28), .O(us_tx[28]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_27 (.I(us_tx_c_27), .O(us_tx[27]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_26 (.I(us_tx_c_26), .O(us_tx[26]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_25 (.I(us_tx_c_25), .O(us_tx[25]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_24 (.I(us_tx_c_24), .O(us_tx[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_23 (.I(us_tx_c_23), .O(us_tx[23]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_22 (.I(us_tx_c_22), .O(us_tx[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_21 (.I(us_tx_c_21), .O(us_tx[21]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_20 (.I(us_tx_c_20), .O(us_tx[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_19 (.I(us_tx_c_19), .O(us_tx[19]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_18 (.I(us_tx_c_18), .O(us_tx[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_17 (.I(us_tx_c_17), .O(us_tx[17]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_16 (.I(us_tx_c_16), .O(us_tx[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_15 (.I(us_tx_c_15), .O(us_tx[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_14 (.I(us_tx_c_14), .O(us_tx[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_13 (.I(us_tx_c_13), .O(us_tx[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_12 (.I(us_tx_c_12), .O(us_tx[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_11 (.I(us_tx_c_11), .O(us_tx[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_10 (.I(us_tx_c_10), .O(us_tx[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_9 (.I(us_tx_c_9), .O(us_tx[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_8 (.I(us_tx_c_8), .O(us_tx[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_7 (.I(us_tx_c_7), .O(us_tx[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_6 (.I(us_tx_c_6), .O(us_tx[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_5 (.I(us_tx_c_5), .O(us_tx[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_4 (.I(us_tx_c_4), .O(us_tx[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_3 (.I(us_tx_c_3), .O(us_tx[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_2 (.I(us_tx_c_2), .O(us_tx[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_1 (.I(us_tx_c_1), .O(us_tx[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_0 (.I(us_tx_c_0), .O(us_tx[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB rgb_data_pad (.I(rgb_data_c), .O(rgb_data));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(66[24:32])
    OB mic_clk_pad (.I(mic_clk_c), .O(mic_clk));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(67[24:31])
    OB spi_mic_miso_pad (.I(spi_mic_miso_c), .O(spi_mic_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(72[24:36])
    IB fpga_clk_8m_pad (.I(fpga_clk_8m), .O(fpga_clk_8m_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(60[24:35])
    IB fpga_cs_n_pad (.I(fpga_cs_n), .O(fpga_cs_n_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:33])
    IB spi1_sck_pad (.I(spi1_sck), .O(spi1_sck_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    IB spi1_mosi_pad (.I(spi1_mosi), .O(spi1_mosi_c_0));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(63[24:33])
    IB mic_data_0_pad (.I(mic_data_0), .O(mic_data_0_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(68[24:34])
    IB mic_data_1_pad (.I(mic_data_1), .O(mic_data_1_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(69[24:34])
    IB spi_mic_cs_n_pad (.I(spi_mic_cs_n), .O(spi_mic_cs_n_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(70[24:36])
    IB spi_mic_sck_pad (.I(spi_mic_sck), .O(spi_mic_sck_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(71[24:35])
    FD1P3AX spi_bitmap_i0_i24 (.D(spi_bitmap[16]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i24.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i25 (.D(spi_bitmap[17]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i25.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i26 (.D(spi_bitmap[18]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i26.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i27 (.D(spi_bitmap[19]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i27.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i28 (.D(spi_bitmap[20]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i28.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i29 (.D(spi_bitmap[21]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i29.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i30 (.D(spi_bitmap[22]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i30.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i31 (.D(spi_bitmap[23]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i31.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i32 (.D(spi_bitmap[24]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i32.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i33 (.D(spi_bitmap[25]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i33.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i34 (.D(spi_bitmap[26]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i34.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i35 (.D(spi_bitmap[27]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i35.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i36 (.D(spi_bitmap[28]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i36.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i37 (.D(spi_bitmap[29]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i37.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i38 (.D(spi_bitmap[30]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i38.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i39 (.D(spi_bitmap[31]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i39.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i40 (.D(spi_bitmap[32]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i40.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i41 (.D(spi_bitmap[33]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i41.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i42 (.D(spi_bitmap[34]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i42.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i43 (.D(spi_bitmap[35]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i43.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i44 (.D(spi_bitmap[36]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i44.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i45 (.D(spi_bitmap[37]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i45.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i46 (.D(spi_bitmap[38]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i46.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i47 (.D(spi_bitmap[39]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i47.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i48 (.D(spi_bitmap[40]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i48.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i49 (.D(spi_bitmap[41]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i49.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i50 (.D(spi_bitmap[42]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i50.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i51 (.D(spi_bitmap[43]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i51.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i52 (.D(spi_bitmap[44]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i52.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i53 (.D(spi_bitmap[45]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i53.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i54 (.D(spi_bitmap[46]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i54.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i55 (.D(spi_bitmap[47]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i55.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i56 (.D(spi_bitmap[48]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i56.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i57 (.D(spi_bitmap[49]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i57.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i58 (.D(spi_bitmap[50]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i58.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i59 (.D(spi_bitmap[51]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i59.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i60 (.D(spi_bitmap[52]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i60.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i61 (.D(spi_bitmap[53]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i61.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i62 (.D(spi_bitmap[54]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i62.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i63 (.D(spi_bitmap[55]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i63.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i64 (.D(spi_bitmap[56]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i64.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i65 (.D(spi_bitmap[57]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i65.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i66 (.D(spi_bitmap[58]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i66.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i67 (.D(spi_bitmap[59]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i67.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i68 (.D(spi_bitmap[60]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i68.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i69 (.D(spi_bitmap[61]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i69.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i70 (.D(spi_bitmap[62]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i70.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i71 (.D(spi_bitmap[63]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i71.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i72 (.D(spi_bitmap[64]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i72.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i73 (.D(spi_bitmap[65]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i73.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i74 (.D(spi_bitmap[66]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i74.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i75 (.D(spi_bitmap[67]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i75.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i76 (.D(spi_bitmap[68]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i76.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i77 (.D(spi_bitmap[69]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i77.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i78 (.D(spi_bitmap[70]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i78.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i79 (.D(spi_bitmap[71]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i79.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i80 (.D(spi_bitmap[72]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i80.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i81 (.D(spi_bitmap[73]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i81.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i82 (.D(spi_bitmap[74]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i82.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i83 (.D(spi_bitmap[75]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i83.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i84 (.D(spi_bitmap[76]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[84])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i84.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i85 (.D(spi_bitmap[77]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[85])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i85.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i86 (.D(spi_bitmap[78]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[86])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i86.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i87 (.D(spi_bitmap[79]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[87])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i87.GSR = "ENABLED";
    FD1P3IX frame_settle__i1 (.D(n14170), .SP(pll_clk_enable_43), .CD(n11583), 
            .CK(pll_clk), .Q(frame_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam frame_settle__i1.GSR = "DISABLED";
    FD1P3IX frame_settle__i2 (.D(n14164), .SP(pll_clk_enable_43), .CD(n11583), 
            .CK(pll_clk), .Q(frame_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam frame_settle__i2.GSR = "DISABLED";
    FD1S3AX phase_frac_i1 (.D(phase_frac_sum[1]), .CK(pll_clk), .Q(phase_frac[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i1.GSR = "DISABLED";
    FD1S3AX phase_frac_i2 (.D(phase_frac_sum[2]), .CK(pll_clk), .Q(phase_frac[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i2.GSR = "DISABLED";
    FD1S3AX phase_frac_i3 (.D(phase_frac_sum[3]), .CK(pll_clk), .Q(phase_frac[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i3.GSR = "DISABLED";
    FD1S3AX phase_frac_i4 (.D(phase_frac_sum[4]), .CK(pll_clk), .Q(phase_frac[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i4.GSR = "DISABLED";
    FD1S3AX phase_frac_i5 (.D(phase_frac_sum[5]), .CK(pll_clk), .Q(phase_frac[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i5.GSR = "DISABLED";
    FD1S3AX phase_frac_i6 (.D(phase_frac_sum[6]), .CK(pll_clk), .Q(phase_frac[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i6.GSR = "DISABLED";
    FD1S3AX phase_frac_i7 (.D(phase_frac_sum[7]), .CK(pll_clk), .Q(phase_frac[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i7.GSR = "DISABLED";
    FD1S3AX phase_frac_i8 (.D(phase_frac_sum[8]), .CK(pll_clk), .Q(phase_frac[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i8.GSR = "DISABLED";
    FD1S3AX phase_frac_i9 (.D(phase_frac_sum[9]), .CK(pll_clk), .Q(phase_frac[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i9.GSR = "DISABLED";
    FD1S3AX phase_frac_i10 (.D(phase_frac_sum[10]), .CK(pll_clk), .Q(phase_frac[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i10.GSR = "DISABLED";
    FD1S3AX phase_frac_i11 (.D(phase_frac_sum[11]), .CK(pll_clk), .Q(phase_frac[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i11.GSR = "DISABLED";
    FD1S3AX phase_frac_i12 (.D(phase_frac_sum[12]), .CK(pll_clk), .Q(phase_frac[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i12.GSR = "DISABLED";
    FD1S3AX phase_frac_i13 (.D(phase_frac_sum[13]), .CK(pll_clk), .Q(phase_frac[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i13.GSR = "DISABLED";
    FD1S3AX phase_frac_i14 (.D(phase_frac_sum[14]), .CK(pll_clk), .Q(phase_frac[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i14.GSR = "DISABLED";
    FD1S3AX phase_frac_i15 (.D(phase_frac_sum[15]), .CK(pll_clk), .Q(phase_frac[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i15.GSR = "DISABLED";
    FD1S3AX phase_frac_i16 (.D(phase_frac_sum[16]), .CK(pll_clk), .Q(phase_frac[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i16.GSR = "DISABLED";
    FD1S3AX phase_frac_i17 (.D(phase_frac_sum[17]), .CK(pll_clk), .Q(phase_frac[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i17.GSR = "DISABLED";
    FD1S3AX phase_frac_i18 (.D(phase_frac_sum[18]), .CK(pll_clk), .Q(phase_frac[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i18.GSR = "DISABLED";
    FD1S3AX phase_frac_i19 (.D(phase_frac_sum[19]), .CK(pll_clk), .Q(phase_frac[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i19.GSR = "DISABLED";
    FD1S3AX phase_frac_i20 (.D(phase_frac_sum[20]), .CK(pll_clk), .Q(phase_frac[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i20.GSR = "DISABLED";
    FD1S3AX phase_frac_i21 (.D(phase_frac_sum[21]), .CK(pll_clk), .Q(phase_frac[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i21.GSR = "DISABLED";
    FD1S3AX phase_frac_i22 (.D(phase_frac_sum[22]), .CK(pll_clk), .Q(phase_frac[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i22.GSR = "DISABLED";
    FD1S3AX phase_frac_i23 (.D(phase_frac_sum[23]), .CK(pll_clk), .Q(phase_frac[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_frac_i23.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i1 (.D(spi_byte_count_15__N_1694[1]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i1.GSR = "ENABLED";
    FD1P3IX init_shadow_i29 (.D(n12885), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i29.GSR = "DISABLED";
    FD1P3IX init_shadow_i28 (.D(n12879), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i28.GSR = "DISABLED";
    FD1P3IX init_shadow_i27 (.D(n12873), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i27.GSR = "DISABLED";
    FD1P3IX init_shadow_i26 (.D(n12867), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i26.GSR = "DISABLED";
    FD1P3IX init_shadow_i25 (.D(n12861), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i25.GSR = "DISABLED";
    FD1P3IX init_shadow_i24 (.D(n12855), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i24.GSR = "DISABLED";
    FD1P3IX init_shadow_i23 (.D(n12849), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i23.GSR = "DISABLED";
    FD1P3IX init_shadow_i22 (.D(n12843), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i22.GSR = "DISABLED";
    FD1P3IX init_shadow_i21 (.D(n12837), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i21.GSR = "DISABLED";
    FD1P3IX init_shadow_i20 (.D(n12831), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i20.GSR = "DISABLED";
    FD1P3IX init_shadow_i19 (.D(n12825), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i19.GSR = "DISABLED";
    FD1P3IX init_shadow_i18 (.D(n12819), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i18.GSR = "DISABLED";
    FD1P3IX init_shadow_i17 (.D(n12813), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i17.GSR = "DISABLED";
    FD1P3IX init_shadow_i16 (.D(n12807), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i16.GSR = "DISABLED";
    FD1P3IX init_shadow_i15 (.D(n12801), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i15.GSR = "DISABLED";
    FD1P3IX init_shadow_i14 (.D(n12795), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i14.GSR = "DISABLED";
    FD1P3IX init_shadow_i13 (.D(n12789), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i13.GSR = "DISABLED";
    FD1P3IX init_shadow_i12 (.D(n12783), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i12.GSR = "DISABLED";
    FD1P3IX init_shadow_i11 (.D(n12777), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i11.GSR = "DISABLED";
    FD1P3IX init_shadow_i10 (.D(n12771), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i10.GSR = "DISABLED";
    FD1P3IX init_shadow_i9 (.D(n12765), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i9.GSR = "DISABLED";
    FD1P3IX init_shadow_i8 (.D(n12759), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i8.GSR = "DISABLED";
    FD1P3IX init_shadow_i7 (.D(n12753), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i7.GSR = "DISABLED";
    FD1P3IX init_shadow_i6 (.D(n12747), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i6.GSR = "DISABLED";
    FD1P3IX init_shadow_i5 (.D(n12741), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i5.GSR = "DISABLED";
    FD1P3IX init_shadow_i4 (.D(n12735), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i4.GSR = "DISABLED";
    FD1P3IX init_shadow_i3 (.D(n12729), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i3.GSR = "DISABLED";
    FD1P3IX init_shadow_i2 (.D(n12723), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i2.GSR = "DISABLED";
    FD1P3IX init_shadow_i1 (.D(n12717), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i1.GSR = "DISABLED";
    FD1P3IX ev_ch_i6 (.D(ev_ch_6__N_1948[6]), .SP(pll_clk_enable_474), .CD(n20303), 
            .CK(pll_clk), .Q(ev_ch[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_ch_i6.GSR = "DISABLED";
    FD1P3IX ev_ch_i5 (.D(ev_ch_6__N_1948[5]), .SP(pll_clk_enable_474), .CD(n20303), 
            .CK(pll_clk), .Q(ev_ch[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_ch_i5.GSR = "DISABLED";
    FD1P3IX ev_ch_i4 (.D(ev_ch_6__N_1948[4]), .SP(pll_clk_enable_474), .CD(n20303), 
            .CK(pll_clk), .Q(ev_ch[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_ch_i4.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i2 (.D(spi_byte_count_15__N_1694[2]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i3 (.D(spi_byte_count_15__N_1694[3]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i4 (.D(spi_byte_count_15__N_1694[4]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i5 (.D(spi_byte_count_15__N_1694[5]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i6 (.D(spi_byte_count_15__N_1694[6]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i7 (.D(spi_byte_count_15__N_1694[7]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i8 (.D(spi_byte_count_15__N_1694[8]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i9 (.D(spi_byte_count_15__N_1694[9]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i10 (.D(spi_byte_count_15__N_1694[10]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i11 (.D(spi_byte_count_15__N_1694[11]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i12 (.D(spi_byte_count_15__N_1694[12]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i13 (.D(spi_byte_count_15__N_1694[13]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i14 (.D(spi_byte_count_15__N_1694[14]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i15 (.D(spi_byte_count_15__N_1694[15]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_byte_count_i0_i15.GSR = "ENABLED";
    FD1P3AX status_hold__i2 (.D(accepted_sequence[25]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i2.GSR = "DISABLED";
    FD1P3IX ev_ch_i3 (.D(ev_ch_6__N_1948[3]), .SP(pll_clk_enable_474), .CD(n20303), 
            .CK(pll_clk), .Q(ev_ch[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_ch_i3.GSR = "DISABLED";
    LUT4 i3_4_lut_adj_149 (.A(spi_byte_count[4]), .B(n23070), .C(spi_byte_count[5]), 
         .D(n23857), .Z(spi1_sck_c_enable_182)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i3_4_lut_adj_149.init = 16'h0800;
    LUT4 accepted_sequence_31__I_0_i4_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[3]), .D(pending_sequence[3]), 
         .Z(accepted_sequence_31__N_1122[3])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i4_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i1_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[0]), .D(pending_sequence[0]), 
         .Z(accepted_sequence_31__N_1122[0])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i1_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_adj_150 (.A(n14467), .B(init_shadow[34]), .C(ev_bit[34]), 
         .Z(n12915)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_150.init = 16'hecec;
    LUT4 accepted_sequence_31__I_0_i5_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[4]), .D(pending_sequence[4]), 
         .Z(accepted_sequence_31__N_1122[4])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i5_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_4_lut_adj_151 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[2]), 
         .D(ev_bit[2]), .Z(ev_wr_data[2])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_151.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_152 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[3]), 
         .D(ev_bit[3]), .Z(ev_wr_data[3])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_152.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_153 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[4]), 
         .D(ev_bit[4]), .Z(ev_wr_data[4])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_153.init = 16'h7770;
    LUT4 i14776_4_lut (.A(status_bit_index[4]), .B(status_bit_index[5]), 
         .C(status_bit_index[3]), .D(n6), .Z(spi1_sck_N_416_enable_7)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i14776_4_lut.init = 16'h7fff;
    LUT4 i1_3_lut_adj_154 (.A(n14467), .B(init_shadow[33]), .C(ev_bit[33]), 
         .Z(n12909)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_154.init = 16'hecec;
    LUT4 i6_4_lut_adj_155 (.A(time_divider[2]), .B(n12_adj_3180), .C(time_divider[6]), 
         .D(time_divider[1]), .Z(pll_clk_enable_639)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_155.init = 16'h8000;
    LUT4 i5_4_lut (.A(time_divider[0]), .B(time_divider[5]), .C(time_divider[4]), 
         .D(time_divider[3]), .Z(n12_adj_3180)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i5_4_lut.init = 16'h8000;
    LUT4 accepted_sequence_31__I_0_i6_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[5]), .D(pending_sequence[5]), 
         .Z(accepted_sequence_31__N_1122[5])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i6_3_lut_4_lut.init = 16'hf960;
    LUT4 i13589_1_lut (.A(spi_bit_count[0]), .Z(n20)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(360[34:54])
    defparam i13589_1_lut.init = 16'h5555;
    LUT4 i1_3_lut_adj_156 (.A(n14467), .B(init_shadow[32]), .C(ev_bit[32]), 
         .Z(n12903)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_156.init = 16'hecec;
    LUT4 i13611_1_lut (.A(mic_sample_count[0]), .Z(n30)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[37:60])
    defparam i13611_1_lut.init = 16'h5555;
    LUT4 accepted_sequence_31__I_0_i7_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[6]), .D(pending_sequence[6]), 
         .Z(accepted_sequence_31__N_1122[6])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i7_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_4_lut_adj_157 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[5]), 
         .D(ev_bit[5]), .Z(ev_wr_data[5])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_157.init = 16'h7770;
    LUT4 accepted_sequence_31__I_0_i8_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[7]), .D(pending_sequence[7]), 
         .Z(accepted_sequence_31__N_1122[7])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i8_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_4_lut_adj_158 (.A(n23143), .B(spi1_sck_c_enable_197), 
         .C(n23881), .D(spi_byte_count[0]), .Z(n23013)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i1_3_lut_4_lut_adj_158.init = 16'hffbf;
    LUT4 accepted_sequence_31__I_0_i9_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[8]), .D(pending_sequence[8]), 
         .Z(accepted_sequence_31__N_1122[8])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i9_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i10_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[9]), .D(pending_sequence[9]), 
         .Z(accepted_sequence_31__N_1122[9])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i10_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i11_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[10]), .D(pending_sequence[10]), 
         .Z(accepted_sequence_31__N_1122[10])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i11_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i12_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[11]), .D(pending_sequence[11]), 
         .Z(accepted_sequence_31__N_1122[11])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i12_3_lut_4_lut.init = 16'hf960;
    FD1P3IX ev_bit_i8 (.D(ev_bit[7]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i8.GSR = "DISABLED";
    FD1P3IX ev_bit_i31 (.D(ev_bit[30]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i31.GSR = "DISABLED";
    LUT4 i11924_1_lut (.A(status_bit_index[1]), .Z(spi1_miso_N_2513[1])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam i11924_1_lut.init = 16'h5555;
    LUT4 accepted_sequence_31__I_0_i13_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[12]), .D(pending_sequence[12]), 
         .Z(accepted_sequence_31__N_1122[12])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i13_3_lut_4_lut.init = 16'hf960;
    LUT4 i14793_3_lut_3_lut (.A(n23856), .B(n14026), .C(n23888), .Z(invalid_frame_spi_N_2542)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(346[30] 356[24])
    defparam i14793_3_lut_3_lut.init = 16'h4040;
    LUT4 accepted_sequence_31__I_0_i14_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[13]), .D(pending_sequence[13]), 
         .Z(accepted_sequence_31__N_1122[13])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i14_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_adj_159 (.A(n14467), .B(init_shadow[10]), .C(ev_bit[10]), 
         .Z(n12771)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_159.init = 16'hecec;
    LUT4 i3_4_lut_rep_64 (.A(spi_command[4]), .B(n23413), .C(spi_version[0]), 
         .D(n23441), .Z(n23856)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i3_4_lut_rep_64.init = 16'h0020;
    LUT4 i35_1_lut (.A(fpga_cs_n_c), .Z(fpga_cs_n_N_2559)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i35_1_lut.init = 16'h5555;
    LUT4 i1_3_lut_adj_160 (.A(n14467), .B(init_shadow[59]), .C(ev_bit[59]), 
         .Z(n13069)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_160.init = 16'hecec;
    LUT4 i1_2_lut_3_lut_adj_161 (.A(spi_channel_field[1]), .B(spi1_sck_c_enable_237), 
         .C(spi_channel_field[0]), .Z(spi1_sck_c_enable_63)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i1_2_lut_3_lut_adj_161.init = 16'h0404;
    LUT4 accepted_sequence_31__I_0_i15_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[14]), .D(pending_sequence[14]), 
         .Z(accepted_sequence_31__N_1122[14])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i15_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i16_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[15]), .D(pending_sequence[15]), 
         .Z(accepted_sequence_31__N_1122[15])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i16_3_lut_4_lut.init = 16'hf960;
    FD1P3AX status_hold__i3 (.D(accepted_sequence[26]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i3.GSR = "DISABLED";
    FD1P3AX status_hold__i4 (.D(accepted_sequence[27]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i4.GSR = "DISABLED";
    FD1P3AX status_hold__i5 (.D(accepted_sequence[28]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i5.GSR = "DISABLED";
    FD1P3AX status_hold__i6 (.D(accepted_sequence[29]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i6.GSR = "DISABLED";
    FD1P3AX status_hold__i7 (.D(accepted_sequence[30]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i7.GSR = "DISABLED";
    FD1P3AX status_hold__i8 (.D(accepted_sequence[31]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i8.GSR = "DISABLED";
    FD1P3AX status_hold__i9 (.D(accepted_sequence[16]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i9.GSR = "DISABLED";
    FD1P3AX status_hold__i10 (.D(accepted_sequence[17]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i10.GSR = "DISABLED";
    FD1P3AX status_hold__i11 (.D(accepted_sequence[18]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i11.GSR = "DISABLED";
    FD1P3AX status_hold__i12 (.D(accepted_sequence[19]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i12.GSR = "DISABLED";
    FD1P3AX status_hold__i13 (.D(accepted_sequence[20]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i13.GSR = "DISABLED";
    FD1P3AX status_hold__i14 (.D(accepted_sequence[21]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i14.GSR = "DISABLED";
    FD1P3AX status_hold__i15 (.D(accepted_sequence[22]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i15.GSR = "DISABLED";
    FD1P3AX status_hold__i16 (.D(accepted_sequence[23]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i16.GSR = "DISABLED";
    FD1P3AX status_hold__i17 (.D(accepted_sequence[8]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i17.GSR = "DISABLED";
    FD1P3AX status_hold__i18 (.D(accepted_sequence[9]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i18.GSR = "DISABLED";
    FD1P3AX status_hold__i19 (.D(accepted_sequence[10]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i19.GSR = "DISABLED";
    FD1P3AX status_hold__i20 (.D(accepted_sequence[11]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i20.GSR = "DISABLED";
    FD1P3AX status_hold__i21 (.D(accepted_sequence[12]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i21.GSR = "DISABLED";
    FD1P3AX status_hold__i22 (.D(accepted_sequence[13]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i22.GSR = "DISABLED";
    FD1P3AX status_hold__i23 (.D(accepted_sequence[14]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i23.GSR = "DISABLED";
    FD1P3AX status_hold__i24 (.D(accepted_sequence[15]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i24.GSR = "DISABLED";
    FD1P3AX status_hold__i25 (.D(accepted_sequence[0]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i25.GSR = "DISABLED";
    FD1P3AX status_hold__i26 (.D(accepted_sequence[1]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i26.GSR = "DISABLED";
    FD1P3AX status_hold__i27 (.D(accepted_sequence[2]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i27.GSR = "DISABLED";
    FD1P3AX status_hold__i28 (.D(accepted_sequence[3]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i28.GSR = "DISABLED";
    FD1P3AX status_hold__i29 (.D(accepted_sequence[4]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i29.GSR = "DISABLED";
    FD1P3AX status_hold__i30 (.D(accepted_sequence[5]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i30.GSR = "DISABLED";
    FD1P3AX status_hold__i31 (.D(accepted_sequence[6]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i31.GSR = "DISABLED";
    FD1P3AX status_hold__i32 (.D(accepted_sequence[7]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i32.GSR = "DISABLED";
    FD1P3AX status_hold__i33 (.D(fpga_time[24]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i33.GSR = "DISABLED";
    FD1P3AX status_hold__i34 (.D(fpga_time[25]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i34.GSR = "DISABLED";
    FD1P3AX status_hold__i35 (.D(fpga_time[26]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i35.GSR = "DISABLED";
    FD1P3AX status_hold__i36 (.D(fpga_time[27]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i36.GSR = "DISABLED";
    FD1P3AX status_hold__i37 (.D(fpga_time[28]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i37.GSR = "DISABLED";
    FD1P3AX status_hold__i38 (.D(fpga_time[29]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i38.GSR = "DISABLED";
    FD1P3AX status_hold__i39 (.D(fpga_time[30]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i39.GSR = "DISABLED";
    FD1P3AX status_hold__i40 (.D(fpga_time[31]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i40.GSR = "DISABLED";
    FD1P3AX status_hold__i41 (.D(fpga_time[16]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i41.GSR = "DISABLED";
    FD1P3AX status_hold__i42 (.D(fpga_time[17]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i42.GSR = "DISABLED";
    FD1P3AX status_hold__i43 (.D(fpga_time[18]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i43.GSR = "DISABLED";
    FD1P3AX status_hold__i44 (.D(fpga_time[19]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i44.GSR = "DISABLED";
    FD1P3AX status_hold__i45 (.D(fpga_time[20]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i45.GSR = "DISABLED";
    FD1P3AX status_hold__i46 (.D(fpga_time[21]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i46.GSR = "DISABLED";
    FD1P3AX status_hold__i47 (.D(fpga_time[22]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i47.GSR = "DISABLED";
    FD1P3AX status_hold__i48 (.D(fpga_time[23]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i48.GSR = "DISABLED";
    FD1P3AX status_hold__i49 (.D(fpga_time[8]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i49.GSR = "DISABLED";
    FD1P3AX status_hold__i50 (.D(fpga_time[9]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i50.GSR = "DISABLED";
    FD1P3AX status_hold__i51 (.D(fpga_time[10]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i51.GSR = "DISABLED";
    FD1P3AX status_hold__i52 (.D(fpga_time[11]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i52.GSR = "DISABLED";
    FD1P3AX status_hold__i53 (.D(fpga_time[12]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i53.GSR = "DISABLED";
    FD1P3AX status_hold__i54 (.D(fpga_time[13]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i54.GSR = "DISABLED";
    FD1P3AX status_hold__i55 (.D(fpga_time[14]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i55.GSR = "DISABLED";
    FD1P3AX status_hold__i56 (.D(fpga_time[15]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i56.GSR = "DISABLED";
    FD1P3AX status_hold__i57 (.D(fpga_time[0]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i57.GSR = "DISABLED";
    FD1P3AX status_hold__i58 (.D(fpga_time[1]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i58.GSR = "DISABLED";
    FD1P3AX status_hold__i59 (.D(fpga_time[2]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i59.GSR = "DISABLED";
    FD1P3AX status_hold__i60 (.D(fpga_time[3]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i60.GSR = "DISABLED";
    FD1P3AX status_hold__i61 (.D(fpga_time[4]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i61.GSR = "DISABLED";
    FD1P3AX status_hold__i62 (.D(fpga_time[5]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i62.GSR = "DISABLED";
    FD1P3AX status_hold__i63 (.D(fpga_time[6]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i63.GSR = "DISABLED";
    FD1P3AX status_hold__i64 (.D(fpga_time[7]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i64.GSR = "DISABLED";
    FD1P3AX status_hold__i65 (.D(status_flags_wire_15__N_1385[2]), .SP(cs_fall), 
            .CK(pll_clk), .Q(status_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i65.GSR = "DISABLED";
    FD1P3AX status_hold__i66 (.D(status_flags_wire_15__N_1401[4]), .SP(cs_fall), 
            .CK(pll_clk), .Q(status_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i66.GSR = "DISABLED";
    FD1P3AX status_hold__i67 (.D(n23870), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i67.GSR = "DISABLED";
    FD1P3AX status_hold__i68 (.D(fifo_credit_wire[0]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[104])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam status_hold__i68.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i1 (.D(accepted_sequence_31__N_1122[1]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i1.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_162 (.A(frame_settle[1]), .B(frame_settle[0]), .Z(n14170)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_162.init = 16'h9999;
    LUT4 accepted_sequence_31__I_0_i17_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[16]), .D(pending_sequence[16]), 
         .Z(accepted_sequence_31__N_1122[16])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i17_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i18_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[17]), .D(pending_sequence[17]), 
         .Z(accepted_sequence_31__N_1122[17])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i18_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_adj_163 (.A(n14467), .B(init_shadow[29]), .C(ev_bit[29]), 
         .Z(n12885)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_163.init = 16'hecec;
    LUT4 i1_3_lut_adj_164 (.A(n14467), .B(init_shadow[28]), .C(ev_bit[28]), 
         .Z(n12879)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_164.init = 16'hecec;
    LUT4 i1_4_lut_adj_165 (.A(ev_state[3]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(n1_adj_3205), .Z(n85)) /* synthesis lut_function=(!(A ((C+(D))+!B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    defparam i1_4_lut_adj_165.init = 16'h444c;
    LUT4 i1_3_lut_adj_166 (.A(n14467), .B(init_shadow[27]), .C(ev_bit[27]), 
         .Z(n12873)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_166.init = 16'hecec;
    LUT4 accepted_sequence_31__I_0_i19_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[18]), .D(pending_sequence[18]), 
         .Z(accepted_sequence_31__N_1122[18])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i19_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_4_lut_adj_167 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[6]), 
         .D(ev_bit[6]), .Z(ev_wr_data[6])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_167.init = 16'h7770;
    LUT4 accepted_sequence_31__I_0_i20_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[19]), .D(pending_sequence[19]), 
         .Z(accepted_sequence_31__N_1122[19])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i20_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i21_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[20]), .D(pending_sequence[20]), 
         .Z(accepted_sequence_31__N_1122[20])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i21_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_4_lut_adj_168 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[7]), 
         .D(ev_bit[7]), .Z(ev_wr_data[7])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_168.init = 16'h7770;
    LUT4 accepted_sequence_31__I_0_i22_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[21]), .D(pending_sequence[21]), 
         .Z(accepted_sequence_31__N_1122[21])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i22_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i23_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[22]), .D(pending_sequence[22]), 
         .Z(accepted_sequence_31__N_1122[22])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i23_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i24_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[23]), .D(pending_sequence[23]), 
         .Z(accepted_sequence_31__N_1122[23])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(452[13:53])
    defparam accepted_sequence_31__I_0_i24_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_adj_169 (.A(n14467), .B(init_shadow[31]), .C(ev_bit[31]), 
         .Z(n12897)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_169.init = 16'hecec;
    LUT4 i1_3_lut_adj_170 (.A(n14467), .B(init_shadow[9]), .C(ev_bit[9]), 
         .Z(n12765)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_170.init = 16'hecec;
    FD1P3AX accepted_sequence_i2 (.D(accepted_sequence_31__N_1122[2]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i3 (.D(accepted_sequence_31__N_1122[3]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i4 (.D(accepted_sequence_31__N_1122[4]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i5 (.D(accepted_sequence_31__N_1122[5]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i6 (.D(accepted_sequence_31__N_1122[6]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i7 (.D(accepted_sequence_31__N_1122[7]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i8 (.D(accepted_sequence_31__N_1122[8]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i9 (.D(accepted_sequence_31__N_1122[9]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i10 (.D(accepted_sequence_31__N_1122[10]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i11 (.D(accepted_sequence_31__N_1122[11]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i12 (.D(accepted_sequence_31__N_1122[12]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i13 (.D(accepted_sequence_31__N_1122[13]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i14 (.D(accepted_sequence_31__N_1122[14]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i15 (.D(accepted_sequence_31__N_1122[15]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i16 (.D(accepted_sequence_31__N_1122[16]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i17 (.D(accepted_sequence_31__N_1122[17]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i18 (.D(accepted_sequence_31__N_1122[18]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i19 (.D(accepted_sequence_31__N_1122[19]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i20 (.D(accepted_sequence_31__N_1122[20]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i21 (.D(accepted_sequence_31__N_1122[21]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i22 (.D(accepted_sequence_31__N_1122[22]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i23 (.D(accepted_sequence_31__N_1122[23]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i24 (.D(accepted_sequence_31__N_1122[24]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i25 (.D(accepted_sequence_31__N_1122[25]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i26 (.D(accepted_sequence_31__N_1122[26]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i27 (.D(accepted_sequence_31__N_1122[27]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i28 (.D(accepted_sequence_31__N_1122[28]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i29 (.D(accepted_sequence_31__N_1122[29]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i30 (.D(accepted_sequence_31__N_1122[30]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i31 (.D(accepted_sequence_31__N_1122[31]), .SP(pll_clk_enable_109), 
            .CK(pll_clk), .Q(accepted_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_i31.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i1 (.D(global_phase_s2[1]), .SP(pll_clk_enable_117), 
            .CK(pll_clk), .Q(run_addr_s3[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam run_addr_s3_i1.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i2 (.D(global_phase_s2[2]), .SP(pll_clk_enable_117), 
            .CK(pll_clk), .Q(run_addr_s3[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam run_addr_s3_i2.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i3 (.D(global_phase_s2[3]), .SP(pll_clk_enable_117), 
            .CK(pll_clk), .Q(run_addr_s3[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam run_addr_s3_i3.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i4 (.D(global_phase_s2[4]), .SP(pll_clk_enable_117), 
            .CK(pll_clk), .Q(run_addr_s3[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam run_addr_s3_i4.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i5 (.D(global_phase_s2[5]), .SP(pll_clk_enable_117), 
            .CK(pll_clk), .Q(run_addr_s3[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam run_addr_s3_i5.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i6 (.D(global_phase_s2[6]), .SP(pll_clk_enable_117), 
            .CK(pll_clk), .Q(run_addr_s3[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam run_addr_s3_i6.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i7 (.D(global_phase_s2[7]), .SP(pll_clk_enable_117), 
            .CK(pll_clk), .Q(run_addr_s3[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam run_addr_s3_i7.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i8 (.D(run_addr_s3_8__N_425[8]), .SP(pll_clk_enable_117), 
            .CK(pll_clk), .Q(run_addr_s3[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam run_addr_s3_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i1 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i2 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i3 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i4 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i5 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i6 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i7 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i8 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i9 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i10 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i11 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i12 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i13 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i14 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i15 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_0_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_l_i0_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i1 (.D(accepted_sequence_spi[1]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i2 (.D(accepted_sequence_spi[2]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i3 (.D(accepted_sequence_spi[3]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i4 (.D(accepted_sequence_spi[4]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i5 (.D(accepted_sequence_spi[5]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i6 (.D(accepted_sequence_spi[6]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i7 (.D(accepted_sequence_spi[7]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i8 (.D(accepted_sequence_spi[8]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i9 (.D(accepted_sequence_spi[9]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i10 (.D(accepted_sequence_spi[10]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i11 (.D(accepted_sequence_spi[11]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i12 (.D(accepted_sequence_spi[12]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i13 (.D(accepted_sequence_spi[13]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i14 (.D(accepted_sequence_spi[14]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i15 (.D(accepted_sequence_spi[15]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i16 (.D(accepted_sequence_spi[16]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i17 (.D(accepted_sequence_spi[17]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i18 (.D(accepted_sequence_spi[18]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i19 (.D(accepted_sequence_spi[19]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i20 (.D(accepted_sequence_spi[20]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i21 (.D(accepted_sequence_spi[21]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i22 (.D(accepted_sequence_spi[22]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i23 (.D(accepted_sequence_spi[23]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i24 (.D(accepted_sequence_spi[24]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i25 (.D(accepted_sequence_spi[25]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i26 (.D(accepted_sequence_spi[26]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i27 (.D(accepted_sequence_spi[27]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i28 (.D(accepted_sequence_spi[28]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i29 (.D(accepted_sequence_spi[29]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i30 (.D(accepted_sequence_spi[30]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i31 (.D(accepted_sequence_spi[31]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_meta_i31.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i1 (.D(accepted_sequence_meta[1]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i2 (.D(accepted_sequence_meta[2]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i3 (.D(accepted_sequence_meta[3]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i4 (.D(accepted_sequence_meta[4]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i5 (.D(accepted_sequence_meta[5]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i6 (.D(accepted_sequence_meta[6]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i7 (.D(accepted_sequence_meta[7]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i8 (.D(accepted_sequence_meta[8]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i9 (.D(accepted_sequence_meta[9]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i10 (.D(accepted_sequence_meta[10]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i11 (.D(accepted_sequence_meta[11]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i12 (.D(accepted_sequence_meta[12]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i13 (.D(accepted_sequence_meta[13]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i14 (.D(accepted_sequence_meta[14]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i15 (.D(accepted_sequence_meta[15]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i16 (.D(accepted_sequence_meta[16]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i17 (.D(accepted_sequence_meta[17]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i18 (.D(accepted_sequence_meta[18]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i19 (.D(accepted_sequence_meta[19]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i20 (.D(accepted_sequence_meta[20]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i21 (.D(accepted_sequence_meta[21]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i22 (.D(accepted_sequence_meta[22]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i23 (.D(accepted_sequence_meta[23]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i24 (.D(accepted_sequence_meta[24]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i25 (.D(accepted_sequence_meta[25]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i26 (.D(accepted_sequence_meta[26]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i27 (.D(accepted_sequence_meta[27]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i28 (.D(accepted_sequence_meta[28]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i29 (.D(accepted_sequence_meta[29]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i30 (.D(accepted_sequence_meta[30]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i31 (.D(accepted_sequence_meta[31]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam accepted_sequence_sync_i31.GSR = "DISABLED";
    FD1P3IX us_tx__i2 (.D(n11407), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i2.GSR = "DISABLED";
    FD1P3IX us_tx__i3 (.D(n11409), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i3.GSR = "DISABLED";
    FD1P3IX us_tx__i4 (.D(n11411), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i4.GSR = "DISABLED";
    FD1P3IX us_tx__i5 (.D(n11413), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i5.GSR = "DISABLED";
    FD1P3IX us_tx__i6 (.D(n11415), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i6.GSR = "DISABLED";
    FD1P3IX us_tx__i7 (.D(n11417), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_6)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i7.GSR = "DISABLED";
    FD1P3IX us_tx__i8 (.D(n11419), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_7)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i8.GSR = "DISABLED";
    FD1P3IX us_tx__i9 (.D(n11421), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_8)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i9.GSR = "DISABLED";
    FD1P3IX us_tx__i10 (.D(n11423), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_9)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i10.GSR = "DISABLED";
    FD1P3IX us_tx__i11 (.D(n11425), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_10)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i11.GSR = "DISABLED";
    FD1P3IX us_tx__i12 (.D(n11427), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_11)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i12.GSR = "DISABLED";
    FD1P3IX us_tx__i13 (.D(n11429), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_12)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i13.GSR = "DISABLED";
    FD1P3IX us_tx__i14 (.D(n11431), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_13)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i14.GSR = "DISABLED";
    FD1P3IX us_tx__i15 (.D(n11433), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_14)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i15.GSR = "DISABLED";
    FD1P3IX us_tx__i16 (.D(n11435), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_15)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i16.GSR = "DISABLED";
    FD1P3IX us_tx__i17 (.D(n11437), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_16)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i17.GSR = "DISABLED";
    FD1P3IX us_tx__i18 (.D(n11439), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_17)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i18.GSR = "DISABLED";
    FD1P3IX us_tx__i19 (.D(n11441), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_18)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i19.GSR = "DISABLED";
    FD1P3IX us_tx__i20 (.D(n11443), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_19)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i20.GSR = "DISABLED";
    FD1P3IX us_tx__i21 (.D(n11445), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_20)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i21.GSR = "DISABLED";
    FD1P3IX us_tx__i22 (.D(n11447), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_21)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i22.GSR = "DISABLED";
    FD1P3IX us_tx__i23 (.D(n11449), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_22)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i23.GSR = "DISABLED";
    FD1P3IX us_tx__i24 (.D(n11451), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_23)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i24.GSR = "DISABLED";
    FD1P3IX us_tx__i25 (.D(n11453), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_24)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i25.GSR = "DISABLED";
    FD1P3IX us_tx__i26 (.D(n11455), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_25)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i26.GSR = "DISABLED";
    FD1P3IX us_tx__i27 (.D(n11457), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_26)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i27.GSR = "DISABLED";
    FD1P3IX us_tx__i28 (.D(n11459), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_27)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i28.GSR = "DISABLED";
    FD1P3IX us_tx__i29 (.D(n11461), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_28)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i29.GSR = "DISABLED";
    FD1P3IX us_tx__i30 (.D(n11463), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_29)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i30.GSR = "DISABLED";
    FD1P3IX us_tx__i31 (.D(n11465), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_30)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i31.GSR = "DISABLED";
    FD1P3IX us_tx__i32 (.D(n11467), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_31)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i32.GSR = "DISABLED";
    FD1P3IX us_tx__i33 (.D(n11469), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_32)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i33.GSR = "DISABLED";
    FD1P3IX us_tx__i34 (.D(n11471), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_33)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i34.GSR = "DISABLED";
    FD1P3IX us_tx__i35 (.D(n11473), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_34)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i35.GSR = "DISABLED";
    FD1P3IX us_tx__i36 (.D(n11475), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_35)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i36.GSR = "DISABLED";
    FD1P3IX us_tx__i37 (.D(n11477), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_36)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i37.GSR = "DISABLED";
    FD1P3IX us_tx__i38 (.D(n11479), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_37)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i38.GSR = "DISABLED";
    FD1P3IX us_tx__i39 (.D(n11481), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_38)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i39.GSR = "DISABLED";
    FD1P3IX us_tx__i40 (.D(n11483), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_39)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i40.GSR = "DISABLED";
    FD1P3IX us_tx__i41 (.D(n11485), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_40)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i41.GSR = "DISABLED";
    FD1P3IX us_tx__i42 (.D(n11487), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_41)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i42.GSR = "DISABLED";
    FD1P3IX us_tx__i43 (.D(n11489), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_42)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i43.GSR = "DISABLED";
    FD1P3IX us_tx__i44 (.D(n11491), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_43)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i44.GSR = "DISABLED";
    FD1P3IX us_tx__i45 (.D(n11493), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_44)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i45.GSR = "DISABLED";
    FD1P3IX us_tx__i46 (.D(n11495), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_45)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i46.GSR = "DISABLED";
    FD1P3IX us_tx__i47 (.D(n11497), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_46)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i47.GSR = "DISABLED";
    FD1P3IX us_tx__i48 (.D(n11499), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_47)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i48.GSR = "DISABLED";
    FD1P3IX us_tx__i49 (.D(n11501), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_48)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i49.GSR = "DISABLED";
    FD1P3IX us_tx__i50 (.D(n11503), .SP(pll_clk_enable_181), .CD(n24239), 
            .CK(pll_clk), .Q(us_tx_c_49)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i50.GSR = "DISABLED";
    FD1P3IX us_tx__i51 (.D(n11505), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_50)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i51.GSR = "DISABLED";
    FD1P3IX us_tx__i52 (.D(n11507), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_51)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i52.GSR = "DISABLED";
    FD1P3IX us_tx__i53 (.D(n11509), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_52)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i53.GSR = "DISABLED";
    FD1P3IX us_tx__i54 (.D(n11511), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_53)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i54.GSR = "DISABLED";
    FD1P3IX us_tx__i55 (.D(n11513), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_54)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i55.GSR = "DISABLED";
    FD1P3IX us_tx__i56 (.D(n11515), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_55)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i56.GSR = "DISABLED";
    FD1P3IX us_tx__i57 (.D(n11517), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_56)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i57.GSR = "DISABLED";
    FD1P3IX us_tx__i58 (.D(n11519), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_57)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i58.GSR = "DISABLED";
    FD1P3IX us_tx__i59 (.D(n11521), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_58)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i59.GSR = "DISABLED";
    FD1P3IX us_tx__i60 (.D(n11523), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_59)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i60.GSR = "DISABLED";
    FD1P3IX us_tx__i61 (.D(n11525), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_60)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i61.GSR = "DISABLED";
    FD1P3IX us_tx__i62 (.D(n11527), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_61)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i62.GSR = "DISABLED";
    FD1P3IX us_tx__i63 (.D(n11529), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_62)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i63.GSR = "DISABLED";
    FD1P3IX us_tx__i64 (.D(n11531), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_63)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i64.GSR = "DISABLED";
    FD1P3IX us_tx__i65 (.D(n11533), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_64)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i65.GSR = "DISABLED";
    FD1P3IX us_tx__i66 (.D(n11535), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_65)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i66.GSR = "DISABLED";
    FD1P3IX us_tx__i67 (.D(n11537), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_66)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i67.GSR = "DISABLED";
    FD1P3IX us_tx__i68 (.D(n11539), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_67)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i68.GSR = "DISABLED";
    FD1P3IX us_tx__i69 (.D(n11541), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_68)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i69.GSR = "DISABLED";
    FD1P3IX us_tx__i70 (.D(n11543), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_69)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i70.GSR = "DISABLED";
    FD1P3IX us_tx__i71 (.D(n11545), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_70)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i71.GSR = "DISABLED";
    FD1P3IX us_tx__i72 (.D(n11547), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_71)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i72.GSR = "DISABLED";
    FD1P3IX us_tx__i73 (.D(n11549), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_72)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i73.GSR = "DISABLED";
    FD1P3IX us_tx__i74 (.D(n11551), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_73)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i74.GSR = "DISABLED";
    FD1P3IX us_tx__i75 (.D(n11553), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_74)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i75.GSR = "DISABLED";
    FD1P3IX us_tx__i76 (.D(n11555), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_75)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i76.GSR = "DISABLED";
    FD1P3IX us_tx__i77 (.D(n11557), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_76)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i77.GSR = "DISABLED";
    FD1P3IX us_tx__i78 (.D(n11559), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_77)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i78.GSR = "DISABLED";
    FD1P3IX us_tx__i79 (.D(n11561), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_78)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i79.GSR = "DISABLED";
    FD1P3IX us_tx__i80 (.D(n11563), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_79)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i80.GSR = "DISABLED";
    FD1P3IX us_tx__i81 (.D(n11565), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_80)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i81.GSR = "DISABLED";
    FD1P3IX us_tx__i82 (.D(n11567), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_81)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i82.GSR = "DISABLED";
    FD1P3IX us_tx__i83 (.D(n11569), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_82)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i83.GSR = "DISABLED";
    FD1P3IX us_tx__i84 (.D(n11571), .SP(pll_clk_enable_215), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_83)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam us_tx__i84.GSR = "DISABLED";
    LUT4 i13640_2_lut (.A(staging_q[8]), .B(staging_q[0]), .Z(build_sum_8__N_2053[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13640_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_adj_171 (.A(n14467), .B(init_shadow[40]), .C(ev_bit[40]), 
         .Z(n12951)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_171.init = 16'hecec;
    LUT4 i14765_3_lut_4_lut_4_lut_4_lut (.A(ev_state[2]), .B(n20289), .C(n85), 
         .D(ev_state[0]), .Z(n20291)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+(D))+!B !((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i14765_3_lut_4_lut_4_lut_4_lut.init = 16'h44f0;
    LUT4 i11491_4_lut_3_lut (.A(ev_state[2]), .B(ev_state_3__N_1940[1]), 
         .C(ev_state[1]), .Z(n20287)) /* synthesis lut_function=(!(A (C)+!A !(B+(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i11491_4_lut_3_lut.init = 16'h5e5e;
    LUT4 i1_3_lut_adj_172 (.A(n14467), .B(init_shadow[8]), .C(ev_bit[8]), 
         .Z(n12759)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_172.init = 16'hecec;
    FD1P3AX ev_state_i2 (.D(ev_state_3__N_697[2]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(ev_state[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_state_i2.GSR = "DISABLED";
    FD1S3AX ev_state_i3 (.D(n23924), .CK(pll_clk), .Q(ev_state[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_state_i3.GSR = "DISABLED";
    FD1P3AX build_phase_i1 (.D(staging_q[9]), .SP(pll_clk_enable_231), .CK(pll_clk), 
            .Q(build_phase[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_phase_i1.GSR = "DISABLED";
    FD1P3AX build_phase_i2 (.D(staging_q[10]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_phase[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_phase_i2.GSR = "DISABLED";
    FD1P3AX build_phase_i3 (.D(staging_q[11]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_phase[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_phase_i3.GSR = "DISABLED";
    FD1P3AX build_phase_i4 (.D(staging_q[12]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_phase[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_phase_i4.GSR = "DISABLED";
    FD1P3AX build_phase_i5 (.D(staging_q[13]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_phase[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_phase_i5.GSR = "DISABLED";
    FD1P3AX build_phase_i6 (.D(staging_q[14]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_phase[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_phase_i6.GSR = "DISABLED";
    FD1P3AX build_phase_i7 (.D(staging_q[15]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_phase[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_phase_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i1 (.D(build_sum_8__N_2053[1]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_sum[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_sum_i1.GSR = "DISABLED";
    FD1P3AX build_sum_i2 (.D(build_sum_8__N_2053[2]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_sum[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_sum_i2.GSR = "DISABLED";
    FD1P3AX build_sum_i3 (.D(build_sum_8__N_2053[3]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_sum[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_sum_i3.GSR = "DISABLED";
    FD1P3AX build_sum_i4 (.D(build_sum_8__N_2053[4]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_sum[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_sum_i4.GSR = "DISABLED";
    FD1P3AX build_sum_i5 (.D(build_sum_8__N_2053[5]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_sum[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_sum_i5.GSR = "DISABLED";
    FD1P3AX build_sum_i6 (.D(build_sum_8__N_2053[6]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_sum[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_sum_i6.GSR = "DISABLED";
    FD1P3AX build_sum_i7 (.D(build_sum_8__N_2053[7]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_sum[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_sum_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i8 (.D(build_sum_8__N_2053[8]), .SP(pll_clk_enable_231), 
            .CK(pll_clk), .Q(build_sum[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam build_sum_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i1 (.D(ev_rd_data[1]), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(ev_rd_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i1.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i2 (.D(ev_rd_data[2]), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(ev_rd_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i2.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i3 (.D(ev_rd_data[3]), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(ev_rd_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i3.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i4 (.D(ev_rd_data[4]), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(ev_rd_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i4.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i5 (.D(ev_rd_data[5]), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(ev_rd_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i5.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i6 (.D(ev_rd_data[6]), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(ev_rd_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i6.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i7 (.D(ev_rd_data[7]), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(ev_rd_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i7.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i8 (.D(ev_rd_data[8]), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(ev_rd_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i9 (.D(ev_rd_data[9]), .SP(pll_clk_enable_280), .CK(pll_clk), 
            .Q(ev_rd_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i9.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i10 (.D(ev_rd_data[10]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i10.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i11 (.D(ev_rd_data[11]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i11.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i12 (.D(ev_rd_data[12]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i12.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i13 (.D(ev_rd_data[13]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i13.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i14 (.D(ev_rd_data[14]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i14.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i15 (.D(ev_rd_data[15]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i15.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i16 (.D(ev_rd_data[16]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i16.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i17 (.D(ev_rd_data[17]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i17.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i18 (.D(ev_rd_data[18]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i18.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i19 (.D(ev_rd_data[19]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i19.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i20 (.D(ev_rd_data[20]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i20.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i21 (.D(ev_rd_data[21]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i21.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i22 (.D(ev_rd_data[22]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i22.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i23 (.D(ev_rd_data[23]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i23.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i24 (.D(ev_rd_data[24]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i24.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i25 (.D(ev_rd_data[25]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i25.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i26 (.D(ev_rd_data[26]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i26.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i27 (.D(ev_rd_data[27]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i27.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i28 (.D(ev_rd_data[28]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i28.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i29 (.D(ev_rd_data[29]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i29.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i30 (.D(ev_rd_data[30]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i30.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i31 (.D(ev_rd_data[31]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i31.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i32 (.D(ev_rd_data[32]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i32.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i33 (.D(ev_rd_data[33]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i33.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i34 (.D(ev_rd_data[34]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i34.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i35 (.D(ev_rd_data[35]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i35.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i36 (.D(ev_rd_data[36]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i36.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i37 (.D(ev_rd_data[37]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i37.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i38 (.D(ev_rd_data[38]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i38.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i39 (.D(ev_rd_data[39]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i39.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i40 (.D(ev_rd_data[40]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i40.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i41 (.D(ev_rd_data[41]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i41.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i42 (.D(ev_rd_data[42]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i42.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i43 (.D(ev_rd_data[43]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i43.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i44 (.D(ev_rd_data[44]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i44.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i45 (.D(ev_rd_data[45]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i45.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i46 (.D(ev_rd_data[46]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i46.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i47 (.D(ev_rd_data[47]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i47.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i48 (.D(ev_rd_data[48]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i48.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i49 (.D(ev_rd_data[49]), .SP(pll_clk_enable_280), 
            .CK(pll_clk), .Q(ev_rd_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i49.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i50 (.D(ev_rd_data[50]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i50.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i51 (.D(ev_rd_data[51]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i51.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i52 (.D(ev_rd_data[52]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i52.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i53 (.D(ev_rd_data[53]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i53.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i54 (.D(ev_rd_data[54]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i54.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i55 (.D(ev_rd_data[55]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i55.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i56 (.D(ev_rd_data[56]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i56.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i57 (.D(ev_rd_data[57]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i57.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i58 (.D(ev_rd_data[58]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i58.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i59 (.D(ev_rd_data[59]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i59.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i60 (.D(ev_rd_data[60]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i60.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i61 (.D(ev_rd_data[61]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i61.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i62 (.D(ev_rd_data[62]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i62.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i63 (.D(ev_rd_data[63]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i63.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i64 (.D(ev_rd_data[64]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i64.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i65 (.D(ev_rd_data[65]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i65.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i66 (.D(ev_rd_data[66]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i66.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i67 (.D(ev_rd_data[67]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i67.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i68 (.D(ev_rd_data[68]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i68.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i69 (.D(ev_rd_data[69]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i69.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i70 (.D(ev_rd_data[70]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i70.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i71 (.D(ev_rd_data[71]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i71.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i72 (.D(ev_rd_data[72]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i72.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i73 (.D(ev_rd_data[73]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i73.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i74 (.D(ev_rd_data[74]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i74.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i75 (.D(ev_rd_data[75]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i75.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i76 (.D(ev_rd_data[76]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i76.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i77 (.D(ev_rd_data[77]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i77.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i78 (.D(ev_rd_data[78]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i78.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i79 (.D(ev_rd_data[79]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i79.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i80 (.D(ev_rd_data[80]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i80.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i81 (.D(ev_rd_data[81]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i81.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i82 (.D(ev_rd_data[82]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i82.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i83 (.D(ev_rd_data[83]), .SP(pll_clk_enable_314), 
            .CK(pll_clk), .Q(ev_rd_hold[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_rd_hold_i83.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_173 (.A(n14467), .B(init_shadow[7]), .C(ev_bit[7]), 
         .Z(n12753)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_173.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_174 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[11]), 
         .D(ev_bit[11]), .Z(ev_wr_data[11])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_174.init = 16'h7770;
    LUT4 i1_3_lut_adj_175 (.A(n14467), .B(init_shadow[39]), .C(ev_bit[39]), 
         .Z(n12945)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_175.init = 16'hecec;
    LUT4 i1_3_lut_adj_176 (.A(n14467), .B(init_shadow[38]), .C(ev_bit[38]), 
         .Z(n12939)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_176.init = 16'hecec;
    LUT4 i1_2_lut_adj_177 (.A(spi_channel_index[0]), .B(spi_channel_index[3]), 
         .Z(n8)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(326[43:69])
    defparam i1_2_lut_adj_177.init = 16'hdddd;
    LUT4 i3_4_lut_adj_178 (.A(spi_channel_index[1]), .B(spi_channel_index[5]), 
         .C(spi_channel_index[4]), .D(spi_channel_index[2]), .Z(n22519)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(326[43:69])
    defparam i3_4_lut_adj_178.init = 16'hffdf;
    FD1S3AX staging_rd_addr_i1 (.D(staging_rd_addr_6__N_901[1]), .CK(pll_clk), 
            .Q(staging_rd_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam staging_rd_addr_i1.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i2 (.D(staging_rd_addr_6__N_901[2]), .CK(pll_clk), 
            .Q(staging_rd_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam staging_rd_addr_i2.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i3 (.D(staging_rd_addr_6__N_901[3]), .CK(pll_clk), 
            .Q(staging_rd_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam staging_rd_addr_i3.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i4 (.D(staging_rd_addr_6__N_901[4]), .CK(pll_clk), 
            .Q(staging_rd_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam staging_rd_addr_i4.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i5 (.D(staging_rd_addr_6__N_901[5]), .CK(pll_clk), 
            .Q(staging_rd_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam staging_rd_addr_i5.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i6 (.D(staging_rd_addr_6__N_901[6]), .CK(pll_clk), 
            .Q(staging_rd_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam staging_rd_addr_i6.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_179 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[30]), 
         .D(ev_bit[30]), .Z(ev_wr_data[30])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_179.init = 16'h7770;
    LUT4 i1_2_lut_rep_110 (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .Z(n23902)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[17] 338[24])
    defparam i1_2_lut_rep_110.init = 16'h4444;
    LUT4 i1_3_lut_adj_180 (.A(n14467), .B(init_shadow[6]), .C(ev_bit[6]), 
         .Z(n12747)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_180.init = 16'hecec;
    LUT4 i2_2_lut_rep_89_3_lut (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .C(spi_byte_count[2]), .Z(n23881)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[17] 338[24])
    defparam i2_2_lut_rep_89_3_lut.init = 16'h0404;
    LUT4 i1_3_lut_adj_181 (.A(n14467), .B(init_shadow[5]), .C(ev_bit[5]), 
         .Z(n12741)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_181.init = 16'hecec;
    FD1P3AX pending_sequence_i1 (.D(accepted_sequence_sync[1]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i1.GSR = "DISABLED";
    FD1P3AX pending_sequence_i2 (.D(accepted_sequence_sync[2]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i2.GSR = "DISABLED";
    FD1P3AX pending_sequence_i3 (.D(accepted_sequence_sync[3]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i3.GSR = "DISABLED";
    FD1P3AX pending_sequence_i4 (.D(accepted_sequence_sync[4]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i4.GSR = "DISABLED";
    FD1P3AX pending_sequence_i5 (.D(accepted_sequence_sync[5]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i5.GSR = "DISABLED";
    FD1P3AX pending_sequence_i6 (.D(accepted_sequence_sync[6]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i6.GSR = "DISABLED";
    FD1P3AX pending_sequence_i7 (.D(accepted_sequence_sync[7]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i7.GSR = "DISABLED";
    FD1P3AX pending_sequence_i8 (.D(accepted_sequence_sync[8]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i8.GSR = "DISABLED";
    FD1P3AX pending_sequence_i9 (.D(accepted_sequence_sync[9]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i9.GSR = "DISABLED";
    FD1P3AX pending_sequence_i10 (.D(accepted_sequence_sync[10]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i10.GSR = "DISABLED";
    FD1P3AX pending_sequence_i11 (.D(accepted_sequence_sync[11]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i11.GSR = "DISABLED";
    FD1P3AX pending_sequence_i12 (.D(accepted_sequence_sync[12]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i12.GSR = "DISABLED";
    FD1P3AX pending_sequence_i13 (.D(accepted_sequence_sync[13]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i13.GSR = "DISABLED";
    FD1P3AX pending_sequence_i14 (.D(accepted_sequence_sync[14]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i14.GSR = "DISABLED";
    FD1P3AX pending_sequence_i15 (.D(accepted_sequence_sync[15]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i15.GSR = "DISABLED";
    FD1P3AX pending_sequence_i16 (.D(accepted_sequence_sync[16]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i16.GSR = "DISABLED";
    FD1P3AX pending_sequence_i17 (.D(accepted_sequence_sync[17]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i17.GSR = "DISABLED";
    FD1P3AX pending_sequence_i18 (.D(accepted_sequence_sync[18]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i18.GSR = "DISABLED";
    FD1P3AX pending_sequence_i19 (.D(accepted_sequence_sync[19]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i19.GSR = "DISABLED";
    FD1P3AX pending_sequence_i20 (.D(accepted_sequence_sync[20]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i20.GSR = "DISABLED";
    FD1P3AX pending_sequence_i21 (.D(accepted_sequence_sync[21]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i21.GSR = "DISABLED";
    FD1P3AX pending_sequence_i22 (.D(accepted_sequence_sync[22]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i22.GSR = "DISABLED";
    FD1P3AX pending_sequence_i23 (.D(accepted_sequence_sync[23]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i23.GSR = "DISABLED";
    FD1P3AX pending_sequence_i24 (.D(accepted_sequence_sync[24]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i24.GSR = "DISABLED";
    FD1P3AX pending_sequence_i25 (.D(accepted_sequence_sync[25]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i25.GSR = "DISABLED";
    FD1P3AX pending_sequence_i26 (.D(accepted_sequence_sync[26]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i26.GSR = "DISABLED";
    FD1P3AX pending_sequence_i27 (.D(accepted_sequence_sync[27]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i27.GSR = "DISABLED";
    FD1P3AX pending_sequence_i28 (.D(accepted_sequence_sync[28]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i28.GSR = "DISABLED";
    FD1P3AX pending_sequence_i29 (.D(accepted_sequence_sync[29]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i29.GSR = "DISABLED";
    FD1P3AX pending_sequence_i30 (.D(accepted_sequence_sync[30]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i30.GSR = "DISABLED";
    FD1P3AX pending_sequence_i31 (.D(accepted_sequence_sync[31]), .SP(pll_clk_enable_345), 
            .CK(pll_clk), .Q(pending_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam pending_sequence_i31.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i1 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i2 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i3 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i4 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i5 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i6 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i7 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i8 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i9 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i10 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i11 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i12 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i13 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i14 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i15 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_360), 
            .CK(pll_clk), .Q(mic_shift_1_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_l_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i2 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i3 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i4 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i5 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i6 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i7 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i8 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i9 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i10 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i11 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i12 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i13 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i14 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i15 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_0_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_0_r__i15.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i2 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i3 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i4 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i5 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i6 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i7 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i8 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i9 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i10 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i11 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i12 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i13 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i14 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i15 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_shift_1_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_shift_1_r__i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i1 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i2 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i3 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i4 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i5 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i6 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i7 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i8 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i9 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i10 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i11 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i12 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i13 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i14 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i15 (.D(mic_shift_1_r[14]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i16 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i16.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i17 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i17.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i18 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i18.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i19 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i19.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i20 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i20.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i21 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i21.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i22 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i22.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i23 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i23.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i24 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i24.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i25 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i26 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i26.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i27 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i27.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i28 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i28.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i29 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i29.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i30 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i30.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i31 (.D(mic_shift_1_l[15]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i31.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i32 (.D(mic_data_0_c), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i32.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i33 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i33.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i34 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i34.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i35 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i35.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i36 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i36.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i37 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i37.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i38 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i38.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i39 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i39.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i40 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i40.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i41 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i41.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i42 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i42.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i43 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i43.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i44 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i44.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i45 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i45.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i46 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i46.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i47 (.D(mic_shift_0_r[14]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i47.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i48 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i48.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i49 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_437), 
            .CK(pll_clk), .Q(mic_latest[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i49.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i50 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i50.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i51 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i51.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i52 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i52.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i53 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i53.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i54 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i54.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i55 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i55.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i56 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i56.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i57 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i57.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i58 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i58.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i59 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i59.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i60 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i60.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i61 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i61.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i62 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i62.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i63 (.D(mic_shift_0_l[15]), .SP(pll_clk_enable_451), 
            .CK(pll_clk), .Q(mic_latest[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mic_latest_i0_i63.GSR = "DISABLED";
    spi_mic_stream mic_stream_i (.mic_latest({mic_latest}), .sck_N_3047(sck_N_3047), 
            .spi_mic_cs_n_c(spi_mic_cs_n_c), .spi_mic_miso_c(spi_mic_miso_c)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(605[20] 608[6])
    LUT4 i81_3_lut_3_lut (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .C(spi_byte_count[2]), .Z(n60)) /* synthesis lut_function=(!(A (B+(C))+!A !(B (C)+!B !(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[17] 338[24])
    defparam i81_3_lut_3_lut.init = 16'h4343;
    FD1P3IX init_shadow_i58 (.D(n13063), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i58.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_182 (.A(n14467), .B(init_shadow[4]), .C(ev_bit[4]), 
         .Z(n12735)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_182.init = 16'hecec;
    LUT4 i1_3_lut_adj_183 (.A(n14467), .B(init_shadow[3]), .C(ev_bit[3]), 
         .Z(n12729)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_183.init = 16'hecec;
    FD1P3IX ev_bit_i7 (.D(ev_bit[6]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i7.GSR = "DISABLED";
    FD1P3IX ev_ch_i2 (.D(ev_ch_6__N_1948[2]), .SP(pll_clk_enable_474), .CD(n20303), 
            .CK(pll_clk), .Q(ev_ch[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_ch_i2.GSR = "DISABLED";
    FD1P3IX ev_ch_i1 (.D(ev_ch_6__N_1948[1]), .SP(pll_clk_enable_474), .CD(n20303), 
            .CK(pll_clk), .Q(ev_ch[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_ch_i1.GSR = "DISABLED";
    FD1P3AX ev_state_i1 (.D(ev_state_3__N_697[1]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(ev_state[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_state_i1.GSR = "DISABLED";
    FD1P3IX frame_settle__i3 (.D(frame_settle_3__N_1832[3]), .SP(pll_clk_enable_483), 
            .CD(pll_clk_enable_3), .CK(pll_clk), .Q(frame_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam frame_settle__i3.GSR = "DISABLED";
    FD1P3IX rgb_hold__i8 (.D(rgb_values[7]), .SP(pll_clk_enable_730), .CD(n20277), 
            .CK(pll_clk), .Q(rgb_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam rgb_hold__i8.GSR = "DISABLED";
    FD1P3IX rgb_hold__i7 (.D(rgb_values[6]), .SP(pll_clk_enable_730), .CD(n20277), 
            .CK(pll_clk), .Q(rgb_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam rgb_hold__i7.GSR = "DISABLED";
    FD1P3IX rgb_hold__i6 (.D(rgb_values[5]), .SP(pll_clk_enable_730), .CD(n20277), 
            .CK(pll_clk), .Q(rgb_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam rgb_hold__i6.GSR = "DISABLED";
    FD1P3IX rgb_hold__i5 (.D(rgb_values[4]), .SP(pll_clk_enable_730), .CD(n20277), 
            .CK(pll_clk), .Q(rgb_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam rgb_hold__i5.GSR = "DISABLED";
    FD1P3IX rgb_hold__i4 (.D(rgb_values[3]), .SP(pll_clk_enable_730), .CD(n20277), 
            .CK(pll_clk), .Q(rgb_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam rgb_hold__i4.GSR = "DISABLED";
    FD1P3IX rgb_hold__i3 (.D(rgb_values[2]), .SP(pll_clk_enable_730), .CD(n20277), 
            .CK(pll_clk), .Q(rgb_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam rgb_hold__i3.GSR = "DISABLED";
    FD1P3IX rgb_hold__i2 (.D(rgb_values[1]), .SP(pll_clk_enable_730), .CD(n20277), 
            .CK(pll_clk), .Q(rgb_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam rgb_hold__i2.GSR = "DISABLED";
    FD1P3IX spi_channel_field_1254__i0 (.D(n15), .SP(spi1_sck_c_enable_237), 
            .CD(spi1_sck_c_enable_236), .CK(spi1_sck_c), .Q(spi_channel_field[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(331[78:102])
    defparam spi_channel_field_1254__i0.GSR = "ENABLED";
    FD1P3AX stop_toggle_spi_479 (.D(stop_toggle_spi_N_2535), .SP(spi1_sck_c_enable_199), 
            .CK(spi1_sck_c), .Q(stop_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam stop_toggle_spi_479.GSR = "DISABLED";
    FD1S3IX wrap_s2_485 (.D(wrap_s2_N_2572), .CK(pll_clk), .CD(n15593), 
            .Q(wrap_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam wrap_s2_485.GSR = "DISABLED";
    FD1P3IX ev_bit_i83 (.D(ev_bit[82]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i83.GSR = "DISABLED";
    FD1P3IX ev_bit_i82 (.D(ev_bit[81]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i82.GSR = "DISABLED";
    FD1P3IX ev_bit_i81 (.D(ev_bit[80]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i81.GSR = "DISABLED";
    FD1P3IX ev_bit_i80 (.D(ev_bit[79]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i80.GSR = "DISABLED";
    FD1P3IX ev_bit_i79 (.D(ev_bit[78]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i79.GSR = "DISABLED";
    FD1P3IX ev_bit_i78 (.D(ev_bit[77]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i78.GSR = "DISABLED";
    FD1P3IX ev_bit_i77 (.D(ev_bit[76]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i77.GSR = "DISABLED";
    FD1P3IX ev_bit_i76 (.D(ev_bit[75]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i76.GSR = "DISABLED";
    FD1P3IX ev_bit_i75 (.D(ev_bit[74]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i75.GSR = "DISABLED";
    FD1P3IX ev_bit_i74 (.D(ev_bit[73]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i74.GSR = "DISABLED";
    FD1P3IX ev_bit_i73 (.D(ev_bit[72]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i73.GSR = "DISABLED";
    FD1P3IX ev_bit_i72 (.D(ev_bit[71]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i72.GSR = "DISABLED";
    FD1P3IX ev_bit_i71 (.D(ev_bit[70]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i71.GSR = "DISABLED";
    FD1P3IX ev_bit_i70 (.D(ev_bit[69]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i70.GSR = "DISABLED";
    FD1P3IX ev_bit_i69 (.D(ev_bit[68]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i69.GSR = "DISABLED";
    FD1P3IX ev_bit_i68 (.D(ev_bit[67]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i68.GSR = "DISABLED";
    FD1P3IX ev_bit_i67 (.D(ev_bit[66]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i67.GSR = "DISABLED";
    FD1P3IX ev_bit_i66 (.D(ev_bit[65]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i66.GSR = "DISABLED";
    FD1P3IX ev_bit_i65 (.D(ev_bit[64]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i65.GSR = "DISABLED";
    FD1P3IX ev_bit_i64 (.D(ev_bit[63]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i64.GSR = "DISABLED";
    FD1P3IX ev_bit_i63 (.D(ev_bit[62]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i63.GSR = "DISABLED";
    FD1P3IX ev_bit_i62 (.D(ev_bit[61]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i62.GSR = "DISABLED";
    FD1P3IX ev_bit_i61 (.D(ev_bit[60]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i61.GSR = "DISABLED";
    FD1P3IX ev_bit_i60 (.D(ev_bit[59]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i60.GSR = "DISABLED";
    FD1P3IX ev_bit_i59 (.D(ev_bit[58]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i59.GSR = "DISABLED";
    FD1P3IX ev_bit_i58 (.D(ev_bit[57]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i58.GSR = "DISABLED";
    FD1P3IX ev_bit_i57 (.D(ev_bit[56]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i57.GSR = "DISABLED";
    FD1P3IX ev_bit_i56 (.D(ev_bit[55]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i56.GSR = "DISABLED";
    FD1P3IX ev_bit_i55 (.D(ev_bit[54]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i55.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_184 (.A(n14467), .B(init_shadow[2]), .C(ev_bit[2]), 
         .Z(n12723)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_184.init = 16'hecec;
    FD1P3IX ev_bit_i51 (.D(ev_bit[50]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i51.GSR = "DISABLED";
    FD1P3IX ev_bit_i50 (.D(ev_bit[49]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i50.GSR = "DISABLED";
    FD1P3IX ev_bit_i49 (.D(ev_bit[48]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i49.GSR = "DISABLED";
    FD1P3IX ev_bit_i48 (.D(ev_bit[47]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i48.GSR = "DISABLED";
    FD1P3IX ev_bit_i47 (.D(ev_bit[46]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i47.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_185 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[31]), 
         .D(ev_bit[31]), .Z(ev_wr_data[31])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_185.init = 16'h7770;
    LUT4 i1_4_lut_then_4_lut (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_hold[76]), .D(status_bit_index[1]), .Z(n23926)) /* synthesis lut_function=(!((B (D)+!B !(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(127[17:33])
    defparam i1_4_lut_then_4_lut.init = 16'h2088;
    LUT4 i1_3_lut_adj_186 (.A(n14467), .B(init_shadow[1]), .C(ev_bit[1]), 
         .Z(n12717)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_186.init = 16'hecec;
    FD1P3AX ev_run_hold_s5_i0_i1 (.D(ev_rd_data[1]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i1.GSR = "DISABLED";
    LUT4 i2_3_lut_rep_113 (.A(spi_byte_count[3]), .B(spi_byte_count[2]), 
         .C(spi_byte_count[4]), .Z(n23905)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_3_lut_rep_113.init = 16'hfefe;
    LUT4 i1359_2_lut_4_lut (.A(spi_byte_count[3]), .B(spi_byte_count[2]), 
         .C(spi_byte_count[4]), .D(spi_byte_count[5]), .Z(n12)) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C (D)))) */ ;
    defparam i1359_2_lut_4_lut.init = 16'hfe00;
    LUT4 i9875_2_lut_rep_114 (.A(spi_byte_count[1]), .B(spi_byte_count[0]), 
         .Z(n23906)) /* synthesis lut_function=(A (B)) */ ;
    defparam i9875_2_lut_rep_114.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_187 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[32]), 
         .D(ev_bit[32]), .Z(ev_wr_data[32])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_187.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_188 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[33]), 
         .D(ev_bit[33]), .Z(ev_wr_data[33])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_188.init = 16'h7770;
    CCU2D fpga_time_1257_add_4_17 (.A0(fpga_time[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22411), .COUT(n22412), .S0(n150), .S1(n149));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_17.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_17.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_17.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_17.INJECT1_1 = "NO";
    LUT4 i68_3_lut_4_lut (.A(spi_byte_count[1]), .B(spi_byte_count[0]), 
         .C(spi_byte_count[2]), .D(spi_byte_count[3]), .Z(n55)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C+(D)))+!A !(C+(D)))) */ ;
    defparam i68_3_lut_4_lut.init = 16'h7ff0;
    LUT4 i1_3_lut_4_lut_adj_189 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[34]), 
         .D(ev_bit[34]), .Z(ev_wr_data[34])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_189.init = 16'h7770;
    LUT4 i2_2_lut_rep_115 (.A(mic_sample_count[0]), .B(mic_sample_count[1]), 
         .Z(n23907)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut_rep_115.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_190 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[35]), 
         .D(ev_bit[35]), .Z(ev_wr_data[35])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_190.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_191 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[36]), 
         .D(ev_bit[36]), .Z(ev_wr_data[36])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_191.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_192 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[37]), 
         .D(ev_bit[37]), .Z(ev_wr_data[37])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_192.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_193 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[38]), 
         .D(ev_bit[38]), .Z(ev_wr_data[38])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_193.init = 16'h7770;
    LUT4 i13647_2_lut (.A(ev_state[3]), .B(ev_state[0]), .Z(n22435)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i13647_2_lut.init = 16'heeee;
    LUT4 i1_2_lut_adj_194 (.A(spi_byte_count[0]), .B(n23074), .Z(spi1_sck_c_enable_110)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_194.init = 16'h8888;
    LUT4 i13627_2_lut_3_lut_4_lut (.A(mic_sample_count[0]), .B(mic_sample_count[1]), 
         .C(mic_sample_count[3]), .D(mic_sample_count[2]), .Z(n27)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;
    defparam i13627_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i5_3_lut_4_lut (.A(mic_sample_count[0]), .B(mic_sample_count[1]), 
         .C(mic_tick), .D(mic_sample_count[2]), .Z(n12_adj_3184)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i5_3_lut_4_lut.init = 16'h8000;
    LUT4 i13620_2_lut_3_lut (.A(mic_sample_count[0]), .B(mic_sample_count[1]), 
         .C(mic_sample_count[2]), .Z(n28)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;
    defparam i13620_2_lut_3_lut.init = 16'h7878;
    LUT4 i1475_2_lut_rep_116 (.A(ev_ch[1]), .B(ev_ch[0]), .Z(n23908)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(518[27:39])
    defparam i1475_2_lut_rep_116.init = 16'h8888;
    LUT4 i1482_2_lut_rep_91_3_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[2]), 
         .Z(n23883)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(518[27:39])
    defparam i1482_2_lut_rep_91_3_lut.init = 16'h8080;
    LUT4 i1480_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[2]), 
         .Z(ev_ch_6__N_1948[2])) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(518[27:39])
    defparam i1480_2_lut_3_lut.init = 16'h7878;
    LUT4 i1489_2_lut_rep_79_3_lut_4_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(n23871)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(518[27:39])
    defparam i1489_2_lut_rep_79_3_lut_4_lut.init = 16'h8000;
    LUT4 i1487_2_lut_3_lut_4_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(ev_ch_6__N_1948[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(518[27:39])
    defparam i1487_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i1_2_lut_rep_117 (.A(swap_pending), .B(frame_req), .Z(n23909)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_2_lut_rep_117.init = 16'h4444;
    LUT4 i1_2_lut_3_lut_adj_195 (.A(swap_pending), .B(frame_req), .C(ev_state[1]), 
         .Z(n4)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_2_lut_3_lut_adj_195.init = 16'h0404;
    LUT4 i20_3_lut_rep_85_4_lut (.A(swap_pending), .B(frame_req), .C(ev_state[0]), 
         .D(ev_state_3__N_1940[1]), .Z(n23877)) /* synthesis lut_function=(A (C (D))+!A (B ((D)+!C)+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i20_3_lut_rep_85_4_lut.init = 16'hf404;
    LUT4 i6821_2_lut_4_lut_3_lut_4_lut (.A(swap_pending), .B(frame_req), 
         .C(n23889), .D(ev_state[0]), .Z(n15703)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i6821_2_lut_4_lut_3_lut_4_lut.init = 16'h0040;
    LUT4 i2_3_lut_rep_92_4_lut (.A(swap_pending), .B(frame_req), .C(n22435), 
         .D(n23920), .Z(pll_clk_enable_345)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i2_3_lut_rep_92_4_lut.init = 16'h0400;
    LUT4 i2_3_lut_rep_118 (.A(ev_state[3]), .B(ev_state[2]), .C(ev_state[0]), 
         .Z(n23910)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i2_3_lut_rep_118.init = 16'h2020;
    LUT4 i1_2_lut_4_lut_adj_196 (.A(ev_state[3]), .B(ev_state[2]), .C(ev_state[0]), 
         .D(n20289), .Z(n23035)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i1_2_lut_4_lut_adj_196.init = 16'h0020;
    LUT4 i1_2_lut_rep_119 (.A(spi_byte_count[11]), .B(spi_byte_count[12]), 
         .Z(n23911)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_119.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_197 (.A(spi_byte_count[11]), .B(spi_byte_count[12]), 
         .C(spi_byte_count[10]), .D(spi_byte_count[9]), .Z(n23006)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_197.init = 16'hfffe;
    LUT4 i1_2_lut_rep_120 (.A(spi_bit_count[0]), .B(spi_bit_count[1]), .Z(n23912)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_rep_120.init = 16'h8888;
    LUT4 i2_2_lut_rep_93_3_lut (.A(spi_bit_count[0]), .B(spi_bit_count[1]), 
         .C(spi_bit_count[2]), .Z(spi1_sck_c_enable_197)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i2_2_lut_rep_93_3_lut.init = 16'h8080;
    LUT4 i13598_2_lut_3_lut (.A(spi_bit_count[0]), .B(spi_bit_count[1]), 
         .C(spi_bit_count[2]), .Z(n18_adj_3183)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;
    defparam i13598_2_lut_3_lut.init = 16'h7878;
    LUT4 i14795_2_lut_2_lut_3_lut_4_lut (.A(spi_bit_count[0]), .B(spi_bit_count[1]), 
         .C(n23858), .D(spi_bit_count[2]), .Z(spi1_sck_c_enable_29)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i14795_2_lut_2_lut_3_lut_4_lut.init = 16'h0800;
    LUT4 i14529_2_lut_rep_121 (.A(ev_state[0]), .B(ev_state[1]), .Z(n23913)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14529_2_lut_rep_121.init = 16'h8888;
    LUT4 i1_2_lut_rep_122 (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .Z(n23914)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_122.init = 16'heeee;
    LUT4 i1_2_lut_rep_82_3_lut (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .C(spi_byte_count[8]), .Z(n23874)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_rep_82_3_lut.init = 16'hfefe;
    LUT4 i1_3_lut_4_lut_adj_198 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[44]), 
         .D(ev_bit[44]), .Z(ev_wr_data[44])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_198.init = 16'h7770;
    LUT4 i14467_4_lut (.A(spi_bitmap[64]), .B(spi_bitmap[25]), .C(spi_bitmap[19]), 
         .D(spi_bitmap[30]), .Z(n23257)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14467_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_199 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[45]), 
         .D(ev_bit[45]), .Z(ev_wr_data[45])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_199.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_200 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[12]), 
         .D(ev_bit[12]), .Z(ev_wr_data[12])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_200.init = 16'h7770;
    LUT4 ev_state_3__I_0_589_i6_2_lut_rep_123 (.A(ev_state[2]), .B(ev_state[3]), 
         .Z(n23915)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(221[55:75])
    defparam ev_state_3__I_0_589_i6_2_lut_rep_123.init = 16'hdddd;
    LUT4 i1_3_lut_4_lut_adj_201 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[13]), 
         .D(ev_bit[13]), .Z(ev_wr_data[13])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_201.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_202 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[14]), 
         .D(ev_bit[14]), .Z(ev_wr_data[14])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_202.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_203 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[15]), 
         .D(ev_bit[15]), .Z(ev_wr_data[15])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_203.init = 16'h7770;
    LUT4 i9668_3_lut_4_lut_4_lut_3_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[1]), .D(ev_state[0]), .Z(n18460)) /* synthesis lut_function=((B+!(C (D)+!C !(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(221[55:75])
    defparam i9668_3_lut_4_lut_4_lut_3_lut_4_lut.init = 16'hdffd;
    LUT4 i1_3_lut_4_lut_adj_204 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[16]), 
         .D(ev_bit[16]), .Z(ev_wr_data[16])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_204.init = 16'h7770;
    FD1P3AX ev_run_hold_s5_i0_i2 (.D(ev_rd_data[2]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i2.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i3 (.D(ev_rd_data[3]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i3.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i4 (.D(ev_rd_data[4]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i4.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i5 (.D(ev_rd_data[5]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i5.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i6 (.D(ev_rd_data[6]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i6.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i7 (.D(ev_rd_data[7]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i7.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i8 (.D(ev_rd_data[8]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i8.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i9 (.D(ev_rd_data[9]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i9.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i10 (.D(ev_rd_data[10]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i10.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i11 (.D(ev_rd_data[11]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i11.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i12 (.D(ev_rd_data[12]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i12.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i13 (.D(ev_rd_data[13]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i13.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i14 (.D(ev_rd_data[14]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i14.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i15 (.D(ev_rd_data[15]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i15.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i16 (.D(ev_rd_data[16]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i16.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i17 (.D(ev_rd_data[17]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i17.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i18 (.D(ev_rd_data[18]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i18.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i19 (.D(ev_rd_data[19]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i19.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i20 (.D(ev_rd_data[20]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i20.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i21 (.D(ev_rd_data[21]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i21.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i22 (.D(ev_rd_data[22]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i22.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i23 (.D(ev_rd_data[23]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i23.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i24 (.D(ev_rd_data[24]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i24.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i25 (.D(ev_rd_data[25]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i25.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i26 (.D(ev_rd_data[26]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i26.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i27 (.D(ev_rd_data[27]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i27.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i28 (.D(ev_rd_data[28]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i28.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i29 (.D(ev_rd_data[29]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i29.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i30 (.D(ev_rd_data[30]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i30.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i31 (.D(ev_rd_data[31]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i31.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i32 (.D(ev_rd_data[32]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i32.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i33 (.D(ev_rd_data[33]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i33.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i34 (.D(ev_rd_data[34]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i34.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i35 (.D(ev_rd_data[35]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i35.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i36 (.D(ev_rd_data[36]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i36.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i37 (.D(ev_rd_data[37]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i37.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i38 (.D(ev_rd_data[38]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i38.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i39 (.D(ev_rd_data[39]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i39.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i40 (.D(ev_rd_data[40]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i40.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i41 (.D(ev_rd_data[41]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i41.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i42 (.D(ev_rd_data[42]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i42.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i43 (.D(ev_rd_data[43]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i43.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i44 (.D(ev_rd_data[44]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i44.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i45 (.D(ev_rd_data[45]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i45.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i46 (.D(ev_rd_data[46]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i46.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i47 (.D(ev_rd_data[47]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i47.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i48 (.D(ev_rd_data[48]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i48.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i49 (.D(ev_rd_data[49]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(ev_run_hold_s5[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i49.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i50 (.D(ev_rd_data[50]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i50.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i51 (.D(ev_rd_data[51]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i51.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i52 (.D(ev_rd_data[52]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i52.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i53 (.D(ev_rd_data[53]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i53.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i54 (.D(ev_rd_data[54]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i54.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i55 (.D(ev_rd_data[55]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i55.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i56 (.D(ev_rd_data[56]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i56.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i57 (.D(ev_rd_data[57]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i57.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i58 (.D(ev_rd_data[58]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i58.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i59 (.D(ev_rd_data[59]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i59.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i60 (.D(ev_rd_data[60]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i60.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i61 (.D(ev_rd_data[61]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i61.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i62 (.D(ev_rd_data[62]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i62.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i63 (.D(ev_rd_data[63]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i63.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i64 (.D(ev_rd_data[64]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i64.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i65 (.D(ev_rd_data[65]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i65.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i66 (.D(ev_rd_data[66]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i66.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i67 (.D(ev_rd_data[67]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i67.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i68 (.D(ev_rd_data[68]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i68.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i69 (.D(ev_rd_data[69]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i69.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i70 (.D(ev_rd_data[70]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i70.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i71 (.D(ev_rd_data[71]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i71.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i72 (.D(ev_rd_data[72]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i72.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i73 (.D(ev_rd_data[73]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i73.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i74 (.D(ev_rd_data[74]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i74.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i75 (.D(ev_rd_data[75]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i75.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i76 (.D(ev_rd_data[76]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i76.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i77 (.D(ev_rd_data[77]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i77.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i78 (.D(ev_rd_data[78]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i78.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i79 (.D(ev_rd_data[79]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i79.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i80 (.D(ev_rd_data[80]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i80.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i81 (.D(ev_rd_data[81]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i81.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i82 (.D(ev_rd_data[82]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i82.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i83 (.D(ev_rd_data[83]), .SP(pll_clk_enable_608), 
            .CK(pll_clk), .Q(ev_run_hold_s5[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_run_hold_s5_i0_i83.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i1 (.D(spi_frame_sequence[1]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i1.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_90_3_lut (.A(ev_state[2]), .B(ev_state[3]), .C(ev_state[0]), 
         .Z(n23882)) /* synthesis lut_function=((B+(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(221[55:75])
    defparam i1_2_lut_rep_90_3_lut.init = 16'hfdfd;
    LUT4 i1431_2_lut_rep_124 (.A(frame_settle[1]), .B(frame_settle[0]), 
         .Z(n23916)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[29:48])
    defparam i1431_2_lut_rep_124.init = 16'heeee;
    LUT4 i2589_4_lut (.A(ev_ch[0]), .B(staging_rd_addr[0]), .C(n9_adj_3199), 
         .D(n23094), .Z(staging_rd_addr_6__N_901[0])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i2589_4_lut.init = 16'hac0c;
    LUT4 i1_2_lut_3_lut_adj_205 (.A(frame_settle[1]), .B(frame_settle[0]), 
         .C(frame_settle[2]), .Z(n14164)) /* synthesis lut_function=(A (C)+!A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[29:48])
    defparam i1_2_lut_3_lut_adj_205.init = 16'he1e1;
    LUT4 run_addr_s3_8__I_0_i5_3_lut (.A(run_addr_s3[4]), .B(ev_rd_slot[4]), 
         .C(n22496), .Z(event_rd_addr[4])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i5_3_lut.init = 16'hacac;
    LUT4 i14780_4_lut (.A(ev_clear_addr[0]), .B(ev_clear_addr[3]), .C(n23407), 
         .D(n23239), .Z(ev_clear_done_N_2575)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(491[34:58])
    defparam i14780_4_lut.init = 16'h4000;
    LUT4 i14615_4_lut (.A(ev_clear_addr[4]), .B(ev_clear_addr[6]), .C(ev_clear_addr[5]), 
         .D(ev_clear_addr[2]), .Z(n23407)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14615_4_lut.init = 16'h8000;
    LUT4 i14449_2_lut (.A(ev_clear_addr[7]), .B(ev_clear_addr[1]), .Z(n23239)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14449_2_lut.init = 16'h8888;
    LUT4 i9690_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_735)) /* synthesis lut_function=(A (B)) */ ;
    defparam i9690_2_lut.init = 16'h8888;
    LUT4 i14773_4_lut (.A(mic_divider[5]), .B(mic_divider[4]), .C(mic_divider[6]), 
         .D(n23411), .Z(mic_tick_N_2573)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(565[21:43])
    defparam i14773_4_lut.init = 16'h0100;
    LUT4 i14705_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[61]), 
         .C(status_hold[60]), .Z(n23498)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14705_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14706_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[63]), 
         .C(status_hold[62]), .Z(n23499)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14706_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14619_4_lut (.A(mic_divider[0]), .B(mic_divider[3]), .C(mic_divider[1]), 
         .D(mic_divider[2]), .Z(n23411)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14619_4_lut.init = 16'h8000;
    FD1P3AX accepted_sequence_spi_i0_i2 (.D(spi_frame_sequence[2]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i3 (.D(spi_frame_sequence[3]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i4 (.D(spi_frame_sequence[4]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i5 (.D(spi_frame_sequence[5]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i6 (.D(spi_frame_sequence[6]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i7 (.D(spi_frame_sequence[7]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i8 (.D(spi_frame_sequence[8]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i9 (.D(spi_frame_sequence[9]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i10 (.D(spi_frame_sequence[10]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i11 (.D(spi_frame_sequence[11]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i12 (.D(spi_frame_sequence[12]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i13 (.D(spi_frame_sequence[13]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i14 (.D(spi_frame_sequence[14]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i15 (.D(spi_frame_sequence[15]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i16 (.D(spi_frame_sequence[16]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i17 (.D(spi_frame_sequence[17]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[17]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i18 (.D(spi_frame_sequence[18]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i19 (.D(spi_frame_sequence[19]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[19]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i20 (.D(spi_frame_sequence[20]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i21 (.D(spi_frame_sequence[21]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[21]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i22 (.D(spi_frame_sequence[22]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i23 (.D(spi_frame_sequence[23]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[23]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i24 (.D(spi_frame_sequence[24]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i25 (.D(spi_frame_sequence[25]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[25]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i26 (.D(spi_frame_sequence[26]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[26]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i27 (.D(spi_frame_sequence[27]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[27]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i28 (.D(spi_frame_sequence[28]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[28]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i29 (.D(spi_frame_sequence[29]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[29]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i30 (.D(spi_frame_sequence[30]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[30]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i31 (.D(spi_frame_sequence[31]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[31]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam accepted_sequence_spi_i0_i31.GSR = "DISABLED";
    FD1P3AX spi_channel_index_1253__i1 (.D(n39_adj_3173), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253__i1.GSR = "ENABLED";
    LUT4 mic_clk_I_0_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(mic_clk_N_2520)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(585[18] 587[12])
    defparam mic_clk_I_0_2_lut.init = 16'h6666;
    LUT4 i6_4_lut_adj_206 (.A(mic_sample_count[3]), .B(n12_adj_3184), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_451)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_adj_206.init = 16'h0800;
    LUT4 i14703_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[57]), 
         .C(status_hold[56]), .Z(n23496)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14703_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_207 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[17]), 
         .D(ev_bit[17]), .Z(ev_wr_data[17])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_207.init = 16'h7770;
    FD1P3AX spi_channel_index_1253__i2 (.D(n38_adj_3174), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253__i2.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1253__i3 (.D(n37_adj_3175), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253__i3.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1253__i4 (.D(n36_adj_3176), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253__i4.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1253__i5 (.D(n35_adj_3177), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253__i5.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1253__i6 (.D(n34_adj_3178), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253__i6.GSR = "ENABLED";
    FD1S3AX time_divider_1258__i1 (.D(n39_adj_3163), .CK(pll_clk), .Q(time_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258__i1.GSR = "DISABLED";
    FD1S3AX time_divider_1258__i2 (.D(n38_adj_3164), .CK(pll_clk), .Q(time_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258__i2.GSR = "DISABLED";
    FD1S3AX time_divider_1258__i3 (.D(n37_adj_3165), .CK(pll_clk), .Q(time_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258__i3.GSR = "DISABLED";
    FD1S3AX time_divider_1258__i4 (.D(n36_adj_3166), .CK(pll_clk), .Q(time_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258__i4.GSR = "DISABLED";
    FD1S3AX time_divider_1258__i5 (.D(n35_adj_3167), .CK(pll_clk), .Q(time_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258__i5.GSR = "DISABLED";
    FD1S3AX time_divider_1258__i6 (.D(n34_adj_3168), .CK(pll_clk), .Q(time_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258__i6.GSR = "DISABLED";
    FD1P3AX status_bit_index_1252__i1 (.D(n39), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252__i1.GSR = "ENABLED";
    LUT4 i14701_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[53]), 
         .C(status_hold[52]), .Z(n23494)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14701_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14704_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[59]), 
         .C(status_hold[58]), .Z(n23497)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14704_3_lut_3_lut.init = 16'he4e4;
    LUT4 mux_1407_i1_3_lut (.A(n10157), .B(n10158), .C(n10156), .Z(rd_data_15__N_2649[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i1_3_lut.init = 16'hcaca;
    FD1P3AX status_bit_index_1252__i2 (.D(n38), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252__i2.GSR = "ENABLED";
    FD1P3AX status_bit_index_1252__i3 (.D(n37), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252__i3.GSR = "ENABLED";
    FD1P3AX status_bit_index_1252__i4 (.D(n36), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252__i4.GSR = "ENABLED";
    FD1P3AX status_bit_index_1252__i5 (.D(n35), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252__i5.GSR = "ENABLED";
    FD1P3AX status_bit_index_1252__i6 (.D(n34), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252__i6.GSR = "ENABLED";
    FD1P3AX fpga_time_1257__i1 (.D(n164), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i1.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_208 (.A(n10155), .B(n23349), .C(n23177), .D(n6_adj_3198), 
         .Z(n10156)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i7_4_lut_adj_208.init = 16'h0002;
    LUT4 i14557_4_lut (.A(n10144), .B(n23173), .C(n5), .D(n10145), .Z(n23349)) /* synthesis lut_function=(A (B+(C+!(D)))+!A (B+(C+(D)))) */ ;
    defparam i14557_4_lut.init = 16'hfdfe;
    LUT4 i14702_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[55]), 
         .C(status_hold[54]), .Z(n23495)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14702_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14700_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[51]), 
         .C(status_hold[50]), .Z(n23493)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14700_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14387_4_lut (.A(n10146), .B(n10140), .C(n10147), .D(n10141), 
         .Z(n23177)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14387_4_lut.init = 16'h7bde;
    LUT4 equal_1388_i6_2_lut (.A(n10150), .B(n10151), .Z(n6_adj_3198)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam equal_1388_i6_2_lut.init = 16'h6666;
    LUT4 i14383_4_lut (.A(n10152), .B(n10142), .C(n10153), .D(n10143), 
         .Z(n23173)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14383_4_lut.init = 16'h7bde;
    LUT4 i14699_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[49]), 
         .C(status_hold[48]), .Z(n23492)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14699_3_lut_3_lut.init = 16'he4e4;
    LUT4 equal_1388_i5_2_lut (.A(n10148), .B(n10149), .Z(n5)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam equal_1388_i5_2_lut.init = 16'h6666;
    FD1P3AX fpga_time_1257__i2 (.D(n163), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i2.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i3 (.D(n162), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i3.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i4 (.D(n161), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i4.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i5 (.D(n160), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i5.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i6 (.D(n159), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i6.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i7 (.D(n158), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i7.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i8 (.D(n157), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i8.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i9 (.D(n156), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i9.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i10 (.D(n155), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[10])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i10.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i11 (.D(n154), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[11])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i11.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i12 (.D(n153), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[12])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i12.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i13 (.D(n152), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[13])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i13.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i14 (.D(n151), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[14])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i14.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i15 (.D(n150), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[15])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i15.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i16 (.D(n149), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[16])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i16.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i17 (.D(n148), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[17])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i17.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i18 (.D(n147), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[18])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i18.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i19 (.D(n146), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[19])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i19.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i20 (.D(n145), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[20])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i20.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i21 (.D(n144), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[21])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i21.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i22 (.D(n143), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[22])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i22.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i23 (.D(n142), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[23])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i23.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i24 (.D(n141), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[24])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i24.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i25 (.D(n140), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[25])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i25.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i26 (.D(n139), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[26])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i26.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i27 (.D(n138), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[27])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i27.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i28 (.D(n137), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[28])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i28.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i29 (.D(n136), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[29])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i29.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i30 (.D(n135), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[30])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i30.GSR = "DISABLED";
    FD1P3AX fpga_time_1257__i31 (.D(n134), .SP(pll_clk_enable_639), .CK(pll_clk), 
            .Q(fpga_time[31])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257__i31.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1256__i1 (.D(n44), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256__i1.GSR = "DISABLED";
    FD1P3IX init_shadow_i37 (.D(n12933), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i37.GSR = "DISABLED";
    LUT4 i4_4_lut_rep_66 (.A(n7), .B(n23914), .C(spi_byte_count[5]), .D(n23905), 
         .Z(n23858)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;
    defparam i4_4_lut_rep_66.init = 16'hffef;
    FD1P3IX init_shadow_i49 (.D(n13009), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i49.GSR = "DISABLED";
    FD1P3IX ev_bit_i30 (.D(ev_bit[29]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i30.GSR = "DISABLED";
    FD1P3IX ev_bit_i29 (.D(ev_bit[28]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i29.GSR = "DISABLED";
    FD1P3IX ev_bit_i28 (.D(ev_bit[27]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i28.GSR = "DISABLED";
    FD1P3IX ev_bit_i27 (.D(ev_bit[26]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i27.GSR = "DISABLED";
    FD1P3IX ev_bit_i26 (.D(ev_bit[25]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i26.GSR = "DISABLED";
    FD1P3IX ev_bit_i25 (.D(ev_bit[24]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i25.GSR = "DISABLED";
    LUT4 i14696_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[43]), 
         .C(status_hold[42]), .Z(n23489)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14696_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14695_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[41]), 
         .C(status_hold[40]), .Z(n23488)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14695_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14694_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[39]), 
         .C(status_hold[38]), .Z(n23487)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14694_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14823_3_lut (.A(n23013), .B(spi_byte_count[4]), .C(spi_byte_count[5]), 
         .Z(spi1_sck_c_enable_103)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i14823_3_lut.init = 16'h0404;
    LUT4 i14693_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[37]), 
         .C(status_hold[36]), .Z(n23486)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14693_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_209 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[18]), 
         .D(ev_bit[18]), .Z(ev_wr_data[18])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_209.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_210 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[19]), 
         .D(ev_bit[19]), .Z(ev_wr_data[19])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_210.init = 16'h7770;
    LUT4 i14698_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[47]), 
         .C(status_hold[46]), .Z(n23491)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14698_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14692_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[35]), 
         .C(status_hold[34]), .Z(n23485)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14692_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14691_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[33]), 
         .C(status_hold[32]), .Z(n23484)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14691_3_lut_3_lut.init = 16'he4e4;
    FD1P3IX init_shadow_i57 (.D(n13057), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i57.GSR = "DISABLED";
    FD1P3IX init_shadow_i36 (.D(n12927), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i36.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_211 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[20]), 
         .D(ev_bit[20]), .Z(ev_wr_data[20])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_211.init = 16'h7770;
    LUT4 i14675_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[31]), 
         .C(status_hold[30]), .Z(n23468)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14675_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14674_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[29]), 
         .C(status_hold[28]), .Z(n23467)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14674_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14673_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[27]), 
         .C(status_hold[26]), .Z(n23466)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14673_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14672_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[25]), 
         .C(status_hold[24]), .Z(n23465)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14672_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_212 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[21]), 
         .D(ev_bit[21]), .Z(ev_wr_data[21])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_212.init = 16'h7770;
    PFUMX i11496 (.BLUT(n20287), .ALUT(n20291), .C0(n23448), .Z(ev_state_3__N_697[1]));
    LUT4 i4_4_lut_adj_213 (.A(n48), .B(n23911), .C(spi_byte_count[9]), 
         .D(n6_adj_3200), .Z(n23143)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut_adj_213.init = 16'hfffe;
    FD1P3IX ev_bit_i24 (.D(ev_bit[23]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i24.GSR = "DISABLED";
    FD1P3IX init_shadow_i56 (.D(n13051), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i56.GSR = "DISABLED";
    FD1P3IX ev_bit_i23 (.D(ev_bit[22]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i23.GSR = "DISABLED";
    LUT4 i14671_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[23]), 
         .C(status_hold[22]), .Z(n23464)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14671_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14697_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[45]), 
         .C(status_hold[44]), .Z(n23490)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14697_3_lut_3_lut.init = 16'he4e4;
    FD1P3IX spi_channel_field_1254__i1 (.D(n14), .SP(spi1_sck_c_enable_237), 
            .CD(spi1_sck_c_enable_236), .CK(spi1_sck_c), .Q(spi_channel_field[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(331[78:102])
    defparam spi_channel_field_1254__i1.GSR = "ENABLED";
    PFUMX i102845_i1 (.BLUT(n23483), .ALUT(n23514), .C0(spi1_miso_N_2513[5]), 
          .Z(n63));
    LUT4 i14670_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[21]), 
         .C(status_hold[20]), .Z(n23463)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14670_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14669_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[19]), 
         .C(status_hold[18]), .Z(n23462)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14669_3_lut_3_lut.init = 16'he4e4;
    PFUMX i14659 (.BLUT(n23450), .ALUT(n23451), .C0(spi1_miso_N_2513[5]), 
          .Z(n23452));
    LUT4 i1_3_lut_4_lut_adj_214 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[22]), 
         .D(ev_bit[22]), .Z(ev_wr_data[22])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_214.init = 16'h7770;
    LUT4 i14668_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[17]), 
         .C(status_hold[16]), .Z(n23461)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14668_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14667_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[15]), 
         .C(status_hold[14]), .Z(n23460)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14667_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14666_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[13]), 
         .C(status_hold[12]), .Z(n23459)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14666_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_adj_215 (.A(n14467), .B(init_shadow[37]), .C(ev_bit[37]), 
         .Z(n12933)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_215.init = 16'hecec;
    LUT4 i14665_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[11]), 
         .C(status_hold[10]), .Z(n23458)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14665_3_lut_3_lut.init = 16'he4e4;
    LUT4 i68_2_lut (.A(spi_byte_count[14]), .B(spi_byte_count[15]), .Z(n48)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i68_2_lut.init = 16'heeee;
    LUT4 i7_4_lut_adj_216 (.A(ev_run_hold_s5[1]), .B(us_tx_c_1), .C(init_shadow[1]), 
         .D(swap_now_s5), .Z(n11407)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_216.init = 16'h5a66;
    LUT4 i14645_4_lut (.A(n23359), .B(n23423), .C(n23393), .D(n23357), 
         .Z(n23437)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14645_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_217 (.A(ev_run_hold_s5[2]), .B(us_tx_c_2), .C(init_shadow[2]), 
         .D(swap_now_s5), .Z(n11409)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_217.init = 16'h5a66;
    L6MUX21 i14688 (.D0(n23477), .D1(n23478), .SD(spi1_miso_N_2513[3]), 
            .Z(n23481));
    LUT4 i7_4_lut_adj_218 (.A(ev_run_hold_s5[3]), .B(us_tx_c_3), .C(init_shadow[3]), 
         .D(swap_now_s5), .Z(n11411)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_218.init = 16'h5a66;
    L6MUX21 i14689 (.D0(n23479), .D1(n23480), .SD(spi1_miso_N_2513[3]), 
            .Z(n23482));
    L6MUX21 i14719 (.D0(n23508), .D1(n23509), .SD(spi1_miso_N_2513[3]), 
            .Z(n23512));
    PFUMX i14856 (.BLUT(n23711), .ALUT(n23710), .C0(ev_state[1]), .Z(ev_state_3__N_697[2]));
    LUT4 i14664_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[9]), 
         .C(status_hold[8]), .Z(n23457)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14664_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_219 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[80]), 
         .D(ev_bit[80]), .Z(ev_wr_data[80])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_219.init = 16'h7770;
    L6MUX21 i14720 (.D0(n23510), .D1(n23511), .SD(spi1_miso_N_2513[3]), 
            .Z(n23513));
    LUT4 i1_3_lut_adj_220 (.A(n14467), .B(init_shadow[49]), .C(ev_bit[49]), 
         .Z(n13009)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_220.init = 16'hecec;
    LUT4 i7_4_lut_adj_221 (.A(ev_run_hold_s5[4]), .B(us_tx_c_4), .C(init_shadow[4]), 
         .D(swap_now_s5), .Z(n11413)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_221.init = 16'h5a66;
    LUT4 i1_3_lut_adj_222 (.A(n14467), .B(init_shadow[57]), .C(ev_bit[57]), 
         .Z(n13057)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_222.init = 16'hecec;
    LUT4 i7_4_lut_adj_223 (.A(ev_run_hold_s5[5]), .B(us_tx_c_5), .C(init_shadow[5]), 
         .D(swap_now_s5), .Z(n11415)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_223.init = 16'h5a66;
    LUT4 i7_4_lut_adj_224 (.A(ev_run_hold_s5[6]), .B(us_tx_c_6), .C(init_shadow[6]), 
         .D(swap_now_s5), .Z(n11417)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_224.init = 16'h5a66;
    CCU2D fpga_time_1257_add_4_15 (.A0(fpga_time[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22410), .COUT(n22411), .S0(n152), .S1(n151));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_15.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_15.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_15.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_15.INJECT1_1 = "NO";
    CCU2D fpga_time_1257_add_4_13 (.A0(fpga_time[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22409), .COUT(n22410), .S0(n154), .S1(n153));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_13.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_13.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_13.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_13.INJECT1_1 = "NO";
    CCU2D fpga_time_1257_add_4_11 (.A0(fpga_time[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22408), .COUT(n22409), .S0(n156), .S1(n155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_11.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_11.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_11.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_11.INJECT1_1 = "NO";
    CCU2D fpga_time_1257_add_4_9 (.A0(fpga_time[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22407), .COUT(n22408), .S0(n158), .S1(n157));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_9.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_9.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_9.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_9.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_225 (.A(ev_run_hold_s5[7]), .B(us_tx_c_7), .C(init_shadow[7]), 
         .D(swap_now_s5), .Z(n11419)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_225.init = 16'h5a66;
    LUT4 i7_4_lut_adj_226 (.A(ev_run_hold_s5[8]), .B(us_tx_c_8), .C(init_shadow[8]), 
         .D(swap_now_s5), .Z(n11421)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_226.init = 16'h5a66;
    CCU2D fpga_time_1257_add_4_7 (.A0(fpga_time[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22406), .COUT(n22407), .S0(n160), .S1(n159));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_7.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_7.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_7.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_7.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_643_3 (.A0(expected_next_15__N_1417[7]), .B0(spi_extension_length[3]), 
          .C0(GND_net), .D0(GND_net), .A1(expected_next_15__N_1458[3]), 
          .B1(spi_extension_length[4]), .C1(GND_net), .D1(GND_net), .CIN(n22385), 
          .COUT(n22386), .S0(expected_next[3]), .S1(expected_next[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[33] 288[86])
    defparam expected_next_15__I_0_643_3.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_643_3.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_643_3.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_643_3.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_227 (.A(ev_run_hold_s5[9]), .B(us_tx_c_9), .C(init_shadow[9]), 
         .D(swap_now_s5), .Z(n11423)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_227.init = 16'h5a66;
    CCU2D expected_next_15__I_0_643_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(expected_next_15__N_1458[3]), .B1(spi_extension_length[2]), 
          .C1(GND_net), .D1(GND_net), .COUT(n22385), .S1(expected_next[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[33] 288[86])
    defparam expected_next_15__I_0_643_1.INIT0 = 16'hF000;
    defparam expected_next_15__I_0_643_1.INIT1 = 16'ha999;
    defparam expected_next_15__I_0_643_1.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_643_1.INJECT1_1 = "NO";
    CCU2D fpga_time_1257_add_4_5 (.A0(fpga_time[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22405), .COUT(n22406), .S0(n162), .S1(n161));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_5.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_5.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_5.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_5.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_228 (.A(ev_run_hold_s5[10]), .B(us_tx_c_10), .C(init_shadow[10]), 
         .D(swap_now_s5), .Z(n11425)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_228.init = 16'h5a66;
    LUT4 i7_4_lut_adj_229 (.A(ev_run_hold_s5[11]), .B(us_tx_c_11), .C(init_shadow[11]), 
         .D(swap_now_s5), .Z(n11427)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_229.init = 16'h5a66;
    LUT4 i7_4_lut_adj_230 (.A(ev_run_hold_s5[12]), .B(us_tx_c_12), .C(init_shadow[12]), 
         .D(swap_now_s5), .Z(n11429)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_230.init = 16'h5a66;
    L6MUX21 i14684 (.D0(n23469), .D1(n23470), .SD(spi1_miso_N_2513[2]), 
            .Z(n23477));
    LUT4 i7_4_lut_adj_231 (.A(ev_run_hold_s5[13]), .B(us_tx_c_13), .C(init_shadow[13]), 
         .D(swap_now_s5), .Z(n11431)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_231.init = 16'h5a66;
    L6MUX21 i14685 (.D0(n23471), .D1(n23472), .SD(spi1_miso_N_2513[2]), 
            .Z(n23478));
    LUT4 i1_3_lut_adj_232 (.A(n14467), .B(init_shadow[36]), .C(ev_bit[36]), 
         .Z(n12927)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_232.init = 16'hecec;
    LUT4 i7_4_lut_adj_233 (.A(ev_run_hold_s5[14]), .B(us_tx_c_14), .C(init_shadow[14]), 
         .D(swap_now_s5), .Z(n11433)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_233.init = 16'h5a66;
    L6MUX21 i14686 (.D0(n23473), .D1(n23474), .SD(spi1_miso_N_2513[2]), 
            .Z(n23479));
    LUT4 i7_4_lut_adj_234 (.A(ev_run_hold_s5[15]), .B(us_tx_c_15), .C(init_shadow[15]), 
         .D(swap_now_s5), .Z(n11435)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_234.init = 16'h5a66;
    L6MUX21 i14687 (.D0(n23475), .D1(n23476), .SD(spi1_miso_N_2513[2]), 
            .Z(n23480));
    LUT4 i7_4_lut_adj_235 (.A(ev_run_hold_s5[16]), .B(us_tx_c_16), .C(init_shadow[16]), 
         .D(swap_now_s5), .Z(n11437)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_235.init = 16'h5a66;
    L6MUX21 i14715 (.D0(n23500), .D1(n23501), .SD(spi1_miso_N_2513[2]), 
            .Z(n23508));
    CCU2D fpga_time_1257_add_4_3 (.A0(fpga_time[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22404), .COUT(n22405), .S0(n164), .S1(n163));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_3.INIT0 = 16'hfaaa;
    defparam fpga_time_1257_add_4_3.INIT1 = 16'hfaaa;
    defparam fpga_time_1257_add_4_3.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_3.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_236 (.A(ev_run_hold_s5[17]), .B(us_tx_c_17), .C(init_shadow[17]), 
         .D(swap_now_s5), .Z(n11439)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_236.init = 16'h5a66;
    CCU2D fpga_time_1257_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22404), .S1(n165));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(412[29:46])
    defparam fpga_time_1257_add_4_1.INIT0 = 16'hF000;
    defparam fpga_time_1257_add_4_1.INIT1 = 16'h0555;
    defparam fpga_time_1257_add_4_1.INJECT1_0 = "NO";
    defparam fpga_time_1257_add_4_1.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_237 (.A(ev_run_hold_s5[18]), .B(us_tx_c_18), .C(init_shadow[18]), 
         .D(swap_now_s5), .Z(n11441)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_237.init = 16'h5a66;
    LUT4 i7_4_lut_adj_238 (.A(ev_run_hold_s5[19]), .B(us_tx_c_19), .C(init_shadow[19]), 
         .D(swap_now_s5), .Z(n11443)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_238.init = 16'h5a66;
    LUT4 i7_4_lut_adj_239 (.A(ev_run_hold_s5[20]), .B(us_tx_c_20), .C(init_shadow[20]), 
         .D(swap_now_s5), .Z(n11445)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_239.init = 16'h5a66;
    LUT4 i7_4_lut_adj_240 (.A(ev_run_hold_s5[21]), .B(us_tx_c_21), .C(init_shadow[21]), 
         .D(swap_now_s5), .Z(n11447)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_240.init = 16'h5a66;
    LUT4 i7_4_lut_adj_241 (.A(ev_run_hold_s5[22]), .B(us_tx_c_22), .C(init_shadow[22]), 
         .D(swap_now_s5), .Z(n11449)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_241.init = 16'h5a66;
    LUT4 i14663_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[7]), 
         .C(status_hold[6]), .Z(n23456)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14663_3_lut_3_lut.init = 16'he4e4;
    L6MUX21 i14716 (.D0(n23502), .D1(n23503), .SD(spi1_miso_N_2513[2]), 
            .Z(n23509));
    L6MUX21 i14717 (.D0(n23504), .D1(n23505), .SD(spi1_miso_N_2513[2]), 
            .Z(n23510));
    LUT4 i7_4_lut_adj_242 (.A(ev_run_hold_s5[23]), .B(us_tx_c_23), .C(init_shadow[23]), 
         .D(swap_now_s5), .Z(n11451)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_242.init = 16'h5a66;
    L6MUX21 i14718 (.D0(n23506), .D1(n23507), .SD(spi1_miso_N_2513[2]), 
            .Z(n23511));
    LUT4 i14825_2_lut (.A(ev_state[0]), .B(ev_state[3]), .Z(n23448)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    defparam i14825_2_lut.init = 16'hdddd;
    PFUMX i14676 (.BLUT(n23453), .ALUT(n23454), .C0(spi1_miso_N_2513[1]), 
          .Z(n23469));
    LUT4 i14820_3_lut (.A(n23013), .B(spi_byte_count[5]), .C(spi_byte_count[4]), 
         .Z(spi1_sck_c_enable_102)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i14820_3_lut.init = 16'h0404;
    LUT4 i14662_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[5]), 
         .C(status_hold[4]), .Z(n23455)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14662_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_243 (.A(ev_run_hold_s5[24]), .B(us_tx_c_24), .C(init_shadow[24]), 
         .D(swap_now_s5), .Z(n11453)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_243.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_244 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[23]), 
         .D(ev_bit[23]), .Z(ev_wr_data[23])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_244.init = 16'h7770;
    LUT4 i7_4_lut_adj_245 (.A(ev_run_hold_s5[25]), .B(us_tx_c_25), .C(init_shadow[25]), 
         .D(swap_now_s5), .Z(n11455)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_245.init = 16'h5a66;
    LUT4 i7_4_lut_adj_246 (.A(ev_run_hold_s5[26]), .B(us_tx_c_26), .C(init_shadow[26]), 
         .D(swap_now_s5), .Z(n11457)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_246.init = 16'h5a66;
    PFUMX i14677 (.BLUT(n23455), .ALUT(n23456), .C0(spi1_miso_N_2513[1]), 
          .Z(n23470));
    LUT4 i7_4_lut_adj_247 (.A(ev_run_hold_s5[27]), .B(us_tx_c_27), .C(init_shadow[27]), 
         .D(swap_now_s5), .Z(n11459)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_247.init = 16'h5a66;
    LUT4 i7_4_lut_adj_248 (.A(ev_run_hold_s5[28]), .B(us_tx_c_28), .C(init_shadow[28]), 
         .D(swap_now_s5), .Z(n11461)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_248.init = 16'h5a66;
    LUT4 i7_4_lut_adj_249 (.A(ev_run_hold_s5[29]), .B(us_tx_c_29), .C(init_shadow[29]), 
         .D(swap_now_s5), .Z(n11463)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_249.init = 16'h5a66;
    PFUMX i14678 (.BLUT(n23457), .ALUT(n23458), .C0(spi1_miso_N_2513[1]), 
          .Z(n23471));
    LUT4 i7_4_lut_adj_250 (.A(ev_run_hold_s5[30]), .B(us_tx_c_30), .C(init_shadow[30]), 
         .D(swap_now_s5), .Z(n11465)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_250.init = 16'h5a66;
    LUT4 i7_4_lut_adj_251 (.A(ev_run_hold_s5[31]), .B(us_tx_c_31), .C(init_shadow[31]), 
         .D(swap_now_s5), .Z(n11467)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_251.init = 16'h5a66;
    LUT4 i14661_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[3]), 
         .C(status_hold[2]), .Z(n23454)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14661_3_lut_3_lut.init = 16'he4e4;
    PFUMX i14679 (.BLUT(n23459), .ALUT(n23460), .C0(spi1_miso_N_2513[1]), 
          .Z(n23472));
    LUT4 i1_3_lut_adj_252 (.A(n14467), .B(init_shadow[56]), .C(ev_bit[56]), 
         .Z(n13051)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_252.init = 16'hecec;
    LUT4 i14660_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[1]), 
         .C(status_hold[0]), .Z(n23453)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i14660_3_lut_3_lut.init = 16'he4e4;
    CCU2D status_bit_index_1252_add_4_7 (.A0(status_bit_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22402), .S0(n35), .S1(n34));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252_add_4_7.INIT0 = 16'hfaaa;
    defparam status_bit_index_1252_add_4_7.INIT1 = 16'hfaaa;
    defparam status_bit_index_1252_add_4_7.INJECT1_0 = "NO";
    defparam status_bit_index_1252_add_4_7.INJECT1_1 = "NO";
    LUT4 i3_4_lut_adj_253 (.A(ev_ch[5]), .B(ev_ch[2]), .C(ev_ch[3]), .D(n23363), 
         .Z(n20289)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(512[21:35])
    defparam i3_4_lut_adj_253.init = 16'hfeff;
    FD1P3IX ev_clear_addr_i7 (.D(ev_clear_addr_7__N_2237[7]), .SP(pll_clk_enable_726), 
            .CD(n15703), .CK(pll_clk), .Q(ev_clear_addr[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_clear_addr_i7.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_254 (.A(ev_run_hold_s5[32]), .B(us_tx_c_32), .C(init_shadow[32]), 
         .D(swap_now_s5), .Z(n11469)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_254.init = 16'h5a66;
    LUT4 i7_4_lut_adj_255 (.A(ev_run_hold_s5[33]), .B(us_tx_c_33), .C(init_shadow[33]), 
         .D(swap_now_s5), .Z(n11471)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_255.init = 16'h5a66;
    LUT4 i7_4_lut_adj_256 (.A(ev_run_hold_s5[34]), .B(us_tx_c_34), .C(init_shadow[34]), 
         .D(swap_now_s5), .Z(n11473)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_256.init = 16'h5a66;
    LUT4 i7_4_lut_adj_257 (.A(ev_run_hold_s5[35]), .B(us_tx_c_35), .C(init_shadow[35]), 
         .D(swap_now_s5), .Z(n11475)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_257.init = 16'h5a66;
    LUT4 i7_4_lut_adj_258 (.A(ev_run_hold_s5[36]), .B(us_tx_c_36), .C(init_shadow[36]), 
         .D(swap_now_s5), .Z(n11477)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_258.init = 16'h5a66;
    LUT4 i7_4_lut_adj_259 (.A(ev_run_hold_s5[37]), .B(us_tx_c_37), .C(init_shadow[37]), 
         .D(swap_now_s5), .Z(n11479)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_259.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i6 (.D(ev_clear_addr_7__N_2237[6]), .SP(pll_clk_enable_726), 
            .CD(n15703), .CK(pll_clk), .Q(ev_clear_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_clear_addr_i6.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_260 (.A(ev_run_hold_s5[38]), .B(us_tx_c_38), .C(init_shadow[38]), 
         .D(swap_now_s5), .Z(n11481)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_260.init = 16'h5a66;
    LUT4 i7_4_lut_adj_261 (.A(ev_run_hold_s5[39]), .B(us_tx_c_39), .C(init_shadow[39]), 
         .D(swap_now_s5), .Z(n11483)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_261.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i5 (.D(ev_clear_addr_7__N_2237[5]), .SP(pll_clk_enable_726), 
            .CD(n15703), .CK(pll_clk), .Q(ev_clear_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_clear_addr_i5.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_262 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[24]), 
         .D(ev_bit[24]), .Z(ev_wr_data[24])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_262.init = 16'h7770;
    LUT4 i4_4_lut_adj_263 (.A(n23419), .B(n23205), .C(expected_next_15__N_1458[3]), 
         .D(n19010), .Z(spi1_sck_c_enable_43)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i4_4_lut_adj_263.init = 16'h1000;
    LUT4 i14571_4_lut (.A(ev_ch[6]), .B(ev_ch[0]), .C(ev_ch[1]), .D(ev_ch[4]), 
         .Z(n23363)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14571_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_264 (.A(ev_run_hold_s5[40]), .B(us_tx_c_40), .C(init_shadow[40]), 
         .D(swap_now_s5), .Z(n11485)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_264.init = 16'h5a66;
    CCU2D status_bit_index_1252_add_4_5 (.A0(status_bit_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22401), .COUT(n22402), .S0(n37), 
          .S1(n36));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252_add_4_5.INIT0 = 16'hfaaa;
    defparam status_bit_index_1252_add_4_5.INIT1 = 16'hfaaa;
    defparam status_bit_index_1252_add_4_5.INJECT1_0 = "NO";
    defparam status_bit_index_1252_add_4_5.INJECT1_1 = "NO";
    FD1P3IX ev_clear_addr_i4 (.D(ev_clear_addr_7__N_2237[4]), .SP(pll_clk_enable_726), 
            .CD(n15703), .CK(pll_clk), .Q(ev_clear_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_clear_addr_i4.GSR = "DISABLED";
    LUT4 i13576_2_lut (.A(spi_channel_field[1]), .B(spi_channel_field[0]), 
         .Z(n14)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(331[78:102])
    defparam i13576_2_lut.init = 16'h6666;
    CCU2D status_bit_index_1252_add_4_3 (.A0(status_bit_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22400), .COUT(n22401), .S0(n39), 
          .S1(n38));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252_add_4_3.INIT0 = 16'hfaaa;
    defparam status_bit_index_1252_add_4_3.INIT1 = 16'hfaaa;
    defparam status_bit_index_1252_add_4_3.INJECT1_0 = "NO";
    defparam status_bit_index_1252_add_4_3.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_265 (.A(ev_run_hold_s5[41]), .B(us_tx_c_41), .C(init_shadow[41]), 
         .D(swap_now_s5), .Z(n11487)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_265.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i3 (.D(ev_clear_addr_7__N_2237[3]), .SP(pll_clk_enable_726), 
            .CD(n15703), .CK(pll_clk), .Q(ev_clear_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_clear_addr_i3.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_266 (.A(ev_run_hold_s5[42]), .B(us_tx_c_42), .C(init_shadow[42]), 
         .D(swap_now_s5), .Z(n11489)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_266.init = 16'h5a66;
    PFUMX i14680 (.BLUT(n23461), .ALUT(n23462), .C0(spi1_miso_N_2513[1]), 
          .Z(n23473));
    CCU2D status_bit_index_1252_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22400), .S1(n40));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam status_bit_index_1252_add_4_1.INIT0 = 16'hF000;
    defparam status_bit_index_1252_add_4_1.INIT1 = 16'h0555;
    defparam status_bit_index_1252_add_4_1.INJECT1_0 = "NO";
    defparam status_bit_index_1252_add_4_1.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_267 (.A(ev_run_hold_s5[43]), .B(us_tx_c_43), .C(init_shadow[43]), 
         .D(swap_now_s5), .Z(n11491)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_267.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i2 (.D(ev_clear_addr_7__N_2237[2]), .SP(pll_clk_enable_726), 
            .CD(n15703), .CK(pll_clk), .Q(ev_clear_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_clear_addr_i2.GSR = "DISABLED";
    LUT4 i2_3_lut_rep_125 (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_bit_index[1]), .Z(n23917)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam i2_3_lut_rep_125.init = 16'h8080;
    LUT4 i1_2_lut_4_lut_adj_268 (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_bit_index[1]), .D(status_bit_index[6]), .Z(n6)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[66:89])
    defparam i1_2_lut_4_lut_adj_268.init = 16'h8000;
    CCU2D time_divider_1258_add_4_7 (.A0(time_divider[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22398), .S0(n35_adj_3167), 
          .S1(n34_adj_3168));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258_add_4_7.INIT0 = 16'hfaaa;
    defparam time_divider_1258_add_4_7.INIT1 = 16'hfaaa;
    defparam time_divider_1258_add_4_7.INJECT1_0 = "NO";
    defparam time_divider_1258_add_4_7.INJECT1_1 = "NO";
    FD1P3IX ev_clear_addr_i1 (.D(ev_clear_addr_7__N_2237[1]), .SP(pll_clk_enable_726), 
            .CD(n15703), .CK(pll_clk), .Q(ev_clear_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_clear_addr_i1.GSR = "DISABLED";
    LUT4 i1_4_lut_else_4_lut (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_hold[76]), .D(status_bit_index[1]), .Z(n23925)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(127[17:33])
    defparam i1_4_lut_else_4_lut.init = 16'h2000;
    FD1P3IX init_shadow_i83 (.D(n13226), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i83.GSR = "DISABLED";
    FD1P3IX init_shadow_i82 (.D(n13220), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i82.GSR = "DISABLED";
    FD1P3IX init_shadow_i81 (.D(n13214), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i81.GSR = "DISABLED";
    CCU2D time_divider_1258_add_4_5 (.A0(time_divider[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22397), .COUT(n22398), .S0(n37_adj_3165), 
          .S1(n36_adj_3166));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258_add_4_5.INIT0 = 16'hfaaa;
    defparam time_divider_1258_add_4_5.INIT1 = 16'hfaaa;
    defparam time_divider_1258_add_4_5.INJECT1_0 = "NO";
    defparam time_divider_1258_add_4_5.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i80 (.D(n13208), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i80.GSR = "DISABLED";
    FD1P3IX init_shadow_i79 (.D(n13200), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i79.GSR = "DISABLED";
    FD1P3IX init_shadow_i78 (.D(n13194), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i78.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_269 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[25]), 
         .D(ev_bit[25]), .Z(ev_wr_data[25])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_269.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_then_4_lut (.A(ev_state[2]), .B(ev_state[0]), .C(ev_state[3]), 
         .D(n1_adj_3205), .Z(n23923)) /* synthesis lut_function=(!(A ((C)+!B)+!A (B (C)+!B ((D)+!C)))) */ ;
    defparam i1_3_lut_4_lut_then_4_lut.init = 16'h0c1c;
    FD1P3IX init_shadow_i77 (.D(n13188), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i77.GSR = "DISABLED";
    CCU2D add_646_11 (.A0(phase_frac[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22366), .COUT(n22367), .S0(phase_frac_sum[9]), 
          .S1(phase_frac_sum[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_11.INIT0 = 16'h5555;
    defparam add_646_11.INIT1 = 16'h5aaa;
    defparam add_646_11.INJECT1_0 = "NO";
    defparam add_646_11.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i76 (.D(n13182), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i76.GSR = "DISABLED";
    FD1P3IX init_shadow_i75 (.D(n13176), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i75.GSR = "DISABLED";
    CCU2D add_160_11 (.A0(spi_byte_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22354), .COUT(n22355), .S0(spi_byte_count_15__N_1694[9]), 
          .S1(spi_byte_count_15__N_1694[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(339[35:57])
    defparam add_160_11.INIT0 = 16'h5aaa;
    defparam add_160_11.INIT1 = 16'h5aaa;
    defparam add_160_11.INJECT1_0 = "NO";
    defparam add_160_11.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i74 (.D(n13164), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i74.GSR = "DISABLED";
    FD1P3IX init_shadow_i73 (.D(n13158), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i73.GSR = "DISABLED";
    LUT4 i9679_2_lut (.A(spi_byte_count[5]), .B(spi_byte_count[4]), .Z(n18471)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i9679_2_lut.init = 16'heeee;
    FD1P3IX init_shadow_i72 (.D(n13152), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i72.GSR = "DISABLED";
    CCU2D add_160_9 (.A0(spi_byte_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22353), .COUT(n22354), .S0(spi_byte_count_15__N_1694[7]), 
          .S1(spi_byte_count_15__N_1694[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(339[35:57])
    defparam add_160_9.INIT0 = 16'h5aaa;
    defparam add_160_9.INIT1 = 16'h5aaa;
    defparam add_160_9.INJECT1_0 = "NO";
    defparam add_160_9.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i71 (.D(n13146), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i71.GSR = "DISABLED";
    FD1P3IX init_shadow_i70 (.D(n13140), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i70.GSR = "DISABLED";
    CCU2D add_646_9 (.A0(phase_frac[7]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[8]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22365), .COUT(n22366), .S0(phase_frac_sum[7]), .S1(phase_frac_sum[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_9.INIT0 = 16'h5555;
    defparam add_646_9.INIT1 = 16'h5aaa;
    defparam add_646_9.INJECT1_0 = "NO";
    defparam add_646_9.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i69 (.D(n13134), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i69.GSR = "DISABLED";
    FD1P3IX init_shadow_i68 (.D(n13128), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i68.GSR = "DISABLED";
    LUT4 i1_3_lut_rep_126 (.A(spi_command[4]), .B(spi_command[1]), .C(spi_command[0]), 
         .Z(n23918)) /* synthesis lut_function=((B (C)+!B !(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i1_3_lut_rep_126.init = 16'hd7d7;
    FD1P3IX init_shadow_i67 (.D(n13122), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i67.GSR = "DISABLED";
    FD1P3IX init_shadow_i66 (.D(n13116), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i66.GSR = "DISABLED";
    CCU2D time_divider_1258_add_4_3 (.A0(time_divider[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22396), .COUT(n22397), .S0(n39_adj_3163), 
          .S1(n38_adj_3164));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258_add_4_3.INIT0 = 16'hfaaa;
    defparam time_divider_1258_add_4_3.INIT1 = 16'hfaaa;
    defparam time_divider_1258_add_4_3.INJECT1_0 = "NO";
    defparam time_divider_1258_add_4_3.INJECT1_1 = "NO";
    CCU2D time_divider_1258_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(time_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22396), .S1(n40_adj_3162));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:48])
    defparam time_divider_1258_add_4_1.INIT0 = 16'hF000;
    defparam time_divider_1258_add_4_1.INIT1 = 16'h0555;
    defparam time_divider_1258_add_4_1.INJECT1_0 = "NO";
    defparam time_divider_1258_add_4_1.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i65 (.D(n13110), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i65.GSR = "DISABLED";
    FD1P3IX init_shadow_i64 (.D(n13099), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i64.GSR = "DISABLED";
    LUT4 i14627_4_lut (.A(spi_byte_count[5]), .B(n14339), .C(spi_byte_count[3]), 
         .D(spi_byte_count[4]), .Z(n23419)) /* synthesis lut_function=(A+(B+(C (D)))) */ ;
    defparam i14627_4_lut.init = 16'hfeee;
    FD1P3IX init_shadow_i63 (.D(n13093), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i63.GSR = "DISABLED";
    CCU2D add_646_7 (.A0(phase_frac[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22364), .COUT(n22365), .S0(phase_frac_sum[5]), .S1(phase_frac_sum[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_7.INIT0 = 16'h5555;
    defparam add_646_7.INIT1 = 16'h5555;
    defparam add_646_7.INJECT1_0 = "NO";
    defparam add_646_7.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i62 (.D(n13087), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i62.GSR = "DISABLED";
    FD1P3IX init_shadow_i61 (.D(n13081), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i61.GSR = "DISABLED";
    CCU2D add_160_7 (.A0(spi_byte_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22352), .COUT(n22353), .S0(spi_byte_count_15__N_1694[5]), 
          .S1(spi_byte_count_15__N_1694[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(339[35:57])
    defparam add_160_7.INIT0 = 16'h5aaa;
    defparam add_160_7.INIT1 = 16'h5aaa;
    defparam add_160_7.INJECT1_0 = "NO";
    defparam add_160_7.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_270 (.A(spi_command[4]), .B(spi_command[1]), 
         .C(spi_command[0]), .D(n14026), .Z(n23062)) /* synthesis lut_function=(A (B (C (D)))+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i1_3_lut_4_lut_adj_270.init = 16'hd000;
    CCU2D add_648_8 (.A0(staging_q[14]), .B0(staging_q[6]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[15]), .B1(staging_q[7]), .C1(GND_net), 
          .D1(GND_net), .CIN(n22348), .COUT(n22349), .S0(build_sum_8__N_2053[6]), 
          .S1(build_sum_8__N_2053[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(507[32:80])
    defparam add_648_8.INIT0 = 16'h5666;
    defparam add_648_8.INIT1 = 16'h5666;
    defparam add_648_8.INJECT1_0 = "NO";
    defparam add_648_8.INJECT1_1 = "NO";
    CCU2D add_646_5 (.A0(phase_frac[3]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22363), .COUT(n22364), .S0(phase_frac_sum[3]), .S1(phase_frac_sum[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_5.INIT0 = 16'h5aaa;
    defparam add_646_5.INIT1 = 16'h5aaa;
    defparam add_646_5.INJECT1_0 = "NO";
    defparam add_646_5.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i60 (.D(n13075), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i60.GSR = "DISABLED";
    CCU2D spi_channel_index_1253_add_4_7 (.A0(spi_channel_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22394), .S0(n35_adj_3177), 
          .S1(n34_adj_3178));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253_add_4_7.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1253_add_4_7.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1253_add_4_7.INJECT1_0 = "NO";
    defparam spi_channel_index_1253_add_4_7.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_127 (.A(spi_command[7]), .B(spi_command[6]), .Z(n23919)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[34:54])
    defparam i1_2_lut_rep_127.init = 16'heeee;
    CCU2D add_646_25 (.A0(phase_frac[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22373), .S0(phase_frac_sum[23]), .S1(phase_frac_sum[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_25.INIT0 = 16'h5aaa;
    defparam add_646_25.INIT1 = 16'h0000;
    defparam add_646_25.INJECT1_0 = "NO";
    defparam add_646_25.INJECT1_1 = "NO";
    LUT4 i2_3_lut_rep_96_4_lut (.A(spi_command[7]), .B(spi_command[6]), 
         .C(n23918), .D(n20519), .Z(n23888)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[34:54])
    defparam i2_3_lut_rep_96_4_lut.init = 16'hfffe;
    FD1P3IX init_shadow_i35 (.D(n12921), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i35.GSR = "DISABLED";
    CCU2D add_646_3 (.A0(phase_frac[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22362), .COUT(n22363), .S0(phase_frac_sum[1]), .S1(phase_frac_sum[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_3.INIT0 = 16'h5aaa;
    defparam add_646_3.INIT1 = 16'h5aaa;
    defparam add_646_3.INJECT1_0 = "NO";
    defparam add_646_3.INJECT1_1 = "NO";
    LUT4 i1634_1_lut (.A(status_bit_index[5]), .Z(spi1_miso_N_2513[5])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i1634_1_lut.init = 16'h5555;
    CCU2D add_648_4 (.A0(staging_q[10]), .B0(staging_q[2]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[11]), .B1(staging_q[3]), .C1(GND_net), 
          .D1(GND_net), .CIN(n22346), .COUT(n22347), .S0(build_sum_8__N_2053[2]), 
          .S1(build_sum_8__N_2053[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(507[32:80])
    defparam add_648_4.INIT0 = 16'h5666;
    defparam add_648_4.INIT1 = 16'h5666;
    defparam add_648_4.INJECT1_0 = "NO";
    defparam add_648_4.INJECT1_1 = "NO";
    LUT4 i8_3_lut_4_lut (.A(spi_command[7]), .B(spi_command[6]), .C(n16), 
         .D(spi_version[3]), .Z(n18)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[34:54])
    defparam i8_3_lut_4_lut.init = 16'hfffe;
    LUT4 i14809_2_lut_rep_128 (.A(ev_state[2]), .B(ev_state[1]), .Z(n23920)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i14809_2_lut_rep_128.init = 16'h1111;
    LUT4 i14803_2_lut_rep_97_3_lut (.A(ev_state[2]), .B(ev_state[1]), .C(ev_state[3]), 
         .Z(n23889)) /* synthesis lut_function=(!(A+(B+(C)))) */ ;
    defparam i14803_2_lut_rep_97_3_lut.init = 16'h0101;
    LUT4 i24_4_lut_4_lut (.A(ev_state[2]), .B(ev_state[1]), .C(ev_state[3]), 
         .D(n23877), .Z(pll_clk_enable_11)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;
    defparam i24_4_lut_4_lut.init = 16'h5150;
    LUT4 i1_2_lut_3_lut_4_lut_adj_271 (.A(ev_state[2]), .B(ev_state[1]), 
         .C(ev_state[0]), .D(ev_state[3]), .Z(pll_clk_enable_16)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_271.init = 16'h0010;
    LUT4 active_bank_I_0_1_lut_rep_129 (.A(active_bank), .Z(n23921)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(217[34:46])
    defparam active_bank_I_0_1_lut_rep_129.init = 16'h5555;
    CCU2D add_646_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22362), .S1(phase_frac_sum[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_1.INIT0 = 16'hF000;
    defparam add_646_1.INIT1 = 16'h5555;
    defparam add_646_1.INJECT1_0 = "NO";
    defparam add_646_1.INJECT1_1 = "NO";
    LUT4 run_addr_s3_8__I_0_i9_3_lut_3_lut (.A(active_bank), .B(n22496), 
         .C(run_addr_s3[8]), .Z(event_rd_addr[8])) /* synthesis lut_function=(A (B (C))+!A ((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(217[34:46])
    defparam run_addr_s3_8__I_0_i9_3_lut_3_lut.init = 16'hd1d1;
    CCU2D add_646_23 (.A0(phase_frac[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22372), .COUT(n22373), .S0(phase_frac_sum[21]), 
          .S1(phase_frac_sum[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_23.INIT0 = 16'h5aaa;
    defparam add_646_23.INIT1 = 16'h5aaa;
    defparam add_646_23.INJECT1_0 = "NO";
    defparam add_646_23.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_272 (.A(ev_run_hold_s5[44]), .B(us_tx_c_44), .C(init_shadow[44]), 
         .D(swap_now_s5), .Z(n11493)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_272.init = 16'h5a66;
    CCU2D spi_channel_index_1253_add_4_5 (.A0(spi_channel_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22393), .COUT(n22394), .S0(n37_adj_3175), 
          .S1(n36_adj_3176));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253_add_4_5.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1253_add_4_5.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1253_add_4_5.INJECT1_0 = "NO";
    defparam spi_channel_index_1253_add_4_5.INJECT1_1 = "NO";
    CCU2D add_646_21 (.A0(phase_frac[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22371), .COUT(n22372), .S0(phase_frac_sum[19]), 
          .S1(phase_frac_sum[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_21.INIT0 = 16'h5aaa;
    defparam add_646_21.INIT1 = 16'h5555;
    defparam add_646_21.INJECT1_0 = "NO";
    defparam add_646_21.INJECT1_1 = "NO";
    CCU2D add_324_9 (.A0(ev_clear_addr[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22361), .S0(ev_clear_addr_7__N_2237[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(490[34:54])
    defparam add_324_9.INIT0 = 16'h5aaa;
    defparam add_324_9.INIT1 = 16'h0000;
    defparam add_324_9.INJECT1_0 = "NO";
    defparam add_324_9.INJECT1_1 = "NO";
    PFUMX i14681 (.BLUT(n23463), .ALUT(n23464), .C0(spi1_miso_N_2513[1]), 
          .Z(n23474));
    CCU2D add_648_6 (.A0(staging_q[12]), .B0(staging_q[4]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[13]), .B1(staging_q[5]), .C1(GND_net), 
          .D1(GND_net), .CIN(n22347), .COUT(n22348), .S0(build_sum_8__N_2053[4]), 
          .S1(build_sum_8__N_2053[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(507[32:80])
    defparam add_648_6.INIT0 = 16'h5666;
    defparam add_648_6.INIT1 = 16'h5666;
    defparam add_648_6.INJECT1_0 = "NO";
    defparam add_648_6.INJECT1_1 = "NO";
    CCU2D equal_1859_17 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n22268), 
          .S0(frame_end_N_2613));
    defparam equal_1859_17.INIT0 = 16'hFFFF;
    defparam equal_1859_17.INIT1 = 16'h0000;
    defparam equal_1859_17.INJECT1_0 = "NO";
    defparam equal_1859_17.INJECT1_1 = "NO";
    CCU2D add_648_2 (.A0(staging_q[8]), .B0(staging_q[0]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[9]), .B1(staging_q[1]), .C1(GND_net), 
          .D1(GND_net), .COUT(n22346), .S1(build_sum_8__N_2053[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(507[32:80])
    defparam add_648_2.INIT0 = 16'h7000;
    defparam add_648_2.INIT1 = 16'h5666;
    defparam add_648_2.INJECT1_0 = "NO";
    defparam add_648_2.INJECT1_1 = "NO";
    CCU2D equal_1859_17_13564 (.A0(spi_expected_length[3]), .B0(spi_byte_count_15__N_1694[3]), 
          .C0(spi_expected_length[2]), .D0(spi_byte_count_15__N_1694[2]), 
          .A1(spi_expected_length[1]), .B1(spi_byte_count_15__N_1694[1]), 
          .C1(spi_expected_length[0]), .D1(spi_byte_count_15__N_1694[0]), 
          .CIN(n22267), .COUT(n22268));
    defparam equal_1859_17_13564.INIT0 = 16'h9009;
    defparam equal_1859_17_13564.INIT1 = 16'h9009;
    defparam equal_1859_17_13564.INJECT1_0 = "YES";
    defparam equal_1859_17_13564.INJECT1_1 = "YES";
    CCU2D equal_1859_13 (.A0(spi_expected_length[11]), .B0(spi_byte_count_15__N_1694[11]), 
          .C0(spi_expected_length[10]), .D0(spi_byte_count_15__N_1694[10]), 
          .A1(spi_expected_length[9]), .B1(spi_byte_count_15__N_1694[9]), 
          .C1(spi_expected_length[8]), .D1(spi_byte_count_15__N_1694[8]), 
          .CIN(n22265), .COUT(n22266));
    defparam equal_1859_13.INIT0 = 16'h9009;
    defparam equal_1859_13.INIT1 = 16'h9009;
    defparam equal_1859_13.INJECT1_0 = "YES";
    defparam equal_1859_13.INJECT1_1 = "YES";
    CCU2D equal_1859_15 (.A0(spi_expected_length[7]), .B0(spi_byte_count_15__N_1694[7]), 
          .C0(spi_expected_length[6]), .D0(spi_byte_count_15__N_1694[6]), 
          .A1(spi_expected_length[5]), .B1(spi_byte_count_15__N_1694[5]), 
          .C1(spi_expected_length[4]), .D1(spi_byte_count_15__N_1694[4]), 
          .CIN(n22266), .COUT(n22267));
    defparam equal_1859_15.INIT0 = 16'h9009;
    defparam equal_1859_15.INIT1 = 16'h9009;
    defparam equal_1859_15.INJECT1_0 = "YES";
    defparam equal_1859_15.INJECT1_1 = "YES";
    LUT4 i7_4_lut_adj_273 (.A(ev_run_hold_s5[45]), .B(us_tx_c_45), .C(init_shadow[45]), 
         .D(swap_now_s5), .Z(n11495)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_273.init = 16'h5a66;
    PFUMX i14914 (.BLUT(n23922), .ALUT(n23923), .C0(ev_state[1]), .Z(n23924));
    LUT4 i1_3_lut_4_lut_adj_274 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[79]), 
         .D(ev_bit[79]), .Z(ev_wr_data[79])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_274.init = 16'h7770;
    LUT4 i7_4_lut_adj_275 (.A(ev_run_hold_s5[46]), .B(us_tx_c_46), .C(init_shadow[46]), 
         .D(swap_now_s5), .Z(n11497)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_275.init = 16'h5a66;
    LUT4 i7_4_lut_adj_276 (.A(ev_run_hold_s5[47]), .B(us_tx_c_47), .C(init_shadow[47]), 
         .D(swap_now_s5), .Z(n11499)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_276.init = 16'h5a66;
    LUT4 i2_4_lut_adj_277 (.A(spi_byte_count[7]), .B(spi_byte_count[5]), 
         .C(spi_byte_count[6]), .D(n4_adj_3196), .Z(n19010)) /* synthesis lut_function=(A (B (C)+!B (C (D)))) */ ;
    defparam i2_4_lut_adj_277.init = 16'ha080;
    PFUMX i14682 (.BLUT(n23465), .ALUT(n23466), .C0(spi1_miso_N_2513[1]), 
          .Z(n23475));
    CCU2D equal_1859_0 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(frame_end_N_2614[16]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22264));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(291[48:95])
    defparam equal_1859_0.INIT0 = 16'hF000;
    defparam equal_1859_0.INIT1 = 16'h5555;
    defparam equal_1859_0.INJECT1_0 = "NO";
    defparam equal_1859_0.INJECT1_1 = "YES";
    LUT4 i7_4_lut_adj_278 (.A(ev_run_hold_s5[48]), .B(us_tx_c_48), .C(init_shadow[48]), 
         .D(swap_now_s5), .Z(n11501)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_278.init = 16'h5a66;
    LUT4 i7_4_lut_adj_279 (.A(ev_run_hold_s5[49]), .B(us_tx_c_49), .C(init_shadow[49]), 
         .D(swap_now_s5), .Z(n11503)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_279.init = 16'h5a66;
    PFUMX i14683 (.BLUT(n23467), .ALUT(n23468), .C0(spi1_miso_N_2513[1]), 
          .Z(n23476));
    LUT4 n1_bdd_2_lut (.A(ev_state[2]), .B(ev_state[3]), .Z(n23711)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam n1_bdd_2_lut.init = 16'h2222;
    LUT4 i7_4_lut_adj_280 (.A(ev_run_hold_s5[50]), .B(us_tx_c_50), .C(init_shadow[50]), 
         .D(swap_now_s5), .Z(n11505)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_280.init = 16'h5a66;
    LUT4 i7_4_lut_adj_281 (.A(ev_run_hold_s5[51]), .B(us_tx_c_51), .C(init_shadow[51]), 
         .D(swap_now_s5), .Z(n11507)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_281.init = 16'h5a66;
    PFUMX i14707 (.BLUT(n23484), .ALUT(n23485), .C0(spi1_miso_N_2513[1]), 
          .Z(n23500));
    LUT4 i7_4_lut_adj_282 (.A(ev_run_hold_s5[52]), .B(us_tx_c_52), .C(init_shadow[52]), 
         .D(swap_now_s5), .Z(n11509)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_282.init = 16'h5a66;
    LUT4 i7_4_lut_adj_283 (.A(ev_run_hold_s5[53]), .B(us_tx_c_53), .C(init_shadow[53]), 
         .D(swap_now_s5), .Z(n11511)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_283.init = 16'h5a66;
    PFUMX i14708 (.BLUT(n23486), .ALUT(n23487), .C0(spi1_miso_N_2513[1]), 
          .Z(n23501));
    LUT4 i1_3_lut_4_lut_adj_284 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[81]), 
         .D(ev_bit[81]), .Z(ev_wr_data[81])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_284.init = 16'h7770;
    LUT4 i7_4_lut_adj_285 (.A(ev_run_hold_s5[54]), .B(us_tx_c_54), .C(init_shadow[54]), 
         .D(swap_now_s5), .Z(n11513)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_285.init = 16'h5a66;
    LUT4 i7_4_lut_adj_286 (.A(ev_run_hold_s5[55]), .B(us_tx_c_55), .C(init_shadow[55]), 
         .D(swap_now_s5), .Z(n11515)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_286.init = 16'h5a66;
    LUT4 i7_4_lut_adj_287 (.A(ev_run_hold_s5[56]), .B(us_tx_c_56), .C(init_shadow[56]), 
         .D(swap_now_s5), .Z(n11517)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_287.init = 16'h5a66;
    LUT4 i7_4_lut_adj_288 (.A(ev_run_hold_s5[57]), .B(us_tx_c_57), .C(init_shadow[57]), 
         .D(swap_now_s5), .Z(n11519)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_288.init = 16'h5a66;
    PFUMX i14709 (.BLUT(n23488), .ALUT(n23489), .C0(spi1_miso_N_2513[1]), 
          .Z(n23502));
    LUT4 i1_2_lut_adj_289 (.A(spi_byte_count[0]), .B(n23074), .Z(spi1_sck_c_enable_117)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_289.init = 16'h4444;
    LUT4 i2_4_lut_adj_290 (.A(n23006), .B(spi1_sck_c_enable_197), .C(spi_byte_count[4]), 
         .D(n23017), .Z(n23074)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i2_4_lut_adj_290.init = 16'h0004;
    LUT4 i4_4_lut_adj_291 (.A(n7_adj_3206), .B(n23874), .C(spi_byte_count[5]), 
         .D(spi_byte_count[3]), .Z(n23017)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut_adj_291.init = 16'hfffe;
    LUT4 i5_4_lut_adj_292 (.A(n9), .B(expected_next_15__N_1417[7]), .C(n8_adj_3197), 
         .D(fpga_cs_n_c), .Z(spi1_sck_c_enable_237)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i5_4_lut_adj_292.init = 16'h0080;
    PFUMX i14710 (.BLUT(n23490), .ALUT(n23491), .C0(spi1_miso_N_2513[1]), 
          .Z(n23503));
    LUT4 i1_3_lut_4_lut_adj_293 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[26]), 
         .D(ev_bit[26]), .Z(ev_wr_data[26])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_293.init = 16'h7770;
    LUT4 mux_1128_i2_3_lut_4_lut (.A(n23889), .B(ev_state[0]), .C(ev_wr_addr_8__N_912[1]), 
         .D(ev_clear_addr[1]), .Z(ev_wr_addr[1])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mux_1128_i2_3_lut_4_lut.init = 16'hf870;
    LUT4 i1_3_lut_4_lut_adj_294 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[47]), 
         .D(ev_bit[47]), .Z(ev_wr_data[47])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_294.init = 16'h7770;
    LUT4 i2_2_lut_adj_295 (.A(spi_byte_count[8]), .B(n23854), .Z(n8_adj_3197)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i2_2_lut_adj_295.init = 16'h4444;
    LUT4 i7_4_lut_adj_296 (.A(ev_run_hold_s5[58]), .B(us_tx_c_58), .C(init_shadow[58]), 
         .D(swap_now_s5), .Z(n11521)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_296.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_297 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[46]), 
         .D(ev_bit[46]), .Z(ev_wr_data[46])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_297.init = 16'h7770;
    LUT4 i7_4_lut_adj_298 (.A(ev_run_hold_s5[59]), .B(us_tx_c_59), .C(init_shadow[59]), 
         .D(swap_now_s5), .Z(n11523)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_298.init = 16'h5a66;
    LUT4 i1_4_lut_adj_299 (.A(n60), .B(n23143), .C(n14336), .D(n18471), 
         .Z(n14339)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[17] 338[24])
    defparam i1_4_lut_adj_299.init = 16'h3032;
    LUT4 mux_1128_i3_3_lut_4_lut (.A(n23889), .B(ev_state[0]), .C(ev_wr_addr_8__N_912[2]), 
         .D(ev_clear_addr[2]), .Z(ev_wr_addr[2])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mux_1128_i3_3_lut_4_lut.init = 16'hf870;
    LUT4 i7_4_lut_adj_300 (.A(ev_run_hold_s5[60]), .B(us_tx_c_60), .C(init_shadow[60]), 
         .D(swap_now_s5), .Z(n11525)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_300.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_301 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[27]), 
         .D(ev_bit[27]), .Z(ev_wr_data[27])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_301.init = 16'h7770;
    PFUMX i14711 (.BLUT(n23492), .ALUT(n23493), .C0(spi1_miso_N_2513[1]), 
          .Z(n23504));
    LUT4 i1_3_lut_adj_302 (.A(n14467), .B(init_shadow[50]), .C(ev_bit[50]), 
         .Z(n13015)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_302.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_303 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[28]), 
         .D(ev_bit[28]), .Z(ev_wr_data[28])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_303.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_304 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[29]), 
         .D(ev_bit[29]), .Z(ev_wr_data[29])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_304.init = 16'h7770;
    LUT4 i2_3_lut_adj_305 (.A(ev_state[1]), .B(build_sum[8]), .C(ev_state[0]), 
         .Z(n14467)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i2_3_lut_adj_305.init = 16'h4040;
    LUT4 i1_3_lut_4_lut_else_4_lut (.A(ev_state[2]), .B(ev_state[0]), .C(ev_state[3]), 
         .Z(n23922)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;
    defparam i1_3_lut_4_lut_else_4_lut.init = 16'h1010;
    LUT4 mux_1128_i4_3_lut_4_lut (.A(n23889), .B(ev_state[0]), .C(ev_wr_addr_8__N_912[3]), 
         .D(ev_clear_addr[3]), .Z(ev_wr_addr[3])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mux_1128_i4_3_lut_4_lut.init = 16'hf870;
    LUT4 cs_sync_d_I_0_2_lut (.A(cs_sync_d), .B(cs_sync), .Z(cs_fall)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(258[20:41])
    defparam cs_sync_d_I_0_2_lut.init = 16'h2222;
    LUT4 run_addr_s3_8__I_0_i6_3_lut (.A(run_addr_s3[5]), .B(ev_rd_slot[5]), 
         .C(n22496), .Z(event_rd_addr[5])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i6_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_4_lut_adj_306 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[66]), 
         .D(ev_bit[66]), .Z(ev_wr_data[66])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_306.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_307 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[67]), 
         .D(ev_bit[67]), .Z(ev_wr_data[67])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_307.init = 16'h7770;
    LUT4 i7_4_lut_adj_308 (.A(ev_run_hold_s5[61]), .B(us_tx_c_61), .C(init_shadow[61]), 
         .D(swap_now_s5), .Z(n11527)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_308.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_309 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[68]), 
         .D(ev_bit[68]), .Z(ev_wr_data[68])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_309.init = 16'h7770;
    LUT4 i11478_3_lut (.A(active_bank), .B(wrap_s2), .C(swap_pending), 
         .Z(run_addr_s3_8__N_425[8])) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(202[28:40])
    defparam i11478_3_lut.init = 16'h6a6a;
    LUT4 i7_4_lut_adj_310 (.A(ev_run_hold_s5[62]), .B(us_tx_c_62), .C(init_shadow[62]), 
         .D(swap_now_s5), .Z(n11529)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_310.init = 16'h5a66;
    CCU2D add_646_19 (.A0(phase_frac[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22370), .COUT(n22371), .S0(phase_frac_sum[17]), 
          .S1(phase_frac_sum[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_19.INIT0 = 16'h5aaa;
    defparam add_646_19.INIT1 = 16'h5555;
    defparam add_646_19.INJECT1_0 = "NO";
    defparam add_646_19.INJECT1_1 = "NO";
    CCU2D add_324_7 (.A0(ev_clear_addr[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22360), .COUT(n22361), .S0(ev_clear_addr_7__N_2237[5]), 
          .S1(ev_clear_addr_7__N_2237[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(490[34:54])
    defparam add_324_7.INIT0 = 16'h5aaa;
    defparam add_324_7.INIT1 = 16'h5aaa;
    defparam add_324_7.INJECT1_0 = "NO";
    defparam add_324_7.INJECT1_1 = "NO";
    LUT4 mux_1407_i7_3_lut (.A(n10169), .B(n10170), .C(n10156), .Z(rd_data_15__N_2649[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i7_3_lut.init = 16'hcaca;
    LUT4 mux_1407_i8_3_lut (.A(n10171), .B(n10172), .C(n10156), .Z(rd_data_15__N_2649[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i8_3_lut.init = 16'hcaca;
    LUT4 i7_4_lut_adj_311 (.A(ev_run_hold_s5[63]), .B(us_tx_c_63), .C(init_shadow[63]), 
         .D(swap_now_s5), .Z(n11531)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_311.init = 16'h5a66;
    LUT4 mux_1407_i9_3_lut (.A(n10173), .B(n10174), .C(n10156), .Z(rd_data_15__N_2649[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i9_3_lut.init = 16'hcaca;
    LUT4 i9698_2_lut (.A(spi_byte_count[2]), .B(spi_byte_count[1]), .Z(n18490)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i9698_2_lut.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_312 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[69]), 
         .D(ev_bit[69]), .Z(ev_wr_data[69])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_312.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_313 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[70]), 
         .D(ev_bit[70]), .Z(ev_wr_data[70])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_313.init = 16'h7770;
    LUT4 i7_4_lut_adj_314 (.A(ev_run_hold_s5[64]), .B(us_tx_c_64), .C(init_shadow[64]), 
         .D(swap_now_s5), .Z(n11533)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_314.init = 16'h5a66;
    LUT4 i78_4_lut (.A(n133), .B(n23405), .C(n91), .D(n23319), .Z(n166)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;
    defparam i78_4_lut.init = 16'hfbff;
    LUT4 i1_3_lut_4_lut_adj_315 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[71]), 
         .D(ev_bit[71]), .Z(ev_wr_data[71])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_315.init = 16'h7770;
    LUT4 i4357_4_lut (.A(ev_ch[6]), .B(staging_rd_addr[6]), .C(n9_adj_3199), 
         .D(n23094), .Z(staging_rd_addr_6__N_901[6])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i4357_4_lut.init = 16'hac0c;
    LUT4 i1_3_lut_4_lut_adj_316 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[72]), 
         .D(ev_bit[72]), .Z(ev_wr_data[72])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_316.init = 16'h7770;
    LUT4 i14813_3_lut_4_lut_4_lut_4_lut_4_lut (.A(ev_state[2]), .B(ev_state[1]), 
         .C(ev_state[3]), .D(ev_state[0]), .Z(ev_we)) /* synthesis lut_function=(!(A ((C+(D))+!B)+!A (B+!(D)))) */ ;
    defparam i14813_3_lut_4_lut_4_lut_4_lut_4_lut.init = 16'h1108;
    LUT4 i4353_4_lut (.A(ev_ch[4]), .B(staging_rd_addr[4]), .C(n9_adj_3199), 
         .D(n23094), .Z(staging_rd_addr_6__N_901[4])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i4353_4_lut.init = 16'hac0c;
    PFUMX i14713 (.BLUT(n23496), .ALUT(n23497), .C0(spi1_miso_N_2513[1]), 
          .Z(n23506));
    LUT4 i1636_1_lut (.A(status_bit_index[3]), .Z(spi1_miso_N_2513[3])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[18:44])
    defparam i1636_1_lut.init = 16'h5555;
    LUT4 i2_4_lut_adj_317 (.A(ev_state[1]), .B(n23221), .C(ev_state_3__N_1940[1]), 
         .D(ev_state[0]), .Z(n9_adj_3199)) /* synthesis lut_function=(!(A (B+(D))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    defparam i2_4_lut_adj_317.init = 16'h1022;
    LUT4 i14431_2_lut (.A(ev_state[3]), .B(ev_state[2]), .Z(n23221)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14431_2_lut.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_318 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[56]), 
         .D(ev_bit[56]), .Z(ev_wr_data[56])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_318.init = 16'h7770;
    LUT4 mux_1128_i5_3_lut_4_lut (.A(n23889), .B(ev_state[0]), .C(ev_wr_addr_8__N_912[4]), 
         .D(ev_clear_addr[4]), .Z(ev_wr_addr[4])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mux_1128_i5_3_lut_4_lut.init = 16'hf870;
    LUT4 i1_2_lut_adj_319 (.A(ev_state[1]), .B(ev_state[0]), .Z(n23094)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_adj_319.init = 16'h2222;
    FD1P3IX ev_bit_i22 (.D(ev_bit[21]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i22.GSR = "DISABLED";
    LUT4 i760_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_360)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(585[18] 587[12])
    defparam i760_2_lut.init = 16'h4444;
    LUT4 i1_3_lut_4_lut_adj_320 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[73]), 
         .D(ev_bit[73]), .Z(ev_wr_data[73])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_320.init = 16'h7770;
    LUT4 i7_4_lut_adj_321 (.A(ev_run_hold_s5[65]), .B(us_tx_c_65), .C(init_shadow[65]), 
         .D(swap_now_s5), .Z(n11535)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_321.init = 16'h5a66;
    LUT4 i7_4_lut_adj_322 (.A(ev_run_hold_s5[66]), .B(us_tx_c_66), .C(init_shadow[66]), 
         .D(swap_now_s5), .Z(n11537)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_322.init = 16'h5a66;
    LUT4 i7_4_lut_adj_323 (.A(ev_run_hold_s5[0]), .B(us_tx_c_0), .C(init_shadow[0]), 
         .D(swap_now_s5), .Z(n10778)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_323.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_324 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[74]), 
         .D(ev_bit[74]), .Z(ev_wr_data[74])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_324.init = 16'h7770;
    LUT4 i7_4_lut_adj_325 (.A(ev_run_hold_s5[67]), .B(us_tx_c_67), .C(init_shadow[67]), 
         .D(swap_now_s5), .Z(n11539)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_325.init = 16'h5a66;
    LUT4 i7_4_lut_adj_326 (.A(ev_run_hold_s5[68]), .B(us_tx_c_68), .C(init_shadow[68]), 
         .D(swap_now_s5), .Z(n11541)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_326.init = 16'h5a66;
    FD1P3IX init_shadow_i55 (.D(n13045), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i55.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_327 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[55]), 
         .D(ev_bit[55]), .Z(ev_wr_data[55])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_327.init = 16'h7770;
    LUT4 mux_1128_i6_3_lut_4_lut (.A(n23889), .B(ev_state[0]), .C(ev_wr_addr_8__N_912[5]), 
         .D(ev_clear_addr[5]), .Z(ev_wr_addr[5])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mux_1128_i6_3_lut_4_lut.init = 16'hf870;
    LUT4 mux_1407_i10_3_lut (.A(n10175), .B(n10176), .C(n10156), .Z(rd_data_15__N_2649[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i10_3_lut.init = 16'hcaca;
    LUT4 i14463_4_lut (.A(spi_bitmap[47]), .B(spi_bitmap[66]), .C(spi_bitmap[71]), 
         .D(spi_bitmap[17]), .Z(n23253)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14463_4_lut.init = 16'h8000;
    LUT4 mux_1407_i11_3_lut (.A(n10177), .B(n10178), .C(n10156), .Z(rd_data_15__N_2649[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i11_3_lut.init = 16'hcaca;
    LUT4 i7_4_lut_adj_328 (.A(ev_run_hold_s5[69]), .B(us_tx_c_69), .C(init_shadow[69]), 
         .D(swap_now_s5), .Z(n11543)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_328.init = 16'h5a66;
    LUT4 i14585_4_lut (.A(spi_bitmap[15]), .B(spi_bitmap[36]), .C(spi_bitmap[29]), 
         .D(spi_bitmap[39]), .Z(n23377)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14585_4_lut.init = 16'h8000;
    LUT4 mux_1407_i12_3_lut (.A(n10179), .B(n10180), .C(n10156), .Z(rd_data_15__N_2649[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i12_3_lut.init = 16'hcaca;
    LUT4 i7_4_lut_adj_329 (.A(ev_run_hold_s5[70]), .B(us_tx_c_70), .C(init_shadow[70]), 
         .D(swap_now_s5), .Z(n11545)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_329.init = 16'h5a66;
    LUT4 i7_4_lut_adj_330 (.A(ev_run_hold_s5[71]), .B(us_tx_c_71), .C(init_shadow[71]), 
         .D(swap_now_s5), .Z(n11547)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_330.init = 16'h5a66;
    LUT4 mux_1128_i7_3_lut_4_lut (.A(n23889), .B(ev_state[0]), .C(ev_wr_addr_8__N_912[6]), 
         .D(ev_clear_addr[6]), .Z(ev_wr_addr[6])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mux_1128_i7_3_lut_4_lut.init = 16'hf870;
    LUT4 i7_4_lut_adj_331 (.A(ev_run_hold_s5[72]), .B(us_tx_c_72), .C(init_shadow[72]), 
         .D(swap_now_s5), .Z(n11549)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_331.init = 16'h5a66;
    LUT4 i7_4_lut_adj_332 (.A(ev_run_hold_s5[73]), .B(us_tx_c_73), .C(init_shadow[73]), 
         .D(swap_now_s5), .Z(n11551)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_332.init = 16'h5a66;
    LUT4 i7_4_lut_adj_333 (.A(ev_run_hold_s5[74]), .B(us_tx_c_74), .C(init_shadow[74]), 
         .D(swap_now_s5), .Z(n11553)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_333.init = 16'h5a66;
    LUT4 mux_1407_i13_3_lut (.A(n10181), .B(n10182), .C(n10156), .Z(rd_data_15__N_2649[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i13_3_lut.init = 16'hcaca;
    LUT4 i2_4_lut_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[1]), .C(n4_adj_3179), 
         .D(spi_byte_count[3]), .Z(n1)) /* synthesis lut_function=(!(A (((D)+!C)+!B)+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i2_4_lut_4_lut.init = 16'h1080;
    LUT4 i7_4_lut_adj_334 (.A(ev_run_hold_s5[75]), .B(us_tx_c_75), .C(init_shadow[75]), 
         .D(swap_now_s5), .Z(n11555)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_334.init = 16'h5a66;
    LUT4 i14635_4_lut (.A(n23283), .B(n23397), .C(n23367), .D(n23281), 
         .Z(n23427)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14635_4_lut.init = 16'h8000;
    LUT4 mux_1128_i8_3_lut_4_lut (.A(n23889), .B(ev_state[0]), .C(ev_wr_addr_8__N_912[7]), 
         .D(ev_clear_addr[7]), .Z(ev_wr_addr[7])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mux_1128_i8_3_lut_4_lut.init = 16'hf870;
    LUT4 i1508_3_lut_4_lut (.A(ev_ch[4]), .B(n23871), .C(ev_ch[5]), .D(ev_ch[6]), 
         .Z(ev_ch_6__N_1948[6])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(518[27:39])
    defparam i1508_3_lut_4_lut.init = 16'h7f80;
    LUT4 i7_4_lut_adj_335 (.A(ev_run_hold_s5[76]), .B(us_tx_c_76), .C(init_shadow[76]), 
         .D(swap_now_s5), .Z(n11557)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_335.init = 16'h5a66;
    LUT4 i7_4_lut_adj_336 (.A(ev_run_hold_s5[77]), .B(us_tx_c_77), .C(init_shadow[77]), 
         .D(swap_now_s5), .Z(n11559)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_336.init = 16'h5a66;
    LUT4 i7_4_lut_adj_337 (.A(ev_run_hold_s5[78]), .B(us_tx_c_78), .C(init_shadow[78]), 
         .D(swap_now_s5), .Z(n11561)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_337.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_338 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[48]), 
         .D(ev_bit[48]), .Z(ev_wr_data[48])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_338.init = 16'h7770;
    LUT4 i7_4_lut_adj_339 (.A(ev_run_hold_s5[79]), .B(us_tx_c_79), .C(init_shadow[79]), 
         .D(swap_now_s5), .Z(n11563)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_339.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_340 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[49]), 
         .D(ev_bit[49]), .Z(ev_wr_data[49])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_340.init = 16'h7770;
    LUT4 i7_4_lut_adj_341 (.A(ev_run_hold_s5[80]), .B(us_tx_c_80), .C(init_shadow[80]), 
         .D(swap_now_s5), .Z(n11565)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_341.init = 16'h5a66;
    FD1P3IX ev_bit_i6 (.D(ev_bit[5]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i6.GSR = "DISABLED";
    FD1P3IX ev_bit_i21 (.D(ev_bit[20]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i21.GSR = "DISABLED";
    FD1P3IX ev_bit_i5 (.D(ev_bit[4]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i5.GSR = "DISABLED";
    FD1P3IX ev_bit_i20 (.D(ev_bit[19]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i20.GSR = "DISABLED";
    FD1P3IX ev_bit_i4 (.D(ev_bit[3]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i4.GSR = "DISABLED";
    FD1P3IX ev_bit_i19 (.D(ev_bit[18]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i19.GSR = "DISABLED";
    FD1P3IX ev_bit_i3 (.D(ev_bit[2]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i3.GSR = "DISABLED";
    FD1P3IX ev_bit_i18 (.D(ev_bit[17]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i18.GSR = "DISABLED";
    FD1P3IX ev_bit_i2 (.D(ev_bit[1]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i2.GSR = "DISABLED";
    FD1P3IX ev_bit_i17 (.D(ev_bit[16]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i17.GSR = "DISABLED";
    FD1P3IX ev_bit_i1 (.D(ev_bit[0]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i1.GSR = "DISABLED";
    FD1P3IX ev_bit_i16 (.D(ev_bit[15]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i16.GSR = "DISABLED";
    LUT4 mux_1407_i14_3_lut (.A(n10183), .B(n10184), .C(n10156), .Z(rd_data_15__N_2649[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i14_3_lut.init = 16'hcaca;
    FD1P3IX ev_bit_i54 (.D(ev_bit[53]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i54.GSR = "DISABLED";
    FD1P3IX ev_bit_i15 (.D(ev_bit[14]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i15.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_adj_342 (.A(spi_channel_field[1]), .B(spi1_sck_c_enable_237), 
         .C(spi_channel_field[0]), .Z(spi1_sck_c_enable_236)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_342.init = 16'h4040;
    LUT4 i14609_4_lut (.A(spi_bitmap[79]), .B(n23371), .C(n23289), .D(spi_bitmap[4]), 
         .Z(n23401)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14609_4_lut.init = 16'h8000;
    LUT4 i4355_4_lut (.A(ev_ch[5]), .B(staging_rd_addr[5]), .C(n9_adj_3199), 
         .D(n23094), .Z(staging_rd_addr_6__N_901[5])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i4355_4_lut.init = 16'hac0c;
    LUT4 i1_3_lut_4_lut_adj_343 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[50]), 
         .D(ev_bit[50]), .Z(ev_wr_data[50])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_343.init = 16'h7770;
    LUT4 mux_1407_i15_3_lut (.A(n10185), .B(n10186), .C(n10156), .Z(rd_data_15__N_2649[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i15_3_lut.init = 16'hcaca;
    LUT4 mux_1407_i16_3_lut (.A(n10187), .B(n10188), .C(n10156), .Z(rd_data_15__N_2649[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i16_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_4_lut_4_lut (.A(ev_state[3]), .B(ev_state[0]), .C(ev_state[1]), 
         .D(ev_state[2]), .Z(pll_clk_enable_314)) /* synthesis lut_function=(!(A (B+(C+(D)))+!A ((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_4_lut_4_lut_4_lut.init = 16'h0402;
    INV i15041 (.A(spi1_sck_c), .Z(spi1_sck_N_416));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    LUT4 i7_4_lut_adj_344 (.A(ev_run_hold_s5[81]), .B(us_tx_c_81), .C(init_shadow[81]), 
         .D(swap_now_s5), .Z(n11567)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_344.init = 16'h5a66;
    LUT4 i14583_4_lut (.A(spi_bitmap[58]), .B(spi_bitmap[69]), .C(spi_bitmap[63]), 
         .D(spi_bitmap[78]), .Z(n23375)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14583_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_345 (.A(ev_run_hold_s5[82]), .B(us_tx_c_82), .C(init_shadow[82]), 
         .D(swap_now_s5), .Z(n11569)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_345.init = 16'h5a66;
    LUT4 i14567_4_lut (.A(spi_bitmap[56]), .B(spi_bitmap[67]), .C(spi_bitmap[60]), 
         .D(spi_bitmap[6]), .Z(n23359)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14567_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_346 (.A(ev_run_hold_s5[83]), .B(us_tx_c_83), .C(init_shadow[83]), 
         .D(swap_now_s5), .Z(n11571)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_346.init = 16'h5a66;
    LUT4 i4_4_lut_adj_347 (.A(ev_state[2]), .B(ev_state[1]), .C(pll_clk_enable_3), 
         .D(n6_adj_3192), .Z(pll_clk_enable_728)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam i4_4_lut_adj_347.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_348 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[75]), 
         .D(ev_bit[75]), .Z(ev_wr_data[75])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_348.init = 16'h7770;
    LUT4 i1_3_lut_adj_349 (.A(n14467), .B(init_shadow[58]), .C(ev_bit[58]), 
         .Z(n13063)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_349.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_350 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[51]), 
         .D(ev_bit[51]), .Z(ev_wr_data[51])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_350.init = 16'h7770;
    LUT4 spi_byte_count_7__bdd_4_lut (.A(spi_byte_count[7]), .B(n12), .C(n100), 
         .D(spi_byte_count[6]), .Z(n23854)) /* synthesis lut_function=(A (C+!(D))+!A (B (C+!(D))+!B (C (D)))) */ ;
    defparam spi_byte_count_7__bdd_4_lut.init = 16'hf0ee;
    LUT4 i7_4_lut_adj_351 (.A(staging_q[0]), .B(n14_adj_3201), .C(n10_adj_3202), 
         .D(staging_q[6]), .Z(n1_adj_3205)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i7_4_lut_adj_351.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_352 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[76]), 
         .D(ev_bit[76]), .Z(ev_wr_data[76])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_352.init = 16'h7770;
    LUT4 i1_4_lut_4_lut_then_4_lut (.A(n1_adj_3205), .B(ev_state[0]), .C(ev_state[2]), 
         .D(ev_state[1]), .Z(n23929)) /* synthesis lut_function=(!(A (B+(C+(D)))+!A (B+(C)))) */ ;
    defparam i1_4_lut_4_lut_then_4_lut.init = 16'h0103;
    LUT4 i1_3_lut_4_lut_adj_353 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[40]), 
         .D(ev_bit[40]), .Z(ev_wr_data[40])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_353.init = 16'h7770;
    LUT4 i9670_4_lut (.A(n13582), .B(fpga_cs_n_c), .C(n63), .D(status_bit_index[6]), 
         .Z(spi1_miso_c)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(278[24:81])
    defparam i9670_4_lut.init = 16'h3022;
    LUT4 i6_4_lut_adj_354 (.A(staging_q[3]), .B(staging_q[1]), .C(staging_q[5]), 
         .D(staging_q[7]), .Z(n14_adj_3201)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i6_4_lut_adj_354.init = 16'hfffe;
    LUT4 i1_4_lut_4_lut_else_4_lut (.A(ev_state[0]), .B(ev_state[2]), .C(n23909), 
         .D(ev_state[1]), .Z(n23928)) /* synthesis lut_function=(!(A+!(B+(C+(D))))) */ ;
    defparam i1_4_lut_4_lut_else_4_lut.init = 16'h5554;
    LUT4 i10069_2_lut (.A(n23452), .B(status_bit_index[3]), .Z(n13582)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(278[55:80])
    defparam i10069_2_lut.init = 16'h2222;
    LUT4 i2_2_lut_adj_355 (.A(staging_q[2]), .B(staging_q[4]), .Z(n10_adj_3202)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i2_2_lut_adj_355.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_356 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[41]), 
         .D(ev_bit[41]), .Z(ev_wr_data[41])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_356.init = 16'h7770;
    LUT4 mux_673_Mux_17_i3_3_lut (.A(rgb_hold[1]), .B(shift_register[16]), 
         .C(state[1]), .Z(shift_register_23__N_2910[17])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_673_Mux_17_i3_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_adj_357 (.A(n23062), .B(spi1_sck_c_enable_38), .C(n23413), 
         .D(spi_command[4]), .Z(spi1_sck_c_enable_35)) /* synthesis lut_function=(A (B (C+(D)))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i1_4_lut_adj_357.init = 16'hccc4;
    LUT4 i14631_4_lut (.A(n23225), .B(n23389), .C(n23265), .D(n23223), 
         .Z(n23423)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14631_4_lut.init = 16'h8000;
    FD1P3IX init_shadow_i48 (.D(n13003), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i48.GSR = "DISABLED";
    FD1P3IX init_shadow_i47 (.D(n12997), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i47.GSR = "DISABLED";
    FD1P3IX init_shadow_i46 (.D(n12987), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i46.GSR = "DISABLED";
    FD1P3IX init_shadow_i45 (.D(n12981), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i45.GSR = "DISABLED";
    FD1P3IX init_shadow_i44 (.D(n12975), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i44.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_358 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[42]), 
         .D(ev_bit[42]), .Z(ev_wr_data[42])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_358.init = 16'h7770;
    FD1P3IX init_shadow_i43 (.D(n12969), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i43.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_359 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[77]), 
         .D(ev_bit[77]), .Z(ev_wr_data[77])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_359.init = 16'h7770;
    FD1P3IX init_shadow_i54 (.D(n13039), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i54.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_360 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[83]), 
         .D(ev_bit[83]), .Z(ev_wr_data[83])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_360.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_361 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[43]), 
         .D(ev_bit[43]), .Z(ev_wr_data[43])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_361.init = 16'h7770;
    LUT4 i1_3_lut_4_lut_adj_362 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[53]), 
         .D(ev_bit[53]), .Z(ev_wr_data[53])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_362.init = 16'h7770;
    LUT4 i14649_4_lut (.A(spi_version[4]), .B(n23435), .C(n14_adj_3182), 
         .D(spi_version[1]), .Z(n23441)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14649_4_lut.init = 16'hfffe;
    LUT4 n20236_bdd_4_lut_then_4_lut (.A(ev_state_3__N_1940[1]), .B(ev_state[1]), 
         .C(ev_state[2]), .D(ev_state[3]), .Z(n23932)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C))+!A (C+!(D)))) */ ;
    defparam n20236_bdd_4_lut_then_4_lut.init = 16'h0f02;
    LUT4 mux_1128_i1_3_lut_4_lut (.A(n23889), .B(ev_state[0]), .C(ev_wr_addr_8__N_912[0]), 
         .D(ev_clear_addr[0]), .Z(ev_wr_addr[0])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam mux_1128_i1_3_lut_4_lut.init = 16'hf870;
    LUT4 i14601_4_lut (.A(spi_bitmap[45]), .B(n23339), .C(n23235), .D(spi_bitmap[5]), 
         .Z(n23393)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14601_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_363 (.A(n23889), .B(ev_state[0]), .C(ev_rd_hold[82]), 
         .D(ev_bit[82]), .Z(ev_wr_data[82])) /* synthesis lut_function=(!(A (B+!(C+(D)))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_3_lut_4_lut_adj_363.init = 16'h7770;
    LUT4 mux_1127_i2_3_lut_4_lut (.A(ev_state[1]), .B(n23882), .C(build_sum[1]), 
         .D(build_phase[1]), .Z(ev_wr_addr_8__N_912[1])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1127_i2_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i14643_4_lut (.A(spi_version[3]), .B(n23417), .C(n23146), .D(spi_extension_length[5]), 
         .Z(n23435)) /* synthesis lut_function=(A+(B+(C (D)))) */ ;
    defparam i14643_4_lut.init = 16'hfeee;
    LUT4 i14817_2_lut_3_lut (.A(spi_byte_count[0]), .B(n1), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_95)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i14817_2_lut_3_lut.init = 16'h4040;
    LUT4 mux_1127_i1_3_lut_4_lut (.A(ev_state[1]), .B(n23882), .C(build_sum[0]), 
         .D(build_phase[0]), .Z(ev_wr_addr_8__N_912[0])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1127_i1_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i14625_4_lut (.A(spi_version[6]), .B(n22579), .C(spi_version[7]), 
         .D(n23335), .Z(n23417)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14625_4_lut.init = 16'hfffe;
    LUT4 mux_1127_i3_3_lut_4_lut (.A(ev_state[1]), .B(n23882), .C(build_sum[2]), 
         .D(build_phase[2]), .Z(ev_wr_addr_8__N_912[2])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1127_i3_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i14543_3_lut (.A(expected_next_15__N_1417[7]), .B(n23437), .C(n174), 
         .Z(n23335)) /* synthesis lut_function=(A ((C)+!B)) */ ;
    defparam i14543_3_lut.init = 16'ha2a2;
    LUT4 n20236_bdd_4_lut_else_4_lut (.A(n23909), .B(ev_state[1]), .C(ev_state[2]), 
         .D(ev_state[3]), .Z(n23931)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam n20236_bdd_4_lut_else_4_lut.init = 16'h0002;
    LUT4 i9_4_lut_adj_364 (.A(spi_rx_shift[4]), .B(n18_adj_3194), .C(spi_command[0]), 
         .D(spi_extension_length[6]), .Z(n20_adj_3193)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i9_4_lut_adj_364.init = 16'hfffe;
    LUT4 mux_1127_i4_3_lut_4_lut (.A(ev_state[1]), .B(n23882), .C(build_sum[3]), 
         .D(build_phase[3]), .Z(ev_wr_addr_8__N_912[3])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1127_i4_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_1127_i5_3_lut_4_lut (.A(ev_state[1]), .B(n23882), .C(build_sum[4]), 
         .D(build_phase[4]), .Z(ev_wr_addr_8__N_912[4])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1127_i5_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i5_2_lut (.A(spi_rx_shift[6]), .B(spi_rx_shift[0]), .Z(n16_adj_3195)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i5_2_lut.init = 16'heeee;
    LUT4 i3_4_lut_adj_365 (.A(spi_extension_length[4]), .B(spi_extension_length[3]), 
         .C(spi_extension_length[2]), .D(n23145), .Z(n23146)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i3_4_lut_adj_365.init = 16'hfffe;
    LUT4 mux_1127_i6_3_lut_4_lut (.A(ev_state[1]), .B(n23882), .C(build_sum[5]), 
         .D(build_phase[5]), .Z(ev_wr_addr_8__N_912[5])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1127_i6_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_1127_i7_3_lut_4_lut (.A(ev_state[1]), .B(n23882), .C(build_sum[6]), 
         .D(build_phase[6]), .Z(ev_wr_addr_8__N_912[6])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1127_i7_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i14565_4_lut (.A(spi_bitmap[12]), .B(spi_bitmap[24]), .C(spi_bitmap[13]), 
         .D(spi_bitmap[27]), .Z(n23357)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14565_4_lut.init = 16'h8000;
    LUT4 i9738_3_lut (.A(ev_ch[0]), .B(ev_state[3]), .C(ev_state[0]), 
         .Z(ev_ch_6__N_709[0])) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i9738_3_lut.init = 16'h4848;
    LUT4 mux_1407_i6_3_lut (.A(n10167), .B(n10168), .C(n10156), .Z(rd_data_15__N_2649[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i6_3_lut.init = 16'hcaca;
    LUT4 i14435_2_lut (.A(spi_bitmap[80]), .B(spi_bitmap[18]), .Z(n23225)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14435_2_lut.init = 16'h8888;
    LUT4 i10_4_lut_adj_366 (.A(spi_rx_shift[5]), .B(n20_adj_3193), .C(n16_adj_3195), 
         .D(spi_rx_shift[1]), .Z(n22579)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i10_4_lut_adj_366.init = 16'hfffe;
    LUT4 i14597_4_lut (.A(spi_bitmap[57]), .B(n23261), .C(n23209), .D(spi_bitmap[77]), 
         .Z(n23389)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14597_4_lut.init = 16'h8000;
    LUT4 mux_1407_i3_3_lut (.A(n10161), .B(n10162), .C(n10156), .Z(rd_data_15__N_2649[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i3_3_lut.init = 16'hcaca;
    VLO i1 (.Z(GND_net));
    TSALL TSALL_INST (.TSALL(GND_net));
    PUR PUR_INST (.PUR(VCC_net));
    defparam PUR_INST.RST_PULSE = 1;
    LUT4 mux_1127_i8_3_lut_4_lut (.A(ev_state[1]), .B(n23882), .C(build_sum[7]), 
         .D(build_phase[7]), .Z(ev_wr_addr_8__N_912[7])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1127_i8_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i14647_4_lut (.A(n23377), .B(n23427), .C(n23401), .D(n23375), 
         .Z(n23439)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14647_4_lut.init = 16'h8000;
    LUT4 i14475_4_lut (.A(spi_bitmap[34]), .B(spi_bitmap[50]), .C(spi_bitmap[35]), 
         .D(spi_bitmap[52]), .Z(n23265)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14475_4_lut.init = 16'h8000;
    CCU2D spi_channel_index_1253_add_4_3 (.A0(spi_channel_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22392), .COUT(n22393), .S0(n39_adj_3173), 
          .S1(n38_adj_3174));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253_add_4_3.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1253_add_4_3.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1253_add_4_3.INJECT1_0 = "NO";
    defparam spi_channel_index_1253_add_4_3.INJECT1_1 = "NO";
    LUT4 i3_4_lut_rep_133 (.A(spi_byte_count[4]), .B(n23070), .C(spi_byte_count[5]), 
         .D(n23857), .Z(spi1_sck_c_enable_239)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i3_4_lut_rep_133.init = 16'h0800;
    CCU2D mic_divider_1260_add_4_7 (.A0(mic_divider[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22426), .S0(n35_adj_3190), .S1(n34_adj_3191));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_divider_1260_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_divider_1260_add_4_7.INJECT1_0 = "NO";
    defparam mic_divider_1260_add_4_7.INJECT1_1 = "NO";
    LUT4 mux_1407_i2_3_lut (.A(n10159), .B(n10160), .C(n10156), .Z(rd_data_15__N_2649[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1407_i2_3_lut.init = 16'hcaca;
    FD1P3IX init_shadow_i53 (.D(n13033), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i53.GSR = "DISABLED";
    LUT4 i1_2_lut_4_lut_rep_135 (.A(n23920), .B(n22435), .C(n23909), .D(n23910), 
         .Z(pll_clk_enable_744)) /* synthesis lut_function=(A (B (D)+!B (C+(D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i1_2_lut_4_lut_rep_135.init = 16'hff20;
    FD1P3IX ev_bit_i14 (.D(ev_bit[13]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i14.GSR = "DISABLED";
    FD1P3IX init_shadow_i34 (.D(n12915), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i34.GSR = "DISABLED";
    LUT4 i2_3_lut_4_lut_rep_137 (.A(status_flags_wire_15__N_1401[4]), .B(pll_locked), 
         .C(pll_clk_enable_3), .D(phase_step_s5), .Z(pll_clk_enable_181)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_3_lut_4_lut_rep_137.init = 16'hfff7;
    FD1P3IX init_shadow_i33 (.D(n12909), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i33.GSR = "DISABLED";
    LUT4 i1_4_lut_4_lut_4_lut_rep_139 (.A(ev_state[3]), .B(ev_state[0]), 
         .C(ev_state[1]), .D(ev_state[2]), .Z(pll_clk_enable_280)) /* synthesis lut_function=(!(A (B+(C+(D)))+!A ((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_4_lut_4_lut_4_lut_rep_139.init = 16'h0402;
    FD1P3IX init_shadow_i32 (.D(n12903), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i32.GSR = "DISABLED";
    FD1S3AX phase_step_s4_493_rep_145 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(pll_clk_enable_574)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_step_s4_493_rep_145.GSR = "DISABLED";
    LUT4 i6_4_lut_rep_141 (.A(mic_sample_count[3]), .B(n12_adj_3184), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_437)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_rep_141.init = 16'h0800;
    FD1P3IX init_shadow_i31 (.D(n12897), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i31.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_4_lut_adj_367 (.A(spi_byte_count[8]), .B(n23914), 
         .C(spi_byte_count[10]), .D(spi_byte_count[13]), .Z(n6_adj_3200)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_367.init = 16'hfffe;
    FD1P3IX init_shadow_i42 (.D(n12963), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i42.GSR = "DISABLED";
    LUT4 i1_4_lut_rep_143 (.A(ev_state[3]), .B(ev_state[0]), .C(ev_state[2]), 
         .D(n4), .Z(pll_clk_enable_743)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_4_lut_rep_143.init = 16'h5150;
    LUT4 run_addr_s3_8__I_0_i1_3_lut (.A(run_addr_s3[0]), .B(ev_rd_slot[0]), 
         .C(n22496), .Z(event_rd_addr[0])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i1_3_lut.init = 16'hacac;
    FD1P3IX init_shadow_i41 (.D(n12957), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i41.GSR = "DISABLED";
    LUT4 run_addr_s3_8__I_0_i7_3_lut (.A(run_addr_s3[6]), .B(ev_rd_slot[6]), 
         .C(n22496), .Z(event_rd_addr[6])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i7_3_lut.init = 16'hacac;
    PFUMX i14714 (.BLUT(n23498), .ALUT(n23499), .C0(spi1_miso_N_2513[1]), 
          .Z(n23507));
    LUT4 i6705_2_lut_4_lut_rep_147 (.A(n23920), .B(n22435), .C(n23909), 
         .D(n23910), .Z(n24237)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(477[9] 550[16])
    defparam i6705_2_lut_4_lut_rep_147.init = 16'h0020;
    FD1S3AX phase_step_s4_493_rep_131 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(n24221)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_step_s4_493_rep_131.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1256__i2 (.D(n43), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256__i2.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1256__i3 (.D(n42), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256__i3.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1256__i4 (.D(n41), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256__i4.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1256__i5 (.D(n40_adj_3171), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256__i5.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1256__i6 (.D(n39_adj_3170), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256__i6.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1256__i7 (.D(n38_adj_3169), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256__i7.GSR = "DISABLED";
    FD1S3IX mic_divider_1260__i1 (.D(n39_adj_3186), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260__i1.GSR = "DISABLED";
    LUT4 mux_673_Mux_18_i3_3_lut (.A(rgb_hold[2]), .B(shift_register[17]), 
         .C(state[1]), .Z(shift_register_23__N_2910[18])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_673_Mux_18_i3_3_lut.init = 16'hcaca;
    FD1P3IX init_shadow_i30 (.D(n12891), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i30.GSR = "DISABLED";
    FD1P3IX init_shadow_i52 (.D(n13027), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i52.GSR = "DISABLED";
    FD1P3IX init_shadow_i51 (.D(n13021), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i51.GSR = "DISABLED";
    FD1P3IX ev_bit_i13 (.D(ev_bit[12]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i13.GSR = "DISABLED";
    FD1P3IX ev_bit_i12 (.D(ev_bit[11]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i12.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_4_lut_rep_149 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(pll_locked), .D(status_flags_wire_15__N_1401[4]), .Z(n24239)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam i1_2_lut_3_lut_4_lut_rep_149.init = 16'h6fff;
    LUT4 i6814_2_lut_4_lut_rep_151 (.A(ev_state[3]), .B(ev_state[0]), .C(ev_state[2]), 
         .D(n4), .Z(n24241)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i6814_2_lut_4_lut_rep_151.init = 16'h0100;
    LUT4 i14433_2_lut (.A(spi_bitmap[48]), .B(spi_bitmap[28]), .Z(n23223)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14433_2_lut.init = 16'h8888;
    LUT4 i14471_4_lut (.A(spi_bitmap[23]), .B(spi_bitmap[2]), .C(spi_bitmap[1]), 
         .D(spi_bitmap[11]), .Z(n23261)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14471_4_lut.init = 16'h8000;
    LUT4 i14419_2_lut (.A(spi_bitmap[3]), .B(spi_bitmap[8]), .Z(n23209)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14419_2_lut.init = 16'h8888;
    FD1P3IX ev_clear_addr_i0 (.D(ev_clear_addr_7__N_2237[0]), .SP(pll_clk_enable_726), 
            .CD(n15703), .CK(pll_clk), .Q(ev_clear_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_clear_addr_i0.GSR = "DISABLED";
    FD1P3IX init_shadow_i0 (.D(n11384), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i0.GSR = "DISABLED";
    FD1P3AX ev_state_i0 (.D(ev_state_3__N_697[0]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(ev_state[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_state_i0.GSR = "DISABLED";
    FD1P3AX ws2812_toggle_spi_480 (.D(ws2812_toggle_spi_N_2554), .SP(spi1_sck_c_enable_238), 
            .CK(spi1_sck_c), .Q(ws2812_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam ws2812_toggle_spi_480.GSR = "DISABLED";
    FD1S3JX swap_pending_489 (.D(swap_pending_N_2586), .CK(pll_clk), .PD(n23035), 
            .Q(swap_pending)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam swap_pending_489.GSR = "DISABLED";
    PFUMX i14920 (.BLUT(n23931), .ALUT(n23932), .C0(ev_state[0]), .Z(pll_clk_enable_474));
    FD1P3IX rgb_hold__i1 (.D(rgb_values[0]), .SP(pll_clk_enable_730), .CD(n20277), 
            .CK(pll_clk), .Q(rgb_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam rgb_hold__i1.GSR = "DISABLED";
    FD1P3AX spi_bitmap_i0_i22 (.D(spi_bitmap[14]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam spi_bitmap_i0_i22.GSR = "ENABLED";
    FD1S3IX mic_divider_1260__i2 (.D(n38_adj_3187), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260__i2.GSR = "DISABLED";
    FD1S3IX mic_divider_1260__i3 (.D(n37_adj_3188), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260__i3.GSR = "DISABLED";
    FD1S3IX mic_divider_1260__i4 (.D(n36_adj_3189), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260__i4.GSR = "DISABLED";
    FD1S3IX mic_divider_1260__i5 (.D(n35_adj_3190), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260__i5.GSR = "DISABLED";
    FD1S3IX mic_divider_1260__i6 (.D(n34_adj_3191), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260__i6.GSR = "DISABLED";
    FD1S3AX spi_bit_count_1255__i1 (.D(n19), .CK(spi1_sck_c), .Q(spi_bit_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(360[34:54])
    defparam spi_bit_count_1255__i1.GSR = "ENABLED";
    FD1S3AX spi_bit_count_1255__i2 (.D(n18_adj_3183), .CK(spi1_sck_c), .Q(spi_bit_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(360[34:54])
    defparam spi_bit_count_1255__i2.GSR = "ENABLED";
    FD1P3AX mic_sample_count_1259__i1 (.D(n29), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_sample_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[37:60])
    defparam mic_sample_count_1259__i1.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1259__i2 (.D(n28), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_sample_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[37:60])
    defparam mic_sample_count_1259__i2.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1259__i3 (.D(n27), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_sample_count[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[37:60])
    defparam mic_sample_count_1259__i3.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1259__i4 (.D(n26), .SP(pll_clk_enable_735), 
            .CK(pll_clk), .Q(mic_sample_count[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[37:60])
    defparam mic_sample_count_1259__i4.GSR = "DISABLED";
    LUT4 mux_673_Mux_19_i3_3_lut (.A(rgb_hold[3]), .B(shift_register[18]), 
         .C(state[1]), .Z(shift_register_23__N_2910[19])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_673_Mux_19_i3_3_lut.init = 16'hcaca;
    LUT4 mux_673_Mux_20_i3_3_lut (.A(rgb_hold[4]), .B(shift_register[19]), 
         .C(state[1]), .Z(shift_register_23__N_2910[20])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_673_Mux_20_i3_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_rep_75 (.A(ev_state[3]), .B(ev_state[0]), .C(ev_state[2]), 
         .D(n4), .Z(pll_clk_enable_739)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i1_4_lut_rep_75.init = 16'h5150;
    FD1S3AX phase_step_s4_493_rep_130 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(pll_clk_enable_608)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam phase_step_s4_493_rep_130.GSR = "DISABLED";
    umh_toggle_ram84 event_ram (.pll_clk(pll_clk), .ev_we(ev_we), .VCC_net(VCC_net), 
            .GND_net(GND_net), .\ev_wr_addr[0] (ev_wr_addr[0]), .event_rd_addr({event_rd_addr}), 
            .\ev_wr_addr[1] (ev_wr_addr[1]), .\ev_wr_addr[2] (ev_wr_addr[2]), 
            .\ev_wr_addr[3] (ev_wr_addr[3]), .\ev_wr_addr[4] (ev_wr_addr[4]), 
            .\ev_wr_addr[5] (ev_wr_addr[5]), .\ev_wr_addr[6] (ev_wr_addr[6]), 
            .\ev_wr_addr[7] (ev_wr_addr[7]), .n23921(n23921), .ev_wr_data({ev_wr_data}), 
            .ev_rd_data({ev_rd_data})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[22] 234[6])
    CCU2D mic_divider_1260_add_4_5 (.A0(mic_divider[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22425), .COUT(n22426), .S0(n37_adj_3188), 
          .S1(n36_adj_3189));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_divider_1260_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_divider_1260_add_4_5.INJECT1_0 = "NO";
    defparam mic_divider_1260_add_4_5.INJECT1_1 = "NO";
    CCU2D equal_1859_11 (.A0(spi_expected_length[15]), .B0(spi_byte_count_15__N_1694[15]), 
          .C0(spi_expected_length[14]), .D0(spi_byte_count_15__N_1694[14]), 
          .A1(spi_expected_length[13]), .B1(spi_byte_count_15__N_1694[13]), 
          .C1(spi_expected_length[12]), .D1(spi_byte_count_15__N_1694[12]), 
          .CIN(n22264), .COUT(n22265));
    defparam equal_1859_11.INIT0 = 16'h9009;
    defparam equal_1859_11.INIT1 = 16'h9009;
    defparam equal_1859_11.INJECT1_0 = "YES";
    defparam equal_1859_11.INJECT1_1 = "YES";
    CCU2D mic_divider_1260_add_4_3 (.A0(mic_divider[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22424), .COUT(n22425), .S0(n39_adj_3186), 
          .S1(n38_adj_3187));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_divider_1260_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_divider_1260_add_4_3.INJECT1_0 = "NO";
    defparam mic_divider_1260_add_4_3.INJECT1_1 = "NO";
    LUT4 i14798_2_lut_3_lut (.A(spi_byte_count[0]), .B(n1), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_80)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i14798_2_lut_3_lut.init = 16'h0404;
    LUT4 i2_2_lut_4_lut_adj_368 (.A(n23909), .B(ev_state_3__N_1940[1]), 
         .C(ev_state[0]), .D(n23889), .Z(n20303)) /* synthesis lut_function=(A (B (D)+!B !(C+!(D)))+!A (B (C (D)))) */ ;
    defparam i2_2_lut_4_lut_adj_368.init = 16'hca00;
    LUT4 i1_2_lut_4_lut_adj_369 (.A(n23909), .B(ev_state_3__N_1940[1]), 
         .C(ev_state[0]), .D(ev_state[3]), .Z(n6_adj_3192)) /* synthesis lut_function=(A (B+((D)+!C))+!A (B (C+(D))+!B (D))) */ ;
    defparam i1_2_lut_4_lut_adj_369.init = 16'hffca;
    CCU2D mic_divider_1260_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22424), .S1(n40_adj_3185));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(586[28:46])
    defparam mic_divider_1260_add_4_1.INIT0 = 16'hF000;
    defparam mic_divider_1260_add_4_1.INIT1 = 16'h0555;
    defparam mic_divider_1260_add_4_1.INJECT1_0 = "NO";
    defparam mic_divider_1260_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_76_4_lut (.A(n23899), .B(frame_settle[0]), .C(pll_clk_enable_25), 
         .D(n23896), .Z(pll_clk_enable_22)) /* synthesis lut_function=(A (D)+!A (B ((D)+!C)+!B (D))) */ ;
    defparam i1_2_lut_rep_76_4_lut.init = 16'hff04;
    CCU2D global_phase_s2_1256_add_4_9 (.A0(global_phase_s2[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22423), .S0(n38_adj_3169));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256_add_4_9.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1256_add_4_9.INIT1 = 16'h0000;
    defparam global_phase_s2_1256_add_4_9.INJECT1_0 = "NO";
    defparam global_phase_s2_1256_add_4_9.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i11 (.D(ev_bit[10]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i11.GSR = "DISABLED";
    PFUMX i14918 (.BLUT(n23928), .ALUT(n23929), .C0(ev_state[3]), .Z(ev_state_3__N_697[0]));
    ws2812_stream ws2812_i (.\rgb_hold[7] (rgb_hold[7]), .state({state[1], 
            Open_0}), .pll_clk(pll_clk), .ws2812_enable(ws2812_enable), 
            .\shift_register[17] (shift_register[17]), .\shift_register_23__N_2910[17] (shift_register_23__N_2910[17]), 
            .\shift_register[18] (shift_register[18]), .\shift_register_23__N_2910[18] (shift_register_23__N_2910[18]), 
            .\shift_register[19] (shift_register[19]), .\shift_register_23__N_2910[19] (shift_register_23__N_2910[19]), 
            .\shift_register_23__N_2910[20] (shift_register_23__N_2910[20]), 
            .\shift_register[21] (shift_register[21]), .\shift_register_23__N_2910[22] (shift_register_23__N_2910[22]), 
            .GND_net(GND_net), .\shift_register[16] (shift_register[16]), 
            .\rgb_hold[0] (rgb_hold[0]), .n23860(n23860), .rgb_data_c(rgb_data_c), 
            .\rgb_hold[5] (rgb_hold[5])) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(592[19] 604[6])
    PFUMX i14916 (.BLUT(n23925), .ALUT(n23926), .C0(status_hold[74]), 
          .Z(n23927));
    CCU2D global_phase_s2_1256_add_4_7 (.A0(global_phase_s2[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22422), .COUT(n22423), .S0(n40_adj_3171), 
          .S1(n39_adj_3170));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256_add_4_7.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1256_add_4_7.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1256_add_4_7.INJECT1_0 = "NO";
    defparam global_phase_s2_1256_add_4_7.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i53 (.D(ev_bit[52]), .SP(pll_clk_enable_737), .CD(n15512), 
            .CK(pll_clk), .Q(ev_bit[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i53.GSR = "DISABLED";
    LUT4 i6814_2_lut_4_lut (.A(ev_state[3]), .B(ev_state[0]), .C(ev_state[2]), 
         .D(n4), .Z(n15620)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam i6814_2_lut_4_lut.init = 16'h0100;
    FD1P3IX ev_bit_i10 (.D(ev_bit[9]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i10.GSR = "DISABLED";
    LUT4 i45_4_lut (.A(spi_bitmap[84]), .B(spi_bitmap[87]), .C(spi_bitmap[14]), 
         .D(spi_bitmap[85]), .Z(n133)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;
    defparam i45_4_lut.init = 16'hffef;
    LUT4 i14613_4_lut (.A(spi_bitmap[82]), .B(n23379), .C(n23315), .D(spi_bitmap[7]), 
         .Z(n23405)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14613_4_lut.init = 16'h8000;
    LUT4 i1_2_lut_3_lut_adj_370 (.A(spi_byte_count[0]), .B(n1), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_71)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18] 362[12])
    defparam i1_2_lut_3_lut_adj_370.init = 16'h0808;
    GSR GSR_INST (.GSR(fpga_cs_n_N_2559));
    CCU2D add_160_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(spi_byte_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22350), .S1(spi_byte_count_15__N_1694[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(339[35:57])
    defparam add_160_1.INIT0 = 16'hF000;
    defparam add_160_1.INIT1 = 16'h5555;
    defparam add_160_1.INJECT1_0 = "NO";
    defparam add_160_1.INJECT1_1 = "NO";
    CCU2D add_324_5 (.A0(ev_clear_addr[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22359), .COUT(n22360), .S0(ev_clear_addr_7__N_2237[3]), 
          .S1(ev_clear_addr_7__N_2237[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(490[34:54])
    defparam add_324_5.INIT0 = 16'h5aaa;
    defparam add_324_5.INIT1 = 16'h5aaa;
    defparam add_324_5.INJECT1_0 = "NO";
    defparam add_324_5.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i59 (.D(n13069), .SP(pll_clk_enable_739), .CD(n15620), 
            .CK(pll_clk), .Q(init_shadow[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i59.GSR = "DISABLED";
    LUT4 m1_lut (.Z(n24216)) /* synthesis lut_function=1, syn_instantiated=1 */ ;
    defparam m1_lut.init = 16'hffff;
    FD1P3IX ev_bit_i9 (.D(ev_bit[8]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i9.GSR = "DISABLED";
    FD1P3IX init_shadow_i40 (.D(n12951), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i40.GSR = "DISABLED";
    FD1P3IX init_shadow_i39 (.D(n12945), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i39.GSR = "DISABLED";
    FD1P3IX init_shadow_i38 (.D(n12939), .SP(pll_clk_enable_743), .CD(n24241), 
            .CK(pll_clk), .Q(init_shadow[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam init_shadow_i38.GSR = "DISABLED";
    CCU2D spi_channel_index_1253_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_channel_index[0]), .B1(spi_channel_index[6]), 
          .C1(n8), .D1(n22519), .COUT(n22392), .S1(n40_adj_3172));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(327[64:88])
    defparam spi_channel_index_1253_add_4_1.INIT0 = 16'hF000;
    defparam spi_channel_index_1253_add_4_1.INIT1 = 16'h5559;
    defparam spi_channel_index_1253_add_4_1.INJECT1_0 = "NO";
    defparam spi_channel_index_1253_add_4_1.INJECT1_1 = "NO";
    CCU2D add_646_17 (.A0(phase_frac[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22369), .COUT(n22370), .S0(phase_frac_sum[15]), 
          .S1(phase_frac_sum[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_17.INIT0 = 16'h5aaa;
    defparam add_646_17.INIT1 = 16'h5aaa;
    defparam add_646_17.INJECT1_0 = "NO";
    defparam add_646_17.INJECT1_1 = "NO";
    LUT4 i3_2_lut (.A(spi_bitmap[86]), .B(spi_bitmap[62]), .Z(n91)) /* synthesis lut_function=(A+!(B)) */ ;
    defparam i3_2_lut.init = 16'hbbbb;
    umh_channel_ram18 staging_ram (.n10152(n10152), .spi1_sck_c(spi1_sck_c), 
            .spi_channel_index({spi_channel_index}), .staging_q({staging_q}), 
            .pll_clk(pll_clk), .rd_data_15__N_2649({rd_data_15__N_2649}), 
            .n10142(n10142), .n10140(n10140), .n10150(n10150), .n10148(n10148), 
            .spi_write(spi_write), .VCC_net(VCC_net), .GND_net(GND_net), 
            .staging_rd_addr_6__N_901({staging_rd_addr_6__N_901}), .spi1_mosi_c_0(spi1_mosi_c_0), 
            .\spi_rx_shift[0] (spi_rx_shift[0]), .\spi_rx_shift[1] (spi_rx_shift[1]), 
            .\spi_rx_shift[2] (spi_rx_shift[2]), .\spi_rx_shift[3] (spi_rx_shift[3]), 
            .\spi_rx_shift[4] (spi_rx_shift[4]), .\spi_rx_shift[5] (spi_rx_shift[5]), 
            .\spi_rx_shift[6] (spi_rx_shift[6]), .spi_phase_pending({spi_phase_pending}), 
            .n10157(n10157), .n10159(n10159), .n10161(n10161), .n10163(n10163), 
            .n10165(n10165), .n10167(n10167), .n10169(n10169), .n10171(n10171), 
            .n10173(n10173), .n10175(n10175), .n10177(n10177), .n10179(n10179), 
            .n10181(n10181), .n10183(n10183), .n10185(n10185), .n10187(n10187), 
            .n10144(n10144), .n10146(n10146)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(207[23] 210[6])
    LUT4 i14527_2_lut (.A(spi_bitmap[41]), .B(spi_bitmap[72]), .Z(n23319)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14527_2_lut.init = 16'h8888;
    CCU2D add_324_3 (.A0(ev_clear_addr[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22358), .COUT(n22359), .S0(ev_clear_addr_7__N_2237[1]), 
          .S1(ev_clear_addr_7__N_2237[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(490[34:54])
    defparam add_324_3.INIT0 = 16'h5aaa;
    defparam add_324_3.INIT1 = 16'h5aaa;
    defparam add_324_3.INJECT1_0 = "NO";
    defparam add_324_3.INJECT1_1 = "NO";
    CCU2D add_646_15 (.A0(phase_frac[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22368), .COUT(n22369), .S0(phase_frac_sum[13]), 
          .S1(phase_frac_sum[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_646_15.INIT0 = 16'h5555;
    defparam add_646_15.INIT1 = 16'h5555;
    defparam add_646_15.INJECT1_0 = "NO";
    defparam add_646_15.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_643_15 (.A0(spi_rx_shift[6]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22391), .S0(expected_next[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(286[33] 288[86])
    defparam expected_next_15__I_0_643_15.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_643_15.INIT1 = 16'h0000;
    defparam expected_next_15__I_0_643_15.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_643_15.INJECT1_1 = "NO";
    CCU2D global_phase_s2_1256_add_4_5 (.A0(global_phase_s2[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22421), .COUT(n22422), .S0(n42), 
          .S1(n41));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(376[32:54])
    defparam global_phase_s2_1256_add_4_5.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1256_add_4_5.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1256_add_4_5.INJECT1_0 = "NO";
    defparam global_phase_s2_1256_add_4_5.INJECT1_1 = "NO";
    LUT4 i14587_4_lut (.A(spi_bitmap[81]), .B(spi_bitmap[54]), .C(spi_bitmap[40]), 
         .D(spi_bitmap[59]), .Z(n23379)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14587_4_lut.init = 16'h8000;
    FD1P3IX ev_bit_i46 (.D(ev_bit[45]), .SP(pll_clk_enable_744), .CD(n24237), 
            .CK(pll_clk), .Q(ev_bit[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[12] 588[8])
    defparam ev_bit_i46.GSR = "DISABLED";
    CCU2D add_324_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(ev_clear_addr[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22358), .S1(ev_clear_addr_7__N_2237[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(490[34:54])
    defparam add_324_1.INIT0 = 16'hF000;
    defparam add_324_1.INIT1 = 16'h5555;
    defparam add_324_1.INJECT1_0 = "NO";
    defparam add_324_1.INJECT1_1 = "NO";
    LUT4 i14523_2_lut (.A(spi_bitmap[9]), .B(spi_bitmap[55]), .Z(n23315)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14523_2_lut.init = 16'h8888;
    
endmodule
//
// Verilog Description of module spi_mic_stream
//

module spi_mic_stream (mic_latest, sck_N_3047, spi_mic_cs_n_c, spi_mic_miso_c) /* synthesis syn_module_defined=1 */ ;
    input [63:0]mic_latest;
    input sck_N_3047;
    input spi_mic_cs_n_c;
    output spi_mic_miso_c;
    
    wire sck_N_3047 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    wire [95:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    
    wire n13;
    wire [95:0]shift_register_95__N_2950;
    wire [6:0]bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(12[11:20])
    
    wire n23890, n23876, n15716, n23865;
    wire [6:0]bit_count_6__N_3048;
    
    wire n15714, sck_N_3047_enable_101, n15778, n15776, n15774, n15772, 
        n15770, n15768, n15766, n15764, n15762, n15760, n15758, 
        n15756, n15754, n15752, n15750, n15748, n15746, n15744, 
        n15742, n15740, n15738, n15736, n15734, n15732, n15730, 
        n15728, n15726, n15724, n15722, n15720, n15718, n12, n23861;
    
    LUT4 shift_register_95__I_0_19_i29_3_lut (.A(mic_latest[19]), .B(shift_register[27]), 
         .C(n13), .Z(shift_register_95__N_2950[28])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i29_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i30_3_lut (.A(mic_latest[20]), .B(shift_register[28]), 
         .C(n13), .Z(shift_register_95__N_2950[29])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i30_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i31_3_lut (.A(mic_latest[21]), .B(shift_register[29]), 
         .C(n13), .Z(shift_register_95__N_2950[30])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i31_3_lut.init = 16'hcaca;
    LUT4 i1525_2_lut_rep_98 (.A(bit_count[1]), .B(bit_count[0]), .Z(n23890)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1525_2_lut_rep_98.init = 16'h8888;
    LUT4 i1532_2_lut_rep_84_3_lut (.A(bit_count[1]), .B(bit_count[0]), .C(bit_count[2]), 
         .Z(n23876)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1532_2_lut_rep_84_3_lut.init = 16'h8080;
    LUT4 i9755_3_lut_4_lut (.A(bit_count[1]), .B(bit_count[0]), .C(n13), 
         .D(bit_count[2]), .Z(n15716)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i9755_3_lut_4_lut.init = 16'h7080;
    LUT4 i1539_2_lut_rep_73_3_lut_4_lut (.A(bit_count[1]), .B(bit_count[0]), 
         .C(bit_count[3]), .D(bit_count[2]), .Z(n23865)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1539_2_lut_rep_73_3_lut_4_lut.init = 16'h8000;
    LUT4 shift_register_95__I_0_19_i32_3_lut (.A(mic_latest[22]), .B(shift_register[30]), 
         .C(n13), .Z(shift_register_95__N_2950[31])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i32_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i33_3_lut (.A(mic_latest[23]), .B(shift_register[31]), 
         .C(n13), .Z(shift_register_95__N_2950[32])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i33_3_lut.init = 16'hcaca;
    FD1S3DX bit_count_i0 (.D(bit_count_6__N_3048[0]), .CK(sck_N_3047), .CD(spi_mic_cs_n_c), 
            .Q(bit_count[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i0.GSR = "DISABLED";
    LUT4 i9745_3_lut (.A(bit_count[1]), .B(n13), .C(bit_count[0]), .Z(n15714)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9745_3_lut.init = 16'h4848;
    LUT4 shift_register_95__I_0_19_i34_3_lut (.A(mic_latest[24]), .B(shift_register[32]), 
         .C(n13), .Z(shift_register_95__N_2950[33])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i34_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i35_3_lut (.A(mic_latest[25]), .B(shift_register[33]), 
         .C(n13), .Z(shift_register_95__N_2950[34])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i35_3_lut.init = 16'hcaca;
    LUT4 i14783_3_lut_4_lut (.A(bit_count[5]), .B(bit_count[6]), .C(n13), 
         .D(bit_count[0]), .Z(bit_count_6__N_3048[0])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D))) */ ;
    defparam i14783_3_lut_4_lut.init = 16'h8f7f;
    LUT4 i14786_2_lut_2_lut_3_lut (.A(bit_count[5]), .B(bit_count[6]), .C(n13), 
         .Z(sck_N_3047_enable_101)) /* synthesis lut_function=(!(A (B (C)))) */ ;
    defparam i14786_2_lut_2_lut_3_lut.init = 16'h7f7f;
    LUT4 i9750_2_lut (.A(mic_latest[0]), .B(n13), .Z(shift_register_95__N_2950[1])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i9750_2_lut.init = 16'h2222;
    LUT4 shift_register_95__I_0_19_i3_3_lut (.A(mic_latest[1]), .B(shift_register[1]), 
         .C(n13), .Z(shift_register_95__N_2950[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i3_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i4_3_lut (.A(mic_latest[2]), .B(shift_register[2]), 
         .C(n13), .Z(shift_register_95__N_2950[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i4_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i5_3_lut (.A(mic_latest[3]), .B(shift_register[3]), 
         .C(n13), .Z(shift_register_95__N_2950[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i5_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i6_3_lut (.A(mic_latest[4]), .B(shift_register[4]), 
         .C(n13), .Z(shift_register_95__N_2950[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i6_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i7_3_lut (.A(mic_latest[5]), .B(shift_register[5]), 
         .C(n13), .Z(shift_register_95__N_2950[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i7_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i8_3_lut (.A(mic_latest[6]), .B(shift_register[6]), 
         .C(n13), .Z(shift_register_95__N_2950[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i8_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i9_3_lut (.A(mic_latest[7]), .B(shift_register[7]), 
         .C(n13), .Z(shift_register_95__N_2950[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i9_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i10_3_lut (.A(mic_latest[8]), .B(shift_register[8]), 
         .C(n13), .Z(shift_register_95__N_2950[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i10_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i11_3_lut (.A(mic_latest[9]), .B(shift_register[9]), 
         .C(n13), .Z(shift_register_95__N_2950[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i11_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i12_3_lut (.A(mic_latest[10]), .B(shift_register[10]), 
         .C(n13), .Z(shift_register_95__N_2950[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i12_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i36_3_lut (.A(mic_latest[26]), .B(shift_register[34]), 
         .C(n13), .Z(shift_register_95__N_2950[35])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i36_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i13_3_lut (.A(mic_latest[11]), .B(shift_register[11]), 
         .C(n13), .Z(shift_register_95__N_2950[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i13_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i14_3_lut (.A(mic_latest[12]), .B(shift_register[12]), 
         .C(n13), .Z(shift_register_95__N_2950[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i14_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i15_3_lut (.A(mic_latest[13]), .B(shift_register[13]), 
         .C(n13), .Z(shift_register_95__N_2950[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i15_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i16_3_lut (.A(mic_latest[14]), .B(shift_register[14]), 
         .C(n13), .Z(shift_register_95__N_2950[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i16_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i17_3_lut (.A(mic_latest[15]), .B(shift_register[15]), 
         .C(n13), .Z(shift_register_95__N_2950[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i17_3_lut.init = 16'hcaca;
    LUT4 i9751_2_lut (.A(shift_register[16]), .B(n13), .Z(shift_register_95__N_2950[17])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i9751_2_lut.init = 16'hbbbb;
    LUT4 i9752_2_lut (.A(shift_register[17]), .B(n13), .Z(shift_register_95__N_2950[18])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i9752_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i26_3_lut (.A(mic_latest[16]), .B(shift_register[24]), 
         .C(n13), .Z(shift_register_95__N_2950[25])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i26_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut (.A(n13), .B(shift_register[95]), .C(spi_mic_cs_n_c), 
         .Z(spi_mic_miso_c)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[15] 26[68])
    defparam i1_3_lut.init = 16'h0808;
    LUT4 shift_register_95__I_0_19_i27_3_lut (.A(mic_latest[17]), .B(shift_register[25]), 
         .C(n13), .Z(shift_register_95__N_2950[26])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i27_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i37_3_lut (.A(mic_latest[27]), .B(shift_register[35]), 
         .C(n13), .Z(shift_register_95__N_2950[36])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i37_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i38_3_lut (.A(mic_latest[28]), .B(shift_register[36]), 
         .C(n13), .Z(shift_register_95__N_2950[37])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i38_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i28_3_lut (.A(mic_latest[18]), .B(shift_register[26]), 
         .C(n13), .Z(shift_register_95__N_2950[27])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i28_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i39_3_lut (.A(mic_latest[29]), .B(shift_register[37]), 
         .C(n13), .Z(shift_register_95__N_2950[38])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i39_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i40_3_lut (.A(mic_latest[30]), .B(shift_register[38]), 
         .C(n13), .Z(shift_register_95__N_2950[39])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i40_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i41_3_lut (.A(mic_latest[31]), .B(shift_register[39]), 
         .C(n13), .Z(shift_register_95__N_2950[40])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i41_3_lut.init = 16'hcaca;
    LUT4 i9753_2_lut (.A(shift_register[41]), .B(n13), .Z(shift_register_95__N_2950[42])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i9753_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i50_3_lut (.A(mic_latest[32]), .B(shift_register[48]), 
         .C(n13), .Z(shift_register_95__N_2950[49])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i50_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i1 (.D(shift_register_95__N_2950[1]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i1.GSR = "DISABLED";
    FD1P3DX shift_register_i2 (.D(shift_register_95__N_2950[2]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i2.GSR = "DISABLED";
    FD1P3DX shift_register_i3 (.D(shift_register_95__N_2950[3]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i3.GSR = "DISABLED";
    FD1P3DX shift_register_i4 (.D(shift_register_95__N_2950[4]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i4.GSR = "DISABLED";
    FD1P3DX shift_register_i5 (.D(shift_register_95__N_2950[5]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i5.GSR = "DISABLED";
    FD1P3DX shift_register_i6 (.D(shift_register_95__N_2950[6]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i6.GSR = "DISABLED";
    FD1P3DX shift_register_i7 (.D(shift_register_95__N_2950[7]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i7.GSR = "DISABLED";
    FD1P3DX shift_register_i8 (.D(shift_register_95__N_2950[8]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i8.GSR = "DISABLED";
    FD1P3DX shift_register_i9 (.D(shift_register_95__N_2950[9]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i9.GSR = "DISABLED";
    FD1P3DX shift_register_i10 (.D(shift_register_95__N_2950[10]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i10.GSR = "DISABLED";
    FD1P3DX shift_register_i11 (.D(shift_register_95__N_2950[11]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i11.GSR = "DISABLED";
    FD1P3DX shift_register_i12 (.D(shift_register_95__N_2950[12]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i12.GSR = "DISABLED";
    FD1P3DX shift_register_i13 (.D(shift_register_95__N_2950[13]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i13.GSR = "DISABLED";
    FD1P3DX shift_register_i14 (.D(shift_register_95__N_2950[14]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i14.GSR = "DISABLED";
    FD1P3DX shift_register_i15 (.D(shift_register_95__N_2950[15]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i15.GSR = "DISABLED";
    FD1P3DX shift_register_i16 (.D(shift_register_95__N_2950[16]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[16])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i16.GSR = "DISABLED";
    FD1P3DX shift_register_i17 (.D(shift_register_95__N_2950[17]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[17])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i17.GSR = "DISABLED";
    FD1P3DX shift_register_i18 (.D(shift_register_95__N_2950[18]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[18])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i18.GSR = "DISABLED";
    FD1P3DX shift_register_i25 (.D(shift_register_95__N_2950[25]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[25])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i25.GSR = "DISABLED";
    FD1P3DX shift_register_i26 (.D(shift_register_95__N_2950[26]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[26])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i26.GSR = "DISABLED";
    FD1P3DX shift_register_i27 (.D(shift_register_95__N_2950[27]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[27])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i27.GSR = "DISABLED";
    FD1P3DX shift_register_i28 (.D(shift_register_95__N_2950[28]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[28])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i28.GSR = "DISABLED";
    FD1P3DX shift_register_i29 (.D(shift_register_95__N_2950[29]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[29])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i29.GSR = "DISABLED";
    FD1P3DX shift_register_i30 (.D(shift_register_95__N_2950[30]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[30])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i30.GSR = "DISABLED";
    FD1P3DX shift_register_i31 (.D(shift_register_95__N_2950[31]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[31])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i31.GSR = "DISABLED";
    FD1P3DX shift_register_i32 (.D(shift_register_95__N_2950[32]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[32])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i32.GSR = "DISABLED";
    FD1P3DX shift_register_i33 (.D(shift_register_95__N_2950[33]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[33])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i33.GSR = "DISABLED";
    FD1P3DX shift_register_i34 (.D(shift_register_95__N_2950[34]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[34])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i34.GSR = "DISABLED";
    FD1P3DX shift_register_i35 (.D(shift_register_95__N_2950[35]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[35])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i35.GSR = "DISABLED";
    FD1P3DX shift_register_i36 (.D(shift_register_95__N_2950[36]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[36])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i36.GSR = "DISABLED";
    FD1P3DX shift_register_i37 (.D(shift_register_95__N_2950[37]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[37])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i37.GSR = "DISABLED";
    FD1P3DX shift_register_i38 (.D(shift_register_95__N_2950[38]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[38])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i38.GSR = "DISABLED";
    FD1P3DX shift_register_i39 (.D(shift_register_95__N_2950[39]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[39])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i39.GSR = "DISABLED";
    FD1P3DX shift_register_i40 (.D(shift_register_95__N_2950[40]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[40])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i40.GSR = "DISABLED";
    FD1P3DX shift_register_i42 (.D(shift_register_95__N_2950[42]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[42])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i42.GSR = "DISABLED";
    FD1P3DX shift_register_i49 (.D(shift_register_95__N_2950[49]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[49])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i49.GSR = "DISABLED";
    FD1P3DX shift_register_i50 (.D(shift_register_95__N_2950[50]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[50])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i50.GSR = "DISABLED";
    FD1P3DX shift_register_i51 (.D(shift_register_95__N_2950[51]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[51])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i51.GSR = "DISABLED";
    FD1P3DX shift_register_i52 (.D(shift_register_95__N_2950[52]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[52])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i52.GSR = "DISABLED";
    FD1P3DX shift_register_i53 (.D(shift_register_95__N_2950[53]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[53])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i53.GSR = "DISABLED";
    FD1P3DX shift_register_i54 (.D(shift_register_95__N_2950[54]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[54])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i54.GSR = "DISABLED";
    FD1P3DX shift_register_i55 (.D(shift_register_95__N_2950[55]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[55])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i55.GSR = "DISABLED";
    FD1P3DX shift_register_i56 (.D(shift_register_95__N_2950[56]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[56])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i56.GSR = "DISABLED";
    FD1P3DX shift_register_i57 (.D(shift_register_95__N_2950[57]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[57])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i57.GSR = "DISABLED";
    FD1P3DX shift_register_i58 (.D(shift_register_95__N_2950[58]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[58])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i58.GSR = "DISABLED";
    FD1P3DX shift_register_i59 (.D(shift_register_95__N_2950[59]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[59])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i59.GSR = "DISABLED";
    FD1P3DX shift_register_i60 (.D(shift_register_95__N_2950[60]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[60])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i60.GSR = "DISABLED";
    FD1P3DX shift_register_i61 (.D(shift_register_95__N_2950[61]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[61])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i61.GSR = "DISABLED";
    FD1P3DX shift_register_i62 (.D(shift_register_95__N_2950[62]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[62])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i62.GSR = "DISABLED";
    FD1P3DX shift_register_i63 (.D(shift_register_95__N_2950[63]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[63])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i63.GSR = "DISABLED";
    FD1P3DX shift_register_i64 (.D(shift_register_95__N_2950[64]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[64])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i64.GSR = "DISABLED";
    FD1P3DX shift_register_i65 (.D(shift_register_95__N_2950[65]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[65])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i65.GSR = "DISABLED";
    FD1P3DX shift_register_i73 (.D(shift_register_95__N_2950[73]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[73])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i73.GSR = "DISABLED";
    FD1P3DX shift_register_i74 (.D(shift_register_95__N_2950[74]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[74])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i74.GSR = "DISABLED";
    FD1P3DX shift_register_i75 (.D(shift_register_95__N_2950[75]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[75])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i75.GSR = "DISABLED";
    FD1P3DX shift_register_i76 (.D(shift_register_95__N_2950[76]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[76])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i76.GSR = "DISABLED";
    FD1P3DX shift_register_i77 (.D(shift_register_95__N_2950[77]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[77])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i77.GSR = "DISABLED";
    FD1P3DX shift_register_i78 (.D(shift_register_95__N_2950[78]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[78])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i78.GSR = "DISABLED";
    FD1P3DX shift_register_i79 (.D(shift_register_95__N_2950[79]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[79])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i79.GSR = "DISABLED";
    FD1P3DX shift_register_i80 (.D(shift_register_95__N_2950[80]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[80])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i80.GSR = "DISABLED";
    FD1P3DX shift_register_i81 (.D(shift_register_95__N_2950[81]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[81])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i81.GSR = "DISABLED";
    FD1P3DX shift_register_i82 (.D(shift_register_95__N_2950[82]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[82])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i82.GSR = "DISABLED";
    FD1P3DX shift_register_i83 (.D(shift_register_95__N_2950[83]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[83])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i83.GSR = "DISABLED";
    FD1P3DX shift_register_i84 (.D(shift_register_95__N_2950[84]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[84])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i84.GSR = "DISABLED";
    FD1P3DX shift_register_i85 (.D(shift_register_95__N_2950[85]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[85])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i85.GSR = "DISABLED";
    FD1P3DX shift_register_i86 (.D(shift_register_95__N_2950[86]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[86])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i86.GSR = "DISABLED";
    FD1P3DX shift_register_i87 (.D(shift_register_95__N_2950[87]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[87])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i87.GSR = "DISABLED";
    FD1P3DX shift_register_i88 (.D(shift_register_95__N_2950[88]), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[88])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i88.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i51_3_lut (.A(mic_latest[33]), .B(shift_register[49]), 
         .C(n13), .Z(shift_register_95__N_2950[50])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i51_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i52_3_lut (.A(mic_latest[34]), .B(shift_register[50]), 
         .C(n13), .Z(shift_register_95__N_2950[51])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i52_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i53_3_lut (.A(mic_latest[35]), .B(shift_register[51]), 
         .C(n13), .Z(shift_register_95__N_2950[52])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i53_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i54_3_lut (.A(mic_latest[36]), .B(shift_register[52]), 
         .C(n13), .Z(shift_register_95__N_2950[53])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i54_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i55_3_lut (.A(mic_latest[37]), .B(shift_register[53]), 
         .C(n13), .Z(shift_register_95__N_2950[54])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i55_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i56_3_lut (.A(mic_latest[38]), .B(shift_register[54]), 
         .C(n13), .Z(shift_register_95__N_2950[55])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i56_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i57_3_lut (.A(mic_latest[39]), .B(shift_register[55]), 
         .C(n13), .Z(shift_register_95__N_2950[56])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i57_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i58_3_lut (.A(mic_latest[40]), .B(shift_register[56]), 
         .C(n13), .Z(shift_register_95__N_2950[57])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i58_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i59_3_lut (.A(mic_latest[41]), .B(shift_register[57]), 
         .C(n13), .Z(shift_register_95__N_2950[58])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i59_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i60_3_lut (.A(mic_latest[42]), .B(shift_register[58]), 
         .C(n13), .Z(shift_register_95__N_2950[59])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i60_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i61_3_lut (.A(mic_latest[43]), .B(shift_register[59]), 
         .C(n13), .Z(shift_register_95__N_2950[60])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i61_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i62_3_lut (.A(mic_latest[44]), .B(shift_register[60]), 
         .C(n13), .Z(shift_register_95__N_2950[61])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i62_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i63_3_lut (.A(mic_latest[45]), .B(shift_register[61]), 
         .C(n13), .Z(shift_register_95__N_2950[62])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i63_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i64_3_lut (.A(mic_latest[46]), .B(shift_register[62]), 
         .C(n13), .Z(shift_register_95__N_2950[63])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i64_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i65_3_lut (.A(mic_latest[47]), .B(shift_register[63]), 
         .C(n13), .Z(shift_register_95__N_2950[64])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i65_3_lut.init = 16'hcaca;
    LUT4 i9754_2_lut (.A(shift_register[64]), .B(n13), .Z(shift_register_95__N_2950[65])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i9754_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i74_3_lut (.A(mic_latest[48]), .B(shift_register[72]), 
         .C(n13), .Z(shift_register_95__N_2950[73])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i74_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i75_3_lut (.A(mic_latest[49]), .B(shift_register[73]), 
         .C(n13), .Z(shift_register_95__N_2950[74])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i75_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i76_3_lut (.A(mic_latest[50]), .B(shift_register[74]), 
         .C(n13), .Z(shift_register_95__N_2950[75])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i76_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i77_3_lut (.A(mic_latest[51]), .B(shift_register[75]), 
         .C(n13), .Z(shift_register_95__N_2950[76])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i77_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i78_3_lut (.A(mic_latest[52]), .B(shift_register[76]), 
         .C(n13), .Z(shift_register_95__N_2950[77])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i78_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i79_3_lut (.A(mic_latest[53]), .B(shift_register[77]), 
         .C(n13), .Z(shift_register_95__N_2950[78])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i79_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i80_3_lut (.A(mic_latest[54]), .B(shift_register[78]), 
         .C(n13), .Z(shift_register_95__N_2950[79])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i80_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i81_3_lut (.A(mic_latest[55]), .B(shift_register[79]), 
         .C(n13), .Z(shift_register_95__N_2950[80])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i81_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i82_3_lut (.A(mic_latest[56]), .B(shift_register[80]), 
         .C(n13), .Z(shift_register_95__N_2950[81])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i82_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i83_3_lut (.A(mic_latest[57]), .B(shift_register[81]), 
         .C(n13), .Z(shift_register_95__N_2950[82])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i83_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i84_3_lut (.A(mic_latest[58]), .B(shift_register[82]), 
         .C(n13), .Z(shift_register_95__N_2950[83])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i84_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i85_3_lut (.A(mic_latest[59]), .B(shift_register[83]), 
         .C(n13), .Z(shift_register_95__N_2950[84])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i85_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i86_3_lut (.A(mic_latest[60]), .B(shift_register[84]), 
         .C(n13), .Z(shift_register_95__N_2950[85])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i86_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i87_3_lut (.A(mic_latest[61]), .B(shift_register[85]), 
         .C(n13), .Z(shift_register_95__N_2950[86])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i87_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i88_3_lut (.A(mic_latest[62]), .B(shift_register[86]), 
         .C(n13), .Z(shift_register_95__N_2950[87])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i88_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i89_3_lut (.A(mic_latest[63]), .B(shift_register[87]), 
         .C(n13), .Z(shift_register_95__N_2950[88])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i89_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i95 (.D(n15778), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[95])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i95.GSR = "DISABLED";
    FD1P3DX shift_register_i94 (.D(n15776), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[94])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i94.GSR = "DISABLED";
    FD1P3DX shift_register_i93 (.D(n15774), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[93])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i93.GSR = "DISABLED";
    FD1P3DX shift_register_i92 (.D(n15772), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[92])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i92.GSR = "DISABLED";
    FD1P3DX shift_register_i91 (.D(n15770), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[91])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i91.GSR = "DISABLED";
    FD1P3DX shift_register_i90 (.D(n15768), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[90])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i90.GSR = "DISABLED";
    FD1P3DX shift_register_i89 (.D(n15766), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[89])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i89.GSR = "DISABLED";
    FD1P3DX shift_register_i72 (.D(n15764), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[72])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i72.GSR = "DISABLED";
    FD1P3DX shift_register_i71 (.D(n15762), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[71])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i71.GSR = "DISABLED";
    FD1P3DX shift_register_i70 (.D(n15760), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[70])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i70.GSR = "DISABLED";
    FD1P3DX shift_register_i69 (.D(n15758), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[69])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i69.GSR = "DISABLED";
    FD1P3DX shift_register_i68 (.D(n15756), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[68])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i68.GSR = "DISABLED";
    FD1P3DX shift_register_i67 (.D(n15754), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[67])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i67.GSR = "DISABLED";
    FD1P3DX shift_register_i66 (.D(n15752), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[66])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i66.GSR = "DISABLED";
    FD1P3DX shift_register_i48 (.D(n15750), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[48])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i48.GSR = "DISABLED";
    FD1P3DX shift_register_i47 (.D(n15748), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[47])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i47.GSR = "DISABLED";
    FD1P3DX shift_register_i46 (.D(n15746), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[46])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i46.GSR = "DISABLED";
    FD1P3DX shift_register_i45 (.D(n15744), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[45])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i45.GSR = "DISABLED";
    FD1P3DX shift_register_i44 (.D(n15742), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[44])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i44.GSR = "DISABLED";
    FD1P3DX shift_register_i43 (.D(n15740), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[43])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i43.GSR = "DISABLED";
    FD1P3DX shift_register_i41 (.D(n15738), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[41])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i41.GSR = "DISABLED";
    FD1P3DX shift_register_i24 (.D(n15736), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[24])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i24.GSR = "DISABLED";
    FD1P3DX shift_register_i23 (.D(n15734), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[23])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i23.GSR = "DISABLED";
    FD1P3DX shift_register_i22 (.D(n15732), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[22])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i22.GSR = "DISABLED";
    FD1P3DX shift_register_i21 (.D(n15730), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[21])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i21.GSR = "DISABLED";
    FD1P3DX shift_register_i20 (.D(n15728), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[20])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i20.GSR = "DISABLED";
    FD1P3DX shift_register_i19 (.D(n15726), .SP(sck_N_3047_enable_101), 
            .CK(sck_N_3047), .CD(spi_mic_cs_n_c), .Q(shift_register[19])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i19.GSR = "DISABLED";
    FD1P3DX bit_count_i6 (.D(n15724), .SP(sck_N_3047_enable_101), .CK(sck_N_3047), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i6.GSR = "DISABLED";
    FD1P3DX bit_count_i5 (.D(n15722), .SP(sck_N_3047_enable_101), .CK(sck_N_3047), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i5.GSR = "DISABLED";
    FD1P3DX bit_count_i4 (.D(n15720), .SP(sck_N_3047_enable_101), .CK(sck_N_3047), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i4.GSR = "DISABLED";
    FD1P3DX bit_count_i3 (.D(n15718), .SP(sck_N_3047_enable_101), .CK(sck_N_3047), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i3.GSR = "DISABLED";
    FD1P3DX bit_count_i2 (.D(n15716), .SP(sck_N_3047_enable_101), .CK(sck_N_3047), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i2.GSR = "DISABLED";
    FD1P3DX bit_count_i1 (.D(n15714), .SP(sck_N_3047_enable_101), .CK(sck_N_3047), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=605, LSE_RLINE=608 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i1.GSR = "DISABLED";
    LUT4 i9786_2_lut (.A(shift_register[94]), .B(n13), .Z(n15778)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9786_2_lut.init = 16'h8888;
    LUT4 i9785_2_lut (.A(shift_register[93]), .B(n13), .Z(n15776)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9785_2_lut.init = 16'h8888;
    LUT4 i9784_2_lut (.A(shift_register[92]), .B(n13), .Z(n15774)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9784_2_lut.init = 16'h8888;
    LUT4 i9783_2_lut (.A(shift_register[91]), .B(n13), .Z(n15772)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9783_2_lut.init = 16'h8888;
    LUT4 i9782_2_lut (.A(shift_register[90]), .B(n13), .Z(n15770)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9782_2_lut.init = 16'h8888;
    LUT4 i9781_2_lut (.A(shift_register[89]), .B(n13), .Z(n15768)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9781_2_lut.init = 16'h8888;
    LUT4 i9780_2_lut (.A(shift_register[88]), .B(n13), .Z(n15766)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9780_2_lut.init = 16'h8888;
    LUT4 i9779_2_lut (.A(shift_register[71]), .B(n13), .Z(n15764)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9779_2_lut.init = 16'h8888;
    LUT4 i9778_2_lut (.A(shift_register[70]), .B(n13), .Z(n15762)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9778_2_lut.init = 16'h8888;
    LUT4 i9777_2_lut (.A(shift_register[69]), .B(n13), .Z(n15760)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9777_2_lut.init = 16'h8888;
    LUT4 i9776_2_lut (.A(shift_register[68]), .B(n13), .Z(n15758)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9776_2_lut.init = 16'h8888;
    LUT4 i9775_2_lut (.A(shift_register[67]), .B(n13), .Z(n15756)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9775_2_lut.init = 16'h8888;
    LUT4 i9774_2_lut (.A(shift_register[66]), .B(n13), .Z(n15754)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9774_2_lut.init = 16'h8888;
    LUT4 i9773_2_lut (.A(shift_register[65]), .B(n13), .Z(n15752)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9773_2_lut.init = 16'h8888;
    LUT4 i9772_2_lut (.A(shift_register[47]), .B(n13), .Z(n15750)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9772_2_lut.init = 16'h8888;
    LUT4 i9771_2_lut (.A(shift_register[46]), .B(n13), .Z(n15748)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9771_2_lut.init = 16'h8888;
    LUT4 i9770_2_lut (.A(shift_register[45]), .B(n13), .Z(n15746)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9770_2_lut.init = 16'h8888;
    LUT4 i9769_2_lut (.A(shift_register[44]), .B(n13), .Z(n15744)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9769_2_lut.init = 16'h8888;
    LUT4 i9768_2_lut (.A(shift_register[43]), .B(n13), .Z(n15742)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9768_2_lut.init = 16'h8888;
    LUT4 i9767_2_lut (.A(shift_register[42]), .B(n13), .Z(n15740)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9767_2_lut.init = 16'h8888;
    LUT4 i6_4_lut (.A(bit_count[2]), .B(n12), .C(bit_count[6]), .D(bit_count[1]), 
         .Z(n13)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(26[16:33])
    defparam i6_4_lut.init = 16'hfffe;
    LUT4 i5_4_lut (.A(bit_count[0]), .B(bit_count[5]), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n12)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(26[16:33])
    defparam i5_4_lut.init = 16'hfffe;
    LUT4 i9766_2_lut (.A(shift_register[40]), .B(n13), .Z(n15738)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9766_2_lut.init = 16'h8888;
    LUT4 i9765_2_lut (.A(shift_register[23]), .B(n13), .Z(n15736)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9765_2_lut.init = 16'h8888;
    LUT4 i9764_2_lut (.A(shift_register[22]), .B(n13), .Z(n15734)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9764_2_lut.init = 16'h8888;
    LUT4 i9758_3_lut_4_lut (.A(bit_count[4]), .B(n23865), .C(n13), .D(bit_count[5]), 
         .Z(n15722)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i9758_3_lut_4_lut.init = 16'h7080;
    LUT4 i9763_2_lut (.A(shift_register[21]), .B(n13), .Z(n15732)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9763_2_lut.init = 16'h8888;
    LUT4 i9762_2_lut (.A(shift_register[20]), .B(n13), .Z(n15730)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9762_2_lut.init = 16'h8888;
    LUT4 i9761_2_lut (.A(shift_register[19]), .B(n13), .Z(n15728)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9761_2_lut.init = 16'h8888;
    LUT4 i9760_2_lut (.A(shift_register[18]), .B(n13), .Z(n15726)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9760_2_lut.init = 16'h8888;
    LUT4 i9759_4_lut (.A(bit_count[6]), .B(n13), .C(bit_count[5]), .D(n23861), 
         .Z(n15724)) /* synthesis lut_function=(!(A ((C (D))+!B)+!A !(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9759_4_lut.init = 16'h4888;
    LUT4 i9757_3_lut_4_lut (.A(bit_count[3]), .B(n23876), .C(n13), .D(bit_count[4]), 
         .Z(n15720)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i9757_3_lut_4_lut.init = 16'h7080;
    LUT4 i9756_3_lut_4_lut (.A(bit_count[2]), .B(n23890), .C(n13), .D(bit_count[3]), 
         .Z(n15718)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i9756_3_lut_4_lut.init = 16'h7080;
    LUT4 i1546_2_lut_rep_69_3_lut_4_lut (.A(bit_count[2]), .B(n23890), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n23861)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1546_2_lut_rep_69_3_lut_4_lut.init = 16'h8000;
    
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
// Verilog Description of module umh_toggle_ram84
//

module umh_toggle_ram84 (pll_clk, ev_we, VCC_net, GND_net, \ev_wr_addr[0] , 
            event_rd_addr, \ev_wr_addr[1] , \ev_wr_addr[2] , \ev_wr_addr[3] , 
            \ev_wr_addr[4] , \ev_wr_addr[5] , \ev_wr_addr[6] , \ev_wr_addr[7] , 
            n23921, ev_wr_data, ev_rd_data) /* synthesis syn_module_defined=1 */ ;
    input pll_clk;
    input ev_we;
    input VCC_net;
    input GND_net;
    input \ev_wr_addr[0] ;
    input [8:0]event_rd_addr;
    input \ev_wr_addr[1] ;
    input \ev_wr_addr[2] ;
    input \ev_wr_addr[3] ;
    input \ev_wr_addr[4] ;
    input \ev_wr_addr[5] ;
    input \ev_wr_addr[6] ;
    input \ev_wr_addr[7] ;
    input n23921;
    input [83:0]ev_wr_data;
    output [83:0]ev_rd_data;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[10:17])
    
    PDPW8KC mem1 (.DI0(ev_wr_data[48]), .DI1(ev_wr_data[49]), .DI2(ev_wr_data[50]), 
            .DI3(ev_wr_data[51]), .DI4(ev_wr_data[52]), .DI5(ev_wr_data[53]), 
            .DI6(ev_wr_data[54]), .DI7(ev_wr_data[55]), .DI8(ev_wr_data[56]), 
            .DI9(ev_wr_data[57]), .DI10(ev_wr_data[58]), .DI11(ev_wr_data[59]), 
            .DI12(ev_wr_data[60]), .DI13(ev_wr_data[61]), .DI14(ev_wr_data[62]), 
            .DI15(ev_wr_data[63]), .DI16(ev_wr_data[64]), .DI17(ev_wr_data[65]), 
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23921), 
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
    PDPW8KC mem4 (.DI0(ev_wr_data[0]), .DI1(ev_wr_data[1]), .DI2(ev_wr_data[2]), 
            .DI3(ev_wr_data[3]), .DI4(ev_wr_data[4]), .DI5(ev_wr_data[5]), 
            .DI6(ev_wr_data[6]), .DI7(ev_wr_data[7]), .DI8(ev_wr_data[8]), 
            .DI9(ev_wr_data[9]), .DI10(ev_wr_data[10]), .DI11(ev_wr_data[11]), 
            .DI12(GND_net), .DI13(GND_net), .DI14(GND_net), .DI15(GND_net), 
            .DI16(GND_net), .DI17(GND_net), .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), 
            .ADW2(\ev_wr_addr[2] ), .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), 
            .ADW5(\ev_wr_addr[5] ), .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), 
            .ADW8(n23921), .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), 
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
    PDPW8KC mem2 (.DI0(ev_wr_data[30]), .DI1(ev_wr_data[31]), .DI2(ev_wr_data[32]), 
            .DI3(ev_wr_data[33]), .DI4(ev_wr_data[34]), .DI5(ev_wr_data[35]), 
            .DI6(ev_wr_data[36]), .DI7(ev_wr_data[37]), .DI8(ev_wr_data[38]), 
            .DI9(ev_wr_data[39]), .DI10(ev_wr_data[40]), .DI11(ev_wr_data[41]), 
            .DI12(ev_wr_data[42]), .DI13(ev_wr_data[43]), .DI14(ev_wr_data[44]), 
            .DI15(ev_wr_data[45]), .DI16(ev_wr_data[46]), .DI17(ev_wr_data[47]), 
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23921), 
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
    PDPW8KC mem3 (.DI0(ev_wr_data[12]), .DI1(ev_wr_data[13]), .DI2(ev_wr_data[14]), 
            .DI3(ev_wr_data[15]), .DI4(ev_wr_data[16]), .DI5(ev_wr_data[17]), 
            .DI6(ev_wr_data[18]), .DI7(ev_wr_data[19]), .DI8(ev_wr_data[20]), 
            .DI9(ev_wr_data[21]), .DI10(ev_wr_data[22]), .DI11(ev_wr_data[23]), 
            .DI12(ev_wr_data[24]), .DI13(ev_wr_data[25]), .DI14(ev_wr_data[26]), 
            .DI15(ev_wr_data[27]), .DI16(ev_wr_data[28]), .DI17(ev_wr_data[29]), 
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23921), 
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
    PDPW8KC mem0 (.DI0(ev_wr_data[66]), .DI1(ev_wr_data[67]), .DI2(ev_wr_data[68]), 
            .DI3(ev_wr_data[69]), .DI4(ev_wr_data[70]), .DI5(ev_wr_data[71]), 
            .DI6(ev_wr_data[72]), .DI7(ev_wr_data[73]), .DI8(ev_wr_data[74]), 
            .DI9(ev_wr_data[75]), .DI10(ev_wr_data[76]), .DI11(ev_wr_data[77]), 
            .DI12(ev_wr_data[78]), .DI13(ev_wr_data[79]), .DI14(ev_wr_data[80]), 
            .DI15(ev_wr_data[81]), .DI16(ev_wr_data[82]), .DI17(ev_wr_data[83]), 
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23921), 
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
    
endmodule
//
// Verilog Description of module ws2812_stream
//

module ws2812_stream (\rgb_hold[7] , state, pll_clk, ws2812_enable, 
            \shift_register[17] , \shift_register_23__N_2910[17] , \shift_register[18] , 
            \shift_register_23__N_2910[18] , \shift_register[19] , \shift_register_23__N_2910[19] , 
            \shift_register_23__N_2910[20] , \shift_register[21] , \shift_register_23__N_2910[22] , 
            GND_net, \shift_register[16] , \rgb_hold[0] , n23860, rgb_data_c, 
            \rgb_hold[5] ) /* synthesis syn_module_defined=1 */ ;
    input \rgb_hold[7] ;
    output [1:0]state;
    input pll_clk;
    input ws2812_enable;
    output \shift_register[17] ;
    input \shift_register_23__N_2910[17] ;
    output \shift_register[18] ;
    input \shift_register_23__N_2910[18] ;
    output \shift_register[19] ;
    input \shift_register_23__N_2910[19] ;
    input \shift_register_23__N_2910[20] ;
    output \shift_register[21] ;
    input \shift_register_23__N_2910[22] ;
    input GND_net;
    output \shift_register[16] ;
    input \rgb_hold[0] ;
    output n23860;
    output rgb_data_c;
    input \rgb_hold[5] ;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[10:17])
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[16:30])
    
    wire n23121;
    wire [23:0]shift_register_23__N_2910;
    wire [7:0]bit_cell_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(27[15:29])
    
    wire n23900, n23522, n23010, n23523;
    wire [1:0]state_c;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    
    wire n23904;
    wire [1:0]state_1__N_2885;
    wire [12:0]reset_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[16:27])
    
    wire pll_clk_enable_473, n23082, n14, n10, n19024;
    wire [4:0]bit_number;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(28[15:25])
    
    wire n23873, n22480, n16480, n23880, n23887, pll_clk_enable_729, 
        n23894, n23864, pll_clk_enable_459;
    wire [12:0]n22;
    
    wire n23083, n23078, n23090, n23089, n23081, n23088, n15, 
        pll_clk_enable_731, n23080, n23058, n30_adj_3160, n23325, 
        n10_adj_3161, n9868, n23079, n23879, n23087, n23086, n23085, 
        n23084, n23863, n23901, n23102, pll_clk_enable_657, n23903, 
        n18959;
    wire [7:0]n70;
    
    wire n23859;
    wire [4:0]n136;
    
    wire n23433, n23415, n23345, n23171, n23028, n4, n22383, n22382, 
        n22381, n22380, n22379, n22378, n22528, n22377, n22376, 
        n22375, n22374;
    
    LUT4 i7616_4_lut (.A(\rgb_hold[7] ), .B(shift_register[22]), .C(state[1]), 
         .D(n23121), .Z(shift_register_23__N_2910[23])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    defparam i7616_4_lut.init = 16'hca0a;
    LUT4 i14751_1_lut_4_lut (.A(bit_cell_count[0]), .B(bit_cell_count[4]), 
         .C(bit_cell_count[5]), .D(n23900), .Z(n23522)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(54[25:53])
    defparam i14751_1_lut_4_lut.init = 16'h0010;
    LUT4 i14752_1_lut_3_lut (.A(n23010), .B(bit_cell_count[2]), .C(bit_cell_count[6]), 
         .Z(n23523)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(72[33:61])
    defparam i14752_1_lut_3_lut.init = 16'h1010;
    FD1S3IX state__i0 (.D(state_1__N_2885[0]), .CK(pll_clk), .CD(n23904), 
            .Q(state_c[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam state__i0.GSR = "DISABLED";
    FD1P3AX reset_count__i0 (.D(n23082), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i0.GSR = "DISABLED";
    LUT4 i7_4_lut (.A(reset_count[0]), .B(n14), .C(n10), .D(reset_count[7]), 
         .Z(n19024)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut.init = 16'h8000;
    LUT4 i6_4_lut (.A(reset_count[2]), .B(reset_count[1]), .C(reset_count[4]), 
         .D(reset_count[11]), .Z(n14)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut.init = 16'h8000;
    LUT4 i2_2_lut (.A(reset_count[3]), .B(reset_count[6]), .Z(n10)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut.init = 16'h8888;
    LUT4 i7594_4_lut (.A(bit_number[0]), .B(n23873), .C(bit_cell_count[7]), 
         .D(n22480), .Z(n16480)) /* synthesis lut_function=(A ((D)+!C)+!A !(B+!(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(27[15:29])
    defparam i7594_4_lut.init = 16'hba1a;
    LUT4 i2_4_lut (.A(n23900), .B(n23880), .C(bit_number[4]), .D(n23887), 
         .Z(n22480)) /* synthesis lut_function=(A+(B+(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(28[15:25])
    defparam i2_4_lut.init = 16'hfeee;
    LUT4 i1_2_lut_rep_101 (.A(state_c[0]), .B(ws2812_enable), .Z(pll_clk_enable_729)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i1_2_lut_rep_101.init = 16'h8888;
    LUT4 i1_2_lut_3_lut_rep_102 (.A(state_c[0]), .B(ws2812_enable), .C(state[1]), 
         .Z(n23894)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i1_2_lut_3_lut_rep_102.init = 16'h0808;
    LUT4 i1_3_lut_4_lut_4_lut (.A(state_c[0]), .B(ws2812_enable), .C(state[1]), 
         .D(n23864), .Z(pll_clk_enable_459)) /* synthesis lut_function=(A (B ((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i1_3_lut_4_lut_4_lut.init = 16'h8808;
    LUT4 i1_2_lut_3_lut (.A(ws2812_enable), .B(state[1]), .C(n22[0]), 
         .Z(n23082)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_43 (.A(ws2812_enable), .B(state[1]), .C(n22[1]), 
         .Z(n23083)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_43.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_44 (.A(ws2812_enable), .B(state[1]), .C(n22[2]), 
         .Z(n23078)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_44.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_45 (.A(ws2812_enable), .B(state[1]), .C(n22[3]), 
         .Z(n23090)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_45.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_46 (.A(ws2812_enable), .B(state[1]), .C(n22[4]), 
         .Z(n23089)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_46.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_47 (.A(ws2812_enable), .B(state[1]), .C(n22[5]), 
         .Z(n23081)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_47.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_48 (.A(ws2812_enable), .B(state[1]), .C(n22[6]), 
         .Z(n23088)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_48.init = 16'h2020;
    LUT4 i2_4_lut_adj_49 (.A(state_c[0]), .B(state[1]), .C(n15), .D(ws2812_enable), 
         .Z(pll_clk_enable_731)) /* synthesis lut_function=(A ((C+!(D))+!B)+!A !(B (D))) */ ;
    defparam i2_4_lut_adj_49.init = 16'hb3ff;
    LUT4 i1_2_lut_rep_108 (.A(bit_cell_count[6]), .B(bit_cell_count[2]), 
         .Z(n23900)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(54[25:53])
    defparam i1_2_lut_rep_108.init = 16'hbbbb;
    LUT4 i1_2_lut_3_lut_adj_50 (.A(ws2812_enable), .B(state[1]), .C(n22[7]), 
         .Z(n23080)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_50.init = 16'h2020;
    LUT4 i33_4_lut (.A(bit_cell_count[1]), .B(n23873), .C(bit_cell_count[7]), 
         .D(n23058), .Z(n15)) /* synthesis lut_function=(!(A (B+!(C))+!A (B (C+!(D))+!B !(C+(D))))) */ ;
    defparam i33_4_lut.init = 16'h3530;
    LUT4 i1_2_lut (.A(bit_cell_count[3]), .B(n30_adj_3160), .Z(n23058)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut.init = 16'h8888;
    LUT4 i1_4_lut (.A(n23325), .B(n23894), .C(n10_adj_3161), .D(n23900), 
         .Z(n9868)) /* synthesis lut_function=(A (B)+!A (B+!((D)+!C))) */ ;
    defparam i1_4_lut.init = 16'hccdc;
    LUT4 i4_4_lut (.A(state[1]), .B(n23880), .C(bit_cell_count[7]), .D(ws2812_enable), 
         .Z(n10_adj_3161)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i4_4_lut.init = 16'h2000;
    LUT4 i1_2_lut_rep_72_3_lut_4_lut (.A(bit_cell_count[6]), .B(bit_cell_count[2]), 
         .C(bit_cell_count[7]), .D(n23880), .Z(n23864)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(54[25:53])
    defparam i1_2_lut_rep_72_3_lut_4_lut.init = 16'h0040;
    LUT4 i1_2_lut_3_lut_adj_51 (.A(ws2812_enable), .B(state[1]), .C(n22[8]), 
         .Z(n23079)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_51.init = 16'h2020;
    LUT4 i1_2_lut_rep_87_3_lut (.A(bit_cell_count[6]), .B(bit_cell_count[2]), 
         .C(bit_cell_count[7]), .Z(n23879)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(54[25:53])
    defparam i1_2_lut_rep_87_3_lut.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_52 (.A(ws2812_enable), .B(state[1]), .C(n22[9]), 
         .Z(n23087)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_52.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_53 (.A(ws2812_enable), .B(state[1]), .C(n22[10]), 
         .Z(n23086)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_53.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_54 (.A(ws2812_enable), .B(state[1]), .C(n22[11]), 
         .Z(n23085)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_54.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_55 (.A(ws2812_enable), .B(state[1]), .C(n22[12]), 
         .Z(n23084)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_55.init = 16'h2020;
    LUT4 i2_3_lut_rep_71 (.A(bit_number[0]), .B(n22480), .C(bit_cell_count[7]), 
         .Z(n23863)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i2_3_lut_rep_71.init = 16'h2020;
    LUT4 i10063_2_lut_rep_109 (.A(bit_number[1]), .B(bit_number[2]), .Z(n23901)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10063_2_lut_rep_109.init = 16'h8888;
    LUT4 i1_2_lut_rep_95_3_lut (.A(bit_number[1]), .B(bit_number[2]), .C(bit_number[3]), 
         .Z(n23887)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i1_2_lut_rep_95_3_lut.init = 16'h0808;
    LUT4 i14533_2_lut_3_lut_4_lut (.A(bit_number[1]), .B(bit_number[2]), 
         .C(n23102), .D(bit_number[3]), .Z(n23325)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i14533_2_lut_3_lut_4_lut.init = 16'h0080;
    FD1S3IX state__i1 (.D(state_1__N_2885[1]), .CK(pll_clk), .CD(n23904), 
            .Q(state[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam state__i1.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i17 (.D(\shift_register_23__N_2910[17] ), .SP(pll_clk_enable_657), 
            .CK(pll_clk), .Q(\shift_register[17] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i17.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i18 (.D(\shift_register_23__N_2910[18] ), .SP(pll_clk_enable_657), 
            .CK(pll_clk), .Q(\shift_register[18] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i18.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i19 (.D(\shift_register_23__N_2910[19] ), .SP(pll_clk_enable_657), 
            .CK(pll_clk), .Q(\shift_register[19] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i19.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i20 (.D(\shift_register_23__N_2910[20] ), .SP(pll_clk_enable_657), 
            .CK(pll_clk), .Q(shift_register[20])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i20.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i21 (.D(shift_register_23__N_2910[21]), .SP(pll_clk_enable_459), 
            .CK(pll_clk), .Q(\shift_register[21] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i21.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i22 (.D(\shift_register_23__N_2910[22] ), .SP(pll_clk_enable_459), 
            .CK(pll_clk), .Q(shift_register[22])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i22.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i23 (.D(shift_register_23__N_2910[23]), .SP(pll_clk_enable_459), 
            .CK(pll_clk), .Q(shift_register[23])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i23.GSR = "DISABLED";
    FD1P3AX reset_count__i1 (.D(n23083), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i1.GSR = "DISABLED";
    LUT4 i2_2_lut_rep_111 (.A(reset_count[8]), .B(reset_count[12]), .Z(n23903)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut_rep_111.init = 16'h8888;
    LUT4 i3_3_lut_4_lut (.A(reset_count[8]), .B(reset_count[12]), .C(reset_count[5]), 
         .D(reset_count[10]), .Z(n18959)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_3_lut_4_lut.init = 16'h8000;
    FD1P3AX reset_count__i2 (.D(n23078), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i2.GSR = "DISABLED";
    FD1P3AX reset_count__i3 (.D(n23090), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i3.GSR = "DISABLED";
    FD1P3AX reset_count__i4 (.D(n23089), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i4.GSR = "DISABLED";
    FD1P3AX reset_count__i5 (.D(n23081), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i5.GSR = "DISABLED";
    FD1P3AX reset_count__i6 (.D(n23088), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i6.GSR = "DISABLED";
    FD1P3AX reset_count__i7 (.D(n23080), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i7.GSR = "DISABLED";
    FD1P3AX reset_count__i8 (.D(n23079), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i8.GSR = "DISABLED";
    FD1P3AX reset_count__i9 (.D(n23087), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i9.GSR = "DISABLED";
    FD1P3AX reset_count__i10 (.D(n23086), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i10.GSR = "DISABLED";
    FD1P3AX reset_count__i11 (.D(n23085), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i11.GSR = "DISABLED";
    FD1P3AX reset_count__i12 (.D(n23084), .SP(pll_clk_enable_473), .CK(pll_clk), 
            .Q(reset_count[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i12.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i7 (.D(n70[7]), .SP(pll_clk_enable_729), .CD(pll_clk_enable_657), 
            .CK(pll_clk), .Q(bit_cell_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i7.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i6 (.D(n70[6]), .SP(pll_clk_enable_729), .CD(pll_clk_enable_657), 
            .CK(pll_clk), .Q(bit_cell_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i6.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i5 (.D(n70[5]), .SP(pll_clk_enable_729), .CD(pll_clk_enable_657), 
            .CK(pll_clk), .Q(bit_cell_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i5.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i4 (.D(n70[4]), .SP(pll_clk_enable_729), .CD(pll_clk_enable_657), 
            .CK(pll_clk), .Q(bit_cell_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i4.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i3 (.D(n70[3]), .SP(pll_clk_enable_729), .CD(pll_clk_enable_657), 
            .CK(pll_clk), .Q(bit_cell_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i3.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i2 (.D(n70[2]), .SP(pll_clk_enable_729), .CD(pll_clk_enable_657), 
            .CK(pll_clk), .Q(bit_cell_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i2.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i1 (.D(n70[1]), .SP(pll_clk_enable_729), .CD(pll_clk_enable_657), 
            .CK(pll_clk), .Q(bit_cell_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i1.GSR = "DISABLED";
    FD1P3IX bit_number_i0_i0 (.D(n16480), .SP(pll_clk_enable_729), .CD(n23894), 
            .CK(pll_clk), .Q(bit_number[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i0.GSR = "DISABLED";
    LUT4 i13_1_lut_rep_112 (.A(ws2812_enable), .Z(n23904)) /* synthesis lut_function=(!(A)) */ ;
    defparam i13_1_lut_rep_112.init = 16'h5555;
    LUT4 i27_3_lut_4_lut (.A(bit_number[2]), .B(n23859), .C(bit_number[4]), 
         .D(bit_number[3]), .Z(n136[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i27_3_lut_4_lut.init = 16'h78f0;
    LUT4 i1_4_lut_adj_56 (.A(n23433), .B(state_1__N_2885[1]), .C(state[1]), 
         .D(reset_count[9]), .Z(state_1__N_2885[0])) /* synthesis lut_function=(A (B+!(C+(D)))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    defparam i1_4_lut_adj_56.init = 16'hccce;
    LUT4 i14641_4_lut (.A(reset_count[11]), .B(n23415), .C(n23903), .D(reset_count[5]), 
         .Z(n23433)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14641_4_lut.init = 16'h8000;
    LUT4 i14623_4_lut (.A(reset_count[10]), .B(n23345), .C(n23171), .D(reset_count[0]), 
         .Z(n23415)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14623_4_lut.init = 16'h8000;
    LUT4 i14553_4_lut (.A(reset_count[6]), .B(reset_count[4]), .C(reset_count[2]), 
         .D(reset_count[1]), .Z(n23345)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14553_4_lut.init = 16'h8000;
    LUT4 i14381_2_lut (.A(reset_count[7]), .B(reset_count[3]), .Z(n23171)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14381_2_lut.init = 16'h8888;
    LUT4 i1_3_lut (.A(state[1]), .B(state_c[0]), .C(n23028), .Z(state_1__N_2885[1])) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    defparam i1_3_lut.init = 16'h4c4c;
    LUT4 i2_4_lut_adj_57 (.A(n23102), .B(n23901), .C(n23879), .D(n4), 
         .Z(n23028)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i2_4_lut_adj_57.init = 16'h0080;
    LUT4 i1_3_lut_adj_58 (.A(bit_cell_count[5]), .B(bit_cell_count[4]), 
         .C(bit_cell_count[0]), .Z(n23010)) /* synthesis lut_function=(A+!(B (C))) */ ;
    defparam i1_3_lut_adj_58.init = 16'hbfbf;
    PFUMX i32 (.BLUT(n23522), .ALUT(n23523), .C0(shift_register[23]), 
          .Z(n30_adj_3160));
    CCU2D add_1075_13 (.A0(reset_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22383), .S0(n22[11]), .S1(n22[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1075_13.INIT0 = 16'h5aaa;
    defparam add_1075_13.INIT1 = 16'h5aaa;
    defparam add_1075_13.INJECT1_0 = "NO";
    defparam add_1075_13.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_59 (.A(bit_number[0]), .B(bit_number[4]), .Z(n23102)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_59.init = 16'h8888;
    CCU2D add_1075_11 (.A0(reset_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22382), .COUT(n22383), .S0(n22[9]), .S1(n22[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1075_11.INIT0 = 16'h5aaa;
    defparam add_1075_11.INIT1 = 16'h5aaa;
    defparam add_1075_11.INJECT1_0 = "NO";
    defparam add_1075_11.INJECT1_1 = "NO";
    CCU2D add_1075_9 (.A0(reset_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22381), .COUT(n22382), .S0(n22[7]), .S1(n22[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1075_9.INIT0 = 16'h5aaa;
    defparam add_1075_9.INIT1 = 16'h5aaa;
    defparam add_1075_9.INJECT1_0 = "NO";
    defparam add_1075_9.INJECT1_1 = "NO";
    CCU2D add_1075_7 (.A0(reset_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22380), .COUT(n22381), .S0(n22[5]), .S1(n22[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1075_7.INIT0 = 16'h5aaa;
    defparam add_1075_7.INIT1 = 16'h5aaa;
    defparam add_1075_7.INJECT1_0 = "NO";
    defparam add_1075_7.INJECT1_1 = "NO";
    FD1P3IX bit_number_i0_i4 (.D(n136[4]), .SP(pll_clk_enable_729), .CD(n23894), 
            .CK(pll_clk), .Q(bit_number[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i4.GSR = "DISABLED";
    CCU2D add_1075_5 (.A0(reset_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22379), .COUT(n22380), .S0(n22[3]), .S1(n22[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1075_5.INIT0 = 16'h5aaa;
    defparam add_1075_5.INIT1 = 16'h5aaa;
    defparam add_1075_5.INJECT1_0 = "NO";
    defparam add_1075_5.INJECT1_1 = "NO";
    FD1P3IX bit_number_i0_i3 (.D(n136[3]), .SP(pll_clk_enable_729), .CD(n23894), 
            .CK(pll_clk), .Q(bit_number[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i3.GSR = "DISABLED";
    CCU2D add_1075_3 (.A0(reset_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22378), .COUT(n22379), .S0(n22[1]), .S1(n22[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1075_3.INIT0 = 16'h5aaa;
    defparam add_1075_3.INIT1 = 16'h5aaa;
    defparam add_1075_3.INJECT1_0 = "NO";
    defparam add_1075_3.INJECT1_1 = "NO";
    FD1P3IX bit_number_i0_i2 (.D(n136[2]), .SP(pll_clk_enable_729), .CD(n23894), 
            .CK(pll_clk), .Q(bit_number[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i2.GSR = "DISABLED";
    FD1P3IX bit_number_i0_i1 (.D(n136[1]), .SP(pll_clk_enable_729), .CD(n23894), 
            .CK(pll_clk), .Q(bit_number[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i1.GSR = "DISABLED";
    FD1P3IX shift_register_i0_i16 (.D(\rgb_hold[0] ), .SP(pll_clk_enable_657), 
            .CD(n22528), .CK(pll_clk), .Q(\shift_register[16] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i16.GSR = "DISABLED";
    CCU2D add_1075_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count[0]), .B1(n19024), .C1(n18959), .D1(reset_count[9]), 
          .COUT(n22378), .S1(n22[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1075_1.INIT0 = 16'hF000;
    defparam add_1075_1.INIT1 = 16'h5595;
    defparam add_1075_1.INJECT1_0 = "NO";
    defparam add_1075_1.INJECT1_1 = "NO";
    CCU2D add_15_9 (.A0(bit_cell_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22377), .S0(n70[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_9.INIT0 = 16'h5aaa;
    defparam add_15_9.INIT1 = 16'h0000;
    defparam add_15_9.INJECT1_0 = "NO";
    defparam add_15_9.INJECT1_1 = "NO";
    CCU2D add_15_7 (.A0(bit_cell_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22376), .COUT(n22377), .S0(n70[5]), .S1(n70[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_7.INIT0 = 16'h5aaa;
    defparam add_15_7.INIT1 = 16'h5aaa;
    defparam add_15_7.INJECT1_0 = "NO";
    defparam add_15_7.INJECT1_1 = "NO";
    CCU2D add_15_5 (.A0(bit_cell_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22375), .COUT(n22376), .S0(n70[3]), .S1(n70[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_5.INIT0 = 16'h5aaa;
    defparam add_15_5.INIT1 = 16'h5aaa;
    defparam add_15_5.INJECT1_0 = "NO";
    defparam add_15_5.INJECT1_1 = "NO";
    CCU2D add_15_3 (.A0(bit_cell_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22374), .COUT(n22375), .S0(n70[1]), .S1(n70[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_3.INIT0 = 16'h5aaa;
    defparam add_15_3.INIT1 = 16'h5aaa;
    defparam add_15_3.INJECT1_0 = "NO";
    defparam add_15_3.INJECT1_1 = "NO";
    CCU2D add_15_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_cell_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22374), .S1(n70[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_1.INIT0 = 16'hF000;
    defparam add_15_1.INIT1 = 16'h5555;
    defparam add_15_1.INJECT1_0 = "NO";
    defparam add_15_1.INJECT1_1 = "NO";
    LUT4 i1586_2_lut_3_lut_4_lut (.A(bit_number[1]), .B(n23863), .C(bit_number[3]), 
         .D(bit_number[2]), .Z(n136[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i1586_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i1579_2_lut_3_lut (.A(bit_number[1]), .B(n23863), .C(bit_number[2]), 
         .Z(n136[2])) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i1579_2_lut_3_lut.init = 16'h7878;
    LUT4 i1574_2_lut_rep_67_4_lut (.A(bit_number[0]), .B(n22480), .C(bit_cell_count[7]), 
         .D(bit_number[1]), .Z(n23859)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i1574_2_lut_rep_67_4_lut.init = 16'h2000;
    LUT4 i1572_2_lut_4_lut (.A(bit_number[0]), .B(n22480), .C(bit_cell_count[7]), 
         .D(bit_number[1]), .Z(n136[1])) /* synthesis lut_function=(A (B (D)+!B !(C (D)+!C !(D)))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i1572_2_lut_4_lut.init = 16'hdf20;
    LUT4 i7565_4_lut_4_lut_4_lut (.A(ws2812_enable), .B(state[1]), .C(state_c[0]), 
         .D(n23028), .Z(pll_clk_enable_473)) /* synthesis lut_function=((B (C (D))+!B !(C))+!A) */ ;
    defparam i7565_4_lut_4_lut_4_lut.init = 16'hd757;
    LUT4 i6723_3_lut_4_lut (.A(bit_cell_count[7]), .B(n23873), .C(state[1]), 
         .D(pll_clk_enable_729), .Z(pll_clk_enable_657)) /* synthesis lut_function=(!(A (B (C+!(D))+!B !(D))+!A (C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i6723_3_lut_4_lut.init = 16'h2f00;
    LUT4 i2_3_lut_4_lut (.A(bit_cell_count[7]), .B(n23873), .C(state[1]), 
         .D(pll_clk_enable_729), .Z(n22528)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i2_3_lut_4_lut.init = 16'h2000;
    LUT4 i1_2_lut_rep_68_3_lut_4_lut (.A(n23900), .B(n23880), .C(state_c[0]), 
         .D(bit_cell_count[7]), .Z(n23860)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i1_2_lut_rep_68_3_lut_4_lut.init = 16'h1000;
    FD1P3IX bit_cell_count_i0_i0 (.D(n70[0]), .SP(pll_clk_enable_729), .CD(pll_clk_enable_657), 
            .CK(pll_clk), .Q(bit_cell_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i0.GSR = "DISABLED";
    FD1P3AX data_out_reg_46 (.D(n9868), .SP(pll_clk_enable_731), .CK(pll_clk), 
            .Q(rgb_data_c)) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=592, LSE_RLINE=604 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam data_out_reg_46.GSR = "DISABLED";
    LUT4 i7609_4_lut (.A(\rgb_hold[5] ), .B(shift_register[20]), .C(state[1]), 
         .D(n23121), .Z(shift_register_23__N_2910[21])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    defparam i7609_4_lut.init = 16'hca0a;
    LUT4 i2_3_lut_4_lut_adj_60 (.A(bit_cell_count[7]), .B(n23900), .C(n23880), 
         .D(state_c[0]), .Z(n23121)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    defparam i2_3_lut_4_lut_adj_60.init = 16'h0200;
    LUT4 i2_3_lut_rep_88 (.A(bit_cell_count[1]), .B(bit_cell_count[3]), 
         .C(n23010), .Z(n23880)) /* synthesis lut_function=(((C)+!B)+!A) */ ;
    defparam i2_3_lut_rep_88.init = 16'hf7f7;
    LUT4 i1_2_lut_4_lut (.A(bit_cell_count[1]), .B(bit_cell_count[3]), .C(n23010), 
         .D(bit_number[3]), .Z(n4)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i1_2_lut_4_lut.init = 16'hfff7;
    LUT4 i1_2_lut_rep_81_4_lut (.A(bit_cell_count[1]), .B(bit_cell_count[3]), 
         .C(n23010), .D(n23900), .Z(n23873)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i1_2_lut_rep_81_4_lut.init = 16'hfff7;
    
endmodule
//
// Verilog Description of module umh_channel_ram18
//

module umh_channel_ram18 (n10152, spi1_sck_c, spi_channel_index, staging_q, 
            pll_clk, rd_data_15__N_2649, n10142, n10140, n10150, n10148, 
            spi_write, VCC_net, GND_net, staging_rd_addr_6__N_901, spi1_mosi_c_0, 
            \spi_rx_shift[0] , \spi_rx_shift[1] , \spi_rx_shift[2] , \spi_rx_shift[3] , 
            \spi_rx_shift[4] , \spi_rx_shift[5] , \spi_rx_shift[6] , spi_phase_pending, 
            n10157, n10159, n10161, n10163, n10165, n10167, n10169, 
            n10171, n10173, n10175, n10177, n10179, n10181, n10183, 
            n10185, n10187, n10144, n10146) /* synthesis syn_module_defined=1 */ ;
    output n10152;
    input spi1_sck_c;
    input [6:0]spi_channel_index;
    output [15:0]staging_q;
    input pll_clk;
    input [15:0]rd_data_15__N_2649;
    output n10142;
    output n10140;
    output n10150;
    output n10148;
    input spi_write;
    input VCC_net;
    input GND_net;
    input [6:0]staging_rd_addr_6__N_901;
    input spi1_mosi_c_0;
    input \spi_rx_shift[0] ;
    input \spi_rx_shift[1] ;
    input \spi_rx_shift[2] ;
    input \spi_rx_shift[3] ;
    input \spi_rx_shift[4] ;
    input \spi_rx_shift[5] ;
    input \spi_rx_shift[6] ;
    input [7:0]spi_phase_pending;
    output n10157;
    output n10159;
    output n10161;
    output n10163;
    output n10165;
    output n10167;
    output n10169;
    output n10171;
    output n10173;
    output n10175;
    output n10177;
    output n10179;
    output n10181;
    output n10183;
    output n10185;
    output n10187;
    output n10144;
    output n10146;
    
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[10:17])
    
    FD1S3AX mem_1388 (.D(spi_channel_index[6]), .CK(spi1_sck_c), .Q(n10152));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1388.GSR = "DISABLED";
    FD1S3AX rd_data_i0 (.D(rd_data_15__N_2649[0]), .CK(pll_clk), .Q(staging_q[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i0.GSR = "DISABLED";
    FD1S3AX mem_1378 (.D(spi_channel_index[1]), .CK(spi1_sck_c), .Q(n10142));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1378.GSR = "DISABLED";
    FD1S3AX mem_1376 (.D(spi_channel_index[0]), .CK(spi1_sck_c), .Q(n10140));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1376.GSR = "DISABLED";
    FD1S3AX mem_1386 (.D(spi_channel_index[5]), .CK(spi1_sck_c), .Q(n10150));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1386.GSR = "DISABLED";
    FD1S3AX mem_1384 (.D(spi_channel_index[4]), .CK(spi1_sck_c), .Q(n10148));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1384.GSR = "DISABLED";
    PDPW8KC mem0 (.DI0(spi1_mosi_c_0), .DI1(\spi_rx_shift[0] ), .DI2(\spi_rx_shift[1] ), 
            .DI3(\spi_rx_shift[2] ), .DI4(\spi_rx_shift[3] ), .DI5(\spi_rx_shift[4] ), 
            .DI6(\spi_rx_shift[5] ), .DI7(\spi_rx_shift[6] ), .DI8(spi_phase_pending[0]), 
            .DI9(spi_phase_pending[1]), .DI10(spi_phase_pending[2]), .DI11(spi_phase_pending[3]), 
            .DI12(spi_phase_pending[4]), .DI13(spi_phase_pending[5]), .DI14(spi_phase_pending[6]), 
            .DI15(spi_phase_pending[7]), .DI16(GND_net), .DI17(GND_net), 
            .ADW0(spi_channel_index[0]), .ADW1(spi_channel_index[1]), .ADW2(spi_channel_index[2]), 
            .ADW3(spi_channel_index[3]), .ADW4(spi_channel_index[4]), .ADW5(spi_channel_index[5]), 
            .ADW6(spi_channel_index[6]), .ADW7(GND_net), .ADW8(GND_net), 
            .BE0(VCC_net), .BE1(VCC_net), .CEW(spi_write), .CLKW(spi1_sck_c), 
            .CSW0(GND_net), .CSW1(GND_net), .CSW2(GND_net), .ADR0(GND_net), 
            .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), .ADR4(staging_rd_addr_6__N_901[0]), 
            .ADR5(staging_rd_addr_6__N_901[1]), .ADR6(staging_rd_addr_6__N_901[2]), 
            .ADR7(staging_rd_addr_6__N_901[3]), .ADR8(staging_rd_addr_6__N_901[4]), 
            .ADR9(staging_rd_addr_6__N_901[5]), .ADR10(staging_rd_addr_6__N_901[6]), 
            .ADR11(GND_net), .ADR12(GND_net), .CER(VCC_net), .OCER(VCC_net), 
            .CLKR(pll_clk), .CSR0(GND_net), .CSR1(GND_net), .CSR2(GND_net), 
            .RST(GND_net), .DO0(n10175), .DO1(n10177), .DO2(n10179), 
            .DO3(n10181), .DO4(n10183), .DO5(n10185), .DO6(n10187), 
            .DO9(n10157), .DO10(n10159), .DO11(n10161), .DO12(n10163), 
            .DO13(n10165), .DO14(n10167), .DO15(n10169), .DO16(n10171), 
            .DO17(n10173));
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
    FD1S3AX mem_1380 (.D(spi_channel_index[2]), .CK(spi1_sck_c), .Q(n10144));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1380.GSR = "DISABLED";
    FD1S3AX mem_1382 (.D(spi_channel_index[3]), .CK(spi1_sck_c), .Q(n10146));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1382.GSR = "DISABLED";
    FD1S3AX rd_data_i1 (.D(rd_data_15__N_2649[1]), .CK(pll_clk), .Q(staging_q[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i1.GSR = "DISABLED";
    FD1S3AX rd_data_i2 (.D(rd_data_15__N_2649[2]), .CK(pll_clk), .Q(staging_q[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i2.GSR = "DISABLED";
    FD1S3AX rd_data_i3 (.D(rd_data_15__N_2649[3]), .CK(pll_clk), .Q(staging_q[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i3.GSR = "DISABLED";
    FD1S3AX rd_data_i4 (.D(rd_data_15__N_2649[4]), .CK(pll_clk), .Q(staging_q[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i4.GSR = "DISABLED";
    FD1S3AX rd_data_i5 (.D(rd_data_15__N_2649[5]), .CK(pll_clk), .Q(staging_q[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i5.GSR = "DISABLED";
    FD1S3AX rd_data_i6 (.D(rd_data_15__N_2649[6]), .CK(pll_clk), .Q(staging_q[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i6.GSR = "DISABLED";
    FD1S3AX rd_data_i7 (.D(rd_data_15__N_2649[7]), .CK(pll_clk), .Q(staging_q[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i7.GSR = "DISABLED";
    FD1S3AX rd_data_i8 (.D(rd_data_15__N_2649[8]), .CK(pll_clk), .Q(staging_q[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i8.GSR = "DISABLED";
    FD1S3AX rd_data_i9 (.D(rd_data_15__N_2649[9]), .CK(pll_clk), .Q(staging_q[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i9.GSR = "DISABLED";
    FD1S3AX rd_data_i10 (.D(rd_data_15__N_2649[10]), .CK(pll_clk), .Q(staging_q[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i10.GSR = "DISABLED";
    FD1S3AX rd_data_i11 (.D(rd_data_15__N_2649[11]), .CK(pll_clk), .Q(staging_q[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i11.GSR = "DISABLED";
    FD1S3AX rd_data_i12 (.D(rd_data_15__N_2649[12]), .CK(pll_clk), .Q(staging_q[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i12.GSR = "DISABLED";
    FD1S3AX rd_data_i13 (.D(rd_data_15__N_2649[13]), .CK(pll_clk), .Q(staging_q[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i13.GSR = "DISABLED";
    FD1S3AX rd_data_i14 (.D(rd_data_15__N_2649[14]), .CK(pll_clk), .Q(staging_q[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i14.GSR = "DISABLED";
    FD1S3AX rd_data_i15 (.D(rd_data_15__N_2649[15]), .CK(pll_clk), .Q(staging_q[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i15.GSR = "DISABLED";
    
endmodule
