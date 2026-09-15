// Verilog netlist produced by program LSE :  version Diamond (64-bit) 3.13.0.56.2
// Netlist written on Tue Sep 15 22:11:33 2026
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
    wire spi1_sck_N_413 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(131[17:33])
    wire sck_N_3271 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    
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
        mic_data_1_c, spi_mic_cs_n_c, spi_mic_miso_c, pll_locked, pll_feedback;
    wire [7:0]spi_rx_shift;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(118[17:29])
    wire [7:0]spi_command;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(118[31:42])
    wire [7:0]spi_version;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(118[44:55])
    wire [7:0]spi_phase_pending;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(118[57:74])
    wire [2:0]spi_bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(119[17:30])
    wire [15:0]spi_byte_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[17:31])
    wire [15:0]spi_extension_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[51:71])
    wire [31:0]spi_frame_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[17:35])
    wire [31:0]spi_expected_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[37:56])
    wire [31:0]accepted_sequence_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[58:79])
    wire [7:0]last_command_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(122[17:33])
    wire [15:0]last_length_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(123[17:32])
    
    wire n165, n164, n163, n162, n161, n160, n159, n158, n157, 
        n156, n155;
    wire [15:0]accepted_update_flags_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(124[17:42])
    
    wire n154;
    wire [6:0]spi_channel_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(125[17:34])
    wire [1:0]spi_channel_field;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(126[17:34])
    wire [3:0]spi_rgb_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(127[17:30])
    
    wire frame_toggle_spi, stop_toggle_spi, invalid_frame_spi, ws2812_toggle_spi, 
        pll_clk_enable_359;
    wire [95:0]rgb_values;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(130[17:27])
    wire [6:0]status_bit_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(131[17:33])
    
    wire spi_write, ws2812_enable;
    wire [23:0]phase_frac;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:27])
    
    wire phase_step_s1;
    wire [7:0]global_phase_s2;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[17:32])
    
    wire phase_step_s2, wrap_s2;
    wire [8:0]run_addr_s3;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(170[17:28])
    
    wire phase_step_s3, swap_now_s3, phase_step_s4, swap_now_s4;
    wire [83:0]ev_run_hold_s5;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:31])
    
    wire phase_step_s5, swap_now_s5;
    wire [24:0]phase_frac_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[17:31])
    wire [31:0]fpga_time;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(181[17:26])
    wire [6:0]time_divider;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(182[17:29])
    wire [6:0]mic_divider;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(185[17:28])
    
    wire mic_tick;
    wire [15:0]mic_shift_0_l;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(188[17:30])
    
    wire n153;
    wire [15:0]mic_shift_0_r;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(188[32:45])
    wire [15:0]mic_shift_1_l;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(189[17:30])
    
    wire n152;
    wire [15:0]mic_shift_1_r;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(189[32:45])
    wire [4:0]mic_sample_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(190[17:33])
    wire [63:0]mic_latest;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(191[17:27])
    wire [3:0]ev_state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[17:25])
    wire [7:0]ev_clear_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[17:30])
    wire [6:0]ev_ch;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(208[17:22])
    wire [83:0]ev_bit;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[17:23])
    wire [83:0]init_shadow;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[25:36])
    wire [7:0]build_phase;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[17:28])
    wire [8:0]build_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[17:26])
    wire [6:0]staging_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(213[17:32])
    
    wire frame_req, swap_pending, running, active_bank;
    wire [15:0]staging_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(218[17:26])
    wire [7:0]ev_rd_slot;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[17:27])
    wire [8:0]event_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(231[17:30])
    
    wire ev_we;
    wire [8:0]ev_wr_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(235[17:27])
    wire [83:0]ev_rd_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(238[17:27])
    wire [83:0]ev_rd_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(239[17:27])
    wire [83:0]ev_wr_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(240[17:27])
    
    wire frame_toggle_meta, frame_toggle_sync, frame_toggle_seen, stop_toggle_meta, 
        stop_toggle_sync, stop_toggle_seen, ws2812_toggle_meta, ws2812_toggle_sync, 
        ws2812_toggle_seen, invalid_frame_meta, invalid_frame_sync;
    wire [31:0]accepted_sequence_meta;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(258[17:39])
    wire [31:0]accepted_sequence_sync;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(258[41:63])
    wire [31:0]pending_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(259[17:33])
    wire [31:0]accepted_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(259[35:52])
    wire [15:0]update_flags_meta;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(260[17:34])
    wire [15:0]update_flags_sync;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(260[36:53])
    wire [3:0]frame_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(261[17:29])
    wire [3:0]ws2812_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(262[17:30])
    
    wire n151, n150, n149, n148, n147, n146, n145, n144, n143, 
        n142, n141, n140, n139, n138, n137, n136, n135, n134, 
        n26027;
    wire [95:0]rgb_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(263[17:25])
    
    wire n15752;
    wire [127:0]status_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(270[18:29])
    
    wire n15758, n15764, n15770, n15776, n26064, n15740, n15734;
    wire [15:0]fifo_credit_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(278[17:33])
    
    wire n15782, n15746;
    wire [15:0]status_flags_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[17:34])
    
    wire n15788;
    wire [15:0]expected_next;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[17:30])
    
    wire frame_end, n15728, n15722, n26009;
    wire [15:0]spi_rgb_payload_byte_N_2770;
    wire [8:0]ev_wr_addr_8__N_925;
    
    wire n22989, n25904, n23000, n15710, n15794, n15800, n15806, 
        n15812, n15818, n15824, n15830, n15836, n15842, n15848, 
        n15854, n15860, n15866, n15716;
    wire [6:0]spi1_miso_N_2715;
    
    wire n26028, spi1_miso_N_2714, fpga_cs_n_N_2723, n15872, pll_clk_enable_610;
    wire [15:0]expected_next_15__N_1475;
    
    wire pll_clk_enable_519, frame_end_N_2872, n15602, n15608;
    wire [31:0]frame_end_N_2840;
    wire [15:0]rd_data_15__N_2873;
    
    wire frame_end_N_2839, frame_end_N_2837, n15878, n17639, n15884, 
        n15890, n15896, n15902, n15908, n15914, n15920, n15926, 
        n15932, n15938, n15944, n15950, n26063, n16064, n16070, 
        n26026, n16078, n25347, n12011, n17384, n87, n26020, n24827, 
        n22, n23, n24, n25, n26025, spi1_sck_c_enable_75, spi1_sck_c_enable_68, 
        spi1_sck_c_enable_195;
    wire [15:0]spi_byte_count_15__N_1628;
    
    wire ws2812_toggle_spi_N_2758, n15956, n15962, n15968, n15974, 
        n15980, n15986, stop_toggle_spi_N_2738, n15992, n15998, n16004, 
        n16567, n16010, n16016, n16022, frame_toggle_spi_N_2728, n16028, 
        n16034, n16040, n15, n19, n16058, n16046, invalid_frame_spi_N_2745, 
        n16086, n40, n39, n38, n37, n36, n35, n34, n26024, 
        n99, n16092, n11999, n16098, n11987, n14, n1, spi1_sck_c_enable_196, 
        wrap_s2_N_2799, n26021, n26035;
    wire [8:0]run_addr_s3_8__N_438;
    
    wire n24775, n25218, n60, n15614, n15698, pll_clk_enable_210, 
        n24826, n3504, n3505, spi1_sck_c_enable_136, n20, n19_adj_3386, 
        n18;
    wire [3:0]frame_settle_3__N_2130;
    
    wire n24774, n14_adj_3387, n6, spi1_sck_c_enable_98, n3882, n24825, 
        n34_adj_3388, n35_adj_3389, n36_adj_3390, n37_adj_3391, n38_adj_3392, 
        n39_adj_3393, n40_adj_3394, pll_clk_enable_390, n38_adj_3395, 
        n39_adj_3396, n40_adj_3397, n41, n42, n43, n44, n45, n26062, 
        pll_clk_enable_650, n25499, n26061, n26023, pll_clk_enable_305, 
        n26060, n26033, n12560;
    wire [31:0]accepted_sequence_31__N_1135;
    
    wire n24911, n4, n8, n32, n25635, swap_pending_N_2813, n29, 
        n40_adj_3398, n39_adj_3399, n38_adj_3400, n37_adj_3401, n36_adj_3402, 
        n35_adj_3403, n34_adj_3404, n28, n78, pll_clk_enable_696, 
        n30, pll_clk_enable_17, n25631, n26058, n25573, n25629, 
        n25981, n25980, n26;
    wire [7:0]ev_clear_addr_7__N_2439;
    
    wire ev_clear_done_N_2802;
    wire [3:0]ev_state_3__N_2142;
    wire [8:0]build_sum_8__N_2255;
    
    wire spi1_sck_c_enable_200, spi1_sck_c_enable_170;
    wire [3:0]ev_state_3__N_2134;
    wire [6:0]ev_ch_6__N_2150;
    wire [3:0]ev_state_3__N_710;
    
    wire n25979;
    wire [6:0]ev_ch_6__N_722;
    
    wire n25619, n25617, n12, n25613, n21575, n26022, n21573, 
        n21742, n25898, n13, n20_adj_3405, n26057, pll_clk_enable_777;
    wire [6:0]staging_rd_addr_6__N_914;
    
    wire n25601, n19_adj_3406, n26056, n49, n25581, n25575, n25895, 
        n22776, n25569, n25894, n4_adj_3407, n25561, spi1_sck_c_enable_83, 
        n13469, mic_tick_N_2800, spi1_sck_c_enable_188, n24824, n34_adj_3408, 
        n35_adj_3409, n36_adj_3410, n37_adj_3411, n38_adj_3412, n39_adj_3413, 
        n40_adj_3414, n26019, n28_adj_3415, n29_adj_3416, n27, n26_adj_3417, 
        n24619, n26271, n25893, mic_clk_N_2724, n25892, n4_adj_3418, 
        n12927, n12926, n12925, n12924, n12923, n12922, n12921, 
        n12920, n12919, n12918, n12917, n12916, n12915, n12914, 
        n12913, n12912, n12911, n12910, n15620, n15626, n15632, 
        n15638, n12879, spi1_sck_N_413_enable_7, n6_adj_3419, n5, 
        n25555, n26269, pll_clk_enable_596, spi1_sck_c_enable_187, n15644, 
        n15650, n15656, n15662, n15668, n15674, n15680, n15686, 
        n15692, n15704;
    wire [1:0]state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[16:30])
    
    wire spi1_sck_c_enable_76, pll_clk_enable_15, n25370, spi1_sck_c_enable_129, 
        n26054, pll_clk_enable_439;
    wire [23:0]shift_register_23__N_3134;
    
    wire n18521, n25471, n25891, n25890, n26053, n12909, n12908, 
        n12907, n12906, n12905, n12904, n12903, n12902, n12901, 
        n12900, n12899, n12898, n12897, n12896, n12895, n12894, 
        n12892, n12891, n12890, n12889, n12888, n12887, n12886, 
        n12885, n12884, n12883, n12882, n12881, n12880, n25543, 
        n25537, pll_clk_enable_271, n25722, n18604, n25525, n18621, 
        n18625, pll_clk_enable_504, n25712, n25711, n25710, n26050, 
        pll_clk_enable_776, n25517, n159_adj_3420, n25709, pll_clk_enable_730, 
        pll_clk_enable_104, spi1_sck_c_enable_199, n8_adj_3421, spi1_sck_c_enable_114, 
        pll_clk_enable_661, n5_adj_3422, n24852, n26049, n6_adj_3423, 
        n24823, n25708, pll_clk_enable_351, n25707, n26103, n25706, 
        n180, n26102, n24851, n17439, n25705, pll_clk_enable_16, 
        n121, n26101, n25704, n24850, n10, n24849, n25703, n25702, 
        n18417, n24822, pll_clk_enable_771, pll_clk_enable_473, n25701, 
        n17658, n25700, n25699, n25698, n17643, n24848, n25697, 
        n26100, n25696, n25695, n25694, n38_adj_3424, n24821, n24847, 
        n25693, n26099, n25692, n36_adj_3425, n26047, n25691, n25690, 
        n25689, n25688, n24846, n26_adj_3426, n26046, n25687, n26098, 
        n26097, n25686, pll_clk_enable_753, n24845, n25685, n24844, 
        n25684, n25683, n26045, n26030, n52, n25350, n25764, n25678, 
        n24820, n24843, n25675, n25674, n25673, n24_adj_3427, n24842, 
        n22_adj_3428, n25672, n25671, spi1_sck_c_enable_37, n24819, 
        n25670, n24773, n24841, n25669, n25668, n18_adj_3429, n26044, 
        n26096, n25667, n12_adj_3430, n26095, n23272, n25666, n26094, 
        n25665, n25664, n25663, n26104, n26092, n25662, n25661, 
        n25660, n25659, n25658, n24818, n25657, n24839, n25433, 
        n26043, pll_clk_enable_737, n25656, pll_clk_enable_375, n25655, 
        n25654, n25637, n24817, n25480, n24816, spi1_sck_c_enable_185, 
        n25472, n25931, n26090, n24772, n25468, n24838, n24815, 
        n24814, n24837, n24813, n24812, n26089, n26088, n24811, 
        n24793, n24792, n24810, n24791, n24790, n24835, n24809, 
        n24789, n24788, n24787, n24786, spi1_sck_c_enable_189, n16052, 
        pll_clk_enable_29, n25924, pll_clk_enable_22, n26086, n24834, 
        n14_adj_3431, n25653, n25652, n14617, n24833, n25345, n24807, 
        n24806, n24832, spi1_sck_c_enable_91, n24805, spi1_sck_c_enable_122, 
        n24831, n24785, n24771, n24784, pll_clk_enable_633, n24769, 
        n24768, spi1_sck_c_enable_106, n25408, pll_clk_enable_30, n24770, 
        n24783, n24782, n26083, n24781, n24767, n24766, n24764, 
        n24757, n24780, n24830, n24779, n24762, n26040, n24760, 
        n24759, n24756, n24761, n24763, n24778, n24777, n24755, 
        n24758, n24623, n24754, n26082, n10_adj_3432, n24622, n24620, 
        n24621, pll_clk_enable_773, n24776, pll_clk_enable_28, n26080, 
        n25591, n26255, n25651, pll_clk_enable_39, pll_clk_enable_139, 
        pll_clk_enable_765, n26250, n14245, n14243, n14241, n14239, 
        n14237, n14235, n14233, n14231, n14229, n14227, n14225, 
        n14223, n14221, n14219, n14217, n14215, n14213, n14211, 
        n14209, n14207, n14205, n14203, n14201, n14199, n14197, 
        n14195, n14193, n14191, n14189, n14187, n14185, n14183, 
        n14181, n14179, n14177, n14175, n14173, n14171, n14169, 
        n14167, n14165, n14163, n14161, n14159, n14157, n14155, 
        n14153, n14151, n14149, n14147, n14145, n14143, n14141, 
        n14139, n14137, n14135, n14133, n14131, n14129, n14127, 
        n14125, n14123, n14121, n14119, n14117, n14115, n14113, 
        n14111, n14109, n14107, n14105, n14103, n14101, n14099, 
        n14097, n14095, n14093, n14091, n14089, n14087, n14085, 
        n14083, n14081, n14058, spi1_sck_c_enable_152, spi1_sck_c_enable_144, 
        n26078, n26038, n26077, n26075, n25650, n25649, n25648, 
        n25647, n25646, n26074, n12_adj_3433, n24829, n26072, pll_clk_enable_775, 
        n26071, n26069, n25503, n26068, n26067, n26066, n26065, 
        n26011;
    
    VHI i2 (.Z(VCC_net));
    INV i14466 (.A(spi_mic_sck_c), .Z(sck_N_3271));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(71[24:35])
    LUT4 i1_3_lut_4_lut (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[42]), 
         .D(ev_bit[42]), .Z(ev_wr_data[42])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut.init = 16'hddd0;
    FD1S3AX mem_1530 (.D(staging_rd_addr_6__N_914[6]), .CK(pll_clk), .Q(n12892));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mem_1530.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i26 (.D(spi_frame_sequence[26]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[26]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i26.GSR = "DISABLED";
    FD1S3AX mem_1523 (.D(staging_rd_addr_6__N_914[3]), .CK(pll_clk), .Q(n12886));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mem_1523.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i25 (.D(spi_frame_sequence[25]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[25]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i25.GSR = "DISABLED";
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
    LUT4 i1_3_lut_4_lut_adj_51 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[43]), 
         .D(ev_bit[43]), .Z(ev_wr_data[43])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_51.init = 16'hddd0;
    FD1P3AX accepted_sequence_spi_i0_i24 (.D(spi_frame_sequence[24]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i24.GSR = "DISABLED";
    L6MUX21 i14251 (.D0(n25705), .D1(n25706), .SD(n26072), .Z(n25710));
    FD1P3AX accepted_sequence_spi_i0_i23 (.D(spi_frame_sequence[23]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[23]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i23.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_52 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[44]), 
         .D(ev_bit[44]), .Z(ev_wr_data[44])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_52.init = 16'hddd0;
    FD1S3AX spi_rx_shift_i1 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(spi_rx_shift[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_rx_shift_i1.GSR = "ENABLED";
    LUT4 ev_state_3__N_2134_1__bdd_3_lut_14455 (.A(ev_state[1]), .B(ev_state[2]), 
         .C(ev_state_3__N_2142[1]), .Z(n26009)) /* synthesis lut_function=(!(A (B)+!A !(B+(C)))) */ ;
    defparam ev_state_3__N_2134_1__bdd_3_lut_14455.init = 16'h7676;
    LUT4 i13371_2_lut_rep_89_3_lut_4_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_185), 
         .C(spi_rgb_index[0]), .D(n25408), .Z(n26023)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(144[40] 147[81])
    defparam i13371_2_lut_rep_89_3_lut_4_lut.init = 16'h4000;
    FD1P3AX accepted_sequence_spi_i0_i22 (.D(spi_frame_sequence[22]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i22.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_53 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[45]), 
         .D(ev_bit[45]), .Z(ev_wr_data[45])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_53.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_54 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[46]), 
         .D(ev_bit[46]), .Z(ev_wr_data[46])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_54.init = 16'hddd0;
    LUT4 i1_3_lut (.A(n17439), .B(init_shadow[39]), .C(ev_bit[39]), .Z(n15830)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut.init = 16'hecec;
    LUT4 i1_3_lut_adj_55 (.A(n17439), .B(init_shadow[38]), .C(ev_bit[38]), 
         .Z(n15824)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_55.init = 16'hecec;
    CCU2D expected_next_15__I_0_632_7 (.A0(spi_rgb_payload_byte_N_2770[6]), 
          .B0(spi_extension_length[7]), .C0(GND_net), .D0(GND_net), .A1(spi1_mosi_c_0), 
          .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24756), .COUT(n24757), 
          .S0(expected_next[7]), .S1(expected_next[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[33] 308[86])
    defparam expected_next_15__I_0_632_7.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_632_7.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_632_7.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_632_7.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_56 (.A(n17439), .B(init_shadow[37]), .C(ev_bit[37]), 
         .Z(n15818)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_56.init = 16'hecec;
    LUT4 mux_662_Mux_17_i3_3_lut (.A(rgb_hold[1]), .B(shift_register[16]), 
         .C(state[1]), .Z(shift_register_23__N_3134[17])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_17_i3_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_57 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[47]), 
         .D(ev_bit[47]), .Z(ev_wr_data[47])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_57.init = 16'hddd0;
    LUT4 i1_2_lut_3_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_185), .C(frame_end), 
         .Z(spi1_sck_c_enable_170)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(144[40] 147[81])
    defparam i1_2_lut_3_lut.init = 16'h4040;
    LUT4 i1_3_lut_4_lut_adj_58 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[66]), 
         .D(ev_bit[66]), .Z(ev_wr_data[66])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_58.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_59 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[67]), 
         .D(ev_bit[67]), .Z(ev_wr_data[67])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_59.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_60 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[68]), 
         .D(ev_bit[68]), .Z(ev_wr_data[68])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_60.init = 16'hddd0;
    OB us_tx_pad_76 (.I(us_tx_c_76), .O(us_tx[76]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3IX ev_bit_i48 (.D(ev_bit[47]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i48.GSR = "DISABLED";
    FD1S3AX mem_1525 (.D(staging_rd_addr_6__N_914[4]), .CK(pll_clk), .Q(n12888));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mem_1525.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i0 (.D(ev_rd_data[0]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i0.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_61 (.A(n17439), .B(init_shadow[36]), .C(ev_bit[36]), 
         .Z(n15812)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_61.init = 16'hecec;
    LUT4 mux_662_Mux_18_i3_3_lut (.A(rgb_hold[2]), .B(shift_register[17]), 
         .C(state[1]), .Z(shift_register_23__N_3134[18])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_18_i3_3_lut.init = 16'hcaca;
    OB us_tx_pad_77 (.I(us_tx_c_77), .O(us_tx[77]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_78 (.I(us_tx_c_78), .O(us_tx[78]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_79 (.I(us_tx_c_79), .O(us_tx[79]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    LUT4 i1_3_lut_4_lut_adj_62 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[69]), 
         .D(ev_bit[69]), .Z(ev_wr_data[69])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_62.init = 16'hddd0;
    LUT4 i6_4_lut (.A(time_divider[2]), .B(n12), .C(time_divider[6]), 
         .D(time_divider[1]), .Z(pll_clk_enable_730)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut.init = 16'h8000;
    OB us_tx_pad_80 (.I(us_tx_c_80), .O(us_tx[80]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX accepted_sequence_spi_i0_i0 (.D(spi_frame_sequence[0]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i0.GSR = "DISABLED";
    LUT4 i5_4_lut (.A(time_divider[0]), .B(time_divider[5]), .C(time_divider[4]), 
         .D(time_divider[3]), .Z(n12)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i5_4_lut.init = 16'h8000;
    FD1P3IX init_shadow_i70 (.D(n16016), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i70.GSR = "DISABLED";
    LUT4 i1_3_lut_rep_90_4_lut (.A(n26098), .B(n26094), .C(n87), .D(n121), 
         .Z(n26024)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i1_3_lut_rep_90_4_lut.init = 16'h0010;
    FD1P3AX spi_command_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_command[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_command_i0_i0.GSR = "ENABLED";
    LUT4 i2_3_lut_rep_113_4_lut (.A(n26098), .B(n26094), .C(spi_byte_count[15]), 
         .D(spi_byte_count[10]), .Z(n26047)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i2_3_lut_rep_113_4_lut.init = 16'hfffe;
    LUT4 i3_4_lut (.A(n21573), .B(n26255), .C(phase_step_s3), .D(phase_step_s2), 
         .Z(n24911)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut.init = 16'hfffe;
    LUT4 mux_1255_i7_3_lut_4_lut (.A(ev_state[1]), .B(n26050), .C(build_sum[6]), 
         .D(build_phase[6]), .Z(ev_wr_addr_8__N_925[6])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam mux_1255_i7_3_lut_4_lut.init = 16'hf2d0;
    FD1P3AX spi_version_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_version[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_version_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_update_flags_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_76), 
            .CK(spi1_sck_c), .Q(spi_rgb_payload_byte_N_2770[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_update_flags_i0_i0.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut_adj_63 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[70]), 
         .D(ev_bit[70]), .Z(ev_wr_data[70])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_63.init = 16'hddd0;
    LUT4 i1_3_lut_adj_64 (.A(n17439), .B(init_shadow[35]), .C(ev_bit[35]), 
         .Z(n15806)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_64.init = 16'hecec;
    LUT4 mux_662_Mux_19_i3_3_lut (.A(rgb_hold[3]), .B(shift_register[18]), 
         .C(state[1]), .Z(shift_register_23__N_3134[19])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_19_i3_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_65 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[71]), 
         .D(ev_bit[71]), .Z(ev_wr_data[71])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_65.init = 16'hddd0;
    FD1P3AX spi_extension_length_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(expected_next[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_98), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_129), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_phase_pending_i0_i0.GSR = "ENABLED";
    FD1P3AX rgb_values_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_136), 
            .CK(spi1_sck_c), .Q(rgb_values[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i0.GSR = "DISABLED";
    FD1P3AX last_command_spi__i1 (.D(spi_command[0]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_command_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_command_spi__i1.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i0 (.D(spi_expected_length[0]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i0.GSR = "DISABLED";
    FD1P3AX accepted_update_flags_spi__i1 (.D(expected_next_15__N_1475[3]), 
            .SP(spi1_sck_c_enable_200), .CK(spi1_sck_c), .Q(accepted_update_flags_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_update_flags_spi__i1.GSR = "DISABLED";
    FD1S3JX ws2812_settle_i0_i0 (.D(n17643), .CK(pll_clk), .PD(pll_clk_enable_29), 
            .Q(ws2812_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ws2812_settle_i0_i0.GSR = "DISABLED";
    FD1S3AX phase_frac_i0 (.D(phase_frac_sum[0]), .CK(pll_clk), .Q(phase_frac[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i0.GSR = "DISABLED";
    FD1P3AX rgb_hold__i1 (.D(rgb_values[0]), .SP(pll_clk_enable_210), .CK(pll_clk), 
            .Q(rgb_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i1.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_66 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[72]), 
         .D(ev_bit[72]), .Z(ev_wr_data[72])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_66.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_67 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[74]), 
         .D(ev_bit[74]), .Z(ev_wr_data[74])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_67.init = 16'hddd0;
    FD1P3IX init_shadow_i69 (.D(n16010), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i69.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i0 (.D(spi_byte_count_15__N_1628[0]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i0.GSR = "ENABLED";
    FD1P3IX ev_bit_i47 (.D(ev_bit[46]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i47.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i21 (.D(spi_frame_sequence[21]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[21]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i20 (.D(spi_frame_sequence[20]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i20.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_68 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[75]), 
         .D(ev_bit[75]), .Z(ev_wr_data[75])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_68.init = 16'hddd0;
    LUT4 i14313_4_lut (.A(n26075), .B(status_bit_index[5]), .C(status_bit_index[3]), 
         .D(n6), .Z(spi1_sck_N_413_enable_7)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i14313_4_lut.init = 16'h7fff;
    FD1P3IX ev_bit_i46 (.D(ev_bit[45]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i46.GSR = "DISABLED";
    LUT4 i1_2_lut (.A(status_bit_index[4]), .B(status_bit_index[6]), .Z(n6)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_69 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[76]), 
         .D(ev_bit[76]), .Z(ev_wr_data[76])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_69.init = 16'hddd0;
    LUT4 i1_3_lut_adj_70 (.A(n17439), .B(init_shadow[34]), .C(ev_bit[34]), 
         .Z(n15800)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_70.init = 16'hecec;
    FD1P3AX accepted_sequence_spi_i0_i19 (.D(spi_frame_sequence[19]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[19]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i19.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_71 (.A(spi_byte_count[2]), .B(n26092), .C(spi_byte_count[5]), 
         .D(spi_byte_count[4]), .Z(n159_adj_3420)) /* synthesis lut_function=(!(A+((C (D)+!C !(D))+!B))) */ ;
    defparam i1_3_lut_4_lut_adj_71.init = 16'h0440;
    FD1P3IX us_tx__i1 (.D(n13469), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_0)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i1.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_72 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[77]), 
         .D(ev_bit[77]), .Z(ev_wr_data[77])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_72.init = 16'hddd0;
    PFUMX i14203 (.BLUT(n25646), .ALUT(n25647), .C0(n26080), .Z(n25662));
    FD1P3AX accepted_sequence_spi_i0_i18 (.D(spi_frame_sequence[18]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i18.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_adj_73 (.A(spi1_sck_c_enable_185), .B(spi_byte_count[0]), 
         .C(n25617), .Z(spi1_sck_c_enable_75)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_73.init = 16'h0808;
    LUT4 i1_3_lut_4_lut_adj_74 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[80]), 
         .D(ev_bit[80]), .Z(ev_wr_data[80])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_74.init = 16'hddd0;
    FD1P3IX ev_bit_i55 (.D(ev_bit[54]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i55.GSR = "DISABLED";
    FD1P3IX init_shadow_i68 (.D(n16004), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i68.GSR = "DISABLED";
    LUT4 mux_662_Mux_20_i3_3_lut (.A(rgb_hold[4]), .B(shift_register[19]), 
         .C(state[1]), .Z(shift_register_23__N_3134[20])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_20_i3_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_75 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[81]), 
         .D(ev_bit[81]), .Z(ev_wr_data[81])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_75.init = 16'hddd0;
    OB us_tx_pad_81 (.I(us_tx_c_81), .O(us_tx[81]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX status_hold__i1 (.D(accepted_sequence[24]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i1.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0 (.D(accepted_sequence_31__N_1135[0]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i0.GSR = "DISABLED";
    LUT4 mux_1255_i6_3_lut_4_lut (.A(ev_state[1]), .B(n26050), .C(build_sum[5]), 
         .D(build_phase[5]), .Z(ev_wr_addr_8__N_925[5])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam mux_1255_i6_3_lut_4_lut.init = 16'hf2d0;
    FD1S3AX phase_step_s1_496 (.D(phase_frac_sum[24]), .CK(pll_clk), .Q(phase_step_s1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_step_s1_496.GSR = "DISABLED";
    LUT4 n25675_bdd_3_lut_14399 (.A(n25675), .B(status_bit_index[4]), .C(n25674), 
         .Z(n25891)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam n25675_bdd_3_lut_14399.init = 16'he2e2;
    LUT4 i1_3_lut_adj_76 (.A(n17439), .B(init_shadow[33]), .C(ev_bit[33]), 
         .Z(n15794)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_76.init = 16'hecec;
    FD1S3AX phase_step_s2_497 (.D(phase_step_s1), .CK(pll_clk), .Q(phase_step_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_step_s2_497.GSR = "DISABLED";
    FD1S3AX phase_step_s3_500 (.D(phase_step_s2), .CK(pll_clk), .Q(phase_step_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_step_s3_500.GSR = "DISABLED";
    FD1S3AX swap_now_s3_501 (.D(pll_clk_enable_39), .CK(pll_clk), .Q(swap_now_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam swap_now_s3_501.GSR = "DISABLED";
    FD1S3AX active_bank_502 (.D(run_addr_s3_8__N_438[8]), .CK(pll_clk), 
            .Q(active_bank)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam active_bank_502.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i0 (.D(global_phase_s2[0]), .SP(pll_clk_enable_359), 
            .CK(pll_clk), .Q(run_addr_s3[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam run_addr_s3_i0.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i0 (.D(accepted_sequence_spi[0]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i0.GSR = "DISABLED";
    FD1S3AX phase_step_s4_507 (.D(phase_step_s3), .CK(pll_clk), .Q(phase_step_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_step_s4_507.GSR = "DISABLED";
    FD1S3AX swap_now_s4_508 (.D(swap_now_s3), .CK(pll_clk), .Q(swap_now_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam swap_now_s4_508.GSR = "DISABLED";
    FD1S3AX phase_step_s5_509 (.D(phase_step_s4), .CK(pll_clk), .Q(phase_step_s5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_step_s5_509.GSR = "DISABLED";
    FD1S3AX swap_now_s5_510 (.D(swap_now_s4), .CK(pll_clk), .Q(swap_now_s5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam swap_now_s5_510.GSR = "DISABLED";
    FD1S3AX frame_toggle_meta_515 (.D(frame_toggle_spi), .CK(pll_clk), .Q(frame_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam frame_toggle_meta_515.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_77 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[82]), 
         .D(ev_bit[82]), .Z(ev_wr_data[82])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_77.init = 16'hddd0;
    FD1S3AX frame_toggle_sync_516 (.D(frame_toggle_meta), .CK(pll_clk), 
            .Q(frame_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam frame_toggle_sync_516.GSR = "DISABLED";
    FD1S3AX stop_toggle_meta_517 (.D(stop_toggle_spi), .CK(pll_clk), .Q(stop_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam stop_toggle_meta_517.GSR = "DISABLED";
    FD1S3AX stop_toggle_sync_518 (.D(stop_toggle_meta), .CK(pll_clk), .Q(stop_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam stop_toggle_sync_518.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_meta_519 (.D(ws2812_toggle_spi), .CK(pll_clk), 
            .Q(ws2812_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ws2812_toggle_meta_519.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_sync_520 (.D(ws2812_toggle_meta), .CK(pll_clk), 
            .Q(ws2812_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ws2812_toggle_sync_520.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i0 (.D(accepted_sequence_meta[0]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i0.GSR = "DISABLED";
    FD1S3AX update_flags_meta_i1 (.D(accepted_update_flags_spi[1]), .CK(pll_clk), 
            .Q(update_flags_meta[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam update_flags_meta_i1.GSR = "DISABLED";
    FD1S3AX update_flags_sync_i1 (.D(update_flags_meta[1]), .CK(pll_clk), 
            .Q(update_flags_sync[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam update_flags_sync_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i0 (.D(mic_data_0_c), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i0.GSR = "DISABLED";
    FD1S3AX invalid_frame_meta_525 (.D(invalid_frame_spi), .CK(pll_clk), 
            .Q(invalid_frame_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam invalid_frame_meta_525.GSR = "DISABLED";
    FD1S3AX invalid_frame_sync_526 (.D(invalid_frame_meta), .CK(pll_clk), 
            .Q(invalid_frame_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam invalid_frame_sync_526.GSR = "DISABLED";
    FD1P3IX frame_req_529 (.D(n26250), .SP(pll_clk_enable_15), .CD(n16567), 
            .CK(pll_clk), .Q(frame_req)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam frame_req_529.GSR = "DISABLED";
    LUT4 i14320_2_lut (.A(spi_byte_count[0]), .B(n1), .Z(spi1_sck_c_enable_83)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i14320_2_lut.init = 16'h4444;
    FD1P3AX ws2812_enable_532 (.D(n26250), .SP(pll_clk_enable_16), .CK(pll_clk), 
            .Q(ws2812_enable)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ws2812_enable_532.GSR = "DISABLED";
    LUT4 mux_1255_i5_3_lut_4_lut (.A(ev_state[1]), .B(n26050), .C(build_sum[4]), 
         .D(build_phase[4]), .Z(ev_wr_addr_8__N_925[4])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam mux_1255_i5_3_lut_4_lut.init = 16'hf2d0;
    FD1P3AX ev_ch_i0 (.D(ev_ch_6__N_722[0]), .SP(pll_clk_enable_17), .CK(pll_clk), 
            .Q(ev_ch[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_ch_i0.GSR = "DISABLED";
    FD1P3AX build_phase_i0 (.D(staging_q[8]), .SP(pll_clk_enable_390), .CK(pll_clk), 
            .Q(build_phase[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_phase_i0.GSR = "DISABLED";
    FD1P3AX build_sum_i0 (.D(build_sum_8__N_2255[0]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_sum[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_sum_i0.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i0 (.D(ev_rd_data[0]), .SP(pll_clk_enable_439), .CK(pll_clk), 
            .Q(ev_rd_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i0.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i0 (.D(staging_rd_addr_6__N_914[0]), .CK(pll_clk), 
            .Q(staging_rd_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam staging_rd_addr_i0.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0 (.D(accepted_sequence_sync[0]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i0.GSR = "DISABLED";
    FD1P3AX ev_clear_done_546 (.D(ev_clear_done_N_2802), .SP(pll_clk_enable_22), 
            .CK(pll_clk), .Q(ev_state_3__N_2142[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_clear_done_546.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i1 (.D(mic_data_0_c), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i1.GSR = "DISABLED";
    FD1S3AX mic_tick_549 (.D(mic_tick_N_2800), .CK(pll_clk), .Q(mic_tick)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_tick_549.GSR = "DISABLED";
    FD1S3AX mic_clock_reg_551 (.D(mic_clk_N_2724), .CK(pll_clk), .Q(mic_clk_c)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_clock_reg_551.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_78 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[83]), 
         .D(ev_bit[83]), .Z(ev_wr_data[83])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_78.init = 16'hddd0;
    FD1P3AX mic_shift_1_r__i1 (.D(mic_data_1_c), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i0.GSR = "DISABLED";
    LUT4 i2_3_lut_4_lut (.A(spi_byte_count[15]), .B(n26068), .C(n26053), 
         .D(n26024), .Z(n22989)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i2_3_lut_4_lut.init = 16'h1000;
    FD1P3IX ev_bit_i0 (.D(n26250), .SP(pll_clk_enable_504), .CD(n26071), 
            .CK(pll_clk), .Q(ev_bit[0])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i0.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_79 (.A(n17439), .B(init_shadow[32]), .C(ev_bit[32]), 
         .Z(n15788)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_79.init = 16'hecec;
    FD1P3AX spi_expected_length_i0 (.D(expected_next[0]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i0.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i1 (.D(expected_next[1]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i1.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i2 (.D(expected_next[2]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[2])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i2.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i3 (.D(expected_next[3]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i3.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i4 (.D(expected_next[4]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i4.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i5 (.D(expected_next[5]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[5])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i5.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i6 (.D(expected_next[6]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i6.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i7 (.D(expected_next[7]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i7.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i8 (.D(expected_next[8]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i8.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i9 (.D(expected_next[9]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i9.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i10 (.D(expected_next[10]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i10.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i11 (.D(expected_next[11]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i11.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i12 (.D(expected_next[12]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i12.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i13 (.D(expected_next[13]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i13.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i14 (.D(expected_next[14]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i14.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i15 (.D(expected_next[15]), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .Q(spi_expected_length[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_expected_length_i15.GSR = "ENABLED";
    PFUMX i12 (.BLUT(n52), .ALUT(n25433), .C0(ev_state[3]), .Z(ev_state_3__N_710[2]));
    LUT4 i3_4_lut_adj_80 (.A(n26028), .B(spi_byte_count[4]), .C(spi_byte_count[5]), 
         .D(n26066), .Z(n1)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(336[17] 401[24])
    defparam i3_4_lut_adj_80.init = 16'h2000;
    LUT4 i14318_2_lut (.A(spi_byte_count[1]), .B(n25468), .Z(spi1_sck_c_enable_98)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i14318_2_lut.init = 16'h2222;
    LUT4 i1_3_lut_adj_81 (.A(n17439), .B(init_shadow[31]), .C(ev_bit[31]), 
         .Z(n15782)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_81.init = 16'hecec;
    PFUMX i14204 (.BLUT(n25648), .ALUT(n25649), .C0(n26080), .Z(n25663));
    PFUMX i14205 (.BLUT(n25650), .ALUT(n25651), .C0(n26080), .Z(n25664));
    LUT4 i1_3_lut_adj_82 (.A(n17439), .B(init_shadow[30]), .C(ev_bit[30]), 
         .Z(n15776)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_82.init = 16'hecec;
    LUT4 i3_4_lut_rep_106 (.A(n26056), .B(ev_state[0]), .C(frame_req), 
         .D(swap_pending), .Z(n26040)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i3_4_lut_rep_106.init = 16'hfffe;
    LUT4 i1_3_lut_adj_83 (.A(n17439), .B(init_shadow[29]), .C(ev_bit[29]), 
         .Z(n15770)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_83.init = 16'hecec;
    LUT4 i14143_2_lut (.A(ev_state[1]), .B(ev_state[3]), .Z(n25601)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14143_2_lut.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_84 (.A(n15), .B(state[1]), .C(ws2812_enable), 
         .D(state[0]), .Z(pll_clk_enable_633)) /* synthesis lut_function=(!(A (B+!(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam i1_3_lut_4_lut_adj_84.init = 16'h7000;
    LUT4 i1_3_lut_adj_85 (.A(n17439), .B(init_shadow[28]), .C(ev_bit[28]), 
         .Z(n15764)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_85.init = 16'hecec;
    LUT4 i1_3_lut_adj_86 (.A(n17439), .B(init_shadow[27]), .C(ev_bit[27]), 
         .Z(n15758)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_86.init = 16'hecec;
    LUT4 i1_3_lut_adj_87 (.A(n17439), .B(init_shadow[26]), .C(ev_bit[26]), 
         .Z(n15752)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_87.init = 16'hecec;
    LUT4 i1_3_lut_adj_88 (.A(n17439), .B(init_shadow[25]), .C(ev_bit[25]), 
         .Z(n15746)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_88.init = 16'hecec;
    LUT4 i1_3_lut_adj_89 (.A(n17439), .B(init_shadow[24]), .C(ev_bit[24]), 
         .Z(n15740)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_89.init = 16'hecec;
    LUT4 mux_1547_i2_3_lut (.A(n12898), .B(n12899), .C(n12895), .Z(rd_data_15__N_2873[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i2_3_lut.init = 16'hcaca;
    LUT4 mux_1547_i3_3_lut (.A(n12900), .B(n12901), .C(n12895), .Z(rd_data_15__N_2873[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i3_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_adj_90 (.A(n17439), .B(init_shadow[23]), .C(ev_bit[23]), 
         .Z(n15734)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_90.init = 16'hecec;
    LUT4 i2_3_lut_4_lut_adj_91 (.A(pll_clk_enable_28), .B(n26057), .C(n25480), 
         .D(ev_state[1]), .Z(pll_clk_enable_375)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[17:25])
    defparam i2_3_lut_4_lut_adj_91.init = 16'hfffe;
    LUT4 i4597_4_lut (.A(ev_ch[4]), .B(staging_rd_addr[4]), .C(n18417), 
         .D(n26077), .Z(staging_rd_addr_6__N_914[4])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(565[9] 638[16])
    defparam i4597_4_lut.init = 16'hcac0;
    LUT4 mux_1547_i4_3_lut (.A(n12902), .B(n12903), .C(n12895), .Z(rd_data_15__N_2873[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i4_3_lut.init = 16'hcaca;
    LUT4 mux_1255_i4_3_lut_4_lut (.A(ev_state[1]), .B(n26050), .C(build_sum[3]), 
         .D(build_phase[3]), .Z(ev_wr_addr_8__N_925[3])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam mux_1255_i4_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i13363_3_lut_4_lut (.A(mic_sample_count[2]), .B(n26078), .C(mic_sample_count[3]), 
         .D(mic_sample_count[4]), .Z(n26_adj_3417)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam i13363_3_lut_4_lut.init = 16'h7f80;
    LUT4 i1_3_lut_adj_92 (.A(n17439), .B(init_shadow[22]), .C(ev_bit[22]), 
         .Z(n15728)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_92.init = 16'hecec;
    LUT4 sub_80_inv_0_i3_1_lut_rep_138 (.A(status_bit_index[2]), .Z(n26072)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(298[55:80])
    defparam sub_80_inv_0_i3_1_lut_rep_138.init = 16'h5555;
    LUT4 mux_1547_i5_3_lut (.A(n12904), .B(n12905), .C(n12895), .Z(rd_data_15__N_2873[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i5_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_adj_93 (.A(n17439), .B(init_shadow[21]), .C(ev_bit[21]), 
         .Z(n15722)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_93.init = 16'hecec;
    LUT4 mux_1547_i6_3_lut (.A(n12906), .B(n12907), .C(n12895), .Z(rd_data_15__N_2873[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i6_3_lut.init = 16'hcaca;
    LUT4 mux_1547_i7_3_lut (.A(n12908), .B(n12909), .C(n12895), .Z(rd_data_15__N_2873[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i7_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut (.A(spi_byte_count[0]), .B(n19), .C(n26028), .D(n21575), 
         .Z(n25468)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i1_4_lut.init = 16'hffbf;
    L6MUX21 i14215 (.D0(n25670), .D1(n25671), .SD(spi1_miso_N_2715[3]), 
            .Z(n25674));
    LUT4 mux_1547_i8_3_lut (.A(n12910), .B(n12911), .C(n12895), .Z(rd_data_15__N_2873[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i8_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_adj_94 (.A(n17439), .B(init_shadow[20]), .C(ev_bit[20]), 
         .Z(n15716)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_94.init = 16'hecec;
    L6MUX21 i14216 (.D0(n25672), .D1(n25673), .SD(spi1_miso_N_2715[3]), 
            .Z(n25675));
    L6MUX21 i14252 (.D0(n25707), .D1(n25708), .SD(spi1_miso_N_2715[3]), 
            .Z(n25711));
    LUT4 n25405_bdd_4_lut (.A(n26054), .B(n78), .C(status_bit_index[4]), 
         .D(status_hold[88]), .Z(n25894)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;
    defparam n25405_bdd_4_lut.init = 16'hcac0;
    LUT4 i1_3_lut_adj_95 (.A(n17439), .B(init_shadow[19]), .C(ev_bit[19]), 
         .Z(n15710)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_95.init = 16'hecec;
    LUT4 mux_1547_i9_3_lut (.A(n12912), .B(n12913), .C(n12895), .Z(rd_data_15__N_2873[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i9_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_adj_96 (.A(n17439), .B(init_shadow[18]), .C(ev_bit[18]), 
         .Z(n15704)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_96.init = 16'hecec;
    LUT4 i2_4_lut (.A(n25637), .B(n23272), .C(n38_adj_3424), .D(spi_command[4]), 
         .Z(n17384)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i2_4_lut.init = 16'h1000;
    L6MUX21 i14253 (.D0(n25709), .D1(n25710), .SD(spi1_miso_N_2715[3]), 
            .Z(n25712));
    LUT4 i13318_1_lut (.A(spi_bit_count[0]), .Z(n20)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam i13318_1_lut.init = 16'h5555;
    LUT4 i14179_4_lut (.A(n25629), .B(spi_extension_length[12]), .C(n25543), 
         .D(spi_extension_length[14]), .Z(n25637)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14179_4_lut.init = 16'hfffe;
    LUT4 i18_4_lut (.A(spi_version[7]), .B(n36_adj_3425), .C(n25537), 
         .D(spi_extension_length[9]), .Z(n38_adj_3424)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i18_4_lut.init = 16'h0004;
    LUT4 i1650_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n26062), .C(ev_ch[5]), 
         .D(ev_ch[4]), .Z(ev_ch_6__N_2150[5])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(606[27:39])
    defparam i1650_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i10147_2_lut (.A(spi_byte_count[5]), .B(spi_byte_count[4]), .Z(n21575)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10147_2_lut.init = 16'heeee;
    LUT4 i14_1_lut_4_lut (.A(n26056), .B(ev_state[0]), .C(frame_req), 
         .D(swap_pending), .Z(fifo_credit_wire[0])) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i14_1_lut_4_lut.init = 16'h0001;
    LUT4 i1_3_lut_adj_97 (.A(n17439), .B(init_shadow[17]), .C(ev_bit[17]), 
         .Z(n15698)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_97.init = 16'hecec;
    LUT4 i14171_4_lut (.A(spi_version[2]), .B(spi_extension_length[8]), 
         .C(spi_extension_length[15]), .D(spi_extension_length[10]), .Z(n25629)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14171_4_lut.init = 16'hfffe;
    PFUMX i14206 (.BLUT(n25652), .ALUT(n25653), .C0(n26080), .Z(n25665));
    LUT4 i1_3_lut_adj_98 (.A(n17439), .B(init_shadow[65]), .C(ev_bit[65]), 
         .Z(n15986)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_98.init = 16'hecec;
    LUT4 build_phase_7__I_0_i7_3_lut_4_lut (.A(n26102), .B(n26074), .C(build_phase[6]), 
         .D(build_sum[6]), .Z(ev_rd_slot[6])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam build_phase_7__I_0_i7_3_lut_4_lut.init = 16'hf1e0;
    LUT4 build_phase_7__I_0_i2_3_lut_4_lut (.A(n26102), .B(n26074), .C(build_phase[1]), 
         .D(build_sum[1]), .Z(ev_rd_slot[1])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam build_phase_7__I_0_i2_3_lut_4_lut.init = 16'hf1e0;
    OB us_tx_pad_82 (.I(us_tx_c_82), .O(us_tx[82]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    LUT4 i3_4_lut_adj_99 (.A(ev_ch[3]), .B(ev_ch[5]), .C(ev_ch[2]), .D(n25613), 
         .Z(ev_state_3__N_2134[1])) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i3_4_lut_adj_99.init = 16'hfeff;
    LUT4 i1_3_lut_adj_100 (.A(n17439), .B(init_shadow[64]), .C(ev_bit[64]), 
         .Z(n15980)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_100.init = 16'hecec;
    LUT4 i1_3_lut_adj_101 (.A(n17439), .B(init_shadow[16]), .C(ev_bit[16]), 
         .Z(n15692)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_101.init = 16'hecec;
    LUT4 mux_1547_i10_3_lut (.A(n12914), .B(n12915), .C(n12895), .Z(rd_data_15__N_2873[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i10_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_adj_102 (.A(n17439), .B(init_shadow[15]), .C(ev_bit[15]), 
         .Z(n15686)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_102.init = 16'hecec;
    LUT4 build_phase_7__I_0_i4_3_lut_4_lut (.A(n26102), .B(n26074), .C(build_phase[3]), 
         .D(build_sum[3]), .Z(ev_rd_slot[3])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam build_phase_7__I_0_i4_3_lut_4_lut.init = 16'hf1e0;
    LUT4 build_phase_7__I_0_i3_3_lut_4_lut (.A(n26102), .B(n26074), .C(build_phase[2]), 
         .D(build_sum[2]), .Z(ev_rd_slot[2])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam build_phase_7__I_0_i3_3_lut_4_lut.init = 16'hf1e0;
    LUT4 i1_3_lut_adj_103 (.A(n17439), .B(init_shadow[14]), .C(ev_bit[14]), 
         .Z(n15680)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_103.init = 16'hecec;
    LUT4 i4_4_lut (.A(n26063), .B(n26022), .C(spi_byte_count[15]), .D(n25591), 
         .Z(spi1_sck_c_enable_129)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i4_4_lut.init = 16'h0008;
    CCU2D fpga_time_1412_add_4_3 (.A0(fpga_time[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24809), .COUT(n24810), .S0(n164), .S1(n163));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_3.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_3.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_3.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_3.INJECT1_1 = "NO";
    CCU2D time_divider_1413_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(time_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24837), .S1(n40_adj_3414));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413_add_4_1.INIT0 = 16'hF000;
    defparam time_divider_1413_add_4_1.INIT1 = 16'h0555;
    defparam time_divider_1413_add_4_1.INJECT1_0 = "NO";
    defparam time_divider_1413_add_4_1.INJECT1_1 = "NO";
    CCU2D add_634_5 (.A0(phase_frac[3]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24783), .COUT(n24784), .S0(phase_frac_sum[3]), .S1(phase_frac_sum[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_5.INIT0 = 16'h5aaa;
    defparam add_634_5.INIT1 = 16'h5aaa;
    defparam add_634_5.INJECT1_0 = "NO";
    defparam add_634_5.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_104 (.A(n17439), .B(init_shadow[63]), .C(ev_bit[63]), 
         .Z(n15974)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_104.init = 16'hecec;
    CCU2D fpga_time_1412_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24809), .S1(n165));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_1.INIT0 = 16'hF000;
    defparam fpga_time_1412_add_4_1.INIT1 = 16'h0555;
    defparam fpga_time_1412_add_4_1.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_1.INJECT1_1 = "NO";
    LUT4 build_phase_7__I_0_i5_3_lut_4_lut (.A(n26102), .B(n26074), .C(build_phase[4]), 
         .D(build_sum[4]), .Z(ev_rd_slot[4])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam build_phase_7__I_0_i5_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_1547_i11_3_lut (.A(n12916), .B(n12917), .C(n12895), .Z(rd_data_15__N_2873[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i11_3_lut.init = 16'hcaca;
    CCU2D spi_channel_index_1407_add_4_7 (.A0(spi_channel_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24807), .S0(n35), .S1(n34));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407_add_4_7.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1407_add_4_7.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1407_add_4_7.INJECT1_0 = "NO";
    defparam spi_channel_index_1407_add_4_7.INJECT1_1 = "NO";
    LUT4 build_phase_7__I_0_i6_3_lut_4_lut (.A(n26102), .B(n26074), .C(build_phase[5]), 
         .D(build_sum[5]), .Z(ev_rd_slot[5])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam build_phase_7__I_0_i6_3_lut_4_lut.init = 16'hf1e0;
    CCU2D add_634_3 (.A0(phase_frac[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24782), .COUT(n24783), .S0(phase_frac_sum[1]), .S1(phase_frac_sum[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_3.INIT0 = 16'h5aaa;
    defparam add_634_3.INIT1 = 16'h5aaa;
    defparam add_634_3.INJECT1_0 = "NO";
    defparam add_634_3.INJECT1_1 = "NO";
    CCU2D sub_1246_add_2_7 (.A0(spi_byte_count[8]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[9]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24763), .COUT(n24764));
    defparam sub_1246_add_2_7.INIT0 = 16'h5555;
    defparam sub_1246_add_2_7.INIT1 = 16'h5555;
    defparam sub_1246_add_2_7.INJECT1_0 = "NO";
    defparam sub_1246_add_2_7.INJECT1_1 = "NO";
    FD1S3AX mem_1517 (.D(staging_rd_addr_6__N_914[0]), .CK(pll_clk), .Q(n12880));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mem_1517.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i8_3_lut_4_lut (.A(n26102), .B(n26074), .C(build_phase[7]), 
         .D(build_sum[7]), .Z(ev_rd_slot[7])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam build_phase_7__I_0_i8_3_lut_4_lut.init = 16'hf1e0;
    LUT4 build_phase_7__I_0_i1_3_lut_4_lut (.A(n26102), .B(n26074), .C(build_phase[0]), 
         .D(build_sum[0]), .Z(ev_rd_slot[0])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam build_phase_7__I_0_i1_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX accepted_sequence_spi_i0_i17 (.D(spi_frame_sequence[17]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[17]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i16 (.D(spi_frame_sequence[16]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i16.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_105 (.A(n17439), .B(init_shadow[62]), .C(ev_bit[62]), 
         .Z(n15968)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_105.init = 16'hecec;
    LUT4 i1_3_lut_adj_106 (.A(n17439), .B(init_shadow[61]), .C(ev_bit[61]), 
         .Z(n15962)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_106.init = 16'hecec;
    FD1P3AX accepted_sequence_spi_i0_i15 (.D(spi_frame_sequence[15]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i15.GSR = "DISABLED";
    LUT4 i14155_4_lut (.A(ev_ch[0]), .B(ev_ch[1]), .C(ev_ch[4]), .D(ev_ch[6]), 
         .Z(n25613)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14155_4_lut.init = 16'h8000;
    FD1P3AX accepted_sequence_spi_i0_i14 (.D(spi_frame_sequence[14]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i14.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_107 (.A(n17439), .B(init_shadow[60]), .C(ev_bit[60]), 
         .Z(n15956)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_107.init = 16'hecec;
    OB us_tx_pad_83 (.I(us_tx_c_83), .O(us_tx[83]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX accepted_sequence_spi_i0_i13 (.D(spi_frame_sequence[13]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i12 (.D(spi_frame_sequence[12]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i11 (.D(spi_frame_sequence[11]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i11.GSR = "DISABLED";
    LUT4 n25675_bdd_3_lut_14378 (.A(n25712), .B(status_bit_index[4]), .C(n25711), 
         .Z(n25890)) /* synthesis lut_function=(A ((C)+!B)+!A (B (C))) */ ;
    defparam n25675_bdd_3_lut_14378.init = 16'he2e2;
    FD1P3AX accepted_sequence_spi_i0_i10 (.D(spi_frame_sequence[10]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i9 (.D(spi_frame_sequence[9]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i8 (.D(spi_frame_sequence[8]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i8.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_108 (.A(n17439), .B(init_shadow[59]), .C(ev_bit[59]), 
         .Z(n15950)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_108.init = 16'hecec;
    FD1P3AX accepted_sequence_spi_i0_i7 (.D(spi_frame_sequence[7]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i6 (.D(spi_frame_sequence[6]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i6.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_109 (.A(n17439), .B(init_shadow[58]), .C(ev_bit[58]), 
         .Z(n15944)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_109.init = 16'hecec;
    OBZ spi1_miso_pad (.I(spi1_miso_N_2714), .T(fpga_cs_n_c), .O(spi1_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(298[12:21])
    FD1P3AX accepted_sequence_spi_i0_i5 (.D(spi_frame_sequence[5]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i5.GSR = "DISABLED";
    FD1P3AX stop_toggle_seen_535 (.D(stop_toggle_sync), .SP(pll_clk_enable_28), 
            .CK(pll_clk), .Q(stop_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam stop_toggle_seen_535.GSR = "DISABLED";
    FD1P3AX ws2812_toggle_seen_533 (.D(ws2812_toggle_sync), .SP(pll_clk_enable_29), 
            .CK(pll_clk), .Q(ws2812_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ws2812_toggle_seen_533.GSR = "DISABLED";
    FD1P3AX frame_toggle_seen_527 (.D(frame_toggle_sync), .SP(pll_clk_enable_30), 
            .CK(pll_clk), .Q(frame_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam frame_toggle_seen_527.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_110 (.A(n17439), .B(init_shadow[57]), .C(ev_bit[57]), 
         .Z(n15938)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_110.init = 16'hecec;
    LUT4 mux_1547_i12_3_lut (.A(n12918), .B(n12919), .C(n12895), .Z(rd_data_15__N_2873[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i12_3_lut.init = 16'hcaca;
    LUT4 mux_1255_i8_3_lut_4_lut (.A(ev_state[1]), .B(n26050), .C(build_sum[7]), 
         .D(build_phase[7]), .Z(ev_wr_addr_8__N_925[7])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam mux_1255_i8_3_lut_4_lut.init = 16'hf2d0;
    L6MUX21 i14211 (.D0(n25662), .D1(n25663), .SD(n26072), .Z(n25670));
    LUT4 i1_3_lut_adj_111 (.A(n17439), .B(init_shadow[56]), .C(ev_bit[56]), 
         .Z(n15932)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_111.init = 16'hecec;
    LUT4 i1_3_lut_adj_112 (.A(n17439), .B(init_shadow[55]), .C(ev_bit[55]), 
         .Z(n15926)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_112.init = 16'hecec;
    LUT4 i1_3_lut_adj_113 (.A(n17439), .B(init_shadow[54]), .C(ev_bit[54]), 
         .Z(n15920)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_113.init = 16'hecec;
    LUT4 i1_3_lut_adj_114 (.A(n17439), .B(init_shadow[53]), .C(ev_bit[53]), 
         .Z(n15914)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_114.init = 16'hecec;
    LUT4 i1_3_lut_adj_115 (.A(n17439), .B(init_shadow[52]), .C(ev_bit[52]), 
         .Z(n15908)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_115.init = 16'hecec;
    FD1P3AX accepted_sequence_spi_i0_i4 (.D(spi_frame_sequence[4]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i3 (.D(spi_frame_sequence[3]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i3.GSR = "DISABLED";
    PFUMX i14207 (.BLUT(n25654), .ALUT(n25655), .C0(n26080), .Z(n25666));
    FD1P3AX accepted_sequence_spi_i0_i2 (.D(spi_frame_sequence[2]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i2.GSR = "DISABLED";
    LUT4 mux_1547_i13_3_lut (.A(n12920), .B(n12921), .C(n12895), .Z(rd_data_15__N_2873[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i13_3_lut.init = 16'hcaca;
    LUT4 i14085_3_lut (.A(n25472), .B(spi_version[3]), .C(spi_extension_length[5]), 
         .Z(n25543)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i14085_3_lut.init = 16'hecec;
    FD1P3AX accepted_sequence_spi_i0_i1 (.D(spi_frame_sequence[1]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i1.GSR = "DISABLED";
    PFUMX i14208 (.BLUT(n25656), .ALUT(n25657), .C0(n26080), .Z(n25667));
    L6MUX21 i14212 (.D0(n25664), .D1(n25665), .SD(n26072), .Z(n25671));
    FD1P3AX ev_run_hold_s5_i0_i83 (.D(ev_rd_data[83]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i83.GSR = "DISABLED";
    L6MUX21 i14213 (.D0(n25666), .D1(n25667), .SD(n26072), .Z(n25672));
    LUT4 i1_3_lut_adj_116 (.A(n17439), .B(init_shadow[13]), .C(ev_bit[13]), 
         .Z(n15674)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_116.init = 16'hecec;
    FD1P3IX init_shadow_i67 (.D(n15998), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i67.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i82 (.D(ev_rd_data[82]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i82.GSR = "DISABLED";
    FD1P3IX init_shadow_i66 (.D(n15992), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i66.GSR = "DISABLED";
    L6MUX21 i14214 (.D0(n25668), .D1(n25669), .SD(n26072), .Z(n25673));
    FD1P3AX ev_run_hold_s5_i0_i81 (.D(ev_rd_data[81]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i81.GSR = "DISABLED";
    L6MUX21 i14248 (.D0(n25699), .D1(n25700), .SD(n26072), .Z(n25707));
    PFUMX i14209 (.BLUT(n25658), .ALUT(n25659), .C0(n26080), .Z(n25668));
    LUT4 i1_3_lut_adj_117 (.A(n17439), .B(init_shadow[12]), .C(ev_bit[12]), 
         .Z(n15668)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_117.init = 16'hecec;
    LUT4 i1_3_lut_adj_118 (.A(n17439), .B(init_shadow[70]), .C(ev_bit[70]), 
         .Z(n16016)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_118.init = 16'hecec;
    LUT4 i1_2_lut_rep_104_4_lut (.A(n26063), .B(spi1_sck_c_enable_185), 
         .C(spi_channel_field[0]), .D(spi_byte_count[15]), .Z(n26038)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam i1_2_lut_rep_104_4_lut.init = 16'h0080;
    LUT4 i2_3_lut (.A(ev_state[1]), .B(build_sum[8]), .C(ev_state[0]), 
         .Z(n17439)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i2_3_lut.init = 16'h4040;
    LUT4 ws2812_settle_3__bdd_4_lut (.A(ws2812_settle[3]), .B(ws2812_settle[2]), 
         .C(ws2812_settle[0]), .D(ws2812_settle[1]), .Z(n26104)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;
    defparam ws2812_settle_3__bdd_4_lut.init = 16'haaa9;
    LUT4 n25405_bdd_3_lut_4_lut (.A(status_bit_index[3]), .B(n26075), .C(status_hold[104]), 
         .D(status_bit_index[4]), .Z(n25893)) /* synthesis lut_function=(!(A+!(B (C+!(D))))) */ ;
    defparam n25405_bdd_3_lut_4_lut.init = 16'h4044;
    L6MUX21 i14249 (.D0(n25701), .D1(n25702), .SD(n26072), .Z(n25708));
    FD1P3AX ev_run_hold_s5_i0_i80 (.D(ev_rd_data[80]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i80.GSR = "DISABLED";
    LUT4 run_addr_s3_8__I_0_i3_3_lut (.A(run_addr_s3[2]), .B(ev_rd_slot[2]), 
         .C(n24911), .Z(event_rd_addr[2])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(231[33:74])
    defparam run_addr_s3_8__I_0_i3_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_adj_119 (.A(n17439), .B(init_shadow[51]), .C(ev_bit[51]), 
         .Z(n15902)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_119.init = 16'hecec;
    L6MUX21 i14250 (.D0(n25703), .D1(n25704), .SD(n26072), .Z(n25709));
    LUT4 i2_3_lut_4_lut_adj_120 (.A(spi_byte_count[15]), .B(n26053), .C(n26068), 
         .D(n26024), .Z(spi1_sck_c_enable_195)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam i2_3_lut_4_lut_adj_120.init = 16'h0400;
    LUT4 i13340_1_lut (.A(mic_sample_count[0]), .Z(n30)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam i13340_1_lut.init = 16'h5555;
    LUT4 i14342_4_lut (.A(n26090), .B(n26030), .C(n4), .D(n25981), .Z(n25764)) /* synthesis lut_function=(!(A (B (C)+!B (C+(D)))+!A !(B+!(D)))) */ ;
    defparam i14342_4_lut.init = 16'h4c5f;
    LUT4 i14339_2_lut_rep_140 (.A(ev_state[0]), .B(ev_state[1]), .Z(n26074)) /* synthesis lut_function=(!(A (B))) */ ;
    defparam i14339_2_lut_rep_140.init = 16'h7777;
    FD1S3IX frame_settle__i0 (.D(n17639), .CK(pll_clk), .CD(n14617), .Q(frame_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam frame_settle__i0.GSR = "DISABLED";
    FD1P3AX frame_toggle_spi_489 (.D(frame_toggle_spi_N_2728), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(frame_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam frame_toggle_spi_489.GSR = "DISABLED";
    FD1S3AX mem_1527 (.D(staging_rd_addr_6__N_914[5]), .CK(pll_clk), .Q(n12890));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mem_1527.GSR = "DISABLED";
    PFUMX i14391 (.BLUT(n25904), .ALUT(n26021), .C0(status_bit_index[3]), 
          .Z(n78));
    LUT4 i1_3_lut_adj_121 (.A(n17439), .B(init_shadow[11]), .C(ev_bit[11]), 
         .Z(n15662)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_121.init = 16'hecec;
    LUT4 mux_662_Mux_1_i3_3_lut (.A(rgb_hold[17]), .B(shift_register[0]), 
         .C(state[1]), .Z(shift_register_23__N_3134[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_1_i3_3_lut.init = 16'hcaca;
    FD1P3IX ev_bit_i49 (.D(ev_bit[48]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i49.GSR = "DISABLED";
    LUT4 mux_1547_i14_3_lut (.A(n12922), .B(n12923), .C(n12895), .Z(rd_data_15__N_2873[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i14_3_lut.init = 16'hcaca;
    LUT4 mux_662_Mux_2_i3_3_lut (.A(rgb_hold[18]), .B(shift_register[1]), 
         .C(state[1]), .Z(shift_register_23__N_3134[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_2_i3_3_lut.init = 16'hcaca;
    LUT4 mux_662_Mux_3_i3_3_lut (.A(rgb_hold[19]), .B(shift_register[2]), 
         .C(state[1]), .Z(shift_register_23__N_3134[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_3_i3_3_lut.init = 16'hcaca;
    LUT4 i13377_2_lut_3_lut_4_lut (.A(n26064), .B(n25408), .C(spi_rgb_index[1]), 
         .D(spi_rgb_index[0]), .Z(n24)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[17:31])
    defparam i13377_2_lut_3_lut_4_lut.init = 16'h78f0;
    FD1P3IX running_504 (.D(n26250), .SP(pll_clk_enable_39), .CD(pll_clk_enable_28), 
            .CK(pll_clk), .Q(running)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam running_504.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i79 (.D(ev_rd_data[79]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i79.GSR = "DISABLED";
    LUT4 i2_3_lut_adj_122 (.A(n25617), .B(spi_byte_count[0]), .C(spi1_sck_c_enable_185), 
         .Z(spi1_sck_c_enable_68)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;
    defparam i2_3_lut_adj_122.init = 16'h1010;
    FD1P3IX init_shadow_i77 (.D(n16058), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i77.GSR = "DISABLED";
    LUT4 mux_662_Mux_4_i3_3_lut (.A(rgb_hold[20]), .B(shift_register[3]), 
         .C(state[1]), .Z(shift_register_23__N_3134[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_4_i3_3_lut.init = 16'hcaca;
    LUT4 mux_1547_i15_3_lut (.A(n12924), .B(n12925), .C(n12895), .Z(rd_data_15__N_2873[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i15_3_lut.init = 16'hcaca;
    FD1S3AX mem (.D(spi_phase_pending[7]), .CK(spi1_sck_c), .Q(n12927));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_adj_123 (.A(ev_state[0]), .B(ev_state[1]), .C(ev_state[2]), 
         .Z(n52)) /* synthesis lut_function=(!(A (B+!(C))+!A !(C))) */ ;
    defparam i1_2_lut_3_lut_adj_123.init = 16'h7070;
    LUT4 i14159_4_lut (.A(n25561), .B(n26095), .C(spi_byte_count[5]), 
         .D(n26094), .Z(n25617)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14159_4_lut.init = 16'hfffe;
    FD1S3AX mem_1547 (.D(spi_phase_pending[6]), .CK(spi1_sck_c), .Q(n12925));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1547.GSR = "DISABLED";
    FD1S3AX mem_1546 (.D(spi_phase_pending[5]), .CK(spi1_sck_c), .Q(n12923));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1546.GSR = "DISABLED";
    FD1S3AX mem_1545 (.D(spi_phase_pending[4]), .CK(spi1_sck_c), .Q(n12921));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1545.GSR = "DISABLED";
    FD1S3AX mem_1544 (.D(spi_phase_pending[3]), .CK(spi1_sck_c), .Q(n12919));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1544.GSR = "DISABLED";
    FD1S3AX mem_1543 (.D(spi_phase_pending[2]), .CK(spi1_sck_c), .Q(n12917));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1543.GSR = "DISABLED";
    FD1S3AX mem_1542 (.D(spi_phase_pending[1]), .CK(spi1_sck_c), .Q(n12915));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1542.GSR = "DISABLED";
    FD1S3AX mem_1541 (.D(spi_phase_pending[0]), .CK(spi1_sck_c), .Q(n12913));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1541.GSR = "DISABLED";
    FD1S3AX mem_1540 (.D(spi_rx_shift[6]), .CK(spi1_sck_c), .Q(n12911));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1540.GSR = "DISABLED";
    FD1S3AX mem_1539 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(n12909));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1539.GSR = "DISABLED";
    FD1S3AX mem_1538 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(n12907));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1538.GSR = "DISABLED";
    FD1S3AX mem_1537 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(n12905));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1537.GSR = "DISABLED";
    FD1S3AX mem_1536 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(n12903));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1536.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i78 (.D(ev_rd_data[78]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i78.GSR = "DISABLED";
    PFUMX i14210 (.BLUT(n25660), .ALUT(n25661), .C0(n26080), .Z(n25669));
    LUT4 mux_662_Mux_5_i3_3_lut (.A(rgb_hold[21]), .B(shift_register[4]), 
         .C(state[1]), .Z(shift_register_23__N_3134[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_5_i3_3_lut.init = 16'hcaca;
    LUT4 i14103_4_lut (.A(n26049), .B(n26100), .C(n26099), .D(n26098), 
         .Z(n25561)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14103_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_adj_124 (.A(n17439), .B(init_shadow[50]), .C(ev_bit[50]), 
         .Z(n15896)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_124.init = 16'hecec;
    LUT4 i1_3_lut_adj_125 (.A(n17439), .B(init_shadow[49]), .C(ev_bit[49]), 
         .Z(n15890)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_125.init = 16'hecec;
    LUT4 mux_662_Mux_6_i3_3_lut (.A(rgb_hold[22]), .B(shift_register[5]), 
         .C(state[1]), .Z(shift_register_23__N_3134[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_6_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_rep_103_3_lut_4_lut (.A(ev_state[3]), .B(n26097), .C(n26069), 
         .D(ev_state[0]), .Z(pll_clk_enable_775)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_rep_103_3_lut_4_lut.init = 16'hf1f0;
    LUT4 i16_4_lut (.A(spi_version[0]), .B(n25631), .C(n25555), .D(n26082), 
         .Z(n36_adj_3425)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i16_4_lut.init = 16'h0002;
    LUT4 i1_3_lut_adj_126 (.A(n17439), .B(init_shadow[48]), .C(ev_bit[48]), 
         .Z(n15884)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_126.init = 16'hecec;
    LUT4 mux_662_Mux_7_i3_3_lut (.A(rgb_hold[23]), .B(shift_register[6]), 
         .C(state[1]), .Z(shift_register_23__N_3134[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_7_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut_4_lut (.A(ev_state[3]), .B(n26097), .C(n26057), 
         .D(pll_clk_enable_28), .Z(pll_clk_enable_777)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_rep_109_4_lut (.A(n26103), .B(ev_state_3__N_2142[1]), 
         .C(ev_state[0]), .D(ev_state[1]), .Z(n26043)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[17:25])
    defparam i1_2_lut_rep_109_4_lut.init = 16'h00ca;
    LUT4 i1_3_lut_adj_127 (.A(n17439), .B(init_shadow[47]), .C(ev_bit[47]), 
         .Z(n15878)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_127.init = 16'hecec;
    LUT4 mux_662_Mux_8_i3_3_lut (.A(rgb_hold[8]), .B(shift_register[7]), 
         .C(state[1]), .Z(shift_register_23__N_3134[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_8_i3_3_lut.init = 16'hcaca;
    LUT4 mux_662_Mux_9_i3_3_lut (.A(rgb_hold[9]), .B(shift_register[8]), 
         .C(state[1]), .Z(shift_register_23__N_3134[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_9_i3_3_lut.init = 16'hcaca;
    FD1S3AX mem_1535 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(n12901));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1535.GSR = "DISABLED";
    FD1S3AX mem_1534 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(n12899));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1534.GSR = "DISABLED";
    FD1S3AX mem_1533 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(n12897));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1533.GSR = "DISABLED";
    FD1S3AX mem_1532 (.D(spi_write), .CK(pll_clk), .Q(n12894));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mem_1532.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i77 (.D(ev_rd_data[77]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i77.GSR = "DISABLED";
    FD1S3AX mem_1519 (.D(staging_rd_addr_6__N_914[1]), .CK(pll_clk), .Q(n12882));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mem_1519.GSR = "DISABLED";
    FD1S3AX mem_1521 (.D(staging_rd_addr_6__N_914[2]), .CK(pll_clk), .Q(n12884));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mem_1521.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_128 (.A(n17439), .B(init_shadow[46]), .C(ev_bit[46]), 
         .Z(n15872)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_128.init = 16'hecec;
    LUT4 i11_3_lut_4_lut (.A(ev_state[0]), .B(ev_state[1]), .C(ev_state[3]), 
         .D(n14_adj_3387), .Z(n25218)) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A (C (D))) */ ;
    defparam i11_3_lut_4_lut.init = 16'hf808;
    LUT4 i1_4_lut_adj_129 (.A(n26030), .B(n21575), .C(n159_adj_3420), 
         .D(n60), .Z(n121)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;
    defparam i1_4_lut_adj_129.init = 16'h5150;
    LUT4 i1_3_lut_adj_130 (.A(n17439), .B(init_shadow[45]), .C(ev_bit[45]), 
         .Z(n15866)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_130.init = 16'hecec;
    LUT4 i3099_2_lut (.A(spi_byte_count[0]), .B(n1), .Z(spi1_sck_c_enable_91)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i3099_2_lut.init = 16'h8888;
    LUT4 i14330_4_lut (.A(spi_byte_count[0]), .B(spi_byte_count[4]), .C(spi_byte_count[5]), 
         .D(n25517), .Z(spi1_sck_c_enable_76)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i14330_4_lut.init = 16'h0400;
    LUT4 i14079_2_lut (.A(spi_version[4]), .B(spi_extension_length[11]), 
         .Z(n25537)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14079_2_lut.init = 16'heeee;
    FD1P3AX ev_run_hold_s5_i0_i76 (.D(ev_rd_data[76]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i76.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i75 (.D(ev_rd_data[75]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i75.GSR = "DISABLED";
    LUT4 i2_3_lut_4_lut_adj_131 (.A(frame_settle[0]), .B(n26083), .C(pll_clk_enable_28), 
         .D(pll_clk_enable_30), .Z(pll_clk_enable_737)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[17:37])
    defparam i2_3_lut_4_lut_adj_131.init = 16'hfffe;
    LUT4 mux_662_Mux_10_i3_3_lut (.A(rgb_hold[10]), .B(shift_register[9]), 
         .C(state[1]), .Z(shift_register_23__N_3134[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_10_i3_3_lut.init = 16'hcaca;
    FD1P3AX ev_run_hold_s5_i0_i74 (.D(ev_rd_data[74]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i74.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i73 (.D(ev_rd_data[73]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i73.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i72 (.D(ev_rd_data[72]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i72.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i71 (.D(ev_rd_data[71]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i71.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i70 (.D(ev_rd_data[70]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i70.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i69 (.D(ev_rd_data[69]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i69.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i68 (.D(ev_rd_data[68]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i68.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i67 (.D(ev_rd_data[67]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i67.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_132 (.A(n17439), .B(init_shadow[44]), .C(ev_bit[44]), 
         .Z(n15860)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_132.init = 16'hecec;
    FD1P3AX ev_run_hold_s5_i0_i66 (.D(ev_rd_data[66]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i66.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i65 (.D(ev_rd_data[65]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i65.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i64 (.D(ev_rd_data[64]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i64.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i63 (.D(ev_rd_data[63]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i63.GSR = "DISABLED";
    LUT4 i752_2_lut_3_lut_4_lut (.A(ws2812_settle[0]), .B(n26086), .C(pll_clk_enable_15), 
         .D(pll_clk_enable_29), .Z(pll_clk_enable_210)) /* synthesis lut_function=(A (B (C)+!B (C+!(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(539[17:38])
    defparam i752_2_lut_3_lut_4_lut.init = 16'hf0f2;
    LUT4 i1_2_lut_3_lut_4_lut_adj_133 (.A(ws2812_settle[0]), .B(n26086), 
         .C(pll_clk_enable_39), .D(pll_clk_enable_29), .Z(pll_clk_enable_351)) /* synthesis lut_function=(A (B (C)+!B (C+!(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(539[17:38])
    defparam i1_2_lut_3_lut_4_lut_adj_133.init = 16'hf0f2;
    LUT4 i129_2_lut_rep_99_3_lut_4_lut (.A(spi_byte_count[2]), .B(n26100), 
         .C(n26095), .D(spi_byte_count[5]), .Z(n26033)) /* synthesis lut_function=(A (C+(D))+!A (B (C+(D))+!B (C))) */ ;
    defparam i129_2_lut_rep_99_3_lut_4_lut.init = 16'hfef0;
    LUT4 i1_3_lut_adj_134 (.A(n17439), .B(init_shadow[43]), .C(ev_bit[43]), 
         .Z(n15854)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_134.init = 16'hecec;
    LUT4 i2_3_lut_rep_141 (.A(status_bit_index[2]), .B(status_bit_index[0]), 
         .C(status_bit_index[1]), .Z(n26075)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(298[55:80])
    defparam i2_3_lut_rep_141.init = 16'h8080;
    LUT4 mux_662_Mux_11_i3_3_lut (.A(rgb_hold[11]), .B(shift_register[10]), 
         .C(state[1]), .Z(shift_register_23__N_3134[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_11_i3_3_lut.init = 16'hcaca;
    LUT4 mux_662_Mux_12_i3_3_lut (.A(rgb_hold[12]), .B(shift_register[11]), 
         .C(state[1]), .Z(shift_register_23__N_3134[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_12_i3_3_lut.init = 16'hcaca;
    LUT4 i1643_2_lut_3_lut_4_lut (.A(ev_ch[2]), .B(n26088), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(ev_ch_6__N_2150[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(606[27:39])
    defparam i1643_2_lut_3_lut_4_lut.init = 16'h78f0;
    FD1P3AX ev_run_hold_s5_i0_i62 (.D(ev_rd_data[62]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i62.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i61 (.D(ev_rd_data[61]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i61.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i60 (.D(ev_rd_data[60]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i60.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i59 (.D(ev_rd_data[59]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i59.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i58 (.D(ev_rd_data[58]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i58.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i57 (.D(ev_rd_data[57]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i57.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i56 (.D(ev_rd_data[56]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i56.GSR = "DISABLED";
    FD1P3AX spi_channel_index_1407__i0 (.D(n40), .SP(spi1_sck_c_enable_195), 
            .CK(spi1_sck_c), .Q(spi_channel_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407__i0.GSR = "ENABLED";
    FD1P3AX ev_run_hold_s5_i0_i55 (.D(ev_rd_data[55]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i55.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i54 (.D(ev_rd_data[54]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i54.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i53 (.D(ev_rd_data[53]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i53.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i52 (.D(ev_rd_data[52]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i52.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i51 (.D(ev_rd_data[51]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i51.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i0 (.D(n165), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i0.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i50 (.D(ev_rd_data[50]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i50.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i49 (.D(ev_rd_data[49]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i49.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i48 (.D(ev_rd_data[48]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i48.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i47 (.D(ev_rd_data[47]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i47.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i46 (.D(ev_rd_data[46]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i46.GSR = "DISABLED";
    FD1P3AX status_bit_index_1406__i0 (.D(n40_adj_3394), .SP(spi1_sck_N_413_enable_7), 
            .CK(spi1_sck_N_413), .Q(status_bit_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406__i0.GSR = "ENABLED";
    FD1P3AX ev_run_hold_s5_i0_i45 (.D(ev_rd_data[45]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i45.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i44 (.D(ev_rd_data[44]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i44.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i43 (.D(ev_rd_data[43]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i43.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i42 (.D(ev_rd_data[42]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i42.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i41 (.D(ev_rd_data[41]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i41.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1411__i0 (.D(n45), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411__i0.GSR = "DISABLED";
    FD1S3AX spi_bit_count_1410__i0 (.D(n20), .CK(spi1_sck_c), .Q(spi_bit_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam spi_bit_count_1410__i0.GSR = "ENABLED";
    CCU2D mic_divider_1415_add_4_7 (.A0(mic_divider[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24835), .S0(n35_adj_3403), .S1(n34_adj_3404));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_divider_1415_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_divider_1415_add_4_7.INJECT1_0 = "NO";
    defparam mic_divider_1415_add_4_7.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_135 (.A(n17439), .B(init_shadow[42]), .C(ev_bit[42]), 
         .Z(n15848)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_135.init = 16'hecec;
    LUT4 i2_3_lut_rep_119_4_lut (.A(spi_channel_field[1]), .B(n26090), .C(spi_channel_field[0]), 
         .D(spi1_sck_c_enable_185), .Z(n26053)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[78:102])
    defparam i2_3_lut_rep_119_4_lut.init = 16'h4000;
    LUT4 mux_662_Mux_13_i3_3_lut (.A(rgb_hold[13]), .B(shift_register[12]), 
         .C(state[1]), .Z(shift_register_23__N_3134[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_13_i3_3_lut.init = 16'hcaca;
    LUT4 mux_662_Mux_14_i3_3_lut (.A(rgb_hold[14]), .B(shift_register[13]), 
         .C(state[1]), .Z(shift_register_23__N_3134[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_14_i3_3_lut.init = 16'hcaca;
    LUT4 i13369_2_lut_3_lut_4_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_185), 
         .C(spi_rgb_index[0]), .D(n25408), .Z(n25)) /* synthesis lut_function=(A (C)+!A !(B (C (D)+!C !(D))+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(144[40] 147[81])
    defparam i13369_2_lut_3_lut_4_lut.init = 16'hb4f0;
    FD1P3IX ev_clear_addr_i7 (.D(ev_clear_addr_7__N_2439[7]), .SP(pll_clk_enable_775), 
            .CD(n18521), .CK(pll_clk), .Q(ev_clear_addr[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_clear_addr_i7.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i6 (.D(ev_clear_addr_7__N_2439[6]), .SP(pll_clk_enable_775), 
            .CD(n18521), .CK(pll_clk), .Q(ev_clear_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_clear_addr_i6.GSR = "DISABLED";
    LUT4 mux_662_Mux_15_i3_3_lut (.A(rgb_hold[15]), .B(shift_register[14]), 
         .C(state[1]), .Z(shift_register_23__N_3134[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_15_i3_3_lut.init = 16'hcaca;
    FD1P3IX ev_clear_addr_i5 (.D(ev_clear_addr_7__N_2439[5]), .SP(pll_clk_enable_775), 
            .CD(n18521), .CK(pll_clk), .Q(ev_clear_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_clear_addr_i5.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_136 (.A(n17439), .B(init_shadow[41]), .C(ev_bit[41]), 
         .Z(n15842)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_136.init = 16'hecec;
    LUT4 i1_3_lut_adj_137 (.A(n17439), .B(init_shadow[40]), .C(ev_bit[40]), 
         .Z(n15836)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_137.init = 16'hecec;
    LUT4 i1_2_lut_rep_91_3_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_185), 
         .C(n25408), .Z(n26025)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(144[40] 147[81])
    defparam i1_2_lut_rep_91_3_lut.init = 16'h4040;
    LUT4 mux_662_Mux_16_i3_3_lut (.A(rgb_hold[0]), .B(shift_register[15]), 
         .C(state[1]), .Z(shift_register_23__N_3134[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_662_Mux_16_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_rep_120_4_lut (.A(status_bit_index[2]), .B(status_bit_index[0]), 
         .C(status_bit_index[1]), .D(status_bit_index[3]), .Z(n26054)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(298[55:80])
    defparam i1_2_lut_rep_120_4_lut.init = 16'h0080;
    FD1P3IX init_shadow_i65 (.D(n15986), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i65.GSR = "DISABLED";
    FD1P3IX init_shadow_i64 (.D(n15980), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i64.GSR = "DISABLED";
    FD1P3IX init_shadow_i63 (.D(n15974), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i63.GSR = "DISABLED";
    LUT4 mux_1547_i16_3_lut (.A(n12926), .B(n12927), .C(n12895), .Z(rd_data_15__N_2873[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i16_3_lut.init = 16'hcaca;
    FD1P3IX init_shadow_i62 (.D(n15968), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i62.GSR = "DISABLED";
    FD1P3IX init_shadow_i61 (.D(n15962), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i61.GSR = "DISABLED";
    FD1P3IX init_shadow_i60 (.D(n15956), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i60.GSR = "DISABLED";
    FD1P3IX init_shadow_i59 (.D(n15950), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i59.GSR = "DISABLED";
    FD1P3IX init_shadow_i58 (.D(n15944), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i58.GSR = "DISABLED";
    FD1P3IX init_shadow_i57 (.D(n15938), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i57.GSR = "DISABLED";
    FD1P3IX init_shadow_i56 (.D(n15932), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i56.GSR = "DISABLED";
    FD1P3IX init_shadow_i55 (.D(n15926), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i55.GSR = "DISABLED";
    FD1P3IX init_shadow_i54 (.D(n15920), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i54.GSR = "DISABLED";
    FD1P3IX init_shadow_i53 (.D(n15914), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i53.GSR = "DISABLED";
    FD1P3IX init_shadow_i52 (.D(n15908), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i52.GSR = "DISABLED";
    FD1P3IX init_shadow_i51 (.D(n15902), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i51.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i40 (.D(ev_rd_data[40]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i40.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i39 (.D(ev_rd_data[39]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i39.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i38 (.D(ev_rd_data[38]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i38.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i37 (.D(ev_rd_data[37]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i37.GSR = "DISABLED";
    CCU2D expected_next_15__I_0_632_5 (.A0(spi_rgb_payload_byte_N_2770[6]), 
          .B0(spi_extension_length[5]), .C0(GND_net), .D0(GND_net), .A1(spi_rgb_payload_byte_N_2770[6]), 
          .B1(spi_extension_length[6]), .C1(GND_net), .D1(GND_net), .CIN(n24755), 
          .COUT(n24756), .S0(expected_next[5]), .S1(expected_next[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[33] 308[86])
    defparam expected_next_15__I_0_632_5.INIT0 = 16'ha999;
    defparam expected_next_15__I_0_632_5.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_632_5.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_632_5.INJECT1_1 = "NO";
    FD1P3AX ev_run_hold_s5_i0_i36 (.D(ev_rd_data[36]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i36.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i35 (.D(ev_rd_data[35]), .SP(pll_clk_enable_104), 
            .CK(pll_clk), .Q(ev_run_hold_s5[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i35.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i34 (.D(ev_rd_data[34]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i34.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i33 (.D(ev_rd_data[33]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i33.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i32 (.D(ev_rd_data[32]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i32.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i31 (.D(ev_rd_data[31]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i31.GSR = "DISABLED";
    FD1S3IX mic_divider_1415__i0 (.D(n40_adj_3398), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415__i0.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i30 (.D(ev_rd_data[30]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i30.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i29 (.D(ev_rd_data[29]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i29.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i28 (.D(ev_rd_data[28]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i28.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i27 (.D(ev_rd_data[27]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i27.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i26 (.D(ev_rd_data[26]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i26.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i25 (.D(ev_rd_data[25]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1414__i0 (.D(n30), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_sample_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam mic_sample_count_1414__i0.GSR = "DISABLED";
    FD1S3AX time_divider_1413__i0 (.D(n40_adj_3414), .CK(pll_clk), .Q(time_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413__i0.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i24 (.D(ev_rd_data[24]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i24.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i23 (.D(ev_rd_data[23]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i23.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i22 (.D(ev_rd_data[22]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i22.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i21 (.D(ev_rd_data[21]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i21.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i20 (.D(ev_rd_data[20]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i20.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i19 (.D(ev_rd_data[19]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i19.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i18 (.D(ev_rd_data[18]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i18.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i17 (.D(ev_rd_data[17]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i17.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i16 (.D(ev_rd_data[16]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i16.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i15 (.D(ev_rd_data[15]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i15.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i14 (.D(ev_rd_data[14]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i14.GSR = "DISABLED";
    LUT4 i13391_3_lut_4_lut (.A(spi_rgb_index[1]), .B(n26023), .C(spi_rgb_index[2]), 
         .D(spi_rgb_index[3]), .Z(n22)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(398[46:66])
    defparam i13391_3_lut_4_lut.init = 16'h7f80;
    FD1P3AX ev_run_hold_s5_i0_i13 (.D(ev_rd_data[13]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i13.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i12 (.D(ev_rd_data[12]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i12.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i11 (.D(ev_rd_data[11]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i11.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i10 (.D(ev_rd_data[10]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i10.GSR = "DISABLED";
    LUT4 run_addr_s3_8__I_0_i4_3_lut (.A(run_addr_s3[3]), .B(ev_rd_slot[3]), 
         .C(n24911), .Z(event_rd_addr[3])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(231[33:74])
    defparam run_addr_s3_8__I_0_i4_3_lut.init = 16'hacac;
    LUT4 i14173_4_lut (.A(spi_extension_length[7]), .B(spi_version[6]), 
         .C(spi_extension_length[6]), .D(spi_version[5]), .Z(n25631)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14173_4_lut.init = 16'hfffe;
    LUT4 i14097_2_lut (.A(spi_extension_length[13]), .B(spi_version[1]), 
         .Z(n25555)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14097_2_lut.init = 16'heeee;
    FD1P3AX ev_run_hold_s5_i0_i9 (.D(ev_rd_data[9]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i9.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i8 (.D(ev_rd_data[8]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i8.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i7 (.D(ev_rd_data[7]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i7.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i6 (.D(ev_rd_data[6]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i6.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i5 (.D(ev_rd_data[5]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i5.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i4 (.D(ev_rd_data[4]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i4.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i3 (.D(ev_rd_data[3]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i3.GSR = "DISABLED";
    FD1P3AX spi_rgb_index_1409__i0 (.D(n25), .SP(spi1_sck_c_enable_199), 
            .CK(spi1_sck_c), .Q(spi_rgb_index[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(398[46:66])
    defparam spi_rgb_index_1409__i0.GSR = "ENABLED";
    LUT4 i3_4_lut_adj_138 (.A(spi_extension_length[3]), .B(spi_extension_length[2]), 
         .C(spi_extension_length[4]), .D(n25471), .Z(n25472)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut_adj_138.init = 16'hfffe;
    OB us_tx_pad_75 (.I(us_tx_c_75), .O(us_tx[75]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX ev_run_hold_s5_i0_i2 (.D(ev_rd_data[2]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i2.GSR = "DISABLED";
    LUT4 i14333_2_lut (.A(spi_byte_count[1]), .B(n25468), .Z(spi1_sck_c_enable_114)) /* synthesis lut_function=(!(A+(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i14333_2_lut.init = 16'h1111;
    FD1P3AX ev_run_hold_s5_i0_i1 (.D(ev_rd_data[1]), .SP(pll_clk_enable_139), 
            .CK(pll_clk), .Q(ev_run_hold_s5[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_run_hold_s5_i0_i1.GSR = "DISABLED";
    FD1S3AX spi_rx_shift_i7 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(spi_rx_shift[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_rx_shift_i7.GSR = "ENABLED";
    FD1P3IX init_shadow_i50 (.D(n15896), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i50.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_139 (.A(n17439), .B(init_shadow[83]), .C(ev_bit[83]), 
         .Z(n16098)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_139.init = 16'hecec;
    LUT4 i2_3_lut_adj_140 (.A(spi_rgb_index[0]), .B(n180), .C(spi_rgb_index[1]), 
         .Z(spi1_sck_c_enable_136)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(398[46:66])
    defparam i2_3_lut_adj_140.init = 16'h0808;
    LUT4 i1_3_lut_adj_141 (.A(n17439), .B(init_shadow[82]), .C(ev_bit[82]), 
         .Z(n16092)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_141.init = 16'hecec;
    LUT4 i1_3_lut_adj_142 (.A(n17439), .B(init_shadow[2]), .C(ev_bit[2]), 
         .Z(n15608)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_142.init = 16'hecec;
    LUT4 i1_3_lut_adj_143 (.A(n17439), .B(init_shadow[1]), .C(ev_bit[1]), 
         .Z(n15602)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_143.init = 16'hecec;
    LUT4 i3_4_lut_adj_144 (.A(spi_byte_count[3]), .B(spi_byte_count[2]), 
         .C(spi_byte_count[1]), .D(n25350), .Z(spi1_sck_c_enable_122)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i3_4_lut_adj_144.init = 16'h0200;
    FD1P3IX init_shadow_i49 (.D(n15890), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i49.GSR = "DISABLED";
    FD1P3IX init_shadow_i48 (.D(n15884), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i48.GSR = "DISABLED";
    FD1P3IX init_shadow_i47 (.D(n15878), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i47.GSR = "DISABLED";
    FD1P3IX init_shadow_i46 (.D(n15872), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i46.GSR = "DISABLED";
    FD1P3IX init_shadow_i45 (.D(n15866), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i45.GSR = "DISABLED";
    FD1P3IX init_shadow_i44 (.D(n15860), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i44.GSR = "DISABLED";
    FD1P3IX init_shadow_i43 (.D(n15854), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i43.GSR = "DISABLED";
    FD1P3IX init_shadow_i42 (.D(n15848), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i42.GSR = "DISABLED";
    FD1P3IX init_shadow_i41 (.D(n15842), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i41.GSR = "DISABLED";
    FD1P3IX init_shadow_i40 (.D(n15836), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i40.GSR = "DISABLED";
    FD1P3IX init_shadow_i39 (.D(n15830), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i39.GSR = "DISABLED";
    FD1P3IX init_shadow_i38 (.D(n15824), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i38.GSR = "DISABLED";
    FD1P3IX init_shadow_i37 (.D(n15818), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i37.GSR = "DISABLED";
    FD1P3IX init_shadow_i36 (.D(n15812), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i36.GSR = "DISABLED";
    FD1P3IX init_shadow_i35 (.D(n15806), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i35.GSR = "DISABLED";
    FD1P3IX init_shadow_i34 (.D(n15800), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i34.GSR = "DISABLED";
    FD1P3IX init_shadow_i33 (.D(n15794), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i33.GSR = "DISABLED";
    FD1P3IX init_shadow_i32 (.D(n15788), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i32.GSR = "DISABLED";
    FD1P3IX init_shadow_i31 (.D(n15782), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i31.GSR = "DISABLED";
    FD1P3IX init_shadow_i30 (.D(n15776), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i30.GSR = "DISABLED";
    FD1P3IX init_shadow_i29 (.D(n15770), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i29.GSR = "DISABLED";
    FD1P3IX init_shadow_i28 (.D(n15764), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i28.GSR = "DISABLED";
    FD1P3IX init_shadow_i27 (.D(n15758), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i27.GSR = "DISABLED";
    FD1P3IX init_shadow_i26 (.D(n15752), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i26.GSR = "DISABLED";
    FD1P3IX init_shadow_i25 (.D(n15746), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i25.GSR = "DISABLED";
    FD1P3IX init_shadow_i24 (.D(n15740), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i24.GSR = "DISABLED";
    FD1P3IX init_shadow_i23 (.D(n15734), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i23.GSR = "DISABLED";
    FD1P3IX init_shadow_i22 (.D(n15728), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i22.GSR = "DISABLED";
    FD1P3IX init_shadow_i21 (.D(n15722), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i21.GSR = "DISABLED";
    FD1P3IX init_shadow_i20 (.D(n15716), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i20.GSR = "DISABLED";
    FD1P3IX init_shadow_i19 (.D(n15710), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i19.GSR = "DISABLED";
    FD1P3IX init_shadow_i18 (.D(n15704), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i18.GSR = "DISABLED";
    FD1P3IX init_shadow_i17 (.D(n15698), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i17.GSR = "DISABLED";
    FD1P3IX init_shadow_i16 (.D(n15692), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i16.GSR = "DISABLED";
    FD1P3IX init_shadow_i15 (.D(n15686), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i15.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_145 (.A(spi_command[3]), .B(spi_command[7]), .Z(n23272)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i1_2_lut_adj_145.init = 16'heeee;
    LUT4 i1_2_lut_rep_143 (.A(ev_state[1]), .B(ev_state[0]), .Z(n26077)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_rep_143.init = 16'h2222;
    LUT4 i1657_3_lut_4_lut (.A(ev_ch[4]), .B(n26046), .C(ev_ch[5]), .D(ev_ch[6]), 
         .Z(ev_ch_6__N_2150[6])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(606[27:39])
    defparam i1657_3_lut_4_lut.init = 16'h7f80;
    FD1P3IX init_shadow_i14 (.D(n15680), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i14.GSR = "DISABLED";
    FD1P3IX init_shadow_i13 (.D(n15674), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i13.GSR = "DISABLED";
    FD1P3IX init_shadow_i12 (.D(n15668), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i12.GSR = "DISABLED";
    LUT4 i13345_2_lut_rep_144 (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .Z(n26078)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam i13345_2_lut_rep_144.init = 16'h8888;
    FD1P3IX init_shadow_i11 (.D(n15662), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i11.GSR = "DISABLED";
    LUT4 i5_3_lut_4_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .C(mic_tick), .D(mic_sample_count[2]), .Z(n12_adj_3433)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam i5_3_lut_4_lut.init = 16'h8000;
    LUT4 i13356_2_lut_3_lut_4_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .C(mic_sample_count[3]), .D(mic_sample_count[2]), .Z(n27)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam i13356_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i4_4_lut_adj_146 (.A(fpga_cs_n_c), .B(n8), .C(n25347), .D(n25569), 
         .Z(n180)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:33])
    defparam i4_4_lut_adj_146.init = 16'h0040;
    LUT4 i13349_2_lut_3_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .C(mic_sample_count[2]), .Z(n28_adj_3415)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam i13349_2_lut_3_lut.init = 16'h7878;
    FD1S3AX phase_step_s4_507_rep_171 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(pll_clk_enable_139)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_step_s4_507_rep_171.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_147 (.A(n17439), .B(init_shadow[10]), .C(ev_bit[10]), 
         .Z(n15656)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_147.init = 16'hecec;
    LUT4 i1_2_lut_rep_145 (.A(swap_pending), .B(wrap_s2), .Z(pll_clk_enable_39)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_rep_145.init = 16'h8888;
    LUT4 i1_3_lut_adj_148 (.A(n17439), .B(init_shadow[9]), .C(ev_bit[9]), 
         .Z(n15650)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_148.init = 16'hecec;
    FD1S3AX spi_rx_shift_i6 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(spi_rx_shift[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_rx_shift_i6.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i5 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(spi_rx_shift[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_rx_shift_i5.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i4 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(spi_rx_shift[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_rx_shift_i4.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i3 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(spi_rx_shift[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_rx_shift_i3.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i2 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(spi_rx_shift[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_rx_shift_i2.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i28 (.D(spi_frame_sequence[28]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[28]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i28.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_149 (.A(n17439), .B(init_shadow[8]), .C(ev_bit[8]), 
         .Z(n15644)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_149.init = 16'hecec;
    LUT4 i6_2_lut_3_lut (.A(swap_pending), .B(wrap_s2), .C(active_bank), 
         .Z(run_addr_s3_8__N_438[8])) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i6_2_lut_3_lut.init = 16'h7878;
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
    LUT4 i3_3_lut (.A(n25408), .B(n121), .C(spi1_sck_c_enable_185), .Z(n8)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:33])
    defparam i3_3_lut.init = 16'h2020;
    FD1P3AX accepted_sequence_spi_i0_i29 (.D(spi_frame_sequence[29]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[29]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i30 (.D(spi_frame_sequence[30]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[30]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i31 (.D(spi_frame_sequence[31]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[31]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i31.GSR = "DISABLED";
    FD1P3AX spi_command_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_command[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_command_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_command[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_command_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_command[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_command_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_command[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_command_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_command[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_command_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_command[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_command_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .Q(spi_command[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_command_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_version[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_version_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_version[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_version_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_version[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_version_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_version[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_version_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_version[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_version_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_version[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_version_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_version[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_version_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_update_flags_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_76), 
            .CK(spi1_sck_c), .Q(expected_next_15__N_1475[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_update_flags_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(expected_next[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_extension_length[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_extension_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_extension_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_extension_length[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_extension_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_extension_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_91), 
            .CK(spi1_sck_c), .Q(spi_extension_length[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_91), 
            .CK(spi1_sck_c), .Q(spi_extension_length[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_91), 
            .CK(spi1_sck_c), .Q(spi_extension_length[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_91), 
            .CK(spi1_sck_c), .Q(spi_extension_length[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_91), 
            .CK(spi1_sck_c), .Q(spi_extension_length[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_91), 
            .CK(spi1_sck_c), .Q(spi_extension_length[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_91), 
            .CK(spi1_sck_c), .Q(spi_extension_length[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_91), 
            .CK(spi1_sck_c), .Q(spi_extension_length[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_extension_length_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_98), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_98), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_98), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_98), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_98), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_98), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_98), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_106), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_106), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_106), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_106), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_106), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_106), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_106), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_106), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_114), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i16.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_114), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i17.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_114), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i18.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_114), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i19.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_114), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i20.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_114), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i21.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_114), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i22.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_114), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i23.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i24 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i24.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i25 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i25.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i26 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i26.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i27 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i27.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i28 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i28.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i29 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i29.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i30 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i30.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i31 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_frame_sequence_i0_i31.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_129), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_phase_pending_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_129), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_phase_pending_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_129), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_phase_pending_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_129), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_phase_pending_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_129), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_phase_pending_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_129), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_phase_pending_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_129), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_phase_pending_i0_i7.GSR = "ENABLED";
    FD1P3AX rgb_values_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_136), 
            .CK(spi1_sck_c), .Q(rgb_values[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i1.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_136), 
            .CK(spi1_sck_c), .Q(rgb_values[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i2.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_136), 
            .CK(spi1_sck_c), .Q(rgb_values[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i3.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_136), 
            .CK(spi1_sck_c), .Q(rgb_values[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i4.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_136), 
            .CK(spi1_sck_c), .Q(rgb_values[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i5.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_136), 
            .CK(spi1_sck_c), .Q(rgb_values[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i6.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_136), 
            .CK(spi1_sck_c), .Q(rgb_values[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i7.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_144), 
            .CK(spi1_sck_c), .Q(rgb_values[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i8.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_144), 
            .CK(spi1_sck_c), .Q(rgb_values[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i9.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_144), 
            .CK(spi1_sck_c), .Q(rgb_values[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i10.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_144), 
            .CK(spi1_sck_c), .Q(rgb_values[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i11.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_144), 
            .CK(spi1_sck_c), .Q(rgb_values[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i12.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_144), 
            .CK(spi1_sck_c), .Q(rgb_values[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i13.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_144), 
            .CK(spi1_sck_c), .Q(rgb_values[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i14.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_144), 
            .CK(spi1_sck_c), .Q(rgb_values[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i15.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .Q(rgb_values[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i16.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .Q(rgb_values[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i17.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .Q(rgb_values[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i18.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .Q(rgb_values[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i19.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .Q(rgb_values[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i20.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .Q(rgb_values[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i21.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .Q(rgb_values[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i22.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .Q(rgb_values[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam rgb_values_i0_i23.GSR = "DISABLED";
    FD1P3AX last_command_spi__i2 (.D(spi_command[1]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_command_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_command_spi__i2.GSR = "DISABLED";
    LUT4 i3_4_lut_adj_150 (.A(spi_rgb_payload_byte_N_2770[6]), .B(n22776), 
         .C(n26033), .D(n25345), .Z(n25347)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(144[17:37])
    defparam i3_4_lut_adj_150.init = 16'hffdf;
    LUT4 i1_3_lut_adj_151 (.A(n17439), .B(init_shadow[7]), .C(ev_bit[7]), 
         .Z(n15638)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_151.init = 16'hecec;
    LUT4 i1_3_lut_adj_152 (.A(n17439), .B(init_shadow[6]), .C(ev_bit[6]), 
         .Z(n15632)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_152.init = 16'hecec;
    LUT4 i1_3_lut_adj_153 (.A(n17439), .B(init_shadow[5]), .C(ev_bit[5]), 
         .Z(n15626)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_153.init = 16'hecec;
    FD1P3AX last_command_spi__i3 (.D(spi_command[2]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_command_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_command_spi__i3.GSR = "DISABLED";
    FD1P3AX last_command_spi__i4 (.D(spi_command[3]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_command_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_command_spi__i4.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i1 (.D(spi_expected_length[1]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i1.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i2 (.D(spi_expected_length[2]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i2.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i3 (.D(spi_expected_length[3]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i3.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i4 (.D(spi_expected_length[4]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i5 (.D(spi_expected_length[5]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i5.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i6 (.D(spi_expected_length[6]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i6.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i7 (.D(spi_expected_length[7]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i7.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i8 (.D(spi_expected_length[8]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i8.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i9 (.D(spi_expected_length[9]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i10 (.D(spi_expected_length[10]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i10.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i11 (.D(spi_expected_length[11]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i11.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i12 (.D(spi_expected_length[12]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i13 (.D(spi_expected_length[13]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i14 (.D(spi_expected_length[14]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i14.GSR = "DISABLED";
    FD1P3AX last_length_spi_i0_i15 (.D(spi_expected_length[15]), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(last_length_spi[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam last_length_spi_i0_i15.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_154 (.A(n17439), .B(init_shadow[4]), .C(ev_bit[4]), 
         .Z(n15620)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_154.init = 16'hecec;
    FD1S3AX phase_frac_i1 (.D(phase_frac_sum[1]), .CK(pll_clk), .Q(phase_frac[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i1.GSR = "DISABLED";
    FD1S3AX phase_frac_i2 (.D(phase_frac_sum[2]), .CK(pll_clk), .Q(phase_frac[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i2.GSR = "DISABLED";
    FD1S3AX phase_frac_i3 (.D(phase_frac_sum[3]), .CK(pll_clk), .Q(phase_frac[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i3.GSR = "DISABLED";
    FD1S3AX phase_frac_i4 (.D(phase_frac_sum[4]), .CK(pll_clk), .Q(phase_frac[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i4.GSR = "DISABLED";
    FD1S3AX phase_frac_i5 (.D(phase_frac_sum[5]), .CK(pll_clk), .Q(phase_frac[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i5.GSR = "DISABLED";
    FD1S3AX phase_frac_i6 (.D(phase_frac_sum[6]), .CK(pll_clk), .Q(phase_frac[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i6.GSR = "DISABLED";
    FD1S3AX phase_frac_i7 (.D(phase_frac_sum[7]), .CK(pll_clk), .Q(phase_frac[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i7.GSR = "DISABLED";
    FD1S3AX phase_frac_i8 (.D(phase_frac_sum[8]), .CK(pll_clk), .Q(phase_frac[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i8.GSR = "DISABLED";
    FD1S3AX phase_frac_i9 (.D(phase_frac_sum[9]), .CK(pll_clk), .Q(phase_frac[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i9.GSR = "DISABLED";
    FD1S3AX phase_frac_i10 (.D(phase_frac_sum[10]), .CK(pll_clk), .Q(phase_frac[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i10.GSR = "DISABLED";
    FD1S3AX phase_frac_i11 (.D(phase_frac_sum[11]), .CK(pll_clk), .Q(phase_frac[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i11.GSR = "DISABLED";
    FD1S3AX phase_frac_i12 (.D(phase_frac_sum[12]), .CK(pll_clk), .Q(phase_frac[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i12.GSR = "DISABLED";
    FD1S3AX phase_frac_i13 (.D(phase_frac_sum[13]), .CK(pll_clk), .Q(phase_frac[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i13.GSR = "DISABLED";
    FD1S3AX phase_frac_i14 (.D(phase_frac_sum[14]), .CK(pll_clk), .Q(phase_frac[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i14.GSR = "DISABLED";
    FD1S3AX phase_frac_i15 (.D(phase_frac_sum[15]), .CK(pll_clk), .Q(phase_frac[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i15.GSR = "DISABLED";
    FD1S3AX phase_frac_i16 (.D(phase_frac_sum[16]), .CK(pll_clk), .Q(phase_frac[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i16.GSR = "DISABLED";
    FD1S3AX phase_frac_i17 (.D(phase_frac_sum[17]), .CK(pll_clk), .Q(phase_frac[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i17.GSR = "DISABLED";
    FD1S3AX phase_frac_i18 (.D(phase_frac_sum[18]), .CK(pll_clk), .Q(phase_frac[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i18.GSR = "DISABLED";
    FD1S3AX phase_frac_i19 (.D(phase_frac_sum[19]), .CK(pll_clk), .Q(phase_frac[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i19.GSR = "DISABLED";
    FD1S3AX phase_frac_i20 (.D(phase_frac_sum[20]), .CK(pll_clk), .Q(phase_frac[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i20.GSR = "DISABLED";
    FD1S3AX phase_frac_i21 (.D(phase_frac_sum[21]), .CK(pll_clk), .Q(phase_frac[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i21.GSR = "DISABLED";
    FD1S3AX phase_frac_i22 (.D(phase_frac_sum[22]), .CK(pll_clk), .Q(phase_frac[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i22.GSR = "DISABLED";
    FD1S3AX phase_frac_i23 (.D(phase_frac_sum[23]), .CK(pll_clk), .Q(phase_frac[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_frac_i23.GSR = "DISABLED";
    FD1P3AX rgb_hold__i2 (.D(rgb_values[1]), .SP(pll_clk_enable_210), .CK(pll_clk), 
            .Q(rgb_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i2.GSR = "DISABLED";
    FD1P3IX init_shadow_i10 (.D(n15656), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i10.GSR = "DISABLED";
    FD1P3IX init_shadow_i9 (.D(n15650), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i9.GSR = "DISABLED";
    FD1P3IX init_shadow_i8 (.D(n15644), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i8.GSR = "DISABLED";
    FD1P3IX init_shadow_i7 (.D(n15638), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i7.GSR = "DISABLED";
    FD1P3IX init_shadow_i6 (.D(n15632), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i6.GSR = "DISABLED";
    FD1P3IX init_shadow_i5 (.D(n15626), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i5.GSR = "DISABLED";
    FD1P3IX init_shadow_i4 (.D(n15620), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i4.GSR = "DISABLED";
    FD1P3IX init_shadow_i3 (.D(n15614), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i3.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_155 (.A(n17439), .B(init_shadow[3]), .C(ev_bit[3]), 
         .Z(n15614)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_155.init = 16'hecec;
    LUT4 i7_4_lut (.A(ev_run_hold_s5[1]), .B(us_tx_c_1), .C(init_shadow[1]), 
         .D(swap_now_s5), .Z(n14081)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut.init = 16'h5a66;
    LUT4 mux_1255_i2_3_lut_4_lut (.A(ev_state[1]), .B(n26050), .C(build_sum[1]), 
         .D(build_phase[1]), .Z(ev_wr_addr_8__N_925[1])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam mux_1255_i2_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i14325_2_lut_4_lut (.A(n26058), .B(spi_command[4]), .C(n26089), 
         .D(spi1_sck_c_enable_188), .Z(spi1_sck_c_enable_187)) /* synthesis lut_function=(A (D)+!A !(B (C+!(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i14325_2_lut_4_lut.init = 16'hbf00;
    LUT4 i14111_2_lut (.A(spi_rgb_index[3]), .B(spi_rgb_index[2]), .Z(n25569)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14111_2_lut.init = 16'heeee;
    LUT4 i7_4_lut_adj_156 (.A(ev_run_hold_s5[2]), .B(us_tx_c_2), .C(init_shadow[2]), 
         .D(swap_now_s5), .Z(n14083)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_156.init = 16'h5a66;
    LUT4 i1_2_lut_4_lut (.A(n26058), .B(spi_command[4]), .C(n26089), .D(stop_toggle_spi), 
         .Z(stop_toggle_spi_N_2738)) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i1_2_lut_4_lut.init = 16'hbf40;
    FD1P3AX rgb_hold__i3 (.D(rgb_values[2]), .SP(pll_clk_enable_210), .CK(pll_clk), 
            .Q(rgb_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i3.GSR = "DISABLED";
    FD1P3AX rgb_hold__i4 (.D(rgb_values[3]), .SP(pll_clk_enable_210), .CK(pll_clk), 
            .Q(rgb_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i4.GSR = "DISABLED";
    FD1P3AX rgb_hold__i5 (.D(rgb_values[4]), .SP(pll_clk_enable_210), .CK(pll_clk), 
            .Q(rgb_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i5.GSR = "DISABLED";
    FD1P3AX rgb_hold__i6 (.D(rgb_values[5]), .SP(pll_clk_enable_210), .CK(pll_clk), 
            .Q(rgb_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i6.GSR = "DISABLED";
    FD1P3AX rgb_hold__i7 (.D(rgb_values[6]), .SP(pll_clk_enable_210), .CK(pll_clk), 
            .Q(rgb_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i7.GSR = "DISABLED";
    FD1P3AX rgb_hold__i8 (.D(rgb_values[7]), .SP(pll_clk_enable_210), .CK(pll_clk), 
            .Q(rgb_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i8.GSR = "DISABLED";
    FD1P3AX rgb_hold__i9 (.D(rgb_values[8]), .SP(pll_clk_enable_210), .CK(pll_clk), 
            .Q(rgb_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i9.GSR = "DISABLED";
    FD1P3AX rgb_hold__i10 (.D(rgb_values[9]), .SP(pll_clk_enable_210), .CK(pll_clk), 
            .Q(rgb_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i10.GSR = "DISABLED";
    FD1P3AX rgb_hold__i11 (.D(rgb_values[10]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i11.GSR = "DISABLED";
    FD1P3AX rgb_hold__i12 (.D(rgb_values[11]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i12.GSR = "DISABLED";
    FD1P3AX rgb_hold__i13 (.D(rgb_values[12]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i13.GSR = "DISABLED";
    FD1P3AX rgb_hold__i14 (.D(rgb_values[13]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i14.GSR = "DISABLED";
    FD1P3AX rgb_hold__i15 (.D(rgb_values[14]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i15.GSR = "DISABLED";
    FD1P3AX rgb_hold__i16 (.D(rgb_values[15]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i16.GSR = "DISABLED";
    FD1P3AX rgb_hold__i17 (.D(rgb_values[16]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i17.GSR = "DISABLED";
    FD1P3AX rgb_hold__i18 (.D(rgb_values[17]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i18.GSR = "DISABLED";
    FD1P3AX rgb_hold__i19 (.D(rgb_values[18]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i19.GSR = "DISABLED";
    FD1P3AX rgb_hold__i20 (.D(rgb_values[19]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i20.GSR = "DISABLED";
    FD1P3AX rgb_hold__i21 (.D(rgb_values[20]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i21.GSR = "DISABLED";
    FD1P3AX rgb_hold__i22 (.D(rgb_values[21]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i22.GSR = "DISABLED";
    FD1P3AX rgb_hold__i23 (.D(rgb_values[22]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i23.GSR = "DISABLED";
    FD1P3AX rgb_hold__i24 (.D(rgb_values[23]), .SP(pll_clk_enable_210), 
            .CK(pll_clk), .Q(rgb_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam rgb_hold__i24.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i1 (.D(spi_byte_count_15__N_1628[1]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i1.GSR = "ENABLED";
    FD1P3IX ev_bit_i45 (.D(ev_bit[44]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i45.GSR = "DISABLED";
    FD1P3IX ev_bit_i44 (.D(ev_bit[43]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i44.GSR = "DISABLED";
    FD1P3IX ev_bit_i43 (.D(ev_bit[42]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i43.GSR = "DISABLED";
    FD1P3IX ev_bit_i42 (.D(ev_bit[41]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i42.GSR = "DISABLED";
    FD1P3IX ev_bit_i41 (.D(ev_bit[40]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i41.GSR = "DISABLED";
    FD1P3IX ev_bit_i40 (.D(ev_bit[39]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i40.GSR = "DISABLED";
    FD1P3IX ev_bit_i39 (.D(ev_bit[38]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i39.GSR = "DISABLED";
    FD1P3IX ev_bit_i38 (.D(ev_bit[37]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i38.GSR = "DISABLED";
    FD1P3IX ev_bit_i37 (.D(ev_bit[36]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i37.GSR = "DISABLED";
    FD1P3IX ev_bit_i36 (.D(ev_bit[35]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i36.GSR = "DISABLED";
    FD1P3IX ev_bit_i35 (.D(ev_bit[34]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i35.GSR = "DISABLED";
    FD1P3IX ev_bit_i34 (.D(ev_bit[33]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i34.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i2 (.D(spi_byte_count_15__N_1628[2]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i3 (.D(spi_byte_count_15__N_1628[3]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i4 (.D(spi_byte_count_15__N_1628[4]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i5 (.D(spi_byte_count_15__N_1628[5]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i6 (.D(spi_byte_count_15__N_1628[6]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i7 (.D(spi_byte_count_15__N_1628[7]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i8 (.D(spi_byte_count_15__N_1628[8]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i9 (.D(spi_byte_count_15__N_1628[9]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i10 (.D(spi_byte_count_15__N_1628[10]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i11 (.D(spi_byte_count_15__N_1628[11]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i12 (.D(spi_byte_count_15__N_1628[12]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i13 (.D(spi_byte_count_15__N_1628[13]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i14 (.D(spi_byte_count_15__N_1628[14]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i15 (.D(spi_byte_count_15__N_1628[15]), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .Q(spi_byte_count[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam spi_byte_count_i0_i15.GSR = "ENABLED";
    LUT4 i7_4_lut_adj_157 (.A(ev_run_hold_s5[3]), .B(us_tx_c_3), .C(init_shadow[3]), 
         .D(swap_now_s5), .Z(n14085)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_157.init = 16'h5a66;
    LUT4 i2_4_lut_adj_158 (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .C(spi_byte_count[5]), .D(n4_adj_3418), .Z(n22776)) /* synthesis lut_function=(A (B (C+(D)))) */ ;
    defparam i2_4_lut_adj_158.init = 16'h8880;
    LUT4 i2403_2_lut_3_lut (.A(swap_pending), .B(wrap_s2), .C(phase_step_s2), 
         .Z(pll_clk_enable_359)) /* synthesis lut_function=(A (B+(C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i2403_2_lut_3_lut.init = 16'hf8f8;
    FD1P3IX us_tx__i2 (.D(n14081), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i2.GSR = "DISABLED";
    FD1P3IX us_tx__i3 (.D(n14083), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i3.GSR = "DISABLED";
    FD1P3IX us_tx__i4 (.D(n14085), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i4.GSR = "DISABLED";
    FD1P3IX us_tx__i5 (.D(n14087), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i5.GSR = "DISABLED";
    FD1P3IX us_tx__i6 (.D(n14089), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i6.GSR = "DISABLED";
    FD1P3IX us_tx__i7 (.D(n14091), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_6)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i7.GSR = "DISABLED";
    FD1P3IX us_tx__i8 (.D(n14093), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_7)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i8.GSR = "DISABLED";
    FD1P3IX us_tx__i9 (.D(n14095), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_8)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i9.GSR = "DISABLED";
    FD1P3IX us_tx__i10 (.D(n14097), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_9)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i10.GSR = "DISABLED";
    FD1P3IX us_tx__i11 (.D(n14099), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_10)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i11.GSR = "DISABLED";
    FD1P3IX us_tx__i12 (.D(n14101), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_11)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i12.GSR = "DISABLED";
    FD1P3IX us_tx__i13 (.D(n14103), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_12)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i13.GSR = "DISABLED";
    FD1P3IX us_tx__i14 (.D(n14105), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_13)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i14.GSR = "DISABLED";
    FD1P3IX us_tx__i15 (.D(n14107), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_14)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i15.GSR = "DISABLED";
    FD1P3IX us_tx__i16 (.D(n14109), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_15)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i16.GSR = "DISABLED";
    FD1P3IX us_tx__i17 (.D(n14111), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_16)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i17.GSR = "DISABLED";
    FD1P3IX us_tx__i18 (.D(n14113), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_17)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i18.GSR = "DISABLED";
    FD1P3IX us_tx__i19 (.D(n14115), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_18)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i19.GSR = "DISABLED";
    FD1P3IX us_tx__i20 (.D(n14117), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_19)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i20.GSR = "DISABLED";
    FD1P3IX us_tx__i21 (.D(n14119), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_20)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i21.GSR = "DISABLED";
    FD1P3IX us_tx__i22 (.D(n14121), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_21)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i22.GSR = "DISABLED";
    FD1P3IX us_tx__i23 (.D(n14123), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_22)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i23.GSR = "DISABLED";
    FD1P3IX us_tx__i24 (.D(n14125), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_23)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i24.GSR = "DISABLED";
    FD1P3IX us_tx__i25 (.D(n14127), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_24)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i25.GSR = "DISABLED";
    FD1P3IX us_tx__i26 (.D(n14129), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_25)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i26.GSR = "DISABLED";
    FD1P3IX us_tx__i27 (.D(n14131), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_26)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i27.GSR = "DISABLED";
    FD1P3IX us_tx__i28 (.D(n14133), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_27)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i28.GSR = "DISABLED";
    FD1P3IX us_tx__i29 (.D(n14135), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_28)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i29.GSR = "DISABLED";
    FD1P3IX us_tx__i30 (.D(n14137), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_29)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i30.GSR = "DISABLED";
    FD1P3IX us_tx__i31 (.D(n14139), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_30)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i31.GSR = "DISABLED";
    FD1P3IX us_tx__i32 (.D(n14141), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_31)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i32.GSR = "DISABLED";
    FD1P3IX us_tx__i33 (.D(n14143), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_32)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i33.GSR = "DISABLED";
    FD1P3IX us_tx__i34 (.D(n14145), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_33)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i34.GSR = "DISABLED";
    FD1P3IX us_tx__i35 (.D(n14147), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_34)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i35.GSR = "DISABLED";
    FD1P3IX us_tx__i36 (.D(n14149), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_35)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i36.GSR = "DISABLED";
    FD1P3IX us_tx__i37 (.D(n14151), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_36)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i37.GSR = "DISABLED";
    FD1P3IX us_tx__i38 (.D(n14153), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_37)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i38.GSR = "DISABLED";
    FD1P3IX us_tx__i39 (.D(n14155), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_38)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i39.GSR = "DISABLED";
    FD1P3IX us_tx__i40 (.D(n14157), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_39)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i40.GSR = "DISABLED";
    FD1P3IX us_tx__i41 (.D(n14159), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_40)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i41.GSR = "DISABLED";
    FD1P3IX us_tx__i42 (.D(n14161), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_41)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i42.GSR = "DISABLED";
    FD1P3IX us_tx__i43 (.D(n14163), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_42)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i43.GSR = "DISABLED";
    FD1P3IX us_tx__i44 (.D(n14165), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_43)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i44.GSR = "DISABLED";
    FD1P3IX us_tx__i45 (.D(n14167), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_44)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i45.GSR = "DISABLED";
    FD1P3IX us_tx__i46 (.D(n14169), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_45)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i46.GSR = "DISABLED";
    FD1P3IX us_tx__i47 (.D(n14171), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_46)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i47.GSR = "DISABLED";
    FD1P3IX us_tx__i48 (.D(n14173), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_47)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i48.GSR = "DISABLED";
    FD1P3IX us_tx__i49 (.D(n14175), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_48)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i49.GSR = "DISABLED";
    FD1P3IX us_tx__i50 (.D(n14177), .SP(pll_clk_enable_271), .CD(n26271), 
            .CK(pll_clk), .Q(us_tx_c_49)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i50.GSR = "DISABLED";
    FD1P3IX us_tx__i51 (.D(n14179), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_50)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i51.GSR = "DISABLED";
    FD1P3IX us_tx__i52 (.D(n14181), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_51)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i52.GSR = "DISABLED";
    FD1P3IX us_tx__i53 (.D(n14183), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_52)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i53.GSR = "DISABLED";
    FD1P3IX us_tx__i54 (.D(n14185), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_53)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i54.GSR = "DISABLED";
    FD1P3IX us_tx__i55 (.D(n14187), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_54)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i55.GSR = "DISABLED";
    FD1P3IX us_tx__i56 (.D(n14189), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_55)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i56.GSR = "DISABLED";
    FD1P3IX us_tx__i57 (.D(n14191), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_56)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i57.GSR = "DISABLED";
    FD1P3IX us_tx__i58 (.D(n14193), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_57)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i58.GSR = "DISABLED";
    FD1P3IX us_tx__i59 (.D(n14195), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_58)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i59.GSR = "DISABLED";
    FD1P3IX us_tx__i60 (.D(n14197), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_59)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i60.GSR = "DISABLED";
    FD1P3IX us_tx__i61 (.D(n14199), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_60)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i61.GSR = "DISABLED";
    FD1P3IX us_tx__i62 (.D(n14201), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_61)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i62.GSR = "DISABLED";
    FD1P3IX us_tx__i63 (.D(n14203), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_62)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i63.GSR = "DISABLED";
    FD1P3IX us_tx__i64 (.D(n14205), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_63)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i64.GSR = "DISABLED";
    FD1P3IX us_tx__i65 (.D(n14207), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_64)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i65.GSR = "DISABLED";
    FD1P3IX us_tx__i66 (.D(n14209), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_65)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i66.GSR = "DISABLED";
    FD1P3IX us_tx__i67 (.D(n14211), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_66)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i67.GSR = "DISABLED";
    FD1P3IX us_tx__i68 (.D(n14213), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_67)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i68.GSR = "DISABLED";
    FD1P3IX us_tx__i69 (.D(n14215), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_68)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i69.GSR = "DISABLED";
    FD1P3IX us_tx__i70 (.D(n14217), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_69)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i70.GSR = "DISABLED";
    FD1P3IX us_tx__i71 (.D(n14219), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_70)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i71.GSR = "DISABLED";
    FD1P3IX us_tx__i72 (.D(n14221), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_71)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i72.GSR = "DISABLED";
    FD1P3IX us_tx__i73 (.D(n14223), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_72)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i73.GSR = "DISABLED";
    FD1P3IX us_tx__i74 (.D(n14225), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_73)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i74.GSR = "DISABLED";
    FD1P3IX us_tx__i75 (.D(n14227), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_74)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i75.GSR = "DISABLED";
    FD1P3IX us_tx__i76 (.D(n14229), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_75)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i76.GSR = "DISABLED";
    FD1P3IX us_tx__i77 (.D(n14231), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_76)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i77.GSR = "DISABLED";
    FD1P3IX us_tx__i78 (.D(n14233), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_77)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i78.GSR = "DISABLED";
    FD1P3IX us_tx__i79 (.D(n14235), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_78)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i79.GSR = "DISABLED";
    FD1P3IX us_tx__i80 (.D(n14237), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_79)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i80.GSR = "DISABLED";
    FD1P3IX us_tx__i81 (.D(n14239), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_80)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i81.GSR = "DISABLED";
    FD1P3IX us_tx__i82 (.D(n14241), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_81)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i82.GSR = "DISABLED";
    FD1P3IX us_tx__i83 (.D(n14243), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_82)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i83.GSR = "DISABLED";
    FD1P3IX us_tx__i84 (.D(n14245), .SP(pll_clk_enable_305), .CD(n12560), 
            .CK(pll_clk), .Q(us_tx_c_83)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam us_tx__i84.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_159 (.A(ev_run_hold_s5[4]), .B(us_tx_c_4), .C(init_shadow[4]), 
         .D(swap_now_s5), .Z(n14087)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_159.init = 16'h5a66;
    LUT4 i1_4_lut_adj_160 (.A(expected_next_15__N_1475[3]), .B(n26047), 
         .C(n21742), .D(n12011), .Z(n25408)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[17:31])
    defparam i1_4_lut_adj_160.init = 16'h0020;
    FD1P3IX init_shadow_i76 (.D(n16052), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i76.GSR = "DISABLED";
    LUT4 i10312_2_lut (.A(spi_byte_count[9]), .B(n11999), .Z(n21742)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10312_2_lut.init = 16'heeee;
    LUT4 i7_4_lut_adj_161 (.A(ev_run_hold_s5[5]), .B(us_tx_c_5), .C(init_shadow[5]), 
         .D(swap_now_s5), .Z(n14089)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_161.init = 16'h5a66;
    LUT4 i1622_2_lut (.A(ev_ch[1]), .B(ev_ch[0]), .Z(ev_ch_6__N_2150[1])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(606[27:39])
    defparam i1622_2_lut.init = 16'h6666;
    LUT4 i7_4_lut_adj_162 (.A(ev_run_hold_s5[6]), .B(us_tx_c_6), .C(init_shadow[6]), 
         .D(swap_now_s5), .Z(n14091)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_162.init = 16'h5a66;
    LUT4 mux_313_i1_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[0]), 
         .D(pending_sequence[0]), .Z(accepted_sequence_31__N_1135[0])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i1_3_lut_4_lut.init = 16'hfe10;
    LUT4 mux_313_i2_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[1]), 
         .D(pending_sequence[1]), .Z(accepted_sequence_31__N_1135[1])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i2_3_lut_4_lut.init = 16'hfe10;
    FD1P3IX ev_bit_i10 (.D(ev_bit[9]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i10.GSR = "DISABLED";
    FD1P3IX ev_bit_i33 (.D(ev_bit[32]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i33.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_163 (.A(ev_run_hold_s5[7]), .B(us_tx_c_7), .C(init_shadow[7]), 
         .D(swap_now_s5), .Z(n14093)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_163.init = 16'h5a66;
    LUT4 sub_80_inv_0_i2_1_lut_rep_146 (.A(status_bit_index[1]), .Z(n26080)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(298[55:80])
    defparam sub_80_inv_0_i2_1_lut_rep_146.init = 16'h5555;
    LUT4 n25898_bdd_4_lut_4_lut (.A(status_bit_index[1]), .B(status_bit_index[2]), 
         .C(n25678), .D(n25898), .Z(n26021)) /* synthesis lut_function=(A (B (D))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(298[55:80])
    defparam n25898_bdd_4_lut_4_lut.init = 16'hc840;
    LUT4 i14238_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[61]), 
         .C(status_hold[60]), .Z(n25697)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14238_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14239_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[63]), 
         .C(status_hold[62]), .Z(n25698)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14239_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_164 (.A(ev_run_hold_s5[8]), .B(us_tx_c_8), .C(init_shadow[8]), 
         .D(swap_now_s5), .Z(n14095)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_164.init = 16'h5a66;
    LUT4 i14237_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[59]), 
         .C(status_hold[58]), .Z(n25696)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14237_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14236_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[57]), 
         .C(status_hold[56]), .Z(n25695)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14236_3_lut_3_lut.init = 16'he4e4;
    PFUMX i14240 (.BLUT(n25683), .ALUT(n25684), .C0(n26080), .Z(n25699));
    FD1P3AX status_hold__i2 (.D(accepted_sequence[25]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i2.GSR = "DISABLED";
    FD1P3AX status_hold__i3 (.D(accepted_sequence[26]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i3.GSR = "DISABLED";
    FD1P3AX status_hold__i4 (.D(accepted_sequence[27]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i4.GSR = "DISABLED";
    FD1P3AX status_hold__i5 (.D(accepted_sequence[28]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i5.GSR = "DISABLED";
    FD1P3AX status_hold__i6 (.D(accepted_sequence[29]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i6.GSR = "DISABLED";
    FD1P3AX status_hold__i7 (.D(accepted_sequence[30]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i7.GSR = "DISABLED";
    FD1P3AX status_hold__i8 (.D(accepted_sequence[31]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i8.GSR = "DISABLED";
    FD1P3AX status_hold__i9 (.D(accepted_sequence[16]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i9.GSR = "DISABLED";
    FD1P3AX status_hold__i10 (.D(accepted_sequence[17]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i10.GSR = "DISABLED";
    FD1P3AX status_hold__i11 (.D(accepted_sequence[18]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i11.GSR = "DISABLED";
    FD1P3AX status_hold__i12 (.D(accepted_sequence[19]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i12.GSR = "DISABLED";
    FD1P3AX status_hold__i13 (.D(accepted_sequence[20]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i13.GSR = "DISABLED";
    FD1P3AX status_hold__i14 (.D(accepted_sequence[21]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i14.GSR = "DISABLED";
    FD1P3AX status_hold__i15 (.D(accepted_sequence[22]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i15.GSR = "DISABLED";
    FD1P3AX status_hold__i16 (.D(accepted_sequence[23]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i16.GSR = "DISABLED";
    FD1P3AX status_hold__i17 (.D(accepted_sequence[8]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i17.GSR = "DISABLED";
    FD1P3AX status_hold__i18 (.D(accepted_sequence[9]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i18.GSR = "DISABLED";
    FD1P3AX status_hold__i19 (.D(accepted_sequence[10]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i19.GSR = "DISABLED";
    FD1P3AX status_hold__i20 (.D(accepted_sequence[11]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i20.GSR = "DISABLED";
    FD1P3AX status_hold__i21 (.D(accepted_sequence[12]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i21.GSR = "DISABLED";
    FD1P3AX status_hold__i22 (.D(accepted_sequence[13]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i22.GSR = "DISABLED";
    FD1P3AX status_hold__i23 (.D(accepted_sequence[14]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i23.GSR = "DISABLED";
    FD1P3AX status_hold__i24 (.D(accepted_sequence[15]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i24.GSR = "DISABLED";
    FD1P3AX status_hold__i25 (.D(accepted_sequence[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i25.GSR = "DISABLED";
    FD1P3AX status_hold__i26 (.D(accepted_sequence[1]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i26.GSR = "DISABLED";
    FD1P3AX status_hold__i27 (.D(accepted_sequence[2]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i27.GSR = "DISABLED";
    FD1P3AX status_hold__i28 (.D(accepted_sequence[3]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i28.GSR = "DISABLED";
    FD1P3AX status_hold__i29 (.D(accepted_sequence[4]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i29.GSR = "DISABLED";
    FD1P3AX status_hold__i30 (.D(accepted_sequence[5]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i30.GSR = "DISABLED";
    FD1P3AX status_hold__i31 (.D(accepted_sequence[6]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i31.GSR = "DISABLED";
    FD1P3AX status_hold__i32 (.D(accepted_sequence[7]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i32.GSR = "DISABLED";
    FD1P3AX status_hold__i33 (.D(fpga_time[24]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i33.GSR = "DISABLED";
    FD1P3AX status_hold__i34 (.D(fpga_time[25]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i34.GSR = "DISABLED";
    FD1P3AX status_hold__i35 (.D(fpga_time[26]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i35.GSR = "DISABLED";
    FD1P3AX status_hold__i36 (.D(fpga_time[27]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i36.GSR = "DISABLED";
    FD1P3AX status_hold__i37 (.D(fpga_time[28]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i37.GSR = "DISABLED";
    FD1P3AX status_hold__i38 (.D(fpga_time[29]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i38.GSR = "DISABLED";
    FD1P3AX status_hold__i39 (.D(fpga_time[30]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i39.GSR = "DISABLED";
    FD1P3AX status_hold__i40 (.D(fpga_time[31]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i40.GSR = "DISABLED";
    FD1P3AX status_hold__i41 (.D(fpga_time[16]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i41.GSR = "DISABLED";
    FD1P3AX status_hold__i42 (.D(fpga_time[17]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i42.GSR = "DISABLED";
    FD1P3AX status_hold__i43 (.D(fpga_time[18]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i43.GSR = "DISABLED";
    FD1P3AX status_hold__i44 (.D(fpga_time[19]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i44.GSR = "DISABLED";
    FD1P3AX status_hold__i45 (.D(fpga_time[20]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i45.GSR = "DISABLED";
    FD1P3AX status_hold__i46 (.D(fpga_time[21]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i46.GSR = "DISABLED";
    FD1P3AX status_hold__i47 (.D(fpga_time[22]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i47.GSR = "DISABLED";
    FD1P3AX status_hold__i48 (.D(fpga_time[23]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i48.GSR = "DISABLED";
    FD1P3AX status_hold__i49 (.D(fpga_time[8]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i49.GSR = "DISABLED";
    FD1P3AX status_hold__i50 (.D(fpga_time[9]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i50.GSR = "DISABLED";
    FD1P3AX status_hold__i51 (.D(fpga_time[10]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i51.GSR = "DISABLED";
    FD1P3AX status_hold__i52 (.D(fpga_time[11]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i52.GSR = "DISABLED";
    FD1P3AX status_hold__i53 (.D(fpga_time[12]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i53.GSR = "DISABLED";
    FD1P3AX status_hold__i54 (.D(fpga_time[13]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i54.GSR = "DISABLED";
    FD1P3AX status_hold__i55 (.D(fpga_time[14]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i55.GSR = "DISABLED";
    FD1P3AX status_hold__i56 (.D(fpga_time[15]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i56.GSR = "DISABLED";
    FD1P3AX status_hold__i57 (.D(fpga_time[0]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i57.GSR = "DISABLED";
    FD1P3AX status_hold__i58 (.D(fpga_time[1]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i58.GSR = "DISABLED";
    FD1P3AX status_hold__i59 (.D(fpga_time[2]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i59.GSR = "DISABLED";
    FD1P3AX status_hold__i60 (.D(fpga_time[3]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i60.GSR = "DISABLED";
    FD1P3AX status_hold__i61 (.D(fpga_time[4]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i61.GSR = "DISABLED";
    FD1P3AX status_hold__i62 (.D(fpga_time[5]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i62.GSR = "DISABLED";
    FD1P3AX status_hold__i63 (.D(fpga_time[6]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i63.GSR = "DISABLED";
    FD1P3AX status_hold__i64 (.D(fpga_time[7]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i64.GSR = "DISABLED";
    FD1P3AX status_hold__i65 (.D(status_flags_wire[8]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i65.GSR = "DISABLED";
    FD1P3AX status_hold__i66 (.D(last_command_spi[1]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i66.GSR = "DISABLED";
    FD1P3AX status_hold__i67 (.D(last_command_spi[2]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i67.GSR = "DISABLED";
    FD1P3AX status_hold__i68 (.D(status_flags_wire[11]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i68.GSR = "DISABLED";
    FD1P3AX status_hold__i69 (.D(invalid_frame_sync), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i69.GSR = "DISABLED";
    FD1P3AX status_hold__i70 (.D(running), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i70.GSR = "DISABLED";
    FD1P3AX status_hold__i71 (.D(n26040), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i71.GSR = "DISABLED";
    FD1P3AX status_hold__i72 (.D(fifo_credit_wire[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[104])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam status_hold__i72.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i1 (.D(accepted_sequence_31__N_1135[1]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i1.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i2 (.D(accepted_sequence_31__N_1135[2]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i3 (.D(accepted_sequence_31__N_1135[3]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i4 (.D(accepted_sequence_31__N_1135[4]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i5 (.D(accepted_sequence_31__N_1135[5]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i6 (.D(accepted_sequence_31__N_1135[6]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i7 (.D(accepted_sequence_31__N_1135[7]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i8 (.D(accepted_sequence_31__N_1135[8]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i9 (.D(accepted_sequence_31__N_1135[9]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i10 (.D(accepted_sequence_31__N_1135[10]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i11 (.D(accepted_sequence_31__N_1135[11]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i12 (.D(accepted_sequence_31__N_1135[12]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i13 (.D(accepted_sequence_31__N_1135[13]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i14 (.D(accepted_sequence_31__N_1135[14]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i15 (.D(accepted_sequence_31__N_1135[15]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i16 (.D(accepted_sequence_31__N_1135[16]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i17 (.D(accepted_sequence_31__N_1135[17]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i18 (.D(accepted_sequence_31__N_1135[18]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i19 (.D(accepted_sequence_31__N_1135[19]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i20 (.D(accepted_sequence_31__N_1135[20]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i21 (.D(accepted_sequence_31__N_1135[21]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i22 (.D(accepted_sequence_31__N_1135[22]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i23 (.D(accepted_sequence_31__N_1135[23]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i24 (.D(accepted_sequence_31__N_1135[24]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i25 (.D(accepted_sequence_31__N_1135[25]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i26 (.D(accepted_sequence_31__N_1135[26]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i27 (.D(accepted_sequence_31__N_1135[27]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i28 (.D(accepted_sequence_31__N_1135[28]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i29 (.D(accepted_sequence_31__N_1135[29]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i30 (.D(accepted_sequence_31__N_1135[30]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i31 (.D(accepted_sequence_31__N_1135[31]), .SP(pll_clk_enable_351), 
            .CK(pll_clk), .Q(accepted_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_i31.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i1 (.D(global_phase_s2[1]), .SP(pll_clk_enable_359), 
            .CK(pll_clk), .Q(run_addr_s3[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam run_addr_s3_i1.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i2 (.D(global_phase_s2[2]), .SP(pll_clk_enable_359), 
            .CK(pll_clk), .Q(run_addr_s3[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam run_addr_s3_i2.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i3 (.D(global_phase_s2[3]), .SP(pll_clk_enable_359), 
            .CK(pll_clk), .Q(run_addr_s3[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam run_addr_s3_i3.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i4 (.D(global_phase_s2[4]), .SP(pll_clk_enable_359), 
            .CK(pll_clk), .Q(run_addr_s3[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam run_addr_s3_i4.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i5 (.D(global_phase_s2[5]), .SP(pll_clk_enable_359), 
            .CK(pll_clk), .Q(run_addr_s3[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam run_addr_s3_i5.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i6 (.D(global_phase_s2[6]), .SP(pll_clk_enable_359), 
            .CK(pll_clk), .Q(run_addr_s3[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam run_addr_s3_i6.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i7 (.D(global_phase_s2[7]), .SP(pll_clk_enable_359), 
            .CK(pll_clk), .Q(run_addr_s3[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam run_addr_s3_i7.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i8 (.D(run_addr_s3_8__N_438[8]), .SP(pll_clk_enable_359), 
            .CK(pll_clk), .Q(run_addr_s3[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam run_addr_s3_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i1 (.D(accepted_sequence_spi[1]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i2 (.D(accepted_sequence_spi[2]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i3 (.D(accepted_sequence_spi[3]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i4 (.D(accepted_sequence_spi[4]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i5 (.D(accepted_sequence_spi[5]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i6 (.D(accepted_sequence_spi[6]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i7 (.D(accepted_sequence_spi[7]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i8 (.D(accepted_sequence_spi[8]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i9 (.D(accepted_sequence_spi[9]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i10 (.D(accepted_sequence_spi[10]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i11 (.D(accepted_sequence_spi[11]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i12 (.D(accepted_sequence_spi[12]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i13 (.D(accepted_sequence_spi[13]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i14 (.D(accepted_sequence_spi[14]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i15 (.D(accepted_sequence_spi[15]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i16 (.D(accepted_sequence_spi[16]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i17 (.D(accepted_sequence_spi[17]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i18 (.D(accepted_sequence_spi[18]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i19 (.D(accepted_sequence_spi[19]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i20 (.D(accepted_sequence_spi[20]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i21 (.D(accepted_sequence_spi[21]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i22 (.D(accepted_sequence_spi[22]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i23 (.D(accepted_sequence_spi[23]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i24 (.D(accepted_sequence_spi[24]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i25 (.D(accepted_sequence_spi[25]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i26 (.D(accepted_sequence_spi[26]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i27 (.D(accepted_sequence_spi[27]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i28 (.D(accepted_sequence_spi[28]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i29 (.D(accepted_sequence_spi[29]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i30 (.D(accepted_sequence_spi[30]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i31 (.D(accepted_sequence_spi[31]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_meta_i31.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i1 (.D(accepted_sequence_meta[1]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i2 (.D(accepted_sequence_meta[2]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i3 (.D(accepted_sequence_meta[3]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i4 (.D(accepted_sequence_meta[4]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i5 (.D(accepted_sequence_meta[5]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i6 (.D(accepted_sequence_meta[6]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i7 (.D(accepted_sequence_meta[7]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i8 (.D(accepted_sequence_meta[8]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i9 (.D(accepted_sequence_meta[9]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i10 (.D(accepted_sequence_meta[10]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i11 (.D(accepted_sequence_meta[11]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i12 (.D(accepted_sequence_meta[12]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i13 (.D(accepted_sequence_meta[13]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i14 (.D(accepted_sequence_meta[14]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i15 (.D(accepted_sequence_meta[15]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i16 (.D(accepted_sequence_meta[16]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i17 (.D(accepted_sequence_meta[17]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i18 (.D(accepted_sequence_meta[18]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i19 (.D(accepted_sequence_meta[19]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i20 (.D(accepted_sequence_meta[20]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i21 (.D(accepted_sequence_meta[21]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i22 (.D(accepted_sequence_meta[22]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i23 (.D(accepted_sequence_meta[23]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i24 (.D(accepted_sequence_meta[24]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i25 (.D(accepted_sequence_meta[25]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i26 (.D(accepted_sequence_meta[26]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i27 (.D(accepted_sequence_meta[27]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i28 (.D(accepted_sequence_meta[28]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i29 (.D(accepted_sequence_meta[29]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i30 (.D(accepted_sequence_meta[30]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i31 (.D(accepted_sequence_meta[31]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam accepted_sequence_sync_i31.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i1 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i2 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i3 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i4 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i5 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i6 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i7 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i8 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i9 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i10 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i11 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i12 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i13 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i14 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i15 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_0_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_l_i0_i15.GSR = "DISABLED";
    LUT4 i14235_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[55]), 
         .C(status_hold[54]), .Z(n25694)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14235_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX ev_state_i2 (.D(ev_state_3__N_710[2]), .SP(pll_clk_enable_375), 
            .CK(pll_clk), .Q(ev_state[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_state_i2.GSR = "DISABLED";
    FD1S3AX ev_state_i3 (.D(n25218), .CK(pll_clk), .Q(ev_state[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_state_i3.GSR = "DISABLED";
    FD1P3AX build_phase_i1 (.D(staging_q[9]), .SP(pll_clk_enable_390), .CK(pll_clk), 
            .Q(build_phase[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_phase_i1.GSR = "DISABLED";
    FD1P3AX build_phase_i2 (.D(staging_q[10]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_phase[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_phase_i2.GSR = "DISABLED";
    FD1P3AX build_phase_i3 (.D(staging_q[11]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_phase[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_phase_i3.GSR = "DISABLED";
    FD1P3AX build_phase_i4 (.D(staging_q[12]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_phase[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_phase_i4.GSR = "DISABLED";
    FD1P3AX build_phase_i5 (.D(staging_q[13]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_phase[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_phase_i5.GSR = "DISABLED";
    FD1P3AX build_phase_i6 (.D(staging_q[14]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_phase[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_phase_i6.GSR = "DISABLED";
    FD1P3AX build_phase_i7 (.D(staging_q[15]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_phase[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_phase_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i1 (.D(build_sum_8__N_2255[1]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_sum[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_sum_i1.GSR = "DISABLED";
    FD1P3AX build_sum_i2 (.D(build_sum_8__N_2255[2]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_sum[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_sum_i2.GSR = "DISABLED";
    FD1P3AX build_sum_i3 (.D(build_sum_8__N_2255[3]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_sum[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_sum_i3.GSR = "DISABLED";
    FD1P3AX build_sum_i4 (.D(build_sum_8__N_2255[4]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_sum[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_sum_i4.GSR = "DISABLED";
    FD1P3AX build_sum_i5 (.D(build_sum_8__N_2255[5]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_sum[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_sum_i5.GSR = "DISABLED";
    FD1P3AX build_sum_i6 (.D(build_sum_8__N_2255[6]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_sum[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_sum_i6.GSR = "DISABLED";
    FD1P3AX build_sum_i7 (.D(build_sum_8__N_2255[7]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_sum[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_sum_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i8 (.D(build_sum_8__N_2255[8]), .SP(pll_clk_enable_390), 
            .CK(pll_clk), .Q(build_sum[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam build_sum_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i1 (.D(ev_rd_data[1]), .SP(pll_clk_enable_439), .CK(pll_clk), 
            .Q(ev_rd_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i1.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i2 (.D(ev_rd_data[2]), .SP(pll_clk_enable_439), .CK(pll_clk), 
            .Q(ev_rd_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i2.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i3 (.D(ev_rd_data[3]), .SP(pll_clk_enable_439), .CK(pll_clk), 
            .Q(ev_rd_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i3.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i4 (.D(ev_rd_data[4]), .SP(pll_clk_enable_439), .CK(pll_clk), 
            .Q(ev_rd_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i4.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i5 (.D(ev_rd_data[5]), .SP(pll_clk_enable_439), .CK(pll_clk), 
            .Q(ev_rd_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i5.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i6 (.D(ev_rd_data[6]), .SP(pll_clk_enable_439), .CK(pll_clk), 
            .Q(ev_rd_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i6.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i7 (.D(ev_rd_data[7]), .SP(pll_clk_enable_439), .CK(pll_clk), 
            .Q(ev_rd_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i7.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i8 (.D(ev_rd_data[8]), .SP(pll_clk_enable_439), .CK(pll_clk), 
            .Q(ev_rd_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i9 (.D(ev_rd_data[9]), .SP(pll_clk_enable_439), .CK(pll_clk), 
            .Q(ev_rd_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i9.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i10 (.D(ev_rd_data[10]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i10.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i11 (.D(ev_rd_data[11]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i11.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i12 (.D(ev_rd_data[12]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i12.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i13 (.D(ev_rd_data[13]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i13.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i14 (.D(ev_rd_data[14]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i14.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i15 (.D(ev_rd_data[15]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i15.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i16 (.D(ev_rd_data[16]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i16.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i17 (.D(ev_rd_data[17]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i17.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i18 (.D(ev_rd_data[18]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i18.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i19 (.D(ev_rd_data[19]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i19.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i20 (.D(ev_rd_data[20]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i20.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i21 (.D(ev_rd_data[21]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i21.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i22 (.D(ev_rd_data[22]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i22.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i23 (.D(ev_rd_data[23]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i23.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i24 (.D(ev_rd_data[24]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i24.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i25 (.D(ev_rd_data[25]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i25.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i26 (.D(ev_rd_data[26]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i26.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i27 (.D(ev_rd_data[27]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i27.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i28 (.D(ev_rd_data[28]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i28.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i29 (.D(ev_rd_data[29]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i29.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i30 (.D(ev_rd_data[30]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i30.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i31 (.D(ev_rd_data[31]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i31.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i32 (.D(ev_rd_data[32]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i32.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i33 (.D(ev_rd_data[33]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i33.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i34 (.D(ev_rd_data[34]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i34.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i35 (.D(ev_rd_data[35]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i35.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i36 (.D(ev_rd_data[36]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i36.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i37 (.D(ev_rd_data[37]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i37.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i38 (.D(ev_rd_data[38]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i38.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i39 (.D(ev_rd_data[39]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i39.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i40 (.D(ev_rd_data[40]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i40.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i41 (.D(ev_rd_data[41]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i41.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i42 (.D(ev_rd_data[42]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i42.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i43 (.D(ev_rd_data[43]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i43.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i44 (.D(ev_rd_data[44]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i44.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i45 (.D(ev_rd_data[45]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i45.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i46 (.D(ev_rd_data[46]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i46.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i47 (.D(ev_rd_data[47]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i47.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i48 (.D(ev_rd_data[48]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i48.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i49 (.D(ev_rd_data[49]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(ev_rd_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i49.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i50 (.D(ev_rd_data[50]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i50.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i51 (.D(ev_rd_data[51]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i51.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i52 (.D(ev_rd_data[52]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i52.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i53 (.D(ev_rd_data[53]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i53.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i54 (.D(ev_rd_data[54]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i54.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i55 (.D(ev_rd_data[55]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i55.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i56 (.D(ev_rd_data[56]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i56.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i57 (.D(ev_rd_data[57]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i57.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i58 (.D(ev_rd_data[58]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i58.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i59 (.D(ev_rd_data[59]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i59.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i60 (.D(ev_rd_data[60]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i60.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i61 (.D(ev_rd_data[61]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i61.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i62 (.D(ev_rd_data[62]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i62.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i63 (.D(ev_rd_data[63]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i63.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i64 (.D(ev_rd_data[64]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i64.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i65 (.D(ev_rd_data[65]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i65.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i66 (.D(ev_rd_data[66]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i66.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i67 (.D(ev_rd_data[67]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i67.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i68 (.D(ev_rd_data[68]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i68.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i69 (.D(ev_rd_data[69]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i69.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i70 (.D(ev_rd_data[70]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i70.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i71 (.D(ev_rd_data[71]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i71.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i72 (.D(ev_rd_data[72]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i72.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i73 (.D(ev_rd_data[73]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i73.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i74 (.D(ev_rd_data[74]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i74.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i75 (.D(ev_rd_data[75]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i75.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i76 (.D(ev_rd_data[76]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i76.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i77 (.D(ev_rd_data[77]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i77.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i78 (.D(ev_rd_data[78]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i78.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i79 (.D(ev_rd_data[79]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i79.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i80 (.D(ev_rd_data[80]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i80.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i81 (.D(ev_rd_data[81]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i81.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i82 (.D(ev_rd_data[82]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i82.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i83 (.D(ev_rd_data[83]), .SP(pll_clk_enable_473), 
            .CK(pll_clk), .Q(ev_rd_hold[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_rd_hold_i83.GSR = "DISABLED";
    LUT4 i14234_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[53]), 
         .C(status_hold[52]), .Z(n25693)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14234_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14233_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[51]), 
         .C(status_hold[50]), .Z(n25692)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14233_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14232_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[49]), 
         .C(status_hold[48]), .Z(n25691)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14232_3_lut_3_lut.init = 16'he4e4;
    LUT4 n25678_bdd_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[65]), 
         .C(status_hold[64]), .Z(n25898)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam n25678_bdd_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14229_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[43]), 
         .C(status_hold[42]), .Z(n25688)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14229_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14228_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[41]), 
         .C(status_hold[40]), .Z(n25687)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14228_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_2_lut_3_lut_4_lut_adj_165 (.A(spi_byte_count[5]), .B(n26061), 
         .C(n22776), .D(n26095), .Z(n5_adj_3422)) /* synthesis lut_function=(!(A (B (C)+!B (C+!(D)))+!A (C+!(D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_165.init = 16'h0f08;
    LUT4 i14231_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[47]), 
         .C(status_hold[46]), .Z(n25690)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14231_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14227_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[39]), 
         .C(status_hold[38]), .Z(n25686)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14227_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14226_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[37]), 
         .C(status_hold[36]), .Z(n25685)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14226_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_166 (.A(ev_run_hold_s5[9]), .B(us_tx_c_9), .C(init_shadow[9]), 
         .D(swap_now_s5), .Z(n14097)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_166.init = 16'h5a66;
    LUT4 i14225_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[35]), 
         .C(status_hold[34]), .Z(n25684)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14225_3_lut_3_lut.init = 16'he4e4;
    FD1S3AX staging_rd_addr_i1 (.D(staging_rd_addr_6__N_914[1]), .CK(pll_clk), 
            .Q(staging_rd_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam staging_rd_addr_i1.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i2 (.D(staging_rd_addr_6__N_914[2]), .CK(pll_clk), 
            .Q(staging_rd_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam staging_rd_addr_i2.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i3 (.D(staging_rd_addr_6__N_914[3]), .CK(pll_clk), 
            .Q(staging_rd_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam staging_rd_addr_i3.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i4 (.D(staging_rd_addr_6__N_914[4]), .CK(pll_clk), 
            .Q(staging_rd_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam staging_rd_addr_i4.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i5 (.D(staging_rd_addr_6__N_914[5]), .CK(pll_clk), 
            .Q(staging_rd_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam staging_rd_addr_i5.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i6 (.D(staging_rd_addr_6__N_914[6]), .CK(pll_clk), 
            .Q(staging_rd_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam staging_rd_addr_i6.GSR = "DISABLED";
    FD1P3AX pending_sequence_i1 (.D(accepted_sequence_sync[1]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i1.GSR = "DISABLED";
    FD1P3AX pending_sequence_i2 (.D(accepted_sequence_sync[2]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i2.GSR = "DISABLED";
    FD1P3AX pending_sequence_i3 (.D(accepted_sequence_sync[3]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i3.GSR = "DISABLED";
    FD1P3AX pending_sequence_i4 (.D(accepted_sequence_sync[4]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i4.GSR = "DISABLED";
    FD1P3AX pending_sequence_i5 (.D(accepted_sequence_sync[5]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i5.GSR = "DISABLED";
    FD1P3AX pending_sequence_i6 (.D(accepted_sequence_sync[6]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i6.GSR = "DISABLED";
    FD1P3AX pending_sequence_i7 (.D(accepted_sequence_sync[7]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i7.GSR = "DISABLED";
    FD1P3AX pending_sequence_i8 (.D(accepted_sequence_sync[8]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i8.GSR = "DISABLED";
    FD1P3AX pending_sequence_i9 (.D(accepted_sequence_sync[9]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i9.GSR = "DISABLED";
    FD1P3AX pending_sequence_i10 (.D(accepted_sequence_sync[10]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i10.GSR = "DISABLED";
    FD1P3AX pending_sequence_i11 (.D(accepted_sequence_sync[11]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i11.GSR = "DISABLED";
    FD1P3AX pending_sequence_i12 (.D(accepted_sequence_sync[12]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i12.GSR = "DISABLED";
    FD1P3AX pending_sequence_i13 (.D(accepted_sequence_sync[13]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i13.GSR = "DISABLED";
    FD1P3AX pending_sequence_i14 (.D(accepted_sequence_sync[14]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i14.GSR = "DISABLED";
    FD1P3AX pending_sequence_i15 (.D(accepted_sequence_sync[15]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i15.GSR = "DISABLED";
    FD1P3AX pending_sequence_i16 (.D(accepted_sequence_sync[16]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i16.GSR = "DISABLED";
    FD1P3AX pending_sequence_i17 (.D(accepted_sequence_sync[17]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i17.GSR = "DISABLED";
    FD1P3AX pending_sequence_i18 (.D(accepted_sequence_sync[18]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i18.GSR = "DISABLED";
    FD1P3AX pending_sequence_i19 (.D(accepted_sequence_sync[19]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i19.GSR = "DISABLED";
    FD1P3AX pending_sequence_i20 (.D(accepted_sequence_sync[20]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i20.GSR = "DISABLED";
    FD1P3AX pending_sequence_i21 (.D(accepted_sequence_sync[21]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i21.GSR = "DISABLED";
    FD1P3AX pending_sequence_i22 (.D(accepted_sequence_sync[22]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i22.GSR = "DISABLED";
    FD1P3AX pending_sequence_i23 (.D(accepted_sequence_sync[23]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i23.GSR = "DISABLED";
    FD1P3AX pending_sequence_i24 (.D(accepted_sequence_sync[24]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i24.GSR = "DISABLED";
    FD1P3AX pending_sequence_i25 (.D(accepted_sequence_sync[25]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i25.GSR = "DISABLED";
    FD1P3AX pending_sequence_i26 (.D(accepted_sequence_sync[26]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i26.GSR = "DISABLED";
    FD1P3AX pending_sequence_i27 (.D(accepted_sequence_sync[27]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i27.GSR = "DISABLED";
    FD1P3AX pending_sequence_i28 (.D(accepted_sequence_sync[28]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i28.GSR = "DISABLED";
    FD1P3AX pending_sequence_i29 (.D(accepted_sequence_sync[29]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i29.GSR = "DISABLED";
    FD1P3AX pending_sequence_i30 (.D(accepted_sequence_sync[30]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i30.GSR = "DISABLED";
    FD1P3AX pending_sequence_i31 (.D(accepted_sequence_sync[31]), .SP(pll_clk_enable_504), 
            .CK(pll_clk), .Q(pending_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam pending_sequence_i31.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i1 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i2 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i3 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i4 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i5 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i6 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i7 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i8 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i9 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i10 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i11 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i12 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i13 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i14 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i15 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_519), 
            .CK(pll_clk), .Q(mic_shift_1_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_l_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i2 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i3 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i4 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i5 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i6 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i7 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i8 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i9 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i10 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i11 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i12 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i13 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i14 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i15 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_0_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_0_r__i15.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i2 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i3 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i4 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i5 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i6 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i7 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i8 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i9 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i10 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i11 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i12 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i13 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i14 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i15 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_shift_1_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_shift_1_r__i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i1 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i2 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i3 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i4 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i5 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i6 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i7 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i8 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i9 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i10 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i11 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i12 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i13 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i14 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i15 (.D(mic_shift_1_r[14]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i16 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i16.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i17 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i17.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i18 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i18.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i19 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i19.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i20 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i20.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i21 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i21.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i22 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i22.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i23 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i23.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i24 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i24.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i25 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i26 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i26.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i27 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i27.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i28 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i28.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i29 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i29.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i30 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i30.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i31 (.D(mic_shift_1_l[15]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i31.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i32 (.D(mic_data_0_c), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i32.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i33 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i33.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i34 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i34.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i35 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i35.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i36 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i36.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i37 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i37.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i38 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i38.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i39 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i39.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i40 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i40.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i41 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i41.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i42 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i42.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i43 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i43.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i44 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i44.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i45 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i45.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i46 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i46.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i47 (.D(mic_shift_0_r[14]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i47.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i48 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i48.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i49 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_596), 
            .CK(pll_clk), .Q(mic_latest[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i49.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i50 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i50.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i51 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i51.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i52 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i52.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i53 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i53.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i54 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i54.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i55 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i55.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i56 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i56.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i57 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i57.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i58 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i58.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i59 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i59.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i60 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i60.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i61 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i61.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i62 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i62.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i63 (.D(mic_shift_0_l[15]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(mic_latest[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam mic_latest_i0_i63.GSR = "DISABLED";
    spi_mic_stream mic_stream_i (.sck_N_3271(sck_N_3271), .spi_mic_cs_n_c(spi_mic_cs_n_c), 
            .spi_mic_miso_c(spi_mic_miso_c), .mic_latest({mic_latest})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(692[20] 695[6])
    LUT4 i7_4_lut_adj_167 (.A(ev_run_hold_s5[10]), .B(us_tx_c_10), .C(init_shadow[10]), 
         .D(swap_now_s5), .Z(n14099)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_167.init = 16'h5a66;
    LUT4 i7_4_lut_adj_168 (.A(ev_run_hold_s5[11]), .B(us_tx_c_11), .C(init_shadow[11]), 
         .D(swap_now_s5), .Z(n14101)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_168.init = 16'h5a66;
    LUT4 i14301_2_lut (.A(ws2812_settle[0]), .B(ws2812_settle[1]), .Z(n25722)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(541[13] 548[16])
    defparam i14301_2_lut.init = 16'h9999;
    FD1P3IX ev_clear_addr_i4 (.D(ev_clear_addr_7__N_2439[4]), .SP(pll_clk_enable_775), 
            .CD(n18521), .CK(pll_clk), .Q(ev_clear_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_clear_addr_i4.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i3 (.D(ev_clear_addr_7__N_2439[3]), .SP(pll_clk_enable_775), 
            .CD(n18521), .CK(pll_clk), .Q(ev_clear_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_clear_addr_i3.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i2 (.D(ev_clear_addr_7__N_2439[2]), .SP(pll_clk_enable_775), 
            .CD(n18521), .CK(pll_clk), .Q(ev_clear_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_clear_addr_i2.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_169 (.A(ev_run_hold_s5[12]), .B(us_tx_c_12), .C(init_shadow[12]), 
         .D(swap_now_s5), .Z(n14103)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_169.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i1 (.D(ev_clear_addr_7__N_2439[1]), .SP(pll_clk_enable_775), 
            .CD(n18521), .CK(pll_clk), .Q(ev_clear_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_clear_addr_i1.GSR = "DISABLED";
    FD1P3IX init_shadow_i83 (.D(n16098), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i83.GSR = "DISABLED";
    FD1P3IX init_shadow_i82 (.D(n16092), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i82.GSR = "DISABLED";
    LUT4 mux_313_i3_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[2]), 
         .D(pending_sequence[2]), .Z(accepted_sequence_31__N_1135[2])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i3_3_lut_4_lut.init = 16'hfe10;
    LUT4 i7_4_lut_adj_170 (.A(ev_run_hold_s5[13]), .B(us_tx_c_13), .C(init_shadow[13]), 
         .D(swap_now_s5), .Z(n14105)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_170.init = 16'h5a66;
    LUT4 i7_4_lut_adj_171 (.A(ev_run_hold_s5[14]), .B(us_tx_c_14), .C(init_shadow[14]), 
         .D(swap_now_s5), .Z(n14107)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_171.init = 16'h5a66;
    PFUMX i14241 (.BLUT(n25685), .ALUT(n25686), .C0(n26080), .Z(n25700));
    LUT4 i7_4_lut_adj_172 (.A(ev_run_hold_s5[15]), .B(us_tx_c_15), .C(init_shadow[15]), 
         .D(swap_now_s5), .Z(n14109)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_172.init = 16'h5a66;
    LUT4 i7_4_lut_adj_173 (.A(ev_run_hold_s5[16]), .B(us_tx_c_16), .C(init_shadow[16]), 
         .D(swap_now_s5), .Z(n14111)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_173.init = 16'h5a66;
    LUT4 i11580_1_lut (.A(spi_channel_field[0]), .Z(n23000)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[78:102])
    defparam i11580_1_lut.init = 16'h5555;
    LUT4 i7_4_lut_adj_174 (.A(ev_run_hold_s5[17]), .B(us_tx_c_17), .C(init_shadow[17]), 
         .D(swap_now_s5), .Z(n14113)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_174.init = 16'h5a66;
    LUT4 mux_313_i4_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[3]), 
         .D(pending_sequence[3]), .Z(accepted_sequence_31__N_1135[3])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i4_3_lut_4_lut.init = 16'hfe10;
    LUT4 i14224_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[33]), 
         .C(status_hold[32]), .Z(n25683)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14224_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_175 (.A(ev_run_hold_s5[18]), .B(us_tx_c_18), .C(init_shadow[18]), 
         .D(swap_now_s5), .Z(n14115)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_175.init = 16'h5a66;
    CCU2D add_634_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24782), .S1(phase_frac_sum[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_1.INIT0 = 16'hF000;
    defparam add_634_1.INIT1 = 16'h5555;
    defparam add_634_1.INJECT1_0 = "NO";
    defparam add_634_1.INJECT1_1 = "NO";
    CCU2D sub_1246_add_2_5 (.A0(spi_byte_count[6]), .B0(spi_rgb_payload_byte_N_2770[6]), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[7]), .B1(spi_rgb_payload_byte_N_2770[6]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24762), .COUT(n24763));
    defparam sub_1246_add_2_5.INIT0 = 16'h5999;
    defparam sub_1246_add_2_5.INIT1 = 16'h5999;
    defparam sub_1246_add_2_5.INJECT1_0 = "NO";
    defparam sub_1246_add_2_5.INJECT1_1 = "NO";
    LUT4 i14219_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[67]), 
         .C(status_hold[66]), .Z(n25678)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14219_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_adj_176 (.A(n17439), .B(init_shadow[69]), .C(ev_bit[69]), 
         .Z(n16010)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_176.init = 16'hecec;
    LUT4 ws2812_toggle_spi_I_0_4_lut (.A(ws2812_toggle_spi), .B(n17384), 
         .C(spi_command[1]), .D(spi_command[0]), .Z(ws2812_toggle_spi_N_2758)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(424[30] 426[24])
    defparam ws2812_toggle_spi_I_0_4_lut.init = 16'h6aaa;
    FD1P3IX ev_bit_i32 (.D(ev_bit[31]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i32.GSR = "DISABLED";
    FD1P3IX ev_bit_i31 (.D(ev_bit[30]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i31.GSR = "DISABLED";
    LUT4 mux_313_i5_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[4]), 
         .D(pending_sequence[4]), .Z(accepted_sequence_31__N_1135[4])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i5_3_lut_4_lut.init = 16'hfe10;
    LUT4 sub_80_inv_0_i6_1_lut (.A(status_bit_index[5]), .Z(spi1_miso_N_2715[5])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(298[55:80])
    defparam sub_80_inv_0_i6_1_lut.init = 16'h5555;
    LUT4 i14202_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[31]), 
         .C(status_hold[30]), .Z(n25661)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14202_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_177 (.A(ev_run_hold_s5[19]), .B(us_tx_c_19), .C(init_shadow[19]), 
         .D(swap_now_s5), .Z(n14117)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_177.init = 16'h5a66;
    LUT4 i14324_4_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_185), .C(frame_end), 
         .D(n26026), .Z(spi1_sck_c_enable_188)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i14324_4_lut.init = 16'h0040;
    LUT4 i7_4_lut_adj_178 (.A(ev_run_hold_s5[20]), .B(us_tx_c_20), .C(init_shadow[20]), 
         .D(swap_now_s5), .Z(n14119)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_178.init = 16'h5a66;
    CCU2D spi_channel_index_1407_add_4_5 (.A0(spi_channel_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24806), .COUT(n24807), .S0(n37), 
          .S1(n36));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407_add_4_5.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1407_add_4_5.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1407_add_4_5.INJECT1_0 = "NO";
    defparam spi_channel_index_1407_add_4_5.INJECT1_1 = "NO";
    LUT4 mux_313_i6_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[5]), 
         .D(pending_sequence[5]), .Z(accepted_sequence_31__N_1135[5])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i6_3_lut_4_lut.init = 16'hfe10;
    LUT4 i14230_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[45]), 
         .C(status_hold[44]), .Z(n25689)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14230_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14201_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[29]), 
         .C(status_hold[28]), .Z(n25660)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14201_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_179 (.A(ev_run_hold_s5[21]), .B(us_tx_c_21), .C(init_shadow[21]), 
         .D(swap_now_s5), .Z(n14121)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_179.init = 16'h5a66;
    LUT4 i7_4_lut_adj_180 (.A(ev_run_hold_s5[22]), .B(us_tx_c_22), .C(init_shadow[22]), 
         .D(swap_now_s5), .Z(n14123)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_180.init = 16'h5a66;
    LUT4 i14200_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[27]), 
         .C(status_hold[26]), .Z(n25659)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14200_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_181 (.A(ev_run_hold_s5[23]), .B(us_tx_c_23), .C(init_shadow[23]), 
         .D(swap_now_s5), .Z(n14125)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_181.init = 16'h5a66;
    LUT4 i7_4_lut_adj_182 (.A(ev_run_hold_s5[24]), .B(us_tx_c_24), .C(init_shadow[24]), 
         .D(swap_now_s5), .Z(n14127)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_182.init = 16'h5a66;
    L6MUX21 i14383 (.D0(n25895), .D1(n25892), .SD(status_bit_index[6]), 
            .Z(spi1_miso_N_2714));
    LUT4 i7_4_lut_adj_183 (.A(ev_run_hold_s5[0]), .B(us_tx_c_0), .C(init_shadow[0]), 
         .D(swap_now_s5), .Z(n13469)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_183.init = 16'h5a66;
    LUT4 i14199_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[25]), 
         .C(status_hold[24]), .Z(n25658)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14199_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14198_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[23]), 
         .C(status_hold[22]), .Z(n25657)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14198_3_lut_3_lut.init = 16'he4e4;
    CCU2D mic_divider_1415_add_4_5 (.A0(mic_divider[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24834), .COUT(n24835), .S0(n37_adj_3401), 
          .S1(n36_adj_3402));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_divider_1415_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_divider_1415_add_4_5.INJECT1_0 = "NO";
    defparam mic_divider_1415_add_4_5.INJECT1_1 = "NO";
    CCU2D spi_channel_index_1407_add_4_3 (.A0(spi_channel_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24805), .COUT(n24806), .S0(n39), 
          .S1(n38));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407_add_4_3.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1407_add_4_3.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1407_add_4_3.INJECT1_0 = "NO";
    defparam spi_channel_index_1407_add_4_3.INJECT1_1 = "NO";
    CCU2D mic_divider_1415_add_4_3 (.A0(mic_divider[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24833), .COUT(n24834), .S0(n39_adj_3399), 
          .S1(n38_adj_3400));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_divider_1415_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_divider_1415_add_4_3.INJECT1_0 = "NO";
    defparam mic_divider_1415_add_4_3.INJECT1_1 = "NO";
    LUT4 i14197_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[21]), 
         .C(status_hold[20]), .Z(n25656)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14197_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_184 (.A(ev_run_hold_s5[25]), .B(us_tx_c_25), .C(init_shadow[25]), 
         .D(swap_now_s5), .Z(n14129)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_184.init = 16'h5a66;
    LUT4 i7_4_lut_adj_185 (.A(ev_run_hold_s5[26]), .B(us_tx_c_26), .C(init_shadow[26]), 
         .D(swap_now_s5), .Z(n14131)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_185.init = 16'h5a66;
    FD1P3IX ev_bit_i9 (.D(ev_bit[8]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i9.GSR = "DISABLED";
    FD1P3IX init_shadow_i2 (.D(n15608), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i2.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_186 (.A(ev_run_hold_s5[27]), .B(us_tx_c_27), .C(init_shadow[27]), 
         .D(swap_now_s5), .Z(n14133)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_186.init = 16'h5a66;
    LUT4 i7_4_lut_adj_187 (.A(ev_run_hold_s5[28]), .B(us_tx_c_28), .C(init_shadow[28]), 
         .D(swap_now_s5), .Z(n14135)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_187.init = 16'h5a66;
    LUT4 i7_4_lut_adj_188 (.A(ev_run_hold_s5[29]), .B(us_tx_c_29), .C(init_shadow[29]), 
         .D(swap_now_s5), .Z(n14137)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_188.init = 16'h5a66;
    FD1P3IX init_shadow_i1 (.D(n15602), .SP(pll_clk_enable_776), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i1.GSR = "DISABLED";
    FD1P3IX ev_ch_i6 (.D(ev_ch_6__N_2150[6]), .SP(pll_clk_enable_650), .CD(n18621), 
            .CK(pll_clk), .Q(ev_ch[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_ch_i6.GSR = "DISABLED";
    FD1P3IX ev_ch_i5 (.D(ev_ch_6__N_2150[5]), .SP(pll_clk_enable_650), .CD(n18621), 
            .CK(pll_clk), .Q(ev_ch[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_ch_i5.GSR = "DISABLED";
    FD1P3IX ev_ch_i4 (.D(ev_ch_6__N_2150[4]), .SP(pll_clk_enable_650), .CD(n18621), 
            .CK(pll_clk), .Q(ev_ch[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_ch_i4.GSR = "DISABLED";
    FD1P3IX ev_ch_i3 (.D(ev_ch_6__N_2150[3]), .SP(pll_clk_enable_650), .CD(n18621), 
            .CK(pll_clk), .Q(ev_ch[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_ch_i3.GSR = "DISABLED";
    FD1P3IX ev_ch_i2 (.D(ev_ch_6__N_2150[2]), .SP(pll_clk_enable_650), .CD(n18621), 
            .CK(pll_clk), .Q(ev_ch[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_ch_i2.GSR = "DISABLED";
    FD1P3IX ev_ch_i1 (.D(ev_ch_6__N_2150[1]), .SP(pll_clk_enable_650), .CD(n18621), 
            .CK(pll_clk), .Q(ev_ch[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_ch_i1.GSR = "DISABLED";
    FD1P3AX ev_state_i1 (.D(ev_state_3__N_710[1]), .SP(pll_clk_enable_777), 
            .CK(pll_clk), .Q(ev_state[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_state_i1.GSR = "DISABLED";
    FD1P3JX ws2812_settle_i0_i3 (.D(n26104), .SP(pll_clk_enable_661), .PD(pll_clk_enable_29), 
            .CK(pll_clk), .Q(ws2812_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ws2812_settle_i0_i3.GSR = "DISABLED";
    FD1P3JX ws2812_settle_i0_i2 (.D(n3882), .SP(pll_clk_enable_661), .PD(pll_clk_enable_29), 
            .CK(pll_clk), .Q(ws2812_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ws2812_settle_i0_i2.GSR = "DISABLED";
    FD1P3JX ws2812_settle_i0_i1 (.D(n25722), .SP(pll_clk_enable_661), .PD(pll_clk_enable_29), 
            .CK(pll_clk), .Q(ws2812_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ws2812_settle_i0_i1.GSR = "DISABLED";
    LUT4 i14196_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[19]), 
         .C(status_hold[18]), .Z(n25655)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14196_3_lut_3_lut.init = 16'he4e4;
    FD1P3IX spi_channel_field_1408__i0 (.D(n23000), .SP(spi1_sck_c_enable_196), 
            .CD(n22989), .CK(spi1_sck_c), .Q(spi_channel_field[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[78:102])
    defparam spi_channel_field_1408__i0.GSR = "ENABLED";
    FD1P3AX ws2812_toggle_spi_494 (.D(ws2812_toggle_spi_N_2758), .SP(spi1_sck_c_enable_187), 
            .CK(spi1_sck_c), .Q(ws2812_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam ws2812_toggle_spi_494.GSR = "DISABLED";
    FD1P3AX stop_toggle_spi_493 (.D(stop_toggle_spi_N_2738), .SP(spi1_sck_c_enable_188), 
            .CK(spi1_sck_c), .Q(stop_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam stop_toggle_spi_493.GSR = "DISABLED";
    FD1S3IX wrap_s2_499 (.D(wrap_s2_N_2799), .CK(pll_clk), .CD(n18604), 
            .Q(wrap_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam wrap_s2_499.GSR = "DISABLED";
    FD1P3AX invalid_frame_spi_492 (.D(invalid_frame_spi_N_2745), .SP(spi1_sck_c_enable_189), 
            .CK(spi1_sck_c), .Q(invalid_frame_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam invalid_frame_spi_492.GSR = "DISABLED";
    FD1P3IX ev_bit_i83 (.D(ev_bit[82]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i83.GSR = "DISABLED";
    FD1P3IX ev_bit_i82 (.D(ev_bit[81]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i82.GSR = "DISABLED";
    FD1P3IX ev_bit_i81 (.D(ev_bit[80]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i81.GSR = "DISABLED";
    FD1P3IX ev_bit_i80 (.D(ev_bit[79]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i80.GSR = "DISABLED";
    FD1P3IX ev_bit_i79 (.D(ev_bit[78]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i79.GSR = "DISABLED";
    FD1P3IX ev_bit_i78 (.D(ev_bit[77]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i78.GSR = "DISABLED";
    FD1P3IX ev_bit_i77 (.D(ev_bit[76]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i77.GSR = "DISABLED";
    FD1P3IX ev_bit_i76 (.D(ev_bit[75]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i76.GSR = "DISABLED";
    FD1P3IX ev_bit_i75 (.D(ev_bit[74]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i75.GSR = "DISABLED";
    FD1P3IX ev_bit_i74 (.D(ev_bit[73]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i74.GSR = "DISABLED";
    FD1P3IX ev_bit_i73 (.D(ev_bit[72]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i73.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_189 (.A(ev_run_hold_s5[30]), .B(us_tx_c_30), .C(init_shadow[30]), 
         .D(swap_now_s5), .Z(n14139)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_189.init = 16'h5a66;
    FD1P3IX ev_bit_i72 (.D(ev_bit[71]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i72.GSR = "DISABLED";
    FD1P3IX ev_bit_i71 (.D(ev_bit[70]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i71.GSR = "DISABLED";
    FD1P3IX ev_bit_i70 (.D(ev_bit[69]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i70.GSR = "DISABLED";
    FD1P3IX ev_bit_i69 (.D(ev_bit[68]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i69.GSR = "DISABLED";
    FD1P3IX ev_bit_i68 (.D(ev_bit[67]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i68.GSR = "DISABLED";
    FD1P3IX ev_bit_i67 (.D(ev_bit[66]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i67.GSR = "DISABLED";
    FD1P3IX ev_bit_i66 (.D(ev_bit[65]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i66.GSR = "DISABLED";
    FD1P3IX ev_bit_i65 (.D(ev_bit[64]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i65.GSR = "DISABLED";
    FD1P3IX ev_bit_i64 (.D(ev_bit[63]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i64.GSR = "DISABLED";
    FD1P3IX ev_bit_i63 (.D(ev_bit[62]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i63.GSR = "DISABLED";
    FD1P3IX ev_bit_i62 (.D(ev_bit[61]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i62.GSR = "DISABLED";
    FD1P3IX ev_bit_i61 (.D(ev_bit[60]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i61.GSR = "DISABLED";
    FD1P3IX ev_bit_i60 (.D(ev_bit[59]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i60.GSR = "DISABLED";
    FD1P3IX ev_bit_i59 (.D(ev_bit[58]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i59.GSR = "DISABLED";
    FD1P3IX ev_bit_i58 (.D(ev_bit[57]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i58.GSR = "DISABLED";
    FD1P3IX ev_bit_i57 (.D(ev_bit[56]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i57.GSR = "DISABLED";
    FD1P3IX ev_bit_i54 (.D(ev_bit[53]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i54.GSR = "DISABLED";
    FD1P3IX ev_bit_i53 (.D(ev_bit[52]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i53.GSR = "DISABLED";
    FD1P3IX ev_bit_i52 (.D(ev_bit[51]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i52.GSR = "DISABLED";
    LUT4 i7079_1_lut (.A(phase_step_s1), .Z(n18604)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i7079_1_lut.init = 16'h5555;
    FD1P3IX ev_bit_i51 (.D(ev_bit[50]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i51.GSR = "DISABLED";
    FD1P3IX ev_bit_i50 (.D(ev_bit[49]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i50.GSR = "DISABLED";
    LUT4 i14195_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[17]), 
         .C(status_hold[16]), .Z(n25654)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14195_3_lut_3_lut.init = 16'he4e4;
    PFUMX i14242 (.BLUT(n25687), .ALUT(n25688), .C0(n26080), .Z(n25701));
    LUT4 i7_4_lut_adj_190 (.A(global_phase_s2[0]), .B(n14_adj_3431), .C(n10_adj_3432), 
         .D(global_phase_s2[6]), .Z(wrap_s2_N_2799)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut_adj_190.init = 16'h8000;
    CCU2D add_340_9 (.A0(ev_clear_addr[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24781), .S0(ev_clear_addr_7__N_2439[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[34:54])
    defparam add_340_9.INIT0 = 16'h5aaa;
    defparam add_340_9.INIT1 = 16'h0000;
    defparam add_340_9.INJECT1_0 = "NO";
    defparam add_340_9.INJECT1_1 = "NO";
    LUT4 i14194_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[15]), 
         .C(status_hold[14]), .Z(n25653)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14194_3_lut_3_lut.init = 16'he4e4;
    LUT4 i6_4_lut_adj_191 (.A(global_phase_s2[3]), .B(global_phase_s2[1]), 
         .C(global_phase_s2[5]), .D(global_phase_s2[7]), .Z(n14_adj_3431)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_191.init = 16'h8000;
    LUT4 i2_2_lut (.A(global_phase_s2[2]), .B(global_phase_s2[4]), .Z(n10_adj_3432)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut.init = 16'h8888;
    LUT4 i6137_1_lut (.A(ws2812_enable), .Z(n17658)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i6137_1_lut.init = 16'h5555;
    LUT4 i7_4_lut_adj_192 (.A(ev_run_hold_s5[31]), .B(us_tx_c_31), .C(init_shadow[31]), 
         .D(swap_now_s5), .Z(n14141)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_192.init = 16'h5a66;
    FD1P3IX frame_settle__i1 (.D(n3505), .SP(pll_clk_enable_696), .CD(n14617), 
            .CK(pll_clk), .Q(frame_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam frame_settle__i1.GSR = "DISABLED";
    LUT4 i14193_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[13]), 
         .C(status_hold[12]), .Z(n25652)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14193_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14192_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[11]), 
         .C(status_hold[10]), .Z(n25651)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14192_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_193 (.A(ev_run_hold_s5[32]), .B(us_tx_c_32), .C(init_shadow[32]), 
         .D(swap_now_s5), .Z(n14143)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_193.init = 16'h5a66;
    LUT4 mux_313_i7_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[6]), 
         .D(pending_sequence[6]), .Z(accepted_sequence_31__N_1135[6])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i7_3_lut_4_lut.init = 16'hfe10;
    LUT4 i7_4_lut_adj_194 (.A(ev_run_hold_s5[33]), .B(us_tx_c_33), .C(init_shadow[33]), 
         .D(swap_now_s5), .Z(n14145)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_194.init = 16'h5a66;
    LUT4 i14191_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[9]), 
         .C(status_hold[8]), .Z(n25650)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14191_3_lut_3_lut.init = 16'he4e4;
    LUT4 mux_313_i8_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[7]), 
         .D(pending_sequence[7]), .Z(accepted_sequence_31__N_1135[7])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i8_3_lut_4_lut.init = 16'hfe10;
    LUT4 i1_2_lut_rep_88_4_lut (.A(n121), .B(n87), .C(n26065), .D(spi1_sck_c_enable_185), 
         .Z(n26022)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam i1_2_lut_rep_88_4_lut.init = 16'h0400;
    LUT4 i1_4_lut_adj_195 (.A(spi_command[4]), .B(spi1_sck_c_enable_170), 
         .C(n6_adj_3423), .D(spi_command[0]), .Z(spi1_sck_c_enable_189)) /* synthesis lut_function=(A (B)+!A (B (C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i1_4_lut_adj_195.init = 16'hc8cc;
    LUT4 mux_313_i9_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[8]), 
         .D(pending_sequence[8]), .Z(accepted_sequence_31__N_1135[8])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i9_3_lut_4_lut.init = 16'hfe10;
    LUT4 i2_4_lut_adj_196 (.A(n23272), .B(spi_command[1]), .C(n25575), 
         .D(spi_command[5]), .Z(n6_adj_3423)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_4_lut_adj_196.init = 16'hfffe;
    LUT4 i2_3_lut_4_lut_adj_197 (.A(n26094), .B(n26068), .C(spi_byte_count[15]), 
         .D(n26098), .Z(n25345)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(144[17:37])
    defparam i2_3_lut_4_lut_adj_197.init = 16'hfffe;
    LUT4 i14117_2_lut (.A(spi_command[2]), .B(spi_command[6]), .Z(n25575)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14117_2_lut.init = 16'heeee;
    FD1P3IX frame_settle__i2 (.D(n3504), .SP(pll_clk_enable_696), .CD(n14617), 
            .CK(pll_clk), .Q(frame_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam frame_settle__i2.GSR = "DISABLED";
    PFUMX i14381 (.BLUT(n25894), .ALUT(n25893), .C0(spi1_miso_N_2715[5]), 
          .Z(n25895));
    LUT4 i14190_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[7]), 
         .C(status_hold[6]), .Z(n25649)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14190_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_198 (.A(ev_run_hold_s5[34]), .B(us_tx_c_34), .C(init_shadow[34]), 
         .D(swap_now_s5), .Z(n14147)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_198.init = 16'h5a66;
    LUT4 i4601_4_lut (.A(ev_ch[6]), .B(staging_rd_addr[6]), .C(n18417), 
         .D(n26077), .Z(staging_rd_addr_6__N_914[6])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(565[9] 638[16])
    defparam i4601_4_lut.init = 16'hcac0;
    LUT4 i7_4_lut_adj_199 (.A(ev_run_hold_s5[35]), .B(us_tx_c_35), .C(init_shadow[35]), 
         .D(swap_now_s5), .Z(n14149)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_199.init = 16'h5a66;
    LUT4 i14189_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[5]), 
         .C(status_hold[4]), .Z(n25648)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14189_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14188_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[3]), 
         .C(status_hold[2]), .Z(n25647)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14188_3_lut_3_lut.init = 16'he4e4;
    PFUMX i14246 (.BLUT(n25695), .ALUT(n25696), .C0(n26080), .Z(n25705));
    LUT4 i14187_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[1]), 
         .C(status_hold[0]), .Z(n25646)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[18:44])
    defparam i14187_3_lut_3_lut.init = 16'he4e4;
    LUT4 i2_3_lut_4_lut_adj_200 (.A(running), .B(pll_locked), .C(pll_clk_enable_28), 
         .D(phase_step_s5), .Z(pll_clk_enable_305)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_3_lut_4_lut_adj_200.init = 16'hfff7;
    LUT4 i7_4_lut_adj_201 (.A(ev_run_hold_s5[36]), .B(us_tx_c_36), .C(init_shadow[36]), 
         .D(swap_now_s5), .Z(n14151)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_201.init = 16'h5a66;
    LUT4 i2_3_lut_rep_92 (.A(n17384), .B(spi_command[1]), .C(spi_command[0]), 
         .Z(n26026)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;
    defparam i2_3_lut_rep_92.init = 16'h0202;
    LUT4 stop_toggle_sync_I_0_2_lut_rep_147 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .Z(pll_clk_enable_28)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(265[23:61])
    defparam stop_toggle_sync_I_0_2_lut_rep_147.init = 16'h6666;
    PFUMX i14243 (.BLUT(n25689), .ALUT(n25690), .C0(n26080), .Z(n25702));
    LUT4 i7_4_lut_adj_202 (.A(ev_run_hold_s5[37]), .B(us_tx_c_37), .C(init_shadow[37]), 
         .D(swap_now_s5), .Z(n14153)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_202.init = 16'h5a66;
    LUT4 i7_4_lut_adj_203 (.A(ev_run_hold_s5[38]), .B(us_tx_c_38), .C(init_shadow[38]), 
         .D(swap_now_s5), .Z(n14155)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_203.init = 16'h5a66;
    LUT4 i7_4_lut_adj_204 (.A(ev_run_hold_s5[39]), .B(us_tx_c_39), .C(init_shadow[39]), 
         .D(swap_now_s5), .Z(n14157)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_204.init = 16'h5a66;
    LUT4 i7_4_lut_adj_205 (.A(ev_run_hold_s5[40]), .B(us_tx_c_40), .C(init_shadow[40]), 
         .D(swap_now_s5), .Z(n14159)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_205.init = 16'h5a66;
    PFUMX i14244 (.BLUT(n25691), .ALUT(n25692), .C0(n26080), .Z(n25703));
    LUT4 i1_2_lut_3_lut_4_lut_adj_206 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(pll_locked), .D(running), .Z(n12560)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(265[23:61])
    defparam i1_2_lut_3_lut_4_lut_adj_206.init = 16'h6fff;
    LUT4 i1_3_lut_4_lut_adj_207 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(wrap_s2), .D(swap_pending), .Z(swap_pending_N_2813)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(265[23:61])
    defparam i1_3_lut_4_lut_adj_207.init = 16'h0900;
    LUT4 i2_3_lut_rep_148 (.A(spi_command[6]), .B(spi_command[5]), .C(spi_command[2]), 
         .Z(n26082)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i2_3_lut_rep_148.init = 16'hfefe;
    LUT4 i1_2_lut_rep_124_4_lut (.A(spi_command[6]), .B(spi_command[5]), 
         .C(spi_command[2]), .D(n23272), .Z(n26058)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i1_2_lut_rep_124_4_lut.init = 16'hfffe;
    LUT4 i2_3_lut_rep_149 (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .Z(n26083)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[17:37])
    defparam i2_3_lut_rep_149.init = 16'hfefe;
    LUT4 i7_4_lut_adj_208 (.A(ev_run_hold_s5[41]), .B(us_tx_c_41), .C(init_shadow[41]), 
         .D(swap_now_s5), .Z(n14161)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_208.init = 16'h5a66;
    LUT4 i7_4_lut_adj_209 (.A(ev_run_hold_s5[42]), .B(us_tx_c_42), .C(init_shadow[42]), 
         .D(swap_now_s5), .Z(n14163)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_209.init = 16'h5a66;
    LUT4 i10235_3_lut_4_lut_4_lut_3_lut (.A(frame_settle[1]), .B(frame_settle[2]), 
         .C(frame_settle[0]), .Z(n3504)) /* synthesis lut_function=(A (B)+!A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[17:37])
    defparam i10235_3_lut_4_lut_4_lut_3_lut.init = 16'hc9c9;
    LUT4 i10234_3_lut_3_lut_2_lut (.A(frame_settle[1]), .B(frame_settle[0]), 
         .Z(n3505)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[17:37])
    defparam i10234_3_lut_3_lut_2_lut.init = 16'h9999;
    LUT4 i6121_2_lut_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .D(frame_settle[0]), .Z(n17639)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[17:37])
    defparam i6121_2_lut_4_lut.init = 16'h00fe;
    LUT4 mux_313_i10_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[9]), 
         .D(pending_sequence[9]), .Z(accepted_sequence_31__N_1135[9])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i10_3_lut_4_lut.init = 16'hfe10;
    LUT4 i1_2_lut_rep_125_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(pll_clk_enable_696)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[17:37])
    defparam i1_2_lut_rep_125_4_lut.init = 16'hfffe;
    LUT4 frame_toggle_sync_I_0_2_lut_rep_150 (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .Z(pll_clk_enable_30)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(505[13:51])
    defparam frame_toggle_sync_I_0_2_lut_rep_150.init = 16'h6666;
    LUT4 i3108_2_lut_3_lut_4_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(stop_toggle_seen), .D(stop_toggle_sync), .Z(n14617)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D)))+!A !(B+!(C (D)+!C !(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(505[13:51])
    defparam i3108_2_lut_3_lut_4_lut.init = 16'h6ff6;
    LUT4 i7_4_lut_adj_210 (.A(ev_run_hold_s5[43]), .B(us_tx_c_43), .C(init_shadow[43]), 
         .D(swap_now_s5), .Z(n14165)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_210.init = 16'h5a66;
    FD1P3IX init_shadow_i81 (.D(n16086), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i81.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_3_lut_4_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n26083), .D(frame_settle[0]), .Z(pll_clk_enable_15)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(505[13:51])
    defparam i1_3_lut_4_lut_3_lut_4_lut.init = 16'h0900;
    LUT4 n25924_bdd_2_lut_3_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n25924), .Z(frame_settle_3__N_2130[3])) /* synthesis lut_function=(A ((C)+!B)+!A (B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(505[13:51])
    defparam n25924_bdd_2_lut_3_lut.init = 16'hf6f6;
    FD1P3IX init_shadow_i80 (.D(n16078), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i80.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_211 (.A(n17439), .B(init_shadow[68]), .C(ev_bit[68]), 
         .Z(n16004)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_211.init = 16'hecec;
    LUT4 i2_3_lut_rep_152 (.A(ws2812_settle[1]), .B(ws2812_settle[3]), .C(ws2812_settle[2]), 
         .Z(n26086)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(539[17:38])
    defparam i2_3_lut_rep_152.init = 16'hfefe;
    LUT4 i1_2_lut_rep_126_4_lut (.A(ws2812_settle[1]), .B(ws2812_settle[3]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(n26060)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(539[17:38])
    defparam i1_2_lut_rep_126_4_lut.init = 16'hfeff;
    LUT4 i6124_2_lut_4_lut (.A(ws2812_settle[1]), .B(ws2812_settle[3]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(n17643)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(539[17:38])
    defparam i6124_2_lut_4_lut.init = 16'h00fe;
    LUT4 i1_2_lut_3_lut_adj_212 (.A(n180), .B(spi_rgb_index[0]), .C(spi_rgb_index[1]), 
         .Z(spi1_sck_c_enable_144)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(398[46:66])
    defparam i1_2_lut_3_lut_adj_212.init = 16'h0202;
    LUT4 i10239_3_lut_4_lut_4_lut_3_lut (.A(ws2812_settle[1]), .B(ws2812_settle[2]), 
         .C(ws2812_settle[0]), .Z(n3882)) /* synthesis lut_function=(A (B)+!A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(539[17:38])
    defparam i10239_3_lut_4_lut_4_lut_3_lut.init = 16'hc9c9;
    LUT4 ws2812_toggle_sync_I_0_2_lut_rep_153 (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .Z(pll_clk_enable_29)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[13:53])
    defparam ws2812_toggle_sync_I_0_2_lut_rep_153.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_213 (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .C(n26086), .D(ws2812_settle[0]), .Z(pll_clk_enable_661)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(526[13:53])
    defparam i1_3_lut_4_lut_adj_213.init = 16'hfff6;
    LUT4 i1_2_lut_3_lut_adj_214 (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[4]), .Z(n4_adj_3418)) /* synthesis lut_function=(A (B+(C))+!A (C)) */ ;
    defparam i1_2_lut_3_lut_adj_214.init = 16'hf8f8;
    LUT4 i10467_3_lut_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(n21575), .D(spi_byte_count[7]), .Z(n99)) /* synthesis lut_function=(!(A (B (D)+!B (C (D)))+!A (C (D)))) */ ;
    defparam i10467_3_lut_4_lut.init = 16'h07ff;
    LUT4 i1624_2_lut_rep_154 (.A(ev_ch[1]), .B(ev_ch[0]), .Z(n26088)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(606[27:39])
    defparam i1624_2_lut_rep_154.init = 16'h8888;
    LUT4 i1631_2_lut_rep_128_3_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[2]), 
         .Z(n26062)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(606[27:39])
    defparam i1631_2_lut_rep_128_3_lut.init = 16'h8080;
    LUT4 i7_4_lut_adj_215 (.A(ev_run_hold_s5[44]), .B(us_tx_c_44), .C(init_shadow[44]), 
         .D(swap_now_s5), .Z(n14167)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_215.init = 16'h5a66;
    LUT4 i1629_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[2]), 
         .Z(ev_ch_6__N_2150[2])) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(606[27:39])
    defparam i1629_2_lut_3_lut.init = 16'h7878;
    LUT4 i1638_2_lut_rep_112_3_lut_4_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(n26046)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(606[27:39])
    defparam i1638_2_lut_rep_112_3_lut_4_lut.init = 16'h8000;
    LUT4 i1_4_lut_adj_216 (.A(n5_adj_3422), .B(n26038), .C(n26098), .D(n26045), 
         .Z(spi_write)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i1_4_lut_adj_216.init = 16'h0008;
    LUT4 i1636_2_lut_3_lut_4_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(ev_ch_6__N_2150[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(606[27:39])
    defparam i1636_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i14077_2_lut_rep_155 (.A(spi_command[1]), .B(spi_command[0]), .Z(n26089)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i14077_2_lut_rep_155.init = 16'h6666;
    LUT4 i7_4_lut_adj_217 (.A(ev_run_hold_s5[45]), .B(us_tx_c_45), .C(init_shadow[45]), 
         .D(swap_now_s5), .Z(n14169)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_217.init = 16'h5a66;
    LUT4 i7_4_lut_adj_218 (.A(ev_run_hold_s5[46]), .B(us_tx_c_46), .C(init_shadow[46]), 
         .D(swap_now_s5), .Z(n14171)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_218.init = 16'h5a66;
    LUT4 i7_4_lut_adj_219 (.A(ev_run_hold_s5[47]), .B(us_tx_c_47), .C(init_shadow[47]), 
         .D(swap_now_s5), .Z(n14173)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_219.init = 16'h5a66;
    LUT4 i7_4_lut_adj_220 (.A(ev_run_hold_s5[48]), .B(us_tx_c_48), .C(init_shadow[48]), 
         .D(swap_now_s5), .Z(n14175)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_220.init = 16'h5a66;
    LUT4 i7_4_lut_adj_221 (.A(ev_run_hold_s5[49]), .B(us_tx_c_49), .C(init_shadow[49]), 
         .D(swap_now_s5), .Z(n14177)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_221.init = 16'h5a66;
    LUT4 i7_4_lut_adj_222 (.A(ev_run_hold_s5[50]), .B(us_tx_c_50), .C(init_shadow[50]), 
         .D(swap_now_s5), .Z(n14179)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_222.init = 16'h5a66;
    LUT4 i7_4_lut_adj_223 (.A(ev_run_hold_s5[51]), .B(us_tx_c_51), .C(init_shadow[51]), 
         .D(swap_now_s5), .Z(n14181)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_223.init = 16'h5a66;
    LUT4 mux_313_i11_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[10]), 
         .D(pending_sequence[10]), .Z(accepted_sequence_31__N_1135[10])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i11_3_lut_4_lut.init = 16'hfe10;
    LUT4 i7_4_lut_adj_224 (.A(ev_run_hold_s5[52]), .B(us_tx_c_52), .C(init_shadow[52]), 
         .D(swap_now_s5), .Z(n14183)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_224.init = 16'h5a66;
    LUT4 i767_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_519)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(672[18] 674[12])
    defparam i767_2_lut.init = 16'h4444;
    LUT4 i1_2_lut_3_lut_4_lut_adj_225 (.A(spi_command[1]), .B(spi_command[0]), 
         .C(spi1_sck_c_enable_170), .D(n17384), .Z(spi1_sck_c_enable_200)) /* synthesis lut_function=(A (B (C (D)))+!A !(B+!(C (D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_225.init = 16'h9000;
    LUT4 i2_4_lut_adj_226 (.A(update_flags_sync[1]), .B(pll_clk_enable_29), 
         .C(pll_clk_enable_15), .D(n26060), .Z(pll_clk_enable_16)) /* synthesis lut_function=(A (B+(C+!(D)))+!A (B+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(535[18] 549[12])
    defparam i2_4_lut_adj_226.init = 16'hecff;
    FD1P3AX spi_channel_index_1407__i1 (.D(n39), .SP(spi1_sck_c_enable_195), 
            .CK(spi1_sck_c), .Q(spi_channel_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407__i1.GSR = "ENABLED";
    LUT4 mux_313_i12_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[11]), 
         .D(pending_sequence[11]), .Z(accepted_sequence_31__N_1135[11])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i12_3_lut_4_lut.init = 16'hfe10;
    FD1P3AX spi_channel_index_1407__i2 (.D(n38), .SP(spi1_sck_c_enable_195), 
            .CK(spi1_sck_c), .Q(spi_channel_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407__i2.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1407__i3 (.D(n37), .SP(spi1_sck_c_enable_195), 
            .CK(spi1_sck_c), .Q(spi_channel_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407__i3.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1407__i4 (.D(n36), .SP(spi1_sck_c_enable_195), 
            .CK(spi1_sck_c), .Q(spi_channel_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407__i4.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1407__i5 (.D(n35), .SP(spi1_sck_c_enable_195), 
            .CK(spi1_sck_c), .Q(spi_channel_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407__i5.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1407__i6 (.D(n34), .SP(spi1_sck_c_enable_195), 
            .CK(spi1_sck_c), .Q(spi_channel_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407__i6.GSR = "ENABLED";
    FD1P3AX fpga_time_1412__i1 (.D(n164), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i1.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_227 (.A(ev_run_hold_s5[53]), .B(us_tx_c_53), .C(init_shadow[53]), 
         .D(swap_now_s5), .Z(n14185)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_227.init = 16'h5a66;
    LUT4 i7_4_lut_adj_228 (.A(ev_run_hold_s5[54]), .B(us_tx_c_54), .C(init_shadow[54]), 
         .D(swap_now_s5), .Z(n14187)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_228.init = 16'h5a66;
    LUT4 i7_4_lut_adj_229 (.A(ev_run_hold_s5[55]), .B(us_tx_c_55), .C(init_shadow[55]), 
         .D(swap_now_s5), .Z(n14189)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_229.init = 16'h5a66;
    LUT4 i1_2_lut_3_lut_adj_230 (.A(n180), .B(spi_rgb_index[0]), .C(spi_rgb_index[1]), 
         .Z(spi1_sck_c_enable_152)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(398[46:66])
    defparam i1_2_lut_3_lut_adj_230.init = 16'h2020;
    FD1P3AX fpga_time_1412__i2 (.D(n163), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i2.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i3 (.D(n162), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i3.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i4 (.D(n161), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i4.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i5 (.D(n160), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i5.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i6 (.D(n159), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i6.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i7 (.D(n158), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i7.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i8 (.D(n157), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i8.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i9 (.D(n156), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i9.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i10 (.D(n155), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[10])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i10.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i11 (.D(n154), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[11])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i11.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i12 (.D(n153), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[12])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i12.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i13 (.D(n152), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[13])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i13.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i14 (.D(n151), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[14])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i14.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i15 (.D(n150), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[15])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i15.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i16 (.D(n149), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[16])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i16.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i17 (.D(n148), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[17])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i17.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i18 (.D(n147), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[18])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i18.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i19 (.D(n146), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[19])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i19.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i20 (.D(n145), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[20])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i20.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i21 (.D(n144), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[21])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i21.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i22 (.D(n143), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[22])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i22.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i23 (.D(n142), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[23])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i23.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i24 (.D(n141), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[24])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i24.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i25 (.D(n140), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[25])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i25.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i26 (.D(n139), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[26])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i26.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i27 (.D(n138), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[27])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i27.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i28 (.D(n137), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[28])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i28.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i29 (.D(n136), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[29])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i29.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i30 (.D(n135), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[30])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i30.GSR = "DISABLED";
    FD1P3AX fpga_time_1412__i31 (.D(n134), .SP(pll_clk_enable_730), .CK(pll_clk), 
            .Q(fpga_time[31])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412__i31.GSR = "DISABLED";
    FD1P3AX status_bit_index_1406__i1 (.D(n39_adj_3393), .SP(spi1_sck_N_413_enable_7), 
            .CK(spi1_sck_N_413), .Q(status_bit_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406__i1.GSR = "ENABLED";
    LUT4 i7_4_lut_adj_231 (.A(ev_run_hold_s5[56]), .B(us_tx_c_56), .C(init_shadow[56]), 
         .D(swap_now_s5), .Z(n14191)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_231.init = 16'h5a66;
    FD1P3AX status_bit_index_1406__i2 (.D(n38_adj_3392), .SP(spi1_sck_N_413_enable_7), 
            .CK(spi1_sck_N_413), .Q(status_bit_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406__i2.GSR = "ENABLED";
    FD1P3AX status_bit_index_1406__i3 (.D(n37_adj_3391), .SP(spi1_sck_N_413_enable_7), 
            .CK(spi1_sck_N_413), .Q(status_bit_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406__i3.GSR = "ENABLED";
    FD1P3AX status_bit_index_1406__i4 (.D(n36_adj_3390), .SP(spi1_sck_N_413_enable_7), 
            .CK(spi1_sck_N_413), .Q(status_bit_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406__i4.GSR = "ENABLED";
    FD1P3AX status_bit_index_1406__i5 (.D(n35_adj_3389), .SP(spi1_sck_N_413_enable_7), 
            .CK(spi1_sck_N_413), .Q(status_bit_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406__i5.GSR = "ENABLED";
    FD1P3AX status_bit_index_1406__i6 (.D(n34_adj_3388), .SP(spi1_sck_N_413_enable_7), 
            .CK(spi1_sck_N_413), .Q(status_bit_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406__i6.GSR = "ENABLED";
    FD1P3AX global_phase_s2_1411__i1 (.D(n44), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411__i1.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_232 (.A(ev_run_hold_s5[57]), .B(us_tx_c_57), .C(init_shadow[57]), 
         .D(swap_now_s5), .Z(n14193)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_232.init = 16'h5a66;
    FD1P3AX global_phase_s2_1411__i2 (.D(n43), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411__i2.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1411__i3 (.D(n42), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411__i3.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1411__i4 (.D(n41), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411__i4.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1411__i5 (.D(n40_adj_3397), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411__i5.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1411__i6 (.D(n39_adj_3396), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411__i6.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1411__i7 (.D(n38_adj_3395), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411__i7.GSR = "DISABLED";
    FD1S3AX spi_bit_count_1410__i1 (.D(n19_adj_3386), .CK(spi1_sck_c), .Q(spi_bit_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam spi_bit_count_1410__i1.GSR = "ENABLED";
    LUT4 i7_4_lut_adj_233 (.A(ev_run_hold_s5[58]), .B(us_tx_c_58), .C(init_shadow[58]), 
         .D(swap_now_s5), .Z(n14195)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_233.init = 16'h5a66;
    LUT4 i7_4_lut_adj_234 (.A(ev_run_hold_s5[59]), .B(us_tx_c_59), .C(init_shadow[59]), 
         .D(swap_now_s5), .Z(n14197)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_234.init = 16'h5a66;
    FD1P3IX ev_bit_i30 (.D(ev_bit[29]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i30.GSR = "DISABLED";
    FD1P3IX ev_bit_i29 (.D(ev_bit[28]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i29.GSR = "DISABLED";
    FD1P3IX ev_bit_i28 (.D(ev_bit[27]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i28.GSR = "DISABLED";
    FD1P3IX ev_bit_i27 (.D(ev_bit[26]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i27.GSR = "DISABLED";
    FD1P3IX ev_bit_i26 (.D(ev_bit[25]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i26.GSR = "DISABLED";
    FD1P3IX ev_bit_i25 (.D(ev_bit[24]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i25.GSR = "DISABLED";
    CCU2D add_340_7 (.A0(ev_clear_addr[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24780), .COUT(n24781), .S0(ev_clear_addr_7__N_2439[5]), 
          .S1(ev_clear_addr_7__N_2439[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[34:54])
    defparam add_340_7.INIT0 = 16'h5aaa;
    defparam add_340_7.INIT1 = 16'h5aaa;
    defparam add_340_7.INJECT1_0 = "NO";
    defparam add_340_7.INJECT1_1 = "NO";
    LUT4 i10243_3_lut (.A(ev_ch[0]), .B(ev_state[3]), .C(ev_state[0]), 
         .Z(ev_ch_6__N_722[0])) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(565[9] 638[16])
    defparam i10243_3_lut.init = 16'h4848;
    LUT4 n25931_bdd_2_lut_3_lut_4_lut (.A(spi_command[1]), .B(spi_command[0]), 
         .C(n25931), .D(n17384), .Z(invalid_frame_spi_N_2745)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C))+!A !(B (C)+!B !((D)+!C)))) */ ;
    defparam n25931_bdd_2_lut_3_lut_4_lut.init = 16'h60f0;
    LUT4 i7_4_lut_adj_235 (.A(ev_run_hold_s5[60]), .B(us_tx_c_60), .C(init_shadow[60]), 
         .D(swap_now_s5), .Z(n14199)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_235.init = 16'h5a66;
    LUT4 mux_313_i13_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[12]), 
         .D(pending_sequence[12]), .Z(accepted_sequence_31__N_1135[12])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i13_3_lut_4_lut.init = 16'hfe10;
    LUT4 i14341_4_lut (.A(ev_state[0]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(ev_state[3]), .Z(pll_clk_enable_390)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam i14341_4_lut.init = 16'h0400;
    LUT4 i13397_2_lut (.A(staging_q[8]), .B(staging_q[0]), .Z(build_sum_8__N_2255[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13397_2_lut.init = 16'h6666;
    LUT4 i7_4_lut_adj_236 (.A(ev_run_hold_s5[61]), .B(us_tx_c_61), .C(init_shadow[61]), 
         .D(swap_now_s5), .Z(n14201)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_236.init = 16'h5a66;
    LUT4 i7_4_lut_adj_237 (.A(ev_run_hold_s5[62]), .B(us_tx_c_62), .C(init_shadow[62]), 
         .D(swap_now_s5), .Z(n14203)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_237.init = 16'h5a66;
    LUT4 i7_4_lut_adj_238 (.A(ev_run_hold_s5[63]), .B(us_tx_c_63), .C(init_shadow[63]), 
         .D(swap_now_s5), .Z(n14205)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_238.init = 16'h5a66;
    LUT4 i7_4_lut_adj_239 (.A(ev_run_hold_s5[64]), .B(us_tx_c_64), .C(init_shadow[64]), 
         .D(swap_now_s5), .Z(n14207)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_239.init = 16'h5a66;
    LUT4 i7_4_lut_adj_240 (.A(ev_run_hold_s5[65]), .B(us_tx_c_65), .C(init_shadow[65]), 
         .D(swap_now_s5), .Z(n14209)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_240.init = 16'h5a66;
    LUT4 i7_4_lut_adj_241 (.A(ev_run_hold_s5[66]), .B(us_tx_c_66), .C(init_shadow[66]), 
         .D(swap_now_s5), .Z(n14211)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_241.init = 16'h5a66;
    LUT4 i7_4_lut_adj_242 (.A(ev_run_hold_s5[67]), .B(us_tx_c_67), .C(init_shadow[67]), 
         .D(swap_now_s5), .Z(n14213)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_242.init = 16'h5a66;
    LUT4 frame_toggle_spi_I_0_2_lut_4_lut (.A(n17384), .B(spi_command[1]), 
         .C(spi_command[0]), .D(frame_toggle_spi), .Z(frame_toggle_spi_N_2728)) /* synthesis lut_function=(A (B (D)+!B (C (D)+!C !(D)))+!A (D)) */ ;
    defparam frame_toggle_spi_I_0_2_lut_4_lut.init = 16'hfd02;
    LUT4 i2635_4_lut (.A(ev_ch[0]), .B(staging_rd_addr[0]), .C(n18417), 
         .D(n26077), .Z(staging_rd_addr_6__N_914[0])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(565[9] 638[16])
    defparam i2635_4_lut.init = 16'hcac0;
    LUT4 i7_4_lut_adj_243 (.A(ev_run_hold_s5[68]), .B(us_tx_c_68), .C(init_shadow[68]), 
         .D(swap_now_s5), .Z(n14215)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_243.init = 16'h5a66;
    LUT4 i1_2_lut_rep_156 (.A(fpga_cs_n_c), .B(spi_rgb_payload_byte_N_2770[6]), 
         .Z(n26090)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:33])
    defparam i1_2_lut_rep_156.init = 16'h4444;
    LUT4 mux_313_i14_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[13]), 
         .D(pending_sequence[13]), .Z(accepted_sequence_31__N_1135[13])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i14_3_lut_4_lut.init = 16'hfe10;
    LUT4 i7_4_lut_adj_244 (.A(ev_run_hold_s5[69]), .B(us_tx_c_69), .C(init_shadow[69]), 
         .D(swap_now_s5), .Z(n14217)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_244.init = 16'h5a66;
    LUT4 i7_4_lut_adj_245 (.A(ev_run_hold_s5[70]), .B(us_tx_c_70), .C(init_shadow[70]), 
         .D(swap_now_s5), .Z(n14219)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_245.init = 16'h5a66;
    LUT4 i7_4_lut_adj_246 (.A(ev_run_hold_s5[71]), .B(us_tx_c_71), .C(init_shadow[71]), 
         .D(swap_now_s5), .Z(n14221)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_246.init = 16'h5a66;
    LUT4 i7_4_lut_adj_247 (.A(ev_run_hold_s5[72]), .B(us_tx_c_72), .C(init_shadow[72]), 
         .D(swap_now_s5), .Z(n14223)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_247.init = 16'h5a66;
    LUT4 i7_4_lut_adj_248 (.A(ev_run_hold_s5[73]), .B(us_tx_c_73), .C(init_shadow[73]), 
         .D(swap_now_s5), .Z(n14225)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_248.init = 16'h5a66;
    LUT4 i7_4_lut_adj_249 (.A(ev_run_hold_s5[74]), .B(us_tx_c_74), .C(init_shadow[74]), 
         .D(swap_now_s5), .Z(n14227)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_249.init = 16'h5a66;
    LUT4 i7_4_lut_adj_250 (.A(ev_run_hold_s5[75]), .B(us_tx_c_75), .C(init_shadow[75]), 
         .D(swap_now_s5), .Z(n14229)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_250.init = 16'h5a66;
    LUT4 i7_4_lut_adj_251 (.A(ev_run_hold_s5[76]), .B(us_tx_c_76), .C(init_shadow[76]), 
         .D(swap_now_s5), .Z(n14231)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_251.init = 16'h5a66;
    CCU2D mic_divider_1415_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24833), .S1(n40_adj_3398));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415_add_4_1.INIT0 = 16'hF000;
    defparam mic_divider_1415_add_4_1.INIT1 = 16'h0555;
    defparam mic_divider_1415_add_4_1.INJECT1_0 = "NO";
    defparam mic_divider_1415_add_4_1.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_252 (.A(ev_run_hold_s5[77]), .B(us_tx_c_77), .C(init_shadow[77]), 
         .D(swap_now_s5), .Z(n14233)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_252.init = 16'h5a66;
    LUT4 i7_4_lut_adj_253 (.A(ev_run_hold_s5[78]), .B(us_tx_c_78), .C(init_shadow[78]), 
         .D(swap_now_s5), .Z(n14235)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_253.init = 16'h5a66;
    LUT4 i7_4_lut_adj_254 (.A(ev_run_hold_s5[79]), .B(us_tx_c_79), .C(init_shadow[79]), 
         .D(swap_now_s5), .Z(n14237)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_254.init = 16'h5a66;
    CCU2D global_phase_s2_1411_add_4_9 (.A0(global_phase_s2[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24832), .S0(n38_adj_3395));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411_add_4_9.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1411_add_4_9.INIT1 = 16'h0000;
    defparam global_phase_s2_1411_add_4_9.INJECT1_0 = "NO";
    defparam global_phase_s2_1411_add_4_9.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_255 (.A(ev_run_hold_s5[80]), .B(us_tx_c_80), .C(init_shadow[80]), 
         .D(swap_now_s5), .Z(n14239)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_255.init = 16'h5a66;
    LUT4 i7_4_lut_adj_256 (.A(ev_run_hold_s5[81]), .B(us_tx_c_81), .C(init_shadow[81]), 
         .D(swap_now_s5), .Z(n14241)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_256.init = 16'h5a66;
    LUT4 i7_4_lut_adj_257 (.A(ev_run_hold_s5[82]), .B(us_tx_c_82), .C(init_shadow[82]), 
         .D(swap_now_s5), .Z(n14243)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_257.init = 16'h5a66;
    CCU2D spi_channel_index_1407_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_channel_index[0]), .B1(spi_channel_index[6]), 
          .C1(n8_adj_3421), .D1(n25370), .COUT(n24805), .S1(n40));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(374[64:88])
    defparam spi_channel_index_1407_add_4_1.INIT0 = 16'hF000;
    defparam spi_channel_index_1407_add_4_1.INIT1 = 16'h5559;
    defparam spi_channel_index_1407_add_4_1.INJECT1_0 = "NO";
    defparam spi_channel_index_1407_add_4_1.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_258 (.A(ev_run_hold_s5[83]), .B(us_tx_c_83), .C(init_shadow[83]), 
         .D(swap_now_s5), .Z(n14245)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:28])
    defparam i7_4_lut_adj_258.init = 16'h5a66;
    LUT4 i14306_4_lut (.A(ev_clear_addr[0]), .B(ev_clear_addr[3]), .C(n25635), 
         .D(n25581), .Z(ev_clear_done_N_2802)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(579[34:58])
    defparam i14306_4_lut.init = 16'h4000;
    LUT4 mux_313_i15_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[14]), 
         .D(pending_sequence[14]), .Z(accepted_sequence_31__N_1135[14])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i15_3_lut_4_lut.init = 16'hfe10;
    LUT4 mux_313_i16_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[15]), 
         .D(pending_sequence[15]), .Z(accepted_sequence_31__N_1135[15])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i16_3_lut_4_lut.init = 16'hfe10;
    LUT4 mux_313_i17_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[16]), 
         .D(pending_sequence[16]), .Z(accepted_sequence_31__N_1135[16])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i17_3_lut_4_lut.init = 16'hfe10;
    LUT4 i1_2_lut_rep_129_3_lut (.A(fpga_cs_n_c), .B(spi_rgb_payload_byte_N_2770[6]), 
         .C(spi_channel_field[1]), .Z(n26063)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:33])
    defparam i1_2_lut_rep_129_3_lut.init = 16'h0404;
    LUT4 mux_313_i18_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[17]), 
         .D(pending_sequence[17]), .Z(accepted_sequence_31__N_1135[17])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i18_3_lut_4_lut.init = 16'hfe10;
    LUT4 mux_313_i19_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[18]), 
         .D(pending_sequence[18]), .Z(accepted_sequence_31__N_1135[18])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i19_3_lut_4_lut.init = 16'hfe10;
    LUT4 i14177_4_lut (.A(ev_clear_addr[4]), .B(ev_clear_addr[6]), .C(ev_clear_addr[5]), 
         .D(ev_clear_addr[2]), .Z(n25635)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14177_4_lut.init = 16'h8000;
    LUT4 i14123_2_lut (.A(ev_clear_addr[7]), .B(ev_clear_addr[1]), .Z(n25581)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14123_2_lut.init = 16'h8888;
    LUT4 i2_3_lut_rep_157 (.A(spi_bit_count[1]), .B(spi_bit_count[0]), .C(spi_bit_count[2]), 
         .Z(spi1_sck_c_enable_185)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam i2_3_lut_rep_157.init = 16'h8080;
    LUT4 i10157_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_771)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10157_2_lut.init = 16'h8888;
    LUT4 i14309_4_lut (.A(mic_divider[2]), .B(n12_adj_3430), .C(mic_divider[5]), 
         .D(mic_divider[6]), .Z(mic_tick_N_2800)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(652[21:43])
    defparam i14309_4_lut.init = 16'h0002;
    LUT4 i5_4_lut_adj_259 (.A(mic_divider[4]), .B(mic_divider[1]), .C(mic_divider[0]), 
         .D(mic_divider[3]), .Z(n12_adj_3430)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;
    defparam i5_4_lut_adj_259.init = 16'hbfff;
    LUT4 i1_2_lut_rep_133_4_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .D(spi_byte_count[0]), .Z(n26067)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam i1_2_lut_rep_133_4_lut.init = 16'h8000;
    LUT4 mic_clk_I_0_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(mic_clk_N_2724)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(672[18] 674[12])
    defparam mic_clk_I_0_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_adj_260 (.A(n17439), .B(init_shadow[76]), .C(ev_bit[76]), 
         .Z(n16052)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_260.init = 16'hecec;
    LUT4 i6_4_lut_adj_261 (.A(mic_sample_count[3]), .B(n12_adj_3433), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_610)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_adj_261.init = 16'h0800;
    LUT4 i1_2_lut_rep_130_4_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .D(fpga_cs_n_c), .Z(n26064)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam i1_2_lut_rep_130_4_lut.init = 16'h0080;
    LUT4 i14343_2_lut_4_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .D(n25764), .Z(spi1_sck_c_enable_199)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam i14343_2_lut_4_lut.init = 16'h8000;
    LUT4 i1_2_lut_rep_158 (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .Z(n26092)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(336[17] 401[24])
    defparam i1_2_lut_rep_158.init = 16'h4444;
    LUT4 i1_2_lut_rep_132_3_lut (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .C(spi_byte_count[2]), .Z(n26066)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(336[17] 401[24])
    defparam i1_2_lut_rep_132_3_lut.init = 16'h0404;
    LUT4 i81_3_lut_3_lut (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .C(spi_byte_count[2]), .Z(n60)) /* synthesis lut_function=(!(A (B+(C))+!A !(B (C)+!B !(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(336[17] 401[24])
    defparam i81_3_lut_3_lut.init = 16'h4343;
    LUT4 i2_3_lut_4_lut_adj_262 (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .C(n25350), .D(spi_byte_count[2]), .Z(spi1_sck_c_enable_106)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(336[17] 401[24])
    defparam i2_3_lut_4_lut_adj_262.init = 16'h4000;
    CCU2D global_phase_s2_1411_add_4_7 (.A0(global_phase_s2[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24831), .COUT(n24832), .S0(n40_adj_3397), 
          .S1(n39_adj_3396));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411_add_4_7.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1411_add_4_7.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1411_add_4_7.INJECT1_0 = "NO";
    defparam global_phase_s2_1411_add_4_7.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_137_3_lut (.A(ev_state[0]), .B(ev_state[3]), .C(ev_state[2]), 
         .Z(n26071)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_rep_137_3_lut.init = 16'h0808;
    LUT4 i1_2_lut_3_lut_4_lut_adj_263 (.A(ev_state[0]), .B(ev_state[3]), 
         .C(ev_state_3__N_2134[1]), .D(ev_state[2]), .Z(n49)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_3_lut_4_lut_adj_263.init = 16'h0008;
    LUT4 i1_2_lut_rep_160 (.A(spi_byte_count[12]), .B(spi_byte_count[11]), 
         .Z(n26094)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i1_2_lut_rep_160.init = 16'heeee;
    LUT4 i1_3_lut_adj_264 (.A(n17439), .B(init_shadow[81]), .C(ev_bit[81]), 
         .Z(n16086)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_264.init = 16'hecec;
    LUT4 i1_3_lut_adj_265 (.A(n17439), .B(init_shadow[80]), .C(ev_bit[80]), 
         .Z(n16078)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_265.init = 16'hecec;
    LUT4 i13320_2_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), .Z(n19_adj_3386)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam i13320_2_lut.init = 16'h6666;
    LUT4 i1_2_lut_rep_111_3_lut_4_lut (.A(spi_byte_count[12]), .B(spi_byte_count[11]), 
         .C(n26096), .D(spi_byte_count[10]), .Z(n26045)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i1_2_lut_rep_111_3_lut_4_lut.init = 16'hfffe;
    LUT4 i10356_2_lut_rep_161 (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .Z(n26095)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10356_2_lut_rep_161.init = 16'heeee;
    LUT4 i5_3_lut_rep_93_4_lut (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .C(spi_byte_count[0]), .D(n10), .Z(n26027)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;
    defparam i5_3_lut_rep_93_4_lut.init = 16'hffef;
    LUT4 i1_2_lut_adj_266 (.A(spi_channel_index[0]), .B(spi_channel_index[3]), 
         .Z(n8_adj_3421)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(373[43:69])
    defparam i1_2_lut_adj_266.init = 16'hdddd;
    LUT4 i2_4_lut_adj_267 (.A(spi_channel_index[1]), .B(spi_channel_index[5]), 
         .C(spi_channel_index[4]), .D(spi_channel_index[2]), .Z(n25370)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(373[43:69])
    defparam i2_4_lut_adj_267.init = 16'hffdf;
    LUT4 i13305_2_lut (.A(spi_channel_field[1]), .B(spi_channel_field[0]), 
         .Z(n14)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[78:102])
    defparam i13305_2_lut.init = 16'h6666;
    LUT4 i10538_2_lut_rep_96_3_lut_4_lut (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .C(n26096), .D(n26047), .Z(n26030)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i10538_2_lut_rep_96_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_rep_162 (.A(spi_byte_count[8]), .B(spi_byte_count[9]), 
         .Z(n26096)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_162.init = 16'heeee;
    LUT4 i1_2_lut_rep_134_3_lut (.A(spi_byte_count[8]), .B(spi_byte_count[9]), 
         .C(spi_byte_count[10]), .Z(n26068)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_rep_134_3_lut.init = 16'hfefe;
    FD1P3IX spi_channel_field_1408__i1 (.D(n14), .SP(spi1_sck_c_enable_196), 
            .CD(n22989), .CK(spi1_sck_c), .Q(spi_channel_field[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[78:102])
    defparam spi_channel_field_1408__i1.GSR = "ENABLED";
    LUT4 i1_2_lut_3_lut_4_lut_adj_268 (.A(spi_byte_count[8]), .B(spi_byte_count[9]), 
         .C(n87), .D(n26047), .Z(n4)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_268.init = 16'h0010;
    LUT4 i1_2_lut_rep_115_3_lut_4_lut (.A(spi_byte_count[8]), .B(spi_byte_count[9]), 
         .C(spi_byte_count[15]), .D(spi_byte_count[10]), .Z(n26049)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_rep_115_3_lut_4_lut.init = 16'hfffe;
    LUT4 i14133_2_lut_3_lut_4_lut (.A(spi_byte_count[8]), .B(spi_byte_count[9]), 
         .C(spi_channel_field[0]), .D(spi_byte_count[10]), .Z(n25591)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14133_2_lut_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_rep_163 (.A(ev_state[1]), .B(ev_state[2]), .Z(n26097)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_163.init = 16'heeee;
    LUT4 i1_2_lut_rep_108_3_lut_4_lut (.A(ev_state[1]), .B(ev_state[2]), 
         .C(ev_state[0]), .D(ev_state[3]), .Z(pll_clk_enable_22)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i1_2_lut_rep_108_3_lut_4_lut.init = 16'h0010;
    FD1P3IX frame_settle__i3 (.D(frame_settle_3__N_2130[3]), .SP(pll_clk_enable_737), 
            .CD(pll_clk_enable_28), .CK(pll_clk), .Q(frame_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam frame_settle__i3.GSR = "DISABLED";
    LUT4 mux_313_i20_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[19]), 
         .D(pending_sequence[19]), .Z(accepted_sequence_31__N_1135[19])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i20_3_lut_4_lut.init = 16'hfe10;
    LUT4 mux_1547_i1_3_lut (.A(n12896), .B(n12897), .C(n12895), .Z(rd_data_15__N_2873[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1547_i1_3_lut.init = 16'hcaca;
    CCU2D global_phase_s2_1411_add_4_5 (.A0(global_phase_s2[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24830), .COUT(n24831), .S0(n42), 
          .S1(n41));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411_add_4_5.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1411_add_4_5.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1411_add_4_5.INJECT1_0 = "NO";
    defparam global_phase_s2_1411_add_4_5.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_122_3_lut (.A(ev_state[1]), .B(ev_state[2]), .C(ev_state[3]), 
         .Z(n26056)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_rep_122_3_lut.init = 16'hfefe;
    LUT4 i7_4_lut_adj_269 (.A(n12894), .B(n25619), .C(n25499), .D(n6_adj_3419), 
         .Z(n12895)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i7_4_lut_adj_269.init = 16'h0002;
    FD1P3IX ev_bit_i24 (.D(ev_bit[23]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i24.GSR = "DISABLED";
    LUT4 mux_313_i21_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[20]), 
         .D(pending_sequence[20]), .Z(accepted_sequence_31__N_1135[20])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i21_3_lut_4_lut.init = 16'hfe10;
    LUT4 i14161_4_lut (.A(n12883), .B(n25525), .C(n5), .D(n12884), .Z(n25619)) /* synthesis lut_function=(A (B+(C+!(D)))+!A (B+(C+(D)))) */ ;
    defparam i14161_4_lut.init = 16'hfdfe;
    LUT4 mux_313_i22_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[21]), 
         .D(pending_sequence[21]), .Z(accepted_sequence_31__N_1135[21])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i22_3_lut_4_lut.init = 16'hfe10;
    LUT4 i10178_2_lut_rep_164 (.A(spi_byte_count[14]), .B(spi_byte_count[13]), 
         .Z(n26098)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10178_2_lut_rep_164.init = 16'heeee;
    LUT4 mux_313_i23_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[22]), 
         .D(pending_sequence[22]), .Z(accepted_sequence_31__N_1135[22])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i23_3_lut_4_lut.init = 16'hfe10;
    LUT4 spi_byte_count_7__bdd_4_lut (.A(spi_byte_count[7]), .B(n26044), 
         .C(n99), .D(spi_byte_count[6]), .Z(n87)) /* synthesis lut_function=(A (C+!(D))+!A (B (C+!(D))+!B (C (D)))) */ ;
    defparam spi_byte_count_7__bdd_4_lut.init = 16'hf0ee;
    LUT4 i1_2_lut_rep_131_3_lut_4_lut (.A(spi_byte_count[14]), .B(spi_byte_count[13]), 
         .C(spi_byte_count[11]), .D(spi_byte_count[12]), .Z(n26065)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_rep_131_3_lut_4_lut.init = 16'hfffe;
    LUT4 i4595_4_lut (.A(ev_ch[3]), .B(staging_rd_addr[3]), .C(n18417), 
         .D(n26077), .Z(staging_rd_addr_6__N_914[3])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(565[9] 638[16])
    defparam i4595_4_lut.init = 16'hcac0;
    LUT4 i10190_2_lut_rep_165 (.A(spi_byte_count[2]), .B(spi_byte_count[1]), 
         .Z(n26099)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10190_2_lut_rep_165.init = 16'heeee;
    LUT4 i30_4_lut_3_lut (.A(spi_byte_count[2]), .B(spi_byte_count[1]), 
         .C(spi_byte_count[3]), .Z(n19)) /* synthesis lut_function=(!(A ((C)+!B)+!A (B+!(C)))) */ ;
    defparam i30_4_lut_3_lut.init = 16'h1818;
    LUT4 i1_2_lut_rep_166 (.A(spi_byte_count[4]), .B(spi_byte_count[3]), 
         .Z(n26100)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_166.init = 16'heeee;
    LUT4 i14041_4_lut (.A(n12885), .B(n12879), .C(n12886), .D(n12880), 
         .Z(n25499)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14041_4_lut.init = 16'h7bde;
    LUT4 run_addr_s3_8__I_0_i5_3_lut (.A(run_addr_s3[4]), .B(ev_rd_slot[4]), 
         .C(n24911), .Z(event_rd_addr[4])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(231[33:74])
    defparam run_addr_s3_8__I_0_i5_3_lut.init = 16'hacac;
    LUT4 i1_2_lut_rep_127_3_lut (.A(spi_byte_count[4]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[2]), .Z(n26061)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_rep_127_3_lut.init = 16'hfefe;
    LUT4 i1_2_lut_rep_110_3_lut_4_lut (.A(spi_byte_count[4]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[5]), .D(spi_byte_count[2]), .Z(n26044)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C (D)))) */ ;
    defparam i1_2_lut_rep_110_3_lut_4_lut.init = 16'hf0e0;
    CCU2D global_phase_s2_1411_add_4_3 (.A0(global_phase_s2[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24829), .COUT(n24830), .S0(n44), 
          .S1(n43));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411_add_4_3.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1411_add_4_3.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1411_add_4_3.INJECT1_0 = "NO";
    defparam global_phase_s2_1411_add_4_3.INJECT1_1 = "NO";
    LUT4 active_bank_I_0_1_lut_rep_167 (.A(active_bank), .Z(n26101)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[34:46])
    defparam active_bank_I_0_1_lut_rep_167.init = 16'h5555;
    LUT4 run_addr_s3_8__I_0_i9_3_lut_3_lut (.A(active_bank), .B(n24911), 
         .C(run_addr_s3[8]), .Z(event_rd_addr[8])) /* synthesis lut_function=(A (B (C))+!A ((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[34:46])
    defparam run_addr_s3_8__I_0_i9_3_lut_3_lut.init = 16'hd1d1;
    CCU2D add_340_5 (.A0(ev_clear_addr[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24779), .COUT(n24780), .S0(ev_clear_addr_7__N_2439[3]), 
          .S1(ev_clear_addr_7__N_2439[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[34:54])
    defparam add_340_5.INIT0 = 16'h5aaa;
    defparam add_340_5.INIT1 = 16'h5aaa;
    defparam add_340_5.INJECT1_0 = "NO";
    defparam add_340_5.INJECT1_1 = "NO";
    LUT4 m1_lut (.Z(n26250)) /* synthesis lut_function=1, syn_instantiated=1 */ ;
    defparam m1_lut.init = 16'hffff;
    CCU2D global_phase_s2_1411_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(global_phase_s2[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24829), .S1(n45));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(446[32:54])
    defparam global_phase_s2_1411_add_4_1.INIT0 = 16'hF000;
    defparam global_phase_s2_1411_add_4_1.INIT1 = 16'h0555;
    defparam global_phase_s2_1411_add_4_1.INJECT1_0 = "NO";
    defparam global_phase_s2_1411_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_270 (.A(n17439), .B(init_shadow[79]), .C(ev_bit[79]), 
         .Z(n16070)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_270.init = 16'hecec;
    LUT4 i1_2_lut_3_lut_4_lut_4_lut_4_lut (.A(ev_state[0]), .B(n26069), 
         .C(ev_state[3]), .D(ev_state[2]), .Z(pll_clk_enable_753)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_3_lut_4_lut_4_lut_4_lut.init = 16'h44e4;
    LUT4 i2_4_lut_4_lut (.A(ev_state[1]), .B(ev_state[2]), .C(ev_state[0]), 
         .D(ev_state[3]), .Z(ev_we)) /* synthesis lut_function=(!(A ((C+(D))+!B)+!A (B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29] 234[49])
    defparam i2_4_lut_4_lut.init = 16'h1018;
    LUT4 i1_4_lut_4_lut_4_lut_4_lut (.A(ev_state[1]), .B(ev_state[2]), .C(n13), 
         .D(ev_state[0]), .Z(n14_adj_3387)) /* synthesis lut_function=(!(A (B+(C+(D)))+!A (B+(D)))) */ ;
    defparam i1_4_lut_4_lut_4_lut_4_lut.init = 16'h0013;
    LUT4 i14115_2_lut (.A(expected_next[5]), .B(expected_next[2]), .Z(n25573)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14115_2_lut.init = 16'h8888;
    LUT4 spi1_miso_N_2715_1__bdd_4_lut_4_lut_4_lut_4_lut (.A(status_bit_index[1]), 
         .B(status_hold[74]), .C(status_hold[76]), .D(status_bit_index[2]), 
         .Z(n26019)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(298[55:80])
    defparam spi1_miso_N_2715_1__bdd_4_lut_4_lut_4_lut_4_lut.init = 16'h44a0;
    LUT4 ev_state_3__bdd_4_lut_14412_rep_182 (.A(ev_state[3]), .B(ev_state[1]), 
         .C(ev_state[0]), .D(ev_state[2]), .Z(pll_clk_enable_439)) /* synthesis lut_function=(!(A (B+(C+(D)))+!A (B+!(C (D))))) */ ;
    defparam ev_state_3__bdd_4_lut_14412_rep_182.init = 16'h1002;
    LUT4 i2_3_lut_4_lut_adj_271 (.A(spi1_sck_c_enable_185), .B(n26024), 
         .C(n26049), .D(n26090), .Z(spi1_sck_c_enable_196)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i2_3_lut_4_lut_adj_271.init = 16'h0800;
    PFUMX i14247 (.BLUT(n25697), .ALUT(n25698), .C0(n26080), .Z(n25706));
    LUT4 i12_4_lut (.A(expected_next[3]), .B(n24_adj_3427), .C(n18_adj_3429), 
         .D(expected_next[10]), .Z(n26_adj_3426)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[48:79])
    defparam i12_4_lut.init = 16'hfffe;
    CCU2D add_340_3 (.A0(ev_clear_addr[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24778), .COUT(n24779), .S0(ev_clear_addr_7__N_2439[1]), 
          .S1(ev_clear_addr_7__N_2439[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[34:54])
    defparam add_340_3.INIT0 = 16'h5aaa;
    defparam add_340_3.INIT1 = 16'h5aaa;
    defparam add_340_3.INJECT1_0 = "NO";
    defparam add_340_3.INJECT1_1 = "NO";
    LUT4 i1_4_lut_rep_170 (.A(ev_state[3]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(n4_adj_3407), .Z(pll_clk_enable_765)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[17:25])
    defparam i1_4_lut_rep_170.init = 16'h5150;
    CCU2D status_bit_index_1406_add_4_7 (.A0(status_bit_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24827), .S0(n35_adj_3389), 
          .S1(n34_adj_3388));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406_add_4_7.INIT0 = 16'hfaaa;
    defparam status_bit_index_1406_add_4_7.INIT1 = 16'hfaaa;
    defparam status_bit_index_1406_add_4_7.INJECT1_0 = "NO";
    defparam status_bit_index_1406_add_4_7.INJECT1_1 = "NO";
    CCU2D status_bit_index_1406_add_4_5 (.A0(status_bit_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24826), .COUT(n24827), .S0(n37_adj_3391), 
          .S1(n36_adj_3390));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406_add_4_5.INIT0 = 16'hfaaa;
    defparam status_bit_index_1406_add_4_5.INIT1 = 16'hfaaa;
    defparam status_bit_index_1406_add_4_5.INJECT1_0 = "NO";
    defparam status_bit_index_1406_add_4_5.INJECT1_1 = "NO";
    LUT4 i4591_4_lut (.A(ev_ch[1]), .B(staging_rd_addr[1]), .C(n18417), 
         .D(n26077), .Z(staging_rd_addr_6__N_914[1])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(565[9] 638[16])
    defparam i4591_4_lut.init = 16'hcac0;
    LUT4 i6_4_lut_rep_184 (.A(mic_sample_count[3]), .B(n12_adj_3433), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_596)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_rep_184.init = 16'h0800;
    CCU2D status_bit_index_1406_add_4_3 (.A0(status_bit_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24825), .COUT(n24826), .S0(n39_adj_3393), 
          .S1(n38_adj_3392));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406_add_4_3.INIT0 = 16'hfaaa;
    defparam status_bit_index_1406_add_4_3.INIT1 = 16'hfaaa;
    defparam status_bit_index_1406_add_4_3.INJECT1_0 = "NO";
    defparam status_bit_index_1406_add_4_3.INJECT1_1 = "NO";
    LUT4 i6990_2_lut_2_lut_4_lut_rep_186 (.A(ev_state[3]), .B(ev_state[1]), 
         .C(ev_state[2]), .D(n4_adj_3407), .Z(n26269)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[17:25])
    defparam i6990_2_lut_2_lut_4_lut_rep_186.init = 16'h0100;
    CCU2D status_bit_index_1406_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24825), .S1(n40_adj_3394));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(302[66:89])
    defparam status_bit_index_1406_add_4_1.INIT0 = 16'hF000;
    defparam status_bit_index_1406_add_4_1.INIT1 = 16'h0555;
    defparam status_bit_index_1406_add_4_1.INJECT1_0 = "NO";
    defparam status_bit_index_1406_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_101_4_lut (.A(n26065), .B(spi_byte_count[10]), .C(spi_byte_count[15]), 
         .D(n26096), .Z(n26035)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_rep_101_4_lut.init = 16'hfffe;
    LUT4 i6990_2_lut_2_lut_4_lut (.A(ev_state[3]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(n4_adj_3407), .Z(n18625)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[17:25])
    defparam i6990_2_lut_2_lut_4_lut.init = 16'h0100;
    CCU2D expected_next_15__I_0_632_11 (.A0(spi_rx_shift[2]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[3]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24758), .COUT(n24759), .S0(expected_next[11]), 
          .S1(expected_next[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[33] 308[86])
    defparam expected_next_15__I_0_632_11.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_632_11.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_632_11.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_632_11.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_94_3_lut_4_lut (.A(n26047), .B(n26096), .C(spi1_sck_c_enable_185), 
         .D(n26095), .Z(n26028)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam i1_2_lut_rep_94_3_lut_4_lut.init = 16'h0010;
    LUT4 i1_2_lut_4_lut_adj_272 (.A(n26095), .B(n10), .C(spi_byte_count[0]), 
         .D(spi1_sck_c_enable_185), .Z(spi1_sck_c_enable_37)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i1_2_lut_4_lut_adj_272.init = 16'h1000;
    LUT4 i1_3_lut_4_lut_adj_273 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[78]), 
         .D(ev_bit[78]), .Z(ev_wr_data[78])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_273.init = 16'hddd0;
    LUT4 i7099_2_lut_3_lut_3_lut (.A(ev_state[2]), .B(ev_state[3]), .C(n26043), 
         .Z(n18621)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i7099_2_lut_3_lut_3_lut.init = 16'h1010;
    LUT4 i1_3_lut_4_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), .C(n26057), 
         .D(ev_state[1]), .Z(pll_clk_enable_17)) /* synthesis lut_function=(!(A+!(B+!((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_3_lut_4_lut_4_lut.init = 16'h4454;
    LUT4 i1_2_lut_3_lut_4_lut_rep_188 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(pll_locked), .D(running), .Z(n26271)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(265[23:61])
    defparam i1_2_lut_3_lut_4_lut_rep_188.init = 16'h6fff;
    LUT4 i1_3_lut_4_lut_adj_274 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[73]), 
         .D(ev_bit[73]), .Z(ev_wr_data[73])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_274.init = 16'hddd0;
    LUT4 ev_state_3__bdd_4_lut_14412 (.A(ev_state[3]), .B(ev_state[1]), 
         .C(ev_state[0]), .D(ev_state[2]), .Z(pll_clk_enable_473)) /* synthesis lut_function=(!(A (B+(C+(D)))+!A (B+!(C (D))))) */ ;
    defparam ev_state_3__bdd_4_lut_14412.init = 16'h1002;
    CCU2D fpga_time_1412_add_4_33 (.A0(fpga_time[31]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24824), .S0(n134));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_33.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_33.INIT1 = 16'h0000;
    defparam fpga_time_1412_add_4_33.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_33.INJECT1_1 = "NO";
    CCU2D add_340_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(ev_clear_addr[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24778), .S1(ev_clear_addr_7__N_2439[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(578[34:54])
    defparam add_340_1.INIT0 = 16'hF000;
    defparam add_340_1.INIT1 = 16'h5555;
    defparam add_340_1.INJECT1_0 = "NO";
    defparam add_340_1.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_275 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[79]), 
         .D(ev_bit[79]), .Z(ev_wr_data[79])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_275.init = 16'hddd0;
    CCU2D fpga_time_1412_add_4_31 (.A0(fpga_time[29]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[30]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24823), .COUT(n24824), .S0(n136), .S1(n135));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_31.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_31.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_31.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_31.INJECT1_1 = "NO";
    LUT4 mux_1256_i1_3_lut_4_lut (.A(ev_state[0]), .B(n26056), .C(ev_wr_addr_8__N_925[0]), 
         .D(ev_clear_addr[0]), .Z(ev_wr_addr[0])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam mux_1256_i1_3_lut_4_lut.init = 16'hf2d0;
    CCU2D add_13291_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24852), 
          .S0(n11987));
    defparam add_13291_cout.INIT0 = 16'h0000;
    defparam add_13291_cout.INIT1 = 16'h0000;
    defparam add_13291_cout.INJECT1_0 = "NO";
    defparam add_13291_cout.INJECT1_1 = "NO";
    LUT4 mux_1256_i2_3_lut_4_lut (.A(ev_state[0]), .B(n26056), .C(ev_wr_addr_8__N_925[1]), 
         .D(ev_clear_addr[1]), .Z(ev_wr_addr[1])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam mux_1256_i2_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i10_4_lut (.A(expected_next[11]), .B(expected_next[9]), .C(expected_next[14]), 
         .D(expected_next[15]), .Z(n24_adj_3427)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[48:79])
    defparam i10_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_adj_276 (.A(ev_state[3]), .B(ev_state[2]), .Z(n25480)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_276.init = 16'heeee;
    CCU2D add_13291_17 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24851), 
          .COUT(n24852));
    defparam add_13291_17.INIT0 = 16'hffff;
    defparam add_13291_17.INIT1 = 16'hffff;
    defparam add_13291_17.INJECT1_0 = "NO";
    defparam add_13291_17.INJECT1_1 = "NO";
    CCU2D fpga_time_1412_add_4_29 (.A0(fpga_time[27]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[28]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24822), .COUT(n24823), .S0(n138), .S1(n137));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_29.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_29.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_29.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_29.INJECT1_1 = "NO";
    CCU2D fpga_time_1412_add_4_27 (.A0(fpga_time[25]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[26]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24821), .COUT(n24822), .S0(n140), .S1(n139));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_27.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_27.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_27.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_27.INJECT1_1 = "NO";
    LUT4 i4_2_lut (.A(expected_next[12]), .B(expected_next[7]), .Z(n18_adj_3429)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[48:79])
    defparam i4_2_lut.init = 16'heeee;
    LUT4 i1_4_lut_rep_100_4_lut (.A(ev_state[2]), .B(ev_state[3]), .C(ev_state[0]), 
         .D(n26043), .Z(pll_clk_enable_650)) /* synthesis lut_function=(!(A+!(B (C)+!B (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_4_lut_rep_100_4_lut.init = 16'h5140;
    LUT4 i2_3_lut_4_lut_4_lut (.A(ev_state[2]), .B(n13), .C(ev_state[0]), 
         .D(ev_state[1]), .Z(n25433)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i2_3_lut_4_lut_4_lut.init = 16'h0400;
    CCU2D add_13291_15 (.A0(spi_expected_length[14]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[15]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24850), .COUT(n24851));
    defparam add_13291_15.INIT0 = 16'hf555;
    defparam add_13291_15.INIT1 = 16'hf555;
    defparam add_13291_15.INJECT1_0 = "NO";
    defparam add_13291_15.INJECT1_1 = "NO";
    LUT4 mux_1256_i3_3_lut_4_lut (.A(ev_state[0]), .B(n26056), .C(ev_wr_addr_8__N_925[2]), 
         .D(ev_clear_addr[2]), .Z(ev_wr_addr[2])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam mux_1256_i3_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i4593_4_lut (.A(ev_ch[2]), .B(staging_rd_addr[2]), .C(n18417), 
         .D(n26077), .Z(staging_rd_addr_6__N_914[2])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(565[9] 638[16])
    defparam i4593_4_lut.init = 16'hcac0;
    LUT4 i1_2_lut_3_lut_4_lut_4_lut_4_lut_rep_174 (.A(ev_state[0]), .B(n26069), 
         .C(ev_state[3]), .D(ev_state[2]), .Z(pll_clk_enable_773)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_3_lut_4_lut_4_lut_4_lut_rep_174.init = 16'h44e4;
    LUT4 mux_1256_i5_3_lut_4_lut (.A(ev_state[0]), .B(n26056), .C(ev_wr_addr_8__N_925[4]), 
         .D(ev_clear_addr[4]), .Z(ev_wr_addr[4])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam mux_1256_i5_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_1256_i4_3_lut_4_lut (.A(ev_state[0]), .B(n26056), .C(ev_wr_addr_8__N_925[3]), 
         .D(ev_clear_addr[3]), .Z(ev_wr_addr[3])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam mux_1256_i4_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_1256_i6_3_lut_4_lut (.A(ev_state[0]), .B(n26056), .C(ev_wr_addr_8__N_925[5]), 
         .D(ev_clear_addr[5]), .Z(ev_wr_addr[5])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam mux_1256_i6_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i94_1_lut (.A(fpga_cs_n_c), .Z(fpga_cs_n_N_2723)) /* synthesis lut_function=(!(A)) */ ;
    defparam i94_1_lut.init = 16'h5555;
    LUT4 mux_1256_i7_3_lut_4_lut (.A(ev_state[0]), .B(n26056), .C(ev_wr_addr_8__N_925[6]), 
         .D(ev_clear_addr[6]), .Z(ev_wr_addr[6])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam mux_1256_i7_3_lut_4_lut.init = 16'hf2d0;
    LUT4 equal_1528_i6_2_lut (.A(n12889), .B(n12890), .Z(n6_adj_3419)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam equal_1528_i6_2_lut.init = 16'h6666;
    LUT4 frame_end_I_27_2_lut (.A(n11987), .B(frame_end_N_2839), .Z(frame_end_N_2837)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(315[48] 316[97])
    defparam frame_end_I_27_2_lut.init = 16'h4444;
    PFUMX i14379 (.BLUT(n25891), .ALUT(n25890), .C0(spi1_miso_N_2715[5]), 
          .Z(n25892));
    LUT4 mux_1256_i8_3_lut_4_lut (.A(ev_state[0]), .B(n26056), .C(ev_wr_addr_8__N_925[7]), 
         .D(ev_clear_addr[7]), .Z(ev_wr_addr[7])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam mux_1256_i8_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i1_3_lut_4_lut_adj_277 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[0]), 
         .D(ev_bit[0]), .Z(ev_wr_data[0])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_277.init = 16'hddd0;
    CCU2D add_13291_13 (.A0(spi_expected_length[12]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[13]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24849), .COUT(n24850));
    defparam add_13291_13.INIT0 = 16'hf555;
    defparam add_13291_13.INIT1 = 16'hf555;
    defparam add_13291_13.INJECT1_0 = "NO";
    defparam add_13291_13.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_278 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[1]), 
         .D(ev_bit[1]), .Z(ev_wr_data[1])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_278.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_279 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[2]), 
         .D(ev_bit[2]), .Z(ev_wr_data[2])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_279.init = 16'hddd0;
    LUT4 status_flags_wire_15__I_0_i9_2_lut (.A(pll_locked), .B(last_command_spi[0]), 
         .Z(status_flags_wire[8])) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[37] 286[72])
    defparam status_flags_wire_15__I_0_i9_2_lut.init = 16'heeee;
    LUT4 expected_next_15__I_0_i32_4_lut (.A(n25573), .B(n26_adj_3426), 
         .C(n22_adj_3428), .D(n25471), .Z(frame_end_N_2872)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[48:79])
    defparam expected_next_15__I_0_i32_4_lut.init = 16'h0002;
    LUT4 i16_4_lut_adj_280 (.A(n19_adj_3406), .B(n32), .C(n28), .D(n20_adj_3405), 
         .Z(status_flags_wire[11])) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[37] 286[72])
    defparam i16_4_lut_adj_280.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_281 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[3]), 
         .D(ev_bit[3]), .Z(ev_wr_data[3])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_281.init = 16'hddd0;
    LUT4 i14067_4_lut (.A(n12891), .B(n12881), .C(n12892), .D(n12882), 
         .Z(n25525)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14067_4_lut.init = 16'h7bde;
    LUT4 i1_4_lut_rep_178 (.A(ev_state[3]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(n4_adj_3407), .Z(pll_clk_enable_776)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[17:25])
    defparam i1_4_lut_rep_178.init = 16'h5150;
    LUT4 equal_1528_i5_2_lut (.A(n12887), .B(n12888), .Z(n5)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam equal_1528_i5_2_lut.init = 16'h6666;
    LUT4 i1_2_lut_rep_116_3_lut_3_lut (.A(ev_state[2]), .B(ev_state[0]), 
         .C(ev_state[3]), .Z(n26050)) /* synthesis lut_function=((B+(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_rep_116_3_lut_3_lut.init = 16'hfdfd;
    LUT4 ev_state_3__N_2134_1__bdd_4_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(n26009), .D(ev_state_3__N_2134[1]), .Z(n26020)) /* synthesis lut_function=(!(A (B+!(C))+!A !(B (D)+!B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_state_3__N_2134_1__bdd_4_lut_4_lut.init = 16'h7430;
    LUT4 i2_3_lut_4_lut_rep_180 (.A(running), .B(pll_locked), .C(pll_clk_enable_28), 
         .D(phase_step_s5), .Z(pll_clk_enable_271)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_3_lut_4_lut_rep_180.init = 16'hfff7;
    LUT4 ev_state_3__I_0_571_i6_2_lut_rep_168 (.A(ev_state[2]), .B(ev_state[3]), 
         .Z(n26102)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_state_3__I_0_571_i6_2_lut_rep_168.init = 16'hdddd;
    LUT4 i8_4_lut (.A(expected_next[6]), .B(expected_next[8]), .C(expected_next[4]), 
         .D(expected_next[13]), .Z(n22_adj_3428)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(314[48:79])
    defparam i8_4_lut.init = 16'hfffe;
    LUT4 i10145_3_lut_4_lut_4_lut_3_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[1]), .D(ev_state[0]), .Z(n21573)) /* synthesis lut_function=((B+!(C (D)+!C !(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i10145_3_lut_4_lut_4_lut_3_lut_4_lut.init = 16'hdffd;
    CCU2D add_13291_11 (.A0(spi_expected_length[10]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[11]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24848), .COUT(n24849));
    defparam add_13291_11.INIT0 = 16'hf555;
    defparam add_13291_11.INIT1 = 16'hf555;
    defparam add_13291_11.INJECT1_0 = "NO";
    defparam add_13291_11.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_169 (.A(swap_pending), .B(frame_req), .Z(n26103)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_rep_169.init = 16'h4444;
    LUT4 i1_3_lut_4_lut_adj_282 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[4]), 
         .D(ev_bit[4]), .Z(ev_wr_data[4])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_282.init = 16'hddd0;
    LUT4 mux_313_i24_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[23]), 
         .D(pending_sequence[23]), .Z(accepted_sequence_31__N_1135[23])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i24_3_lut_4_lut.init = 16'hfe10;
    LUT4 run_addr_s3_8__I_0_i1_3_lut (.A(run_addr_s3[0]), .B(ev_rd_slot[0]), 
         .C(n24911), .Z(event_rd_addr[0])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(231[33:74])
    defparam run_addr_s3_8__I_0_i1_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_4_lut_adj_283 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[5]), 
         .D(ev_bit[5]), .Z(ev_wr_data[5])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_283.init = 16'hddd0;
    CCU2D add_13291_9 (.A0(spi_expected_length[8]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[9]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24847), .COUT(n24848));
    defparam add_13291_9.INIT0 = 16'hf555;
    defparam add_13291_9.INIT1 = 16'hf555;
    defparam add_13291_9.INJECT1_0 = "NO";
    defparam add_13291_9.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_284 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[6]), 
         .D(ev_bit[6]), .Z(ev_wr_data[6])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_284.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_285 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[7]), 
         .D(ev_bit[7]), .Z(ev_wr_data[7])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_285.init = 16'hddd0;
    LUT4 i2_2_lut_adj_286 (.A(last_length_spi[12]), .B(last_length_spi[2]), 
         .Z(n19_adj_3406)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[37] 286[72])
    defparam i2_2_lut_adj_286.init = 16'heeee;
    LUT4 i1_2_lut_adj_287 (.A(expected_next[0]), .B(expected_next[1]), .Z(n25471)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_287.init = 16'heeee;
    LUT4 i4_4_lut_adj_288 (.A(spi_byte_count[1]), .B(n26061), .C(spi_byte_count[5]), 
         .D(n25345), .Z(n10)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;
    defparam i4_4_lut_adj_288.init = 16'hffdf;
    LUT4 i1_3_lut_4_lut_adj_289 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[8]), 
         .D(ev_bit[8]), .Z(ev_wr_data[8])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_289.init = 16'hddd0;
    LUT4 i8122_3_lut_rep_123_4_lut (.A(swap_pending), .B(frame_req), .C(ev_state[0]), 
         .D(ev_state_3__N_2142[1]), .Z(n26057)) /* synthesis lut_function=(A (C (D))+!A (B ((D)+!C)+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i8122_3_lut_rep_123_4_lut.init = 16'hf404;
    LUT4 i2_3_lut_rep_135_4_lut (.A(swap_pending), .B(frame_req), .C(ev_state[2]), 
         .D(n25601), .Z(n26069)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i2_3_lut_rep_135_4_lut.init = 16'h0004;
    LUT4 frame_settle_3__bdd_4_lut (.A(frame_settle[3]), .B(frame_settle[1]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(n25924)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;
    defparam frame_settle_3__bdd_4_lut.init = 16'haaa9;
    LUT4 i14045_3_lut_4_lut (.A(swap_pending), .B(frame_req), .C(ev_state[2]), 
         .D(ev_state[1]), .Z(n25503)) /* synthesis lut_function=(A (C+(D))+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i14045_3_lut_4_lut.init = 16'hfff4;
    LUT4 i1_3_lut_4_lut_adj_290 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[9]), 
         .D(ev_bit[9]), .Z(ev_wr_data[9])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_290.init = 16'hddd0;
    LUT4 i6991_2_lut_3_lut_4_lut_2_lut (.A(ev_state[0]), .B(n26069), .Z(n18521)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i6991_2_lut_3_lut_4_lut_2_lut.init = 16'h4444;
    LUT4 i1_3_lut_4_lut_adj_291 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[10]), 
         .D(ev_bit[10]), .Z(ev_wr_data[10])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_291.init = 16'hddd0;
    LUT4 i15_4_lut (.A(n29), .B(last_length_spi[6]), .C(n26), .D(last_length_spi[0]), 
         .Z(n32)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[37] 286[72])
    defparam i15_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_292 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[11]), 
         .D(ev_bit[11]), .Z(ev_wr_data[11])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_292.init = 16'hddd0;
    CCU2D add_13291_7 (.A0(spi_expected_length[6]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[7]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24846), .COUT(n24847));
    defparam add_13291_7.INIT0 = 16'hf555;
    defparam add_13291_7.INIT1 = 16'hf555;
    defparam add_13291_7.INJECT1_0 = "NO";
    defparam add_13291_7.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_3_lut (.A(ev_state[0]), .B(frame_req), .C(swap_pending), 
         .Z(n4_adj_3407)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_3_lut_3_lut.init = 16'h0404;
    LUT4 i1_2_lut_rep_117_4_lut_4_lut (.A(ev_state[0]), .B(ev_state[2]), 
         .C(n26103), .D(n25601), .Z(pll_clk_enable_504)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i1_2_lut_rep_117_4_lut_4_lut.init = 16'h0010;
    LUT4 i11_4_lut (.A(last_length_spi[14]), .B(last_length_spi[9]), .C(last_length_spi[3]), 
         .D(last_length_spi[10]), .Z(n28)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[37] 286[72])
    defparam i11_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_adj_293 (.A(n17439), .B(init_shadow[75]), .C(ev_bit[75]), 
         .Z(n16046)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_293.init = 16'hecec;
    LUT4 i5057_2_lut_3_lut_4_lut_4_lut (.A(ev_state[0]), .B(n26069), .C(stop_toggle_seen), 
         .D(stop_toggle_sync), .Z(n16567)) /* synthesis lut_function=(!(A (C (D)+!C !(D))+!A !(B+!(C (D)+!C !(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam i5057_2_lut_3_lut_4_lut_4_lut.init = 16'h4ff4;
    LUT4 i3_2_lut (.A(last_length_spi[4]), .B(last_length_spi[11]), .Z(n20_adj_3405)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[37] 286[72])
    defparam i3_2_lut.init = 16'heeee;
    LUT4 i12_4_lut_adj_294 (.A(last_length_spi[1]), .B(last_length_spi[7]), 
         .C(last_length_spi[15]), .D(last_length_spi[13]), .Z(n29)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[37] 286[72])
    defparam i12_4_lut_adj_294.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_295 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[48]), 
         .D(ev_bit[48]), .Z(ev_wr_data[48])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_295.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_296 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[49]), 
         .D(ev_bit[49]), .Z(ev_wr_data[49])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_296.init = 16'hddd0;
    LUT4 i9_3_lut (.A(last_command_spi[3]), .B(last_length_spi[8]), .C(last_length_spi[5]), 
         .Z(n26)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(282[37] 286[72])
    defparam i9_3_lut.init = 16'hfefe;
    LUT4 mux_313_i25_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[24]), 
         .D(pending_sequence[24]), .Z(accepted_sequence_31__N_1135[24])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i25_3_lut_4_lut.init = 16'hfe10;
    LUT4 n715_bdd_2_lut_14395 (.A(n26019), .B(status_bit_index[0]), .Z(n25904)) /* synthesis lut_function=(A (B)) */ ;
    defparam n715_bdd_2_lut_14395.init = 16'h8888;
    LUT4 i1_3_lut_adj_297 (.A(n17439), .B(init_shadow[74]), .C(ev_bit[74]), 
         .Z(n16040)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_297.init = 16'hecec;
    LUT4 i14059_2_lut_3_lut_4_lut (.A(n26095), .B(n26035), .C(n26066), 
         .D(spi1_sck_c_enable_185), .Z(n25517)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i14059_2_lut_3_lut_4_lut.init = 16'h1000;
    LUT4 mux_313_i26_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[25]), 
         .D(pending_sequence[25]), .Z(accepted_sequence_31__N_1135[25])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i26_3_lut_4_lut.init = 16'hfe10;
    LUT4 sub_80_inv_0_i4_1_lut (.A(status_bit_index[3]), .Z(spi1_miso_N_2715[3])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(298[55:80])
    defparam sub_80_inv_0_i4_1_lut.init = 16'h5555;
    LUT4 n25480_bdd_4_lut (.A(n25480), .B(ev_state_3__N_2142[1]), .C(ev_state[1]), 
         .D(ev_state[0]), .Z(n18417)) /* synthesis lut_function=(A+(B (C (D)+!C !(D))+!B ((D)+!C))) */ ;
    defparam n25480_bdd_4_lut.init = 16'hfbaf;
    LUT4 i1_3_lut_4_lut_adj_298 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[50]), 
         .D(ev_bit[50]), .Z(ev_wr_data[50])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_298.init = 16'hddd0;
    LUT4 run_addr_s3_8__I_0_i6_3_lut (.A(run_addr_s3[5]), .B(ev_rd_slot[5]), 
         .C(n24911), .Z(event_rd_addr[5])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(231[33:74])
    defparam run_addr_s3_8__I_0_i6_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_4_lut_adj_299 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[51]), 
         .D(ev_bit[51]), .Z(ev_wr_data[51])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_299.init = 16'hddd0;
    CCU2D fpga_time_1412_add_4_25 (.A0(fpga_time[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[24]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24820), .COUT(n24821), .S0(n142), .S1(n141));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_25.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_25.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_25.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_25.INJECT1_1 = "NO";
    LUT4 mux_313_i27_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[26]), 
         .D(pending_sequence[26]), .Z(accepted_sequence_31__N_1135[26])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i27_3_lut_4_lut.init = 16'hfe10;
    LUT4 mux_313_i28_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[27]), 
         .D(pending_sequence[27]), .Z(accepted_sequence_31__N_1135[27])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i28_3_lut_4_lut.init = 16'hfe10;
    LUT4 i13384_2_lut_3_lut_4_lut (.A(n26025), .B(spi_rgb_index[0]), .C(spi_rgb_index[2]), 
         .D(spi_rgb_index[1]), .Z(n23)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(398[46:66])
    defparam i13384_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 mux_313_i29_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[28]), 
         .D(pending_sequence[28]), .Z(accepted_sequence_31__N_1135[28])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i29_3_lut_4_lut.init = 16'hfe10;
    CCU2D add_13291_5 (.A0(spi_expected_length[4]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[5]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24845), .COUT(n24846));
    defparam add_13291_5.INIT0 = 16'hf555;
    defparam add_13291_5.INIT1 = 16'h0aaa;
    defparam add_13291_5.INJECT1_0 = "NO";
    defparam add_13291_5.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_300 (.A(n17439), .B(init_shadow[73]), .C(ev_bit[73]), 
         .Z(n16034)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_300.init = 16'hecec;
    CCU2D add_13291_3 (.A0(spi_expected_length[2]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[3]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24844), .COUT(n24845));
    defparam add_13291_3.INIT0 = 16'h0aaa;
    defparam add_13291_3.INIT1 = 16'hf555;
    defparam add_13291_3.INJECT1_0 = "NO";
    defparam add_13291_3.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i8 (.D(ev_bit[7]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i8.GSR = "DISABLED";
    FD1P3IX ev_bit_i23 (.D(ev_bit[22]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i23.GSR = "DISABLED";
    CCU2D fpga_time_1412_add_4_23 (.A0(fpga_time[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24819), .COUT(n24820), .S0(n144), .S1(n143));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_23.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_23.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_23.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_23.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_301 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[52]), 
         .D(ev_bit[52]), .Z(ev_wr_data[52])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_301.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_302 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[53]), 
         .D(ev_bit[53]), .Z(ev_wr_data[53])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_302.init = 16'hddd0;
    FD1P3IX ev_bit_i7 (.D(ev_bit[6]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i7.GSR = "DISABLED";
    FD1P3IX ev_bit_i22 (.D(ev_bit[21]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i22.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_303 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[54]), 
         .D(ev_bit[54]), .Z(ev_wr_data[54])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_303.init = 16'hddd0;
    LUT4 mux_313_i30_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[29]), 
         .D(pending_sequence[29]), .Z(accepted_sequence_31__N_1135[29])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i30_3_lut_4_lut.init = 16'hfe10;
    LUT4 i2_3_lut_4_lut_adj_304 (.A(n26095), .B(n26035), .C(n21575), .D(n26067), 
         .Z(n25350)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i2_3_lut_4_lut_adj_304.init = 16'h0100;
    FD1P3IX ev_bit_i6 (.D(ev_bit[5]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i6.GSR = "DISABLED";
    FD1P3IX ev_bit_i21 (.D(ev_bit[20]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i21.GSR = "DISABLED";
    LUT4 spi_command_1__bdd_4_lut (.A(spi_command[1]), .B(spi_command[0]), 
         .C(n26058), .D(spi_command[4]), .Z(n25931)) /* synthesis lut_function=(A (B+(C+!(D)))+!A ((C)+!B)) */ ;
    defparam spi_command_1__bdd_4_lut.init = 16'hf9fb;
    LUT4 i1_3_lut_4_lut_adj_305 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[55]), 
         .D(ev_bit[55]), .Z(ev_wr_data[55])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_305.init = 16'hddd0;
    LUT4 mux_313_i31_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[30]), 
         .D(pending_sequence[30]), .Z(accepted_sequence_31__N_1135[30])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i31_3_lut_4_lut.init = 16'hfe10;
    LUT4 mux_313_i32_3_lut_4_lut (.A(pll_clk_enable_29), .B(n26060), .C(accepted_sequence_sync[31]), 
         .D(pending_sequence[31]), .Z(accepted_sequence_31__N_1135[31])) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C))) */ ;
    defparam mux_313_i32_3_lut_4_lut.init = 16'hfe10;
    FD1P3IX ev_bit_i5 (.D(ev_bit[4]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i5.GSR = "DISABLED";
    FD1P3IX ev_bit_i20 (.D(ev_bit[19]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i20.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_306 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[56]), 
         .D(ev_bit[56]), .Z(ev_wr_data[56])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_306.init = 16'hddd0;
    FD1P3IX ev_bit_i4 (.D(ev_bit[3]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i4.GSR = "DISABLED";
    FD1P3IX ev_bit_i19 (.D(ev_bit[18]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i19.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_307 (.A(n17439), .B(init_shadow[72]), .C(ev_bit[72]), 
         .Z(n16028)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_307.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_308 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[57]), 
         .D(ev_bit[57]), .Z(ev_wr_data[57])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_308.init = 16'hddd0;
    LUT4 i1_3_lut_adj_309 (.A(n17439), .B(init_shadow[71]), .C(ev_bit[71]), 
         .Z(n16022)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_309.init = 16'hecec;
    FD1P3IX ev_bit_i3 (.D(ev_bit[2]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i3.GSR = "DISABLED";
    FD1P3IX ev_bit_i18 (.D(ev_bit[17]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i18.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_310 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[58]), 
         .D(ev_bit[58]), .Z(ev_wr_data[58])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_310.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_311 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[59]), 
         .D(ev_bit[59]), .Z(ev_wr_data[59])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_311.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_312 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[60]), 
         .D(ev_bit[60]), .Z(ev_wr_data[60])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_312.init = 16'hddd0;
    LUT4 i1_3_lut_adj_313 (.A(n17439), .B(init_shadow[78]), .C(ev_bit[78]), 
         .Z(n16064)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_313.init = 16'hecec;
    LUT4 i1_3_lut_adj_314 (.A(n17439), .B(init_shadow[67]), .C(ev_bit[67]), 
         .Z(n15998)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_314.init = 16'hecec;
    CCU2D add_13291_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(spi_expected_length[0]), .B1(spi_expected_length[1]), .C1(GND_net), 
          .D1(GND_net), .COUT(n24844));
    defparam add_13291_1.INIT0 = 16'hF000;
    defparam add_13291_1.INIT1 = 16'ha666;
    defparam add_13291_1.INJECT1_0 = "NO";
    defparam add_13291_1.INJECT1_1 = "NO";
    CCU2D fpga_time_1412_add_4_21 (.A0(fpga_time[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24818), .COUT(n24819), .S0(n146), .S1(n145));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_21.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_21.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_21.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_21.INJECT1_1 = "NO";
    LUT4 run_addr_s3_8__I_0_i7_3_lut (.A(run_addr_s3[6]), .B(ev_rd_slot[6]), 
         .C(n24911), .Z(event_rd_addr[6])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(231[33:74])
    defparam run_addr_s3_8__I_0_i7_3_lut.init = 16'hacac;
    FD1P3IX ev_bit_i17 (.D(ev_bit[16]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i17.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_315 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[61]), 
         .D(ev_bit[61]), .Z(ev_wr_data[61])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_315.init = 16'hddd0;
    PFUMX i14410 (.BLUT(n25980), .ALUT(n25979), .C0(spi_byte_count[1]), 
          .Z(n25981));
    LUT4 i1_3_lut_4_lut_adj_316 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[62]), 
         .D(ev_bit[62]), .Z(ev_wr_data[62])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_316.init = 16'hddd0;
    LUT4 i1_3_lut_adj_317 (.A(n17439), .B(init_shadow[66]), .C(ev_bit[66]), 
         .Z(n15992)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_317.init = 16'hecec;
    LUT4 mux_1255_i3_3_lut_4_lut (.A(ev_state[1]), .B(n26050), .C(build_sum[2]), 
         .D(build_phase[2]), .Z(ev_wr_addr_8__N_925[2])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam mux_1255_i3_3_lut_4_lut.init = 16'hf2d0;
    CCU2D fpga_time_1412_add_4_19 (.A0(fpga_time[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24817), .COUT(n24818), .S0(n148), .S1(n147));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_19.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_19.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_19.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_19.INJECT1_1 = "NO";
    LUT4 i4599_4_lut (.A(ev_ch[5]), .B(staging_rd_addr[5]), .C(n18417), 
         .D(n26077), .Z(staging_rd_addr_6__N_914[5])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(565[9] 638[16])
    defparam i4599_4_lut.init = 16'hcac0;
    FD1P3IX ev_bit_i56 (.D(ev_bit[55]), .SP(pll_clk_enable_753), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i56.GSR = "DISABLED";
    CCU2D add_13292_7 (.A0(spi_byte_count[8]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24843), .S1(n11999));
    defparam add_13292_7.INIT0 = 16'h5555;
    defparam add_13292_7.INIT1 = 16'h0000;
    defparam add_13292_7.INJECT1_0 = "NO";
    defparam add_13292_7.INJECT1_1 = "NO";
    CCU2D add_13292_5 (.A0(spi_byte_count[6]), .B0(spi_rgb_payload_byte_N_2770[6]), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[7]), .B1(spi_rgb_payload_byte_N_2770[6]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24842), .COUT(n24843));
    defparam add_13292_5.INIT0 = 16'h5999;
    defparam add_13292_5.INIT1 = 16'h5999;
    defparam add_13292_5.INJECT1_0 = "NO";
    defparam add_13292_5.INJECT1_1 = "NO";
    CCU2D fpga_time_1412_add_4_17 (.A0(fpga_time[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24816), .COUT(n24817), .S0(n150), .S1(n149));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_17.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_17.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_17.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_17.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_318 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[63]), 
         .D(ev_bit[63]), .Z(ev_wr_data[63])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_318.init = 16'hddd0;
    FD1P3IX ev_bit_i16 (.D(ev_bit[15]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i16.GSR = "DISABLED";
    LUT4 run_addr_s3_8__I_0_i8_3_lut (.A(run_addr_s3[7]), .B(ev_rd_slot[7]), 
         .C(n24911), .Z(event_rd_addr[7])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(231[33:74])
    defparam run_addr_s3_8__I_0_i8_3_lut.init = 16'hacac;
    FD1P3IX ev_bit_i15 (.D(ev_bit[14]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i15.GSR = "DISABLED";
    FD1P3IX ev_bit_i14 (.D(ev_bit[13]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i14.GSR = "DISABLED";
    FD1P3IX init_shadow_i75 (.D(n16046), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i75.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_319 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[64]), 
         .D(ev_bit[64]), .Z(ev_wr_data[64])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_319.init = 16'hddd0;
    FD1P3IX init_shadow_i74 (.D(n16040), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i74.GSR = "DISABLED";
    FD1P3IX init_shadow_i73 (.D(n16034), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i73.GSR = "DISABLED";
    CCU2D add_166_17 (.A0(spi_byte_count[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24777), .S0(spi_byte_count_15__N_1628[15]), .S1(frame_end_N_2840[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(402[35:57])
    defparam add_166_17.INIT0 = 16'h5aaa;
    defparam add_166_17.INIT1 = 16'h0000;
    defparam add_166_17.INJECT1_0 = "NO";
    defparam add_166_17.INJECT1_1 = "NO";
    CCU2D add_166_15 (.A0(spi_byte_count[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24776), .COUT(n24777), .S0(spi_byte_count_15__N_1628[13]), 
          .S1(spi_byte_count_15__N_1628[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(402[35:57])
    defparam add_166_15.INIT0 = 16'h5aaa;
    defparam add_166_15.INIT1 = 16'h5aaa;
    defparam add_166_15.INJECT1_0 = "NO";
    defparam add_166_15.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i72 (.D(n16028), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i72.GSR = "DISABLED";
    CCU2D add_166_13 (.A0(spi_byte_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24775), .COUT(n24776), .S0(spi_byte_count_15__N_1628[11]), 
          .S1(spi_byte_count_15__N_1628[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(402[35:57])
    defparam add_166_13.INIT0 = 16'h5aaa;
    defparam add_166_13.INIT1 = 16'h5aaa;
    defparam add_166_13.INJECT1_0 = "NO";
    defparam add_166_13.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i71 (.D(n16022), .SP(pll_clk_enable_765), .CD(n26269), 
            .CK(pll_clk), .Q(init_shadow[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i71.GSR = "DISABLED";
    FD1P3IX init_shadow_i78 (.D(n16064), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i78.GSR = "DISABLED";
    FD1P3IX ev_bit_i13 (.D(ev_bit[12]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i13.GSR = "DISABLED";
    GSR GSR_INST (.GSR(fpga_cs_n_N_2723));
    FD1P3IX init_shadow_i79 (.D(n16070), .SP(pll_clk_enable_765), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i79.GSR = "DISABLED";
    CCU2D add_166_11 (.A0(spi_byte_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24774), .COUT(n24775), .S0(spi_byte_count_15__N_1628[9]), 
          .S1(spi_byte_count_15__N_1628[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(402[35:57])
    defparam add_166_11.INIT0 = 16'h5aaa;
    defparam add_166_11.INIT1 = 16'h5aaa;
    defparam add_166_11.INJECT1_0 = "NO";
    defparam add_166_11.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_320 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[65]), 
         .D(ev_bit[65]), .Z(ev_wr_data[65])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_320.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_321 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[12]), 
         .D(ev_bit[12]), .Z(ev_wr_data[12])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_321.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_322 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[13]), 
         .D(ev_bit[13]), .Z(ev_wr_data[13])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_322.init = 16'hddd0;
    LUT4 mux_1255_i1_3_lut_4_lut (.A(ev_state[1]), .B(n26050), .C(build_sum[0]), 
         .D(build_phase[0]), .Z(ev_wr_addr_8__N_925[0])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[33:53])
    defparam mux_1255_i1_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i1_3_lut_4_lut_adj_323 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[14]), 
         .D(ev_bit[14]), .Z(ev_wr_data[14])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_323.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_324 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[15]), 
         .D(ev_bit[15]), .Z(ev_wr_data[15])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_324.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_325 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[16]), 
         .D(ev_bit[16]), .Z(ev_wr_data[16])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_325.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_326 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[17]), 
         .D(ev_bit[17]), .Z(ev_wr_data[17])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_326.init = 16'hddd0;
    CCU2D fpga_time_1412_add_4_15 (.A0(fpga_time[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24815), .COUT(n24816), .S0(n152), .S1(n151));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_15.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_15.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_15.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_15.INJECT1_1 = "NO";
    CCU2D add_634_25 (.A0(phase_frac[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24793), .S0(phase_frac_sum[23]), .S1(phase_frac_sum[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_25.INIT0 = 16'h5aaa;
    defparam add_634_25.INIT1 = 16'h0000;
    defparam add_634_25.INJECT1_0 = "NO";
    defparam add_634_25.INJECT1_1 = "NO";
    CCU2D add_166_9 (.A0(spi_byte_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24773), .COUT(n24774), .S0(spi_byte_count_15__N_1628[7]), 
          .S1(spi_byte_count_15__N_1628[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(402[35:57])
    defparam add_166_9.INIT0 = 16'h5aaa;
    defparam add_166_9.INIT1 = 16'h5aaa;
    defparam add_166_9.INJECT1_0 = "NO";
    defparam add_166_9.INJECT1_1 = "NO";
    CCU2D add_166_7 (.A0(spi_byte_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24772), .COUT(n24773), .S0(spi_byte_count_15__N_1628[5]), 
          .S1(spi_byte_count_15__N_1628[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(402[35:57])
    defparam add_166_7.INIT0 = 16'h5aaa;
    defparam add_166_7.INIT1 = 16'h5aaa;
    defparam add_166_7.INJECT1_0 = "NO";
    defparam add_166_7.INJECT1_1 = "NO";
    CCU2D add_634_23 (.A0(phase_frac[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24792), .COUT(n24793), .S0(phase_frac_sum[21]), 
          .S1(phase_frac_sum[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_23.INIT0 = 16'h5aaa;
    defparam add_634_23.INIT1 = 16'h5aaa;
    defparam add_634_23.INJECT1_0 = "NO";
    defparam add_634_23.INJECT1_1 = "NO";
    CCU2D add_166_5 (.A0(spi_byte_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24771), .COUT(n24772), .S0(spi_byte_count_15__N_1628[3]), 
          .S1(spi_byte_count_15__N_1628[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(402[35:57])
    defparam add_166_5.INIT0 = 16'h5aaa;
    defparam add_166_5.INIT1 = 16'h5aaa;
    defparam add_166_5.INJECT1_0 = "NO";
    defparam add_166_5.INJECT1_1 = "NO";
    CCU2D add_166_3 (.A0(spi_byte_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24770), .COUT(n24771), .S0(spi_byte_count_15__N_1628[1]), 
          .S1(spi_byte_count_15__N_1628[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(402[35:57])
    defparam add_166_3.INIT0 = 16'h5aaa;
    defparam add_166_3.INIT1 = 16'h5aaa;
    defparam add_166_3.INJECT1_0 = "NO";
    defparam add_166_3.INJECT1_1 = "NO";
    CCU2D add_13292_3 (.A0(spi_byte_count[4]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[5]), .B1(spi_rgb_payload_byte_N_2770[6]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24841), .COUT(n24842));
    defparam add_13292_3.INIT0 = 16'h5555;
    defparam add_13292_3.INIT1 = 16'h5666;
    defparam add_13292_3.INJECT1_0 = "NO";
    defparam add_13292_3.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_327 (.A(n17439), .B(init_shadow[77]), .C(ev_bit[77]), 
         .Z(n16058)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_327.init = 16'hecec;
    INV i14467 (.A(spi1_sck_c), .Z(spi1_sck_N_413));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    FD1S3AX phase_step_s4_507_rep_176 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(pll_clk_enable_104)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_step_s4_507_rep_176.GSR = "DISABLED";
    PFUMX frame_end_I_0 (.BLUT(frame_end_N_2872), .ALUT(frame_end_N_2837), 
          .C0(n26027), .Z(frame_end));
    CCU2D add_13292_1 (.A0(spi_byte_count[2]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[3]), .B1(spi_rgb_payload_byte_N_2770[6]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24841));
    defparam add_13292_1.INIT0 = 16'h5000;
    defparam add_13292_1.INIT1 = 16'h5999;
    defparam add_13292_1.INJECT1_0 = "NO";
    defparam add_13292_1.INJECT1_1 = "NO";
    CCU2D fpga_time_1412_add_4_13 (.A0(fpga_time[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24814), .COUT(n24815), .S0(n154), .S1(n153));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_13.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_13.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_13.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_13.INJECT1_1 = "NO";
    CCU2D add_634_21 (.A0(phase_frac[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24791), .COUT(n24792), .S0(phase_frac_sum[19]), 
          .S1(phase_frac_sum[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_21.INIT0 = 16'h5aaa;
    defparam add_634_21.INIT1 = 16'h5555;
    defparam add_634_21.INJECT1_0 = "NO";
    defparam add_634_21.INJECT1_1 = "NO";
    CCU2D add_634_19 (.A0(phase_frac[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24790), .COUT(n24791), .S0(phase_frac_sum[17]), 
          .S1(phase_frac_sum[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_19.INIT0 = 16'h5aaa;
    defparam add_634_19.INIT1 = 16'h5555;
    defparam add_634_19.INJECT1_0 = "NO";
    defparam add_634_19.INJECT1_1 = "NO";
    CCU2D fpga_time_1412_add_4_11 (.A0(fpga_time[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24813), .COUT(n24814), .S0(n156), .S1(n155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_11.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_11.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_11.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_11.INJECT1_1 = "NO";
    CCU2D add_634_17 (.A0(phase_frac[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24789), .COUT(n24790), .S0(phase_frac_sum[15]), 
          .S1(phase_frac_sum[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_17.INIT0 = 16'h5aaa;
    defparam add_634_17.INIT1 = 16'h5aaa;
    defparam add_634_17.INJECT1_0 = "NO";
    defparam add_634_17.INJECT1_1 = "NO";
    CCU2D add_634_15 (.A0(phase_frac[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24788), .COUT(n24789), .S0(phase_frac_sum[13]), 
          .S1(phase_frac_sum[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_15.INIT0 = 16'h5555;
    defparam add_634_15.INIT1 = 16'h5555;
    defparam add_634_15.INJECT1_0 = "NO";
    defparam add_634_15.INJECT1_1 = "NO";
    CCU2D time_divider_1413_add_4_7 (.A0(time_divider[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24839), .S0(n35_adj_3409), 
          .S1(n34_adj_3408));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413_add_4_7.INIT0 = 16'hfaaa;
    defparam time_divider_1413_add_4_7.INIT1 = 16'hfaaa;
    defparam time_divider_1413_add_4_7.INJECT1_0 = "NO";
    defparam time_divider_1413_add_4_7.INJECT1_1 = "NO";
    CCU2D time_divider_1413_add_4_5 (.A0(time_divider[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24838), .COUT(n24839), .S0(n37_adj_3411), 
          .S1(n36_adj_3410));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413_add_4_5.INIT0 = 16'hfaaa;
    defparam time_divider_1413_add_4_5.INIT1 = 16'hfaaa;
    defparam time_divider_1413_add_4_5.INJECT1_0 = "NO";
    defparam time_divider_1413_add_4_5.INJECT1_1 = "NO";
    CCU2D fpga_time_1412_add_4_9 (.A0(fpga_time[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24812), .COUT(n24813), .S0(n158), .S1(n157));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_9.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_9.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_9.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_9.INJECT1_1 = "NO";
    CCU2D add_634_13 (.A0(phase_frac[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24787), .COUT(n24788), .S0(phase_frac_sum[11]), 
          .S1(phase_frac_sum[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_13.INIT0 = 16'h5555;
    defparam add_634_13.INIT1 = 16'h5555;
    defparam add_634_13.INJECT1_0 = "NO";
    defparam add_634_13.INJECT1_1 = "NO";
    CCU2D add_166_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(spi_byte_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24770), .S1(spi_byte_count_15__N_1628[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(402[35:57])
    defparam add_166_1.INIT0 = 16'hF000;
    defparam add_166_1.INIT1 = 16'h5555;
    defparam add_166_1.INJECT1_0 = "NO";
    defparam add_166_1.INJECT1_1 = "NO";
    LUT4 i13327_3_lut (.A(spi_bit_count[2]), .B(spi_bit_count[1]), .C(spi_bit_count[0]), 
         .Z(n18)) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam i13327_3_lut.init = 16'h6a6a;
    LUT4 i1_3_lut_4_lut_adj_328 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[18]), 
         .D(ev_bit[18]), .Z(ev_wr_data[18])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_328.init = 16'hddd0;
    LUT4 i13342_2_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .Z(n29_adj_3416)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam i13342_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_329 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[19]), 
         .D(ev_bit[19]), .Z(ev_wr_data[19])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_329.init = 16'hddd0;
    CCU2D add_636_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24769), 
          .S0(build_sum_8__N_2255[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[32:80])
    defparam add_636_cout.INIT0 = 16'h0000;
    defparam add_636_cout.INIT1 = 16'h0000;
    defparam add_636_cout.INJECT1_0 = "NO";
    defparam add_636_cout.INJECT1_1 = "NO";
    CCU2D add_634_11 (.A0(phase_frac[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24786), .COUT(n24787), .S0(phase_frac_sum[9]), 
          .S1(phase_frac_sum[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_11.INIT0 = 16'h5555;
    defparam add_634_11.INIT1 = 16'h5aaa;
    defparam add_634_11.INJECT1_0 = "NO";
    defparam add_634_11.INJECT1_1 = "NO";
    CCU2D add_636_8 (.A0(staging_q[14]), .B0(staging_q[6]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[15]), .B1(staging_q[7]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24768), .COUT(n24769), .S0(build_sum_8__N_2255[6]), 
          .S1(build_sum_8__N_2255[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[32:80])
    defparam add_636_8.INIT0 = 16'h5666;
    defparam add_636_8.INIT1 = 16'h5666;
    defparam add_636_8.INJECT1_0 = "NO";
    defparam add_636_8.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_632_9 (.A0(spi_rx_shift[0]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[1]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24757), .COUT(n24758), .S0(expected_next[9]), 
          .S1(expected_next[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[33] 308[86])
    defparam expected_next_15__I_0_632_9.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_632_9.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_632_9.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_632_9.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_330 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[20]), 
         .D(ev_bit[20]), .Z(ev_wr_data[20])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_330.init = 16'hddd0;
    CCU2D expected_next_15__I_0_632_15 (.A0(spi_rx_shift[6]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24760), .S0(expected_next[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[33] 308[86])
    defparam expected_next_15__I_0_632_15.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_632_15.INIT1 = 16'h0000;
    defparam expected_next_15__I_0_632_15.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_632_15.INJECT1_1 = "NO";
    CCU2D add_636_6 (.A0(staging_q[12]), .B0(staging_q[4]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[13]), .B1(staging_q[5]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24767), .COUT(n24768), .S0(build_sum_8__N_2255[4]), 
          .S1(build_sum_8__N_2255[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[32:80])
    defparam add_636_6.INIT0 = 16'h5666;
    defparam add_636_6.INIT1 = 16'h5666;
    defparam add_636_6.INJECT1_0 = "NO";
    defparam add_636_6.INJECT1_1 = "NO";
    CCU2D fpga_time_1412_add_4_7 (.A0(fpga_time[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24811), .COUT(n24812), .S0(n160), .S1(n159));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_7.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_7.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_7.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_7.INJECT1_1 = "NO";
    CCU2D add_634_9 (.A0(phase_frac[7]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[8]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24785), .COUT(n24786), .S0(phase_frac_sum[7]), .S1(phase_frac_sum[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_9.INIT0 = 16'h5555;
    defparam add_634_9.INIT1 = 16'h5aaa;
    defparam add_634_9.INJECT1_0 = "NO";
    defparam add_634_9.INJECT1_1 = "NO";
    CCU2D add_636_4 (.A0(staging_q[10]), .B0(staging_q[2]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[11]), .B1(staging_q[3]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24766), .COUT(n24767), .S0(build_sum_8__N_2255[2]), 
          .S1(build_sum_8__N_2255[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[32:80])
    defparam add_636_4.INIT0 = 16'h5666;
    defparam add_636_4.INIT1 = 16'h5666;
    defparam add_636_4.INJECT1_0 = "NO";
    defparam add_636_4.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_331 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[21]), 
         .D(ev_bit[21]), .Z(ev_wr_data[21])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_331.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_332 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[22]), 
         .D(ev_bit[22]), .Z(ev_wr_data[22])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_332.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_333 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[23]), 
         .D(ev_bit[23]), .Z(ev_wr_data[23])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_333.init = 16'hddd0;
    LUT4 spi_byte_count_1__bdd_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[4]), .D(spi_byte_count[5]), .Z(n25979)) /* synthesis lut_function=(!(A (B+(C+(D)))+!A (B+(C (D)+!C !(D))))) */ ;
    defparam spi_byte_count_1__bdd_4_lut.init = 16'h0112;
    CCU2D sub_1246_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[3]), .B1(spi_rgb_payload_byte_N_2770[6]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24761));
    defparam sub_1246_add_2_1.INIT0 = 16'h0000;
    defparam sub_1246_add_2_1.INIT1 = 16'h5999;
    defparam sub_1246_add_2_1.INJECT1_0 = "NO";
    defparam sub_1246_add_2_1.INJECT1_1 = "NO";
    LUT4 run_addr_s3_8__I_0_i2_3_lut (.A(run_addr_s3[1]), .B(ev_rd_slot[1]), 
         .C(n24911), .Z(event_rd_addr[1])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(231[33:74])
    defparam run_addr_s3_8__I_0_i2_3_lut.init = 16'hacac;
    CCU2D expected_next_15__I_0_632_13 (.A0(spi_rx_shift[4]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[5]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24759), .COUT(n24760), .S0(expected_next[13]), 
          .S1(expected_next[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[33] 308[86])
    defparam expected_next_15__I_0_632_13.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_632_13.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_632_13.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_632_13.INJECT1_1 = "NO";
    CCU2D add_636_2 (.A0(staging_q[8]), .B0(staging_q[0]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[9]), .B1(staging_q[1]), .C1(GND_net), 
          .D1(GND_net), .COUT(n24766), .S1(build_sum_8__N_2255[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(595[32:80])
    defparam add_636_2.INIT0 = 16'h7000;
    defparam add_636_2.INIT1 = 16'h5666;
    defparam add_636_2.INJECT1_0 = "NO";
    defparam add_636_2.INJECT1_1 = "NO";
    CCU2D add_634_7 (.A0(phase_frac[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24784), .COUT(n24785), .S0(phase_frac_sum[5]), .S1(phase_frac_sum[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[34:66])
    defparam add_634_7.INIT0 = 16'h5555;
    defparam add_634_7.INIT1 = 16'h5555;
    defparam add_634_7.INJECT1_0 = "NO";
    defparam add_634_7.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_334 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[24]), 
         .D(ev_bit[24]), .Z(ev_wr_data[24])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_334.init = 16'hddd0;
    LUT4 i1_3_lut_adj_335 (.A(n17439), .B(init_shadow[0]), .C(ev_bit[0]), 
         .Z(n14058)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_335.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_336 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[25]), 
         .D(ev_bit[25]), .Z(ev_wr_data[25])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_336.init = 16'hddd0;
    CCU2D sub_1246_add_2_3 (.A0(spi_byte_count[4]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[5]), .B1(spi_rgb_payload_byte_N_2770[6]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24761), .COUT(n24762));
    defparam sub_1246_add_2_3.INIT0 = 16'h5aaa;
    defparam sub_1246_add_2_3.INIT1 = 16'h5666;
    defparam sub_1246_add_2_3.INJECT1_0 = "NO";
    defparam sub_1246_add_2_3.INJECT1_1 = "NO";
    CCU2D equal_1981_17 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24623), 
          .S0(frame_end_N_2839));
    defparam equal_1981_17.INIT0 = 16'hFFFF;
    defparam equal_1981_17.INIT1 = 16'h0000;
    defparam equal_1981_17.INJECT1_0 = "NO";
    defparam equal_1981_17.INJECT1_1 = "NO";
    CCU2D equal_1981_17_13291 (.A0(spi_expected_length[3]), .B0(spi_byte_count_15__N_1628[3]), 
          .C0(spi_expected_length[2]), .D0(spi_byte_count_15__N_1628[2]), 
          .A1(spi_expected_length[1]), .B1(spi_byte_count_15__N_1628[1]), 
          .C1(spi_expected_length[0]), .D1(spi_byte_count_15__N_1628[0]), 
          .CIN(n24622), .COUT(n24623));
    defparam equal_1981_17_13291.INIT0 = 16'h9009;
    defparam equal_1981_17_13291.INIT1 = 16'h9009;
    defparam equal_1981_17_13291.INJECT1_0 = "YES";
    defparam equal_1981_17_13291.INJECT1_1 = "YES";
    CCU2D expected_next_15__I_0_632_3 (.A0(spi_rgb_payload_byte_N_2770[6]), 
          .B0(spi_extension_length[3]), .C0(GND_net), .D0(GND_net), .A1(expected_next_15__N_1475[3]), 
          .B1(spi_extension_length[4]), .C1(GND_net), .D1(GND_net), .CIN(n24754), 
          .COUT(n24755), .S0(expected_next[3]), .S1(expected_next[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[33] 308[86])
    defparam expected_next_15__I_0_632_3.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_632_3.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_632_3.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_632_3.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_632_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(expected_next_15__N_1475[3]), .B1(spi_extension_length[2]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24754), .S1(expected_next[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(306[33] 308[86])
    defparam expected_next_15__I_0_632_1.INIT0 = 16'hF000;
    defparam expected_next_15__I_0_632_1.INIT1 = 16'ha999;
    defparam expected_next_15__I_0_632_1.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_632_1.INJECT1_1 = "NO";
    CCU2D equal_1981_15 (.A0(spi_expected_length[7]), .B0(spi_byte_count_15__N_1628[7]), 
          .C0(spi_expected_length[6]), .D0(spi_byte_count_15__N_1628[6]), 
          .A1(spi_expected_length[5]), .B1(spi_byte_count_15__N_1628[5]), 
          .C1(spi_expected_length[4]), .D1(spi_byte_count_15__N_1628[4]), 
          .CIN(n24621), .COUT(n24622));
    defparam equal_1981_15.INIT0 = 16'h9009;
    defparam equal_1981_15.INIT1 = 16'h9009;
    defparam equal_1981_15.INJECT1_0 = "YES";
    defparam equal_1981_15.INJECT1_1 = "YES";
    CCU2D equal_1981_11 (.A0(spi_expected_length[15]), .B0(spi_byte_count_15__N_1628[15]), 
          .C0(spi_expected_length[14]), .D0(spi_byte_count_15__N_1628[14]), 
          .A1(spi_expected_length[13]), .B1(spi_byte_count_15__N_1628[13]), 
          .C1(spi_expected_length[12]), .D1(spi_byte_count_15__N_1628[12]), 
          .CIN(n24619), .COUT(n24620));
    defparam equal_1981_11.INIT0 = 16'h9009;
    defparam equal_1981_11.INIT1 = 16'h9009;
    defparam equal_1981_11.INJECT1_0 = "YES";
    defparam equal_1981_11.INJECT1_1 = "YES";
    CCU2D equal_1981_13 (.A0(spi_expected_length[11]), .B0(spi_byte_count_15__N_1628[11]), 
          .C0(spi_expected_length[10]), .D0(spi_byte_count_15__N_1628[10]), 
          .A1(spi_expected_length[9]), .B1(spi_byte_count_15__N_1628[9]), 
          .C1(spi_expected_length[8]), .D1(spi_byte_count_15__N_1628[8]), 
          .CIN(n24620), .COUT(n24621));
    defparam equal_1981_13.INIT0 = 16'h9009;
    defparam equal_1981_13.INIT1 = 16'h9009;
    defparam equal_1981_13.INJECT1_0 = "YES";
    defparam equal_1981_13.INJECT1_1 = "YES";
    LUT4 i24_4_lut (.A(n25503), .B(n14_adj_3387), .C(ev_state[3]), .D(ev_state[0]), 
         .Z(ev_state_3__N_710[0])) /* synthesis lut_function=(A (B (C+!(D))+!B !(C+(D)))+!A (B (C))) */ ;
    defparam i24_4_lut.init = 16'hc0ca;
    CCU2D sub_1246_add_2_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24764), .S0(n12011));
    defparam sub_1246_add_2_cout.INIT0 = 16'h0000;
    defparam sub_1246_add_2_cout.INIT1 = 16'h0000;
    defparam sub_1246_add_2_cout.INJECT1_0 = "NO";
    defparam sub_1246_add_2_cout.INJECT1_1 = "NO";
    CCU2D time_divider_1413_add_4_3 (.A0(time_divider[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24837), .COUT(n24838), .S0(n39_adj_3413), 
          .S1(n38_adj_3412));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413_add_4_3.INIT0 = 16'hfaaa;
    defparam time_divider_1413_add_4_3.INIT1 = 16'hfaaa;
    defparam time_divider_1413_add_4_3.INJECT1_0 = "NO";
    defparam time_divider_1413_add_4_3.INJECT1_1 = "NO";
    PFUMX i14245 (.BLUT(n25693), .ALUT(n25694), .C0(n26080), .Z(n25704));
    VLO i1 (.Z(GND_net));
    TSALL TSALL_INST (.TSALL(GND_net));
    PUR PUR_INST (.PUR(VCC_net));
    defparam PUR_INST.RST_PULSE = 1;
    LUT4 spi_byte_count_1__bdd_3_lut (.A(spi_byte_count[2]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[5]), .Z(n25980)) /* synthesis lut_function=(!(A+(B+(C)))) */ ;
    defparam spi_byte_count_1__bdd_3_lut.init = 16'h0101;
    LUT4 i1_3_lut_4_lut_adj_337 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[26]), 
         .D(ev_bit[26]), .Z(ev_wr_data[26])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_337.init = 16'hddd0;
    FD1P3IX ev_bit_i12 (.D(ev_bit[11]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i12.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_338 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[27]), 
         .D(ev_bit[27]), .Z(ev_wr_data[27])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_338.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_339 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[28]), 
         .D(ev_bit[28]), .Z(ev_wr_data[28])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_339.init = 16'hddd0;
    FD1S3AX phase_step_s4_507_rep_172 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(n26255)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam phase_step_s4_507_rep_172.GSR = "DISABLED";
    FD1P3IX ev_bit_i11 (.D(ev_bit[10]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i11.GSR = "DISABLED";
    FD1S3AX spi_bit_count_1410__i2 (.D(n18), .CK(spi1_sck_c), .Q(spi_bit_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(430[34:54])
    defparam spi_bit_count_1410__i2.GSR = "ENABLED";
    FD1S3IX mic_divider_1415__i1 (.D(n39_adj_3399), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415__i1.GSR = "DISABLED";
    FD1S3IX mic_divider_1415__i2 (.D(n38_adj_3400), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415__i2.GSR = "DISABLED";
    FD1S3IX mic_divider_1415__i3 (.D(n37_adj_3401), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415__i3.GSR = "DISABLED";
    FD1S3IX mic_divider_1415__i4 (.D(n36_adj_3402), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415__i4.GSR = "DISABLED";
    FD1S3IX mic_divider_1415__i5 (.D(n35_adj_3403), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415__i5.GSR = "DISABLED";
    FD1S3IX mic_divider_1415__i6 (.D(n34_adj_3404), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(673[28:46])
    defparam mic_divider_1415__i6.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1414__i1 (.D(n29_adj_3416), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_sample_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam mic_sample_count_1414__i1.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_340 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[29]), 
         .D(ev_bit[29]), .Z(ev_wr_data[29])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_340.init = 16'hddd0;
    FD1P3AX mic_sample_count_1414__i2 (.D(n28_adj_3415), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_sample_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam mic_sample_count_1414__i2.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1414__i3 (.D(n27), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_sample_count[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam mic_sample_count_1414__i3.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1414__i4 (.D(n26_adj_3417), .SP(pll_clk_enable_771), 
            .CK(pll_clk), .Q(mic_sample_count[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(665[37:60])
    defparam mic_sample_count_1414__i4.GSR = "DISABLED";
    FD1S3AX time_divider_1413__i1 (.D(n39_adj_3413), .CK(pll_clk), .Q(time_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413__i1.GSR = "DISABLED";
    FD1S3AX time_divider_1413__i2 (.D(n38_adj_3412), .CK(pll_clk), .Q(time_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413__i2.GSR = "DISABLED";
    FD1S3AX time_divider_1413__i3 (.D(n37_adj_3411), .CK(pll_clk), .Q(time_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413__i3.GSR = "DISABLED";
    FD1S3AX time_divider_1413__i4 (.D(n36_adj_3410), .CK(pll_clk), .Q(time_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413__i4.GSR = "DISABLED";
    FD1S3AX time_divider_1413__i5 (.D(n35_adj_3409), .CK(pll_clk), .Q(time_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413__i5.GSR = "DISABLED";
    FD1S3AX time_divider_1413__i6 (.D(n34_adj_3408), .CK(pll_clk), .Q(time_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[29:48])
    defparam time_divider_1413__i6.GSR = "DISABLED";
    FD1P3AX spi_rgb_index_1409__i1 (.D(n24), .SP(spi1_sck_c_enable_199), 
            .CK(spi1_sck_c), .Q(spi_rgb_index[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(398[46:66])
    defparam spi_rgb_index_1409__i1.GSR = "ENABLED";
    FD1P3AX spi_rgb_index_1409__i2 (.D(n23), .SP(spi1_sck_c_enable_199), 
            .CK(spi1_sck_c), .Q(spi_rgb_index[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(398[46:66])
    defparam spi_rgb_index_1409__i2.GSR = "ENABLED";
    FD1P3AX spi_rgb_index_1409__i3 (.D(n22), .SP(spi1_sck_c_enable_199), 
            .CK(spi1_sck_c), .Q(spi_rgb_index[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(398[46:66])
    defparam spi_rgb_index_1409__i3.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut_adj_341 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[30]), 
         .D(ev_bit[30]), .Z(ev_wr_data[30])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_341.init = 16'hddd0;
    CCU2D fpga_time_1412_add_4_5 (.A0(fpga_time[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24810), .COUT(n24811), .S0(n162), .S1(n161));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[29:46])
    defparam fpga_time_1412_add_4_5.INIT0 = 16'hfaaa;
    defparam fpga_time_1412_add_4_5.INIT1 = 16'hfaaa;
    defparam fpga_time_1412_add_4_5.INJECT1_0 = "NO";
    defparam fpga_time_1412_add_4_5.INJECT1_1 = "NO";
    umh_toggle_ram84 event_ram (.pll_clk(pll_clk), .ev_we(ev_we), .VCC_net(VCC_net), 
            .GND_net(GND_net), .\ev_wr_addr[0] (ev_wr_addr[0]), .event_rd_addr({event_rd_addr}), 
            .\ev_wr_addr[1] (ev_wr_addr[1]), .\ev_wr_addr[2] (ev_wr_addr[2]), 
            .\ev_wr_addr[3] (ev_wr_addr[3]), .\ev_wr_addr[4] (ev_wr_addr[4]), 
            .\ev_wr_addr[5] (ev_wr_addr[5]), .\ev_wr_addr[6] (ev_wr_addr[6]), 
            .\ev_wr_addr[7] (ev_wr_addr[7]), .n26101(n26101), .ev_wr_data({ev_wr_data}), 
            .ev_rd_data({ev_rd_data})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(241[22] 246[6])
    LUT4 i1_3_lut_4_lut_adj_342 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[31]), 
         .D(ev_bit[31]), .Z(ev_wr_data[31])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_342.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_343 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[32]), 
         .D(ev_bit[32]), .Z(ev_wr_data[32])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_343.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_344 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[33]), 
         .D(ev_bit[33]), .Z(ev_wr_data[33])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_344.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_345 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[34]), 
         .D(ev_bit[34]), .Z(ev_wr_data[34])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_345.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_346 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[35]), 
         .D(ev_bit[35]), .Z(ev_wr_data[35])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_346.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_347 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[36]), 
         .D(ev_bit[36]), .Z(ev_wr_data[36])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_347.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_348 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[37]), 
         .D(ev_bit[37]), .Z(ev_wr_data[37])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_348.init = 16'hddd0;
    LUT4 ev_state_0__bdd_4_lut (.A(ev_state[1]), .B(ev_state[3]), .C(n13), 
         .D(ev_state[2]), .Z(n26011)) /* synthesis lut_function=(!((B (C+(D)))+!A)) */ ;
    defparam ev_state_0__bdd_4_lut.init = 16'h222a;
    LUT4 n26011_bdd_3_lut (.A(n26011), .B(n26020), .C(ev_state[0]), .Z(ev_state_3__N_710[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;
    defparam n26011_bdd_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_349 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[38]), 
         .D(ev_bit[38]), .Z(ev_wr_data[38])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_349.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_350 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[39]), 
         .D(ev_bit[39]), .Z(ev_wr_data[39])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_350.init = 16'hddd0;
    FD1P3IX ev_bit_i2 (.D(ev_bit[1]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i2.GSR = "DISABLED";
    FD1P3IX ev_bit_i1 (.D(ev_bit[0]), .SP(pll_clk_enable_773), .CD(n18521), 
            .CK(pll_clk), .Q(ev_bit[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_bit_i1.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_351 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[40]), 
         .D(ev_bit[40]), .Z(ev_wr_data[40])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_351.init = 16'hddd0;
    FD1P3IX ev_clear_addr_i0 (.D(ev_clear_addr_7__N_2439[0]), .SP(pll_clk_enable_775), 
            .CD(n18521), .CK(pll_clk), .Q(ev_clear_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_clear_addr_i0.GSR = "DISABLED";
    FD1P3IX init_shadow_i0 (.D(n14058), .SP(pll_clk_enable_776), .CD(n18625), 
            .CK(pll_clk), .Q(init_shadow[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam init_shadow_i0.GSR = "DISABLED";
    FD1P3AX ev_state_i0 (.D(ev_state_3__N_710[0]), .SP(pll_clk_enable_777), 
            .CK(pll_clk), .Q(ev_state[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam ev_state_i0.GSR = "DISABLED";
    FD1S3JX swap_pending_503 (.D(swap_pending_N_2813), .CK(pll_clk), .PD(n49), 
            .Q(swap_pending)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[12] 675[8])
    defparam swap_pending_503.GSR = "DISABLED";
    umh_channel_ram18 staging_ram (.n12887(n12887), .spi1_sck_c(spi1_sck_c), 
            .spi_channel_index({spi_channel_index}), .staging_q({staging_q}), 
            .pll_clk(pll_clk), .rd_data_15__N_2873({rd_data_15__N_2873}), 
            .n12881(n12881), .n12879(n12879), .n12889(n12889), .n12891(n12891), 
            .n13(n13), .spi_write(spi_write), .VCC_net(VCC_net), .GND_net(GND_net), 
            .staging_rd_addr_6__N_914({staging_rd_addr_6__N_914}), .spi1_mosi_c_0(spi1_mosi_c_0), 
            .\spi_rx_shift[0] (spi_rx_shift[0]), .\spi_rx_shift[1] (spi_rx_shift[1]), 
            .\spi_rx_shift[2] (spi_rx_shift[2]), .\spi_rx_shift[3] (spi_rx_shift[3]), 
            .\spi_rx_shift[4] (spi_rx_shift[4]), .\spi_rx_shift[5] (spi_rx_shift[5]), 
            .\spi_rx_shift[6] (spi_rx_shift[6]), .spi_phase_pending({spi_phase_pending}), 
            .n12896(n12896), .n12898(n12898), .n12900(n12900), .n12902(n12902), 
            .n12904(n12904), .n12906(n12906), .n12908(n12908), .n12910(n12910), 
            .n12912(n12912), .n12914(n12914), .n12916(n12916), .n12918(n12918), 
            .n12920(n12920), .n12922(n12922), .n12924(n12924), .n12926(n12926), 
            .n12883(n12883), .n12885(n12885)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[23] 222[6])
    FD1P3AX accepted_sequence_spi_i0_i27 (.D(spi_frame_sequence[27]), .SP(spi1_sck_c_enable_200), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[27]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[18] 432[12])
    defparam accepted_sequence_spi_i0_i27.GSR = "DISABLED";
    ws2812_stream ws2812_i (.n15(n15), .state({state}), .pll_clk(pll_clk), 
            .n17658(n17658), .\rgb_hold[5] (rgb_hold[5]), .\rgb_hold[6] (rgb_hold[6]), 
            .\rgb_hold[7] (rgb_hold[7]), .ws2812_enable(ws2812_enable), 
            .\shift_register[1] (shift_register[1]), .\shift_register_23__N_3134[1] (shift_register_23__N_3134[1]), 
            .\shift_register[2] (shift_register[2]), .\shift_register_23__N_3134[2] (shift_register_23__N_3134[2]), 
            .\shift_register[3] (shift_register[3]), .\shift_register_23__N_3134[3] (shift_register_23__N_3134[3]), 
            .\shift_register[4] (shift_register[4]), .\shift_register_23__N_3134[4] (shift_register_23__N_3134[4]), 
            .\shift_register[5] (shift_register[5]), .\shift_register_23__N_3134[5] (shift_register_23__N_3134[5]), 
            .\shift_register[6] (shift_register[6]), .\shift_register_23__N_3134[6] (shift_register_23__N_3134[6]), 
            .\shift_register[7] (shift_register[7]), .\shift_register_23__N_3134[7] (shift_register_23__N_3134[7]), 
            .\shift_register[8] (shift_register[8]), .\shift_register_23__N_3134[8] (shift_register_23__N_3134[8]), 
            .\shift_register[9] (shift_register[9]), .\shift_register_23__N_3134[9] (shift_register_23__N_3134[9]), 
            .\shift_register[10] (shift_register[10]), .\shift_register_23__N_3134[10] (shift_register_23__N_3134[10]), 
            .\shift_register[11] (shift_register[11]), .\shift_register_23__N_3134[11] (shift_register_23__N_3134[11]), 
            .\shift_register[12] (shift_register[12]), .\shift_register_23__N_3134[12] (shift_register_23__N_3134[12]), 
            .\shift_register[13] (shift_register[13]), .\shift_register_23__N_3134[13] (shift_register_23__N_3134[13]), 
            .\shift_register[14] (shift_register[14]), .\shift_register_23__N_3134[14] (shift_register_23__N_3134[14]), 
            .\shift_register[15] (shift_register[15]), .\shift_register_23__N_3134[15] (shift_register_23__N_3134[15]), 
            .\shift_register[16] (shift_register[16]), .\shift_register_23__N_3134[16] (shift_register_23__N_3134[16]), 
            .\shift_register[17] (shift_register[17]), .\shift_register_23__N_3134[17] (shift_register_23__N_3134[17]), 
            .\shift_register[18] (shift_register[18]), .\shift_register_23__N_3134[18] (shift_register_23__N_3134[18]), 
            .\shift_register[19] (shift_register[19]), .\shift_register_23__N_3134[19] (shift_register_23__N_3134[19]), 
            .\shift_register_23__N_3134[20] (shift_register_23__N_3134[20]), 
            .pll_clk_enable_633(pll_clk_enable_633), .GND_net(GND_net), 
            .\shift_register[0] (shift_register[0]), .\rgb_hold[16] (rgb_hold[16]), 
            .rgb_data_c(rgb_data_c)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(679[19] 691[6])
    CCU2D equal_1981_0 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(frame_end_N_2840[16]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24619));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(316[49:96])
    defparam equal_1981_0.INIT0 = 16'hF000;
    defparam equal_1981_0.INIT1 = 16'h5555;
    defparam equal_1981_0.INJECT1_0 = "NO";
    defparam equal_1981_0.INJECT1_1 = "YES";
    LUT4 i1_3_lut_4_lut_adj_352 (.A(ev_state[0]), .B(n26056), .C(ev_rd_hold[41]), 
         .D(ev_bit[41]), .Z(ev_wr_data[41])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[29:51])
    defparam i1_3_lut_4_lut_adj_352.init = 16'hddd0;
    
endmodule
//
// Verilog Description of module spi_mic_stream
//

module spi_mic_stream (sck_N_3271, spi_mic_cs_n_c, spi_mic_miso_c, mic_latest) /* synthesis syn_module_defined=1 */ ;
    input sck_N_3271;
    input spi_mic_cs_n_c;
    output spi_mic_miso_c;
    input [63:0]mic_latest;
    
    wire sck_N_3271 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    wire [6:0]bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(12[11:20])
    
    wire n13;
    wire [6:0]bit_count_6__N_3272;
    
    wire sck_N_3271_enable_101, n26055, n18725, n26076, n18723, n26032;
    wire [95:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    wire [95:0]shift_register_95__N_3174;
    
    wire n18721, n26041, n18719, n18783, n18781, n18779, n18777, 
        n18775, n18773, n18771, n18769, n18767, n18765, n18763, 
        n18761, n18759, n18757, n18755, n18753, n18751, n18749, 
        n18747, n18745, n18743, n18741, n18739, n18737, n18735, 
        n18733, n18731, n18729, n18727, n12;
    
    LUT4 i14316_3_lut_4_lut (.A(bit_count[5]), .B(bit_count[6]), .C(n13), 
         .D(bit_count[0]), .Z(bit_count_6__N_3272[0])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D))) */ ;
    defparam i14316_3_lut_4_lut.init = 16'h8f7f;
    LUT4 i14328_2_lut_2_lut_3_lut (.A(bit_count[5]), .B(bit_count[6]), .C(n13), 
         .Z(sck_N_3271_enable_101)) /* synthesis lut_function=(!(A (B (C)))) */ ;
    defparam i14328_2_lut_2_lut_3_lut.init = 16'h7f7f;
    LUT4 i10269_3_lut_4_lut (.A(bit_count[3]), .B(n26055), .C(n13), .D(bit_count[4]), 
         .Z(n18725)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10269_3_lut_4_lut.init = 16'h7080;
    FD1S3DX bit_count_i0 (.D(bit_count_6__N_3272[0]), .CK(sck_N_3271), .CD(spi_mic_cs_n_c), 
            .Q(bit_count[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i0.GSR = "DISABLED";
    LUT4 i10268_3_lut_4_lut (.A(bit_count[2]), .B(n26076), .C(n13), .D(bit_count[3]), 
         .Z(n18723)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10268_3_lut_4_lut.init = 16'h7080;
    LUT4 i1717_2_lut_rep_98_3_lut_4_lut (.A(bit_count[2]), .B(n26076), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n26032)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1717_2_lut_rep_98_3_lut_4_lut.init = 16'h8000;
    LUT4 i1_3_lut (.A(n13), .B(shift_register[95]), .C(spi_mic_cs_n_c), 
         .Z(spi_mic_miso_c)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[15] 26[68])
    defparam i1_3_lut.init = 16'h0808;
    LUT4 shift_register_95__I_0_19_i82_3_lut (.A(mic_latest[56]), .B(shift_register[80]), 
         .C(n13), .Z(shift_register_95__N_3174[81])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i82_3_lut.init = 16'hcaca;
    LUT4 i1696_2_lut_rep_142 (.A(bit_count[1]), .B(bit_count[0]), .Z(n26076)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1696_2_lut_rep_142.init = 16'h8888;
    LUT4 i1703_2_lut_rep_121_3_lut (.A(bit_count[1]), .B(bit_count[0]), 
         .C(bit_count[2]), .Z(n26055)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1703_2_lut_rep_121_3_lut.init = 16'h8080;
    LUT4 shift_register_95__I_0_19_i83_3_lut (.A(mic_latest[57]), .B(shift_register[81]), 
         .C(n13), .Z(shift_register_95__N_3174[82])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i83_3_lut.init = 16'hcaca;
    LUT4 i10267_3_lut_4_lut (.A(bit_count[1]), .B(bit_count[0]), .C(n13), 
         .D(bit_count[2]), .Z(n18721)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10267_3_lut_4_lut.init = 16'h7080;
    LUT4 i1710_2_lut_rep_107_3_lut_4_lut (.A(bit_count[1]), .B(bit_count[0]), 
         .C(bit_count[3]), .D(bit_count[2]), .Z(n26041)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1710_2_lut_rep_107_3_lut_4_lut.init = 16'h8000;
    LUT4 shift_register_95__I_0_19_i84_3_lut (.A(mic_latest[58]), .B(shift_register[82]), 
         .C(n13), .Z(shift_register_95__N_3174[83])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i84_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i85_3_lut (.A(mic_latest[59]), .B(shift_register[83]), 
         .C(n13), .Z(shift_register_95__N_3174[84])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i85_3_lut.init = 16'hcaca;
    FD1P3DX bit_count_i1 (.D(n18719), .SP(sck_N_3271_enable_101), .CK(sck_N_3271), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i1.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i86_3_lut (.A(mic_latest[60]), .B(shift_register[84]), 
         .C(n13), .Z(shift_register_95__N_3174[85])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i86_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i87_3_lut (.A(mic_latest[61]), .B(shift_register[85]), 
         .C(n13), .Z(shift_register_95__N_3174[86])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i87_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i1 (.D(shift_register_95__N_3174[1]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i1.GSR = "DISABLED";
    FD1P3DX shift_register_i2 (.D(shift_register_95__N_3174[2]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i2.GSR = "DISABLED";
    FD1P3DX shift_register_i3 (.D(shift_register_95__N_3174[3]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i3.GSR = "DISABLED";
    FD1P3DX shift_register_i4 (.D(shift_register_95__N_3174[4]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i4.GSR = "DISABLED";
    FD1P3DX shift_register_i5 (.D(shift_register_95__N_3174[5]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i5.GSR = "DISABLED";
    FD1P3DX shift_register_i6 (.D(shift_register_95__N_3174[6]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i6.GSR = "DISABLED";
    FD1P3DX shift_register_i7 (.D(shift_register_95__N_3174[7]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i7.GSR = "DISABLED";
    FD1P3DX shift_register_i8 (.D(shift_register_95__N_3174[8]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i8.GSR = "DISABLED";
    FD1P3DX shift_register_i9 (.D(shift_register_95__N_3174[9]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i9.GSR = "DISABLED";
    FD1P3DX shift_register_i10 (.D(shift_register_95__N_3174[10]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i10.GSR = "DISABLED";
    FD1P3DX shift_register_i11 (.D(shift_register_95__N_3174[11]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i11.GSR = "DISABLED";
    FD1P3DX shift_register_i12 (.D(shift_register_95__N_3174[12]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i12.GSR = "DISABLED";
    FD1P3DX shift_register_i13 (.D(shift_register_95__N_3174[13]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i13.GSR = "DISABLED";
    FD1P3DX shift_register_i14 (.D(shift_register_95__N_3174[14]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i14.GSR = "DISABLED";
    FD1P3DX shift_register_i15 (.D(shift_register_95__N_3174[15]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i15.GSR = "DISABLED";
    FD1P3DX shift_register_i16 (.D(shift_register_95__N_3174[16]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[16])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i16.GSR = "DISABLED";
    FD1P3DX shift_register_i17 (.D(shift_register_95__N_3174[17]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[17])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i17.GSR = "DISABLED";
    FD1P3DX shift_register_i18 (.D(shift_register_95__N_3174[18]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[18])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i18.GSR = "DISABLED";
    FD1P3DX shift_register_i25 (.D(shift_register_95__N_3174[25]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[25])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i25.GSR = "DISABLED";
    FD1P3DX shift_register_i26 (.D(shift_register_95__N_3174[26]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[26])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i26.GSR = "DISABLED";
    FD1P3DX shift_register_i27 (.D(shift_register_95__N_3174[27]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[27])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i27.GSR = "DISABLED";
    FD1P3DX shift_register_i28 (.D(shift_register_95__N_3174[28]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[28])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i28.GSR = "DISABLED";
    FD1P3DX shift_register_i29 (.D(shift_register_95__N_3174[29]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[29])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i29.GSR = "DISABLED";
    FD1P3DX shift_register_i30 (.D(shift_register_95__N_3174[30]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[30])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i30.GSR = "DISABLED";
    FD1P3DX shift_register_i31 (.D(shift_register_95__N_3174[31]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[31])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i31.GSR = "DISABLED";
    FD1P3DX shift_register_i32 (.D(shift_register_95__N_3174[32]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[32])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i32.GSR = "DISABLED";
    FD1P3DX shift_register_i33 (.D(shift_register_95__N_3174[33]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[33])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i33.GSR = "DISABLED";
    FD1P3DX shift_register_i34 (.D(shift_register_95__N_3174[34]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[34])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i34.GSR = "DISABLED";
    FD1P3DX shift_register_i35 (.D(shift_register_95__N_3174[35]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[35])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i35.GSR = "DISABLED";
    FD1P3DX shift_register_i36 (.D(shift_register_95__N_3174[36]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[36])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i36.GSR = "DISABLED";
    FD1P3DX shift_register_i37 (.D(shift_register_95__N_3174[37]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[37])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i37.GSR = "DISABLED";
    FD1P3DX shift_register_i38 (.D(shift_register_95__N_3174[38]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[38])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i38.GSR = "DISABLED";
    FD1P3DX shift_register_i39 (.D(shift_register_95__N_3174[39]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[39])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i39.GSR = "DISABLED";
    FD1P3DX shift_register_i40 (.D(shift_register_95__N_3174[40]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[40])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i40.GSR = "DISABLED";
    FD1P3DX shift_register_i42 (.D(shift_register_95__N_3174[42]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[42])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i42.GSR = "DISABLED";
    FD1P3DX shift_register_i49 (.D(shift_register_95__N_3174[49]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[49])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i49.GSR = "DISABLED";
    FD1P3DX shift_register_i50 (.D(shift_register_95__N_3174[50]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[50])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i50.GSR = "DISABLED";
    FD1P3DX shift_register_i51 (.D(shift_register_95__N_3174[51]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[51])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i51.GSR = "DISABLED";
    FD1P3DX shift_register_i52 (.D(shift_register_95__N_3174[52]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[52])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i52.GSR = "DISABLED";
    FD1P3DX shift_register_i53 (.D(shift_register_95__N_3174[53]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[53])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i53.GSR = "DISABLED";
    FD1P3DX shift_register_i54 (.D(shift_register_95__N_3174[54]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[54])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i54.GSR = "DISABLED";
    FD1P3DX shift_register_i55 (.D(shift_register_95__N_3174[55]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[55])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i55.GSR = "DISABLED";
    FD1P3DX shift_register_i56 (.D(shift_register_95__N_3174[56]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[56])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i56.GSR = "DISABLED";
    FD1P3DX shift_register_i57 (.D(shift_register_95__N_3174[57]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[57])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i57.GSR = "DISABLED";
    FD1P3DX shift_register_i58 (.D(shift_register_95__N_3174[58]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[58])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i58.GSR = "DISABLED";
    FD1P3DX shift_register_i59 (.D(shift_register_95__N_3174[59]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[59])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i59.GSR = "DISABLED";
    FD1P3DX shift_register_i60 (.D(shift_register_95__N_3174[60]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[60])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i60.GSR = "DISABLED";
    FD1P3DX shift_register_i61 (.D(shift_register_95__N_3174[61]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[61])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i61.GSR = "DISABLED";
    FD1P3DX shift_register_i62 (.D(shift_register_95__N_3174[62]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[62])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i62.GSR = "DISABLED";
    FD1P3DX shift_register_i63 (.D(shift_register_95__N_3174[63]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[63])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i63.GSR = "DISABLED";
    FD1P3DX shift_register_i64 (.D(shift_register_95__N_3174[64]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[64])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i64.GSR = "DISABLED";
    FD1P3DX shift_register_i65 (.D(shift_register_95__N_3174[65]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[65])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i65.GSR = "DISABLED";
    FD1P3DX shift_register_i73 (.D(shift_register_95__N_3174[73]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[73])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i73.GSR = "DISABLED";
    FD1P3DX shift_register_i74 (.D(shift_register_95__N_3174[74]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[74])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i74.GSR = "DISABLED";
    FD1P3DX shift_register_i75 (.D(shift_register_95__N_3174[75]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[75])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i75.GSR = "DISABLED";
    FD1P3DX shift_register_i76 (.D(shift_register_95__N_3174[76]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[76])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i76.GSR = "DISABLED";
    FD1P3DX shift_register_i77 (.D(shift_register_95__N_3174[77]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[77])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i77.GSR = "DISABLED";
    FD1P3DX shift_register_i78 (.D(shift_register_95__N_3174[78]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[78])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i78.GSR = "DISABLED";
    FD1P3DX shift_register_i79 (.D(shift_register_95__N_3174[79]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[79])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i79.GSR = "DISABLED";
    FD1P3DX shift_register_i80 (.D(shift_register_95__N_3174[80]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[80])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i80.GSR = "DISABLED";
    FD1P3DX shift_register_i81 (.D(shift_register_95__N_3174[81]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[81])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i81.GSR = "DISABLED";
    FD1P3DX shift_register_i82 (.D(shift_register_95__N_3174[82]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[82])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i82.GSR = "DISABLED";
    FD1P3DX shift_register_i83 (.D(shift_register_95__N_3174[83]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[83])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i83.GSR = "DISABLED";
    FD1P3DX shift_register_i84 (.D(shift_register_95__N_3174[84]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[84])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i84.GSR = "DISABLED";
    FD1P3DX shift_register_i85 (.D(shift_register_95__N_3174[85]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[85])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i85.GSR = "DISABLED";
    FD1P3DX shift_register_i86 (.D(shift_register_95__N_3174[86]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[86])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i86.GSR = "DISABLED";
    FD1P3DX shift_register_i87 (.D(shift_register_95__N_3174[87]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[87])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i87.GSR = "DISABLED";
    FD1P3DX shift_register_i88 (.D(shift_register_95__N_3174[88]), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[88])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i88.GSR = "DISABLED";
    LUT4 i10262_2_lut (.A(mic_latest[0]), .B(n13), .Z(shift_register_95__N_3174[1])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10262_2_lut.init = 16'h2222;
    LUT4 shift_register_95__I_0_19_i3_3_lut (.A(mic_latest[1]), .B(shift_register[1]), 
         .C(n13), .Z(shift_register_95__N_3174[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i3_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i4_3_lut (.A(mic_latest[2]), .B(shift_register[2]), 
         .C(n13), .Z(shift_register_95__N_3174[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i4_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i5_3_lut (.A(mic_latest[3]), .B(shift_register[3]), 
         .C(n13), .Z(shift_register_95__N_3174[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i5_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i6_3_lut (.A(mic_latest[4]), .B(shift_register[4]), 
         .C(n13), .Z(shift_register_95__N_3174[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i6_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i7_3_lut (.A(mic_latest[5]), .B(shift_register[5]), 
         .C(n13), .Z(shift_register_95__N_3174[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i7_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i8_3_lut (.A(mic_latest[6]), .B(shift_register[6]), 
         .C(n13), .Z(shift_register_95__N_3174[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i8_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i9_3_lut (.A(mic_latest[7]), .B(shift_register[7]), 
         .C(n13), .Z(shift_register_95__N_3174[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i9_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i10_3_lut (.A(mic_latest[8]), .B(shift_register[8]), 
         .C(n13), .Z(shift_register_95__N_3174[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i10_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i11_3_lut (.A(mic_latest[9]), .B(shift_register[9]), 
         .C(n13), .Z(shift_register_95__N_3174[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i11_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i12_3_lut (.A(mic_latest[10]), .B(shift_register[10]), 
         .C(n13), .Z(shift_register_95__N_3174[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i12_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i13_3_lut (.A(mic_latest[11]), .B(shift_register[11]), 
         .C(n13), .Z(shift_register_95__N_3174[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i13_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i14_3_lut (.A(mic_latest[12]), .B(shift_register[12]), 
         .C(n13), .Z(shift_register_95__N_3174[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i14_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i15_3_lut (.A(mic_latest[13]), .B(shift_register[13]), 
         .C(n13), .Z(shift_register_95__N_3174[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i15_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i16_3_lut (.A(mic_latest[14]), .B(shift_register[14]), 
         .C(n13), .Z(shift_register_95__N_3174[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i16_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i17_3_lut (.A(mic_latest[15]), .B(shift_register[15]), 
         .C(n13), .Z(shift_register_95__N_3174[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i17_3_lut.init = 16'hcaca;
    LUT4 i10263_2_lut (.A(shift_register[16]), .B(n13), .Z(shift_register_95__N_3174[17])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10263_2_lut.init = 16'hbbbb;
    LUT4 i10264_2_lut (.A(shift_register[17]), .B(n13), .Z(shift_register_95__N_3174[18])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10264_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i26_3_lut (.A(mic_latest[16]), .B(shift_register[24]), 
         .C(n13), .Z(shift_register_95__N_3174[25])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i26_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i27_3_lut (.A(mic_latest[17]), .B(shift_register[25]), 
         .C(n13), .Z(shift_register_95__N_3174[26])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i27_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i28_3_lut (.A(mic_latest[18]), .B(shift_register[26]), 
         .C(n13), .Z(shift_register_95__N_3174[27])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i28_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i29_3_lut (.A(mic_latest[19]), .B(shift_register[27]), 
         .C(n13), .Z(shift_register_95__N_3174[28])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i29_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i30_3_lut (.A(mic_latest[20]), .B(shift_register[28]), 
         .C(n13), .Z(shift_register_95__N_3174[29])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i30_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i31_3_lut (.A(mic_latest[21]), .B(shift_register[29]), 
         .C(n13), .Z(shift_register_95__N_3174[30])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i31_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i32_3_lut (.A(mic_latest[22]), .B(shift_register[30]), 
         .C(n13), .Z(shift_register_95__N_3174[31])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i32_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i33_3_lut (.A(mic_latest[23]), .B(shift_register[31]), 
         .C(n13), .Z(shift_register_95__N_3174[32])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i33_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i34_3_lut (.A(mic_latest[24]), .B(shift_register[32]), 
         .C(n13), .Z(shift_register_95__N_3174[33])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i34_3_lut.init = 16'hcaca;
    LUT4 i10251_3_lut (.A(bit_count[1]), .B(n13), .C(bit_count[0]), .Z(n18719)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10251_3_lut.init = 16'h4848;
    LUT4 shift_register_95__I_0_19_i35_3_lut (.A(mic_latest[25]), .B(shift_register[33]), 
         .C(n13), .Z(shift_register_95__N_3174[34])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i35_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i36_3_lut (.A(mic_latest[26]), .B(shift_register[34]), 
         .C(n13), .Z(shift_register_95__N_3174[35])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i36_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i37_3_lut (.A(mic_latest[27]), .B(shift_register[35]), 
         .C(n13), .Z(shift_register_95__N_3174[36])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i37_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i38_3_lut (.A(mic_latest[28]), .B(shift_register[36]), 
         .C(n13), .Z(shift_register_95__N_3174[37])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i38_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i39_3_lut (.A(mic_latest[29]), .B(shift_register[37]), 
         .C(n13), .Z(shift_register_95__N_3174[38])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i39_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i95 (.D(n18783), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[95])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i95.GSR = "DISABLED";
    FD1P3DX shift_register_i94 (.D(n18781), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[94])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i94.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i40_3_lut (.A(mic_latest[30]), .B(shift_register[38]), 
         .C(n13), .Z(shift_register_95__N_3174[39])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i40_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i93 (.D(n18779), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[93])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i93.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i41_3_lut (.A(mic_latest[31]), .B(shift_register[39]), 
         .C(n13), .Z(shift_register_95__N_3174[40])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i41_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i92 (.D(n18777), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[92])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i92.GSR = "DISABLED";
    LUT4 i10298_2_lut (.A(shift_register[94]), .B(n13), .Z(n18783)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10298_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i91 (.D(n18775), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[91])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i91.GSR = "DISABLED";
    LUT4 i10297_2_lut (.A(shift_register[93]), .B(n13), .Z(n18781)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10297_2_lut.init = 16'h8888;
    LUT4 i10265_2_lut (.A(shift_register[41]), .B(n13), .Z(shift_register_95__N_3174[42])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10265_2_lut.init = 16'hbbbb;
    FD1P3DX shift_register_i90 (.D(n18773), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[90])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i90.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i50_3_lut (.A(mic_latest[32]), .B(shift_register[48]), 
         .C(n13), .Z(shift_register_95__N_3174[49])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i50_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i89 (.D(n18771), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[89])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i89.GSR = "DISABLED";
    LUT4 i10296_2_lut (.A(shift_register[92]), .B(n13), .Z(n18779)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10296_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i72 (.D(n18769), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[72])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i72.GSR = "DISABLED";
    FD1P3DX shift_register_i71 (.D(n18767), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[71])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i71.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i51_3_lut (.A(mic_latest[33]), .B(shift_register[49]), 
         .C(n13), .Z(shift_register_95__N_3174[50])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i51_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i70 (.D(n18765), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[70])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i70.GSR = "DISABLED";
    LUT4 i10295_2_lut (.A(shift_register[91]), .B(n13), .Z(n18777)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10295_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i69 (.D(n18763), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[69])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i69.GSR = "DISABLED";
    LUT4 i10294_2_lut (.A(shift_register[90]), .B(n13), .Z(n18775)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10294_2_lut.init = 16'h8888;
    LUT4 i10293_2_lut (.A(shift_register[89]), .B(n13), .Z(n18773)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10293_2_lut.init = 16'h8888;
    LUT4 i10292_2_lut (.A(shift_register[88]), .B(n13), .Z(n18771)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10292_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i68 (.D(n18761), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[68])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i68.GSR = "DISABLED";
    LUT4 i10291_2_lut (.A(shift_register[71]), .B(n13), .Z(n18769)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10291_2_lut.init = 16'h8888;
    LUT4 i10290_2_lut (.A(shift_register[70]), .B(n13), .Z(n18767)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10290_2_lut.init = 16'h8888;
    LUT4 i10289_2_lut (.A(shift_register[69]), .B(n13), .Z(n18765)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10289_2_lut.init = 16'h8888;
    LUT4 i10288_2_lut (.A(shift_register[68]), .B(n13), .Z(n18763)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10288_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i67 (.D(n18759), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[67])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i67.GSR = "DISABLED";
    LUT4 i10287_2_lut (.A(shift_register[67]), .B(n13), .Z(n18761)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10287_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i66 (.D(n18757), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[66])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i66.GSR = "DISABLED";
    LUT4 i10286_2_lut (.A(shift_register[66]), .B(n13), .Z(n18759)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10286_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i48 (.D(n18755), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[48])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i48.GSR = "DISABLED";
    LUT4 i10285_2_lut (.A(shift_register[65]), .B(n13), .Z(n18757)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10285_2_lut.init = 16'h8888;
    LUT4 i10284_2_lut (.A(shift_register[47]), .B(n13), .Z(n18755)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10284_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i47 (.D(n18753), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[47])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i47.GSR = "DISABLED";
    LUT4 i10283_2_lut (.A(shift_register[46]), .B(n13), .Z(n18753)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10283_2_lut.init = 16'h8888;
    LUT4 i10282_2_lut (.A(shift_register[45]), .B(n13), .Z(n18751)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10282_2_lut.init = 16'h8888;
    LUT4 i10281_2_lut (.A(shift_register[44]), .B(n13), .Z(n18749)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10281_2_lut.init = 16'h8888;
    LUT4 i10280_2_lut (.A(shift_register[43]), .B(n13), .Z(n18747)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10280_2_lut.init = 16'h8888;
    LUT4 i10279_2_lut (.A(shift_register[42]), .B(n13), .Z(n18745)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10279_2_lut.init = 16'h8888;
    LUT4 i10278_2_lut (.A(shift_register[40]), .B(n13), .Z(n18743)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10278_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i46 (.D(n18751), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[46])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i46.GSR = "DISABLED";
    LUT4 i10277_2_lut (.A(shift_register[23]), .B(n13), .Z(n18741)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10277_2_lut.init = 16'h8888;
    LUT4 i10276_2_lut (.A(shift_register[22]), .B(n13), .Z(n18739)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10276_2_lut.init = 16'h8888;
    LUT4 i10275_2_lut (.A(shift_register[21]), .B(n13), .Z(n18737)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10275_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i45 (.D(n18749), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[45])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i45.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i52_3_lut (.A(mic_latest[34]), .B(shift_register[50]), 
         .C(n13), .Z(shift_register_95__N_3174[51])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i52_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i44 (.D(n18747), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[44])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i44.GSR = "DISABLED";
    FD1P3DX shift_register_i43 (.D(n18745), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[43])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i43.GSR = "DISABLED";
    FD1P3DX shift_register_i41 (.D(n18743), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[41])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i41.GSR = "DISABLED";
    LUT4 i10274_2_lut (.A(shift_register[20]), .B(n13), .Z(n18735)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10274_2_lut.init = 16'h8888;
    LUT4 i10273_2_lut (.A(shift_register[19]), .B(n13), .Z(n18733)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10273_2_lut.init = 16'h8888;
    LUT4 i10272_2_lut (.A(shift_register[18]), .B(n13), .Z(n18731)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10272_2_lut.init = 16'h8888;
    LUT4 i10271_4_lut (.A(bit_count[6]), .B(n13), .C(bit_count[5]), .D(n26032), 
         .Z(n18729)) /* synthesis lut_function=(!(A ((C (D))+!B)+!A !(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10271_4_lut.init = 16'h4888;
    FD1P3DX shift_register_i24 (.D(n18741), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[24])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i24.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i53_3_lut (.A(mic_latest[35]), .B(shift_register[51]), 
         .C(n13), .Z(shift_register_95__N_3174[52])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i53_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i23 (.D(n18739), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[23])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i23.GSR = "DISABLED";
    FD1P3DX shift_register_i22 (.D(n18737), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[22])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i22.GSR = "DISABLED";
    FD1P3DX shift_register_i21 (.D(n18735), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[21])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i21.GSR = "DISABLED";
    FD1P3DX shift_register_i20 (.D(n18733), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[20])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i20.GSR = "DISABLED";
    FD1P3DX shift_register_i19 (.D(n18731), .SP(sck_N_3271_enable_101), 
            .CK(sck_N_3271), .CD(spi_mic_cs_n_c), .Q(shift_register[19])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i19.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i54_3_lut (.A(mic_latest[36]), .B(shift_register[52]), 
         .C(n13), .Z(shift_register_95__N_3174[53])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i54_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i55_3_lut (.A(mic_latest[37]), .B(shift_register[53]), 
         .C(n13), .Z(shift_register_95__N_3174[54])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i55_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i56_3_lut (.A(mic_latest[38]), .B(shift_register[54]), 
         .C(n13), .Z(shift_register_95__N_3174[55])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i56_3_lut.init = 16'hcaca;
    FD1P3DX bit_count_i6 (.D(n18729), .SP(sck_N_3271_enable_101), .CK(sck_N_3271), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i6.GSR = "DISABLED";
    FD1P3DX bit_count_i5 (.D(n18727), .SP(sck_N_3271_enable_101), .CK(sck_N_3271), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i5.GSR = "DISABLED";
    FD1P3DX bit_count_i4 (.D(n18725), .SP(sck_N_3271_enable_101), .CK(sck_N_3271), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i4.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i57_3_lut (.A(mic_latest[39]), .B(shift_register[55]), 
         .C(n13), .Z(shift_register_95__N_3174[56])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i57_3_lut.init = 16'hcaca;
    FD1P3DX bit_count_i3 (.D(n18723), .SP(sck_N_3271_enable_101), .CK(sck_N_3271), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i3.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i58_3_lut (.A(mic_latest[40]), .B(shift_register[56]), 
         .C(n13), .Z(shift_register_95__N_3174[57])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i58_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i59_3_lut (.A(mic_latest[41]), .B(shift_register[57]), 
         .C(n13), .Z(shift_register_95__N_3174[58])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i59_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i60_3_lut (.A(mic_latest[42]), .B(shift_register[58]), 
         .C(n13), .Z(shift_register_95__N_3174[59])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i60_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i61_3_lut (.A(mic_latest[43]), .B(shift_register[59]), 
         .C(n13), .Z(shift_register_95__N_3174[60])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i61_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i62_3_lut (.A(mic_latest[44]), .B(shift_register[60]), 
         .C(n13), .Z(shift_register_95__N_3174[61])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i62_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i63_3_lut (.A(mic_latest[45]), .B(shift_register[61]), 
         .C(n13), .Z(shift_register_95__N_3174[62])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i63_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i64_3_lut (.A(mic_latest[46]), .B(shift_register[62]), 
         .C(n13), .Z(shift_register_95__N_3174[63])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i64_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i65_3_lut (.A(mic_latest[47]), .B(shift_register[63]), 
         .C(n13), .Z(shift_register_95__N_3174[64])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i65_3_lut.init = 16'hcaca;
    LUT4 i10266_2_lut (.A(shift_register[64]), .B(n13), .Z(shift_register_95__N_3174[65])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10266_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i74_3_lut (.A(mic_latest[48]), .B(shift_register[72]), 
         .C(n13), .Z(shift_register_95__N_3174[73])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i74_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i75_3_lut (.A(mic_latest[49]), .B(shift_register[73]), 
         .C(n13), .Z(shift_register_95__N_3174[74])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i75_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i76_3_lut (.A(mic_latest[50]), .B(shift_register[74]), 
         .C(n13), .Z(shift_register_95__N_3174[75])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i76_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i77_3_lut (.A(mic_latest[51]), .B(shift_register[75]), 
         .C(n13), .Z(shift_register_95__N_3174[76])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i77_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i78_3_lut (.A(mic_latest[52]), .B(shift_register[76]), 
         .C(n13), .Z(shift_register_95__N_3174[77])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i78_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i80_3_lut (.A(mic_latest[54]), .B(shift_register[78]), 
         .C(n13), .Z(shift_register_95__N_3174[79])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i80_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i81_3_lut (.A(mic_latest[55]), .B(shift_register[79]), 
         .C(n13), .Z(shift_register_95__N_3174[80])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i81_3_lut.init = 16'hcaca;
    LUT4 i6_4_lut (.A(bit_count[2]), .B(n12), .C(bit_count[6]), .D(bit_count[1]), 
         .Z(n13)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(26[16:33])
    defparam i6_4_lut.init = 16'hfffe;
    LUT4 shift_register_95__I_0_19_i79_3_lut (.A(mic_latest[53]), .B(shift_register[77]), 
         .C(n13), .Z(shift_register_95__N_3174[78])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i79_3_lut.init = 16'hcaca;
    FD1P3DX bit_count_i2 (.D(n18721), .SP(sck_N_3271_enable_101), .CK(sck_N_3271), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=692, LSE_RLINE=695 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i2.GSR = "DISABLED";
    LUT4 i5_4_lut (.A(bit_count[0]), .B(bit_count[5]), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n12)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(26[16:33])
    defparam i5_4_lut.init = 16'hfffe;
    LUT4 shift_register_95__I_0_19_i88_3_lut (.A(mic_latest[62]), .B(shift_register[86]), 
         .C(n13), .Z(shift_register_95__N_3174[87])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i88_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i89_3_lut (.A(mic_latest[63]), .B(shift_register[87]), 
         .C(n13), .Z(shift_register_95__N_3174[88])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i89_3_lut.init = 16'hcaca;
    LUT4 i10270_3_lut_4_lut (.A(bit_count[4]), .B(n26041), .C(n13), .D(bit_count[5]), 
         .Z(n18727)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10270_3_lut_4_lut.init = 16'h7080;
    
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
            n26101, ev_wr_data, ev_rd_data) /* synthesis syn_module_defined=1 */ ;
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
    input n26101;
    input [83:0]ev_wr_data;
    output [83:0]ev_rd_data;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[10:17])
    
    PDPW8KC mem4 (.DI0(ev_wr_data[0]), .DI1(ev_wr_data[1]), .DI2(ev_wr_data[2]), 
            .DI3(ev_wr_data[3]), .DI4(ev_wr_data[4]), .DI5(ev_wr_data[5]), 
            .DI6(ev_wr_data[6]), .DI7(ev_wr_data[7]), .DI8(ev_wr_data[8]), 
            .DI9(ev_wr_data[9]), .DI10(ev_wr_data[10]), .DI11(ev_wr_data[11]), 
            .DI12(GND_net), .DI13(GND_net), .DI14(GND_net), .DI15(GND_net), 
            .DI16(GND_net), .DI17(GND_net), .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), 
            .ADW2(\ev_wr_addr[2] ), .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), 
            .ADW5(\ev_wr_addr[5] ), .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), 
            .ADW8(n26101), .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), 
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
    PDPW8KC mem1 (.DI0(ev_wr_data[48]), .DI1(ev_wr_data[49]), .DI2(ev_wr_data[50]), 
            .DI3(ev_wr_data[51]), .DI4(ev_wr_data[52]), .DI5(ev_wr_data[53]), 
            .DI6(ev_wr_data[54]), .DI7(ev_wr_data[55]), .DI8(ev_wr_data[56]), 
            .DI9(ev_wr_data[57]), .DI10(ev_wr_data[58]), .DI11(ev_wr_data[59]), 
            .DI12(ev_wr_data[60]), .DI13(ev_wr_data[61]), .DI14(ev_wr_data[62]), 
            .DI15(ev_wr_data[63]), .DI16(ev_wr_data[64]), .DI17(ev_wr_data[65]), 
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n26101), 
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
    PDPW8KC mem3 (.DI0(ev_wr_data[12]), .DI1(ev_wr_data[13]), .DI2(ev_wr_data[14]), 
            .DI3(ev_wr_data[15]), .DI4(ev_wr_data[16]), .DI5(ev_wr_data[17]), 
            .DI6(ev_wr_data[18]), .DI7(ev_wr_data[19]), .DI8(ev_wr_data[20]), 
            .DI9(ev_wr_data[21]), .DI10(ev_wr_data[22]), .DI11(ev_wr_data[23]), 
            .DI12(ev_wr_data[24]), .DI13(ev_wr_data[25]), .DI14(ev_wr_data[26]), 
            .DI15(ev_wr_data[27]), .DI16(ev_wr_data[28]), .DI17(ev_wr_data[29]), 
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n26101), 
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
    PDPW8KC mem2 (.DI0(ev_wr_data[30]), .DI1(ev_wr_data[31]), .DI2(ev_wr_data[32]), 
            .DI3(ev_wr_data[33]), .DI4(ev_wr_data[34]), .DI5(ev_wr_data[35]), 
            .DI6(ev_wr_data[36]), .DI7(ev_wr_data[37]), .DI8(ev_wr_data[38]), 
            .DI9(ev_wr_data[39]), .DI10(ev_wr_data[40]), .DI11(ev_wr_data[41]), 
            .DI12(ev_wr_data[42]), .DI13(ev_wr_data[43]), .DI14(ev_wr_data[44]), 
            .DI15(ev_wr_data[45]), .DI16(ev_wr_data[46]), .DI17(ev_wr_data[47]), 
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n26101), 
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
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n26101), 
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
// Verilog Description of module umh_channel_ram18
//

module umh_channel_ram18 (n12887, spi1_sck_c, spi_channel_index, staging_q, 
            pll_clk, rd_data_15__N_2873, n12881, n12879, n12889, n12891, 
            n13, spi_write, VCC_net, GND_net, staging_rd_addr_6__N_914, 
            spi1_mosi_c_0, \spi_rx_shift[0] , \spi_rx_shift[1] , \spi_rx_shift[2] , 
            \spi_rx_shift[3] , \spi_rx_shift[4] , \spi_rx_shift[5] , \spi_rx_shift[6] , 
            spi_phase_pending, n12896, n12898, n12900, n12902, n12904, 
            n12906, n12908, n12910, n12912, n12914, n12916, n12918, 
            n12920, n12922, n12924, n12926, n12883, n12885) /* synthesis syn_module_defined=1 */ ;
    output n12887;
    input spi1_sck_c;
    input [6:0]spi_channel_index;
    output [15:0]staging_q;
    input pll_clk;
    input [15:0]rd_data_15__N_2873;
    output n12881;
    output n12879;
    output n12889;
    output n12891;
    output n13;
    input spi_write;
    input VCC_net;
    input GND_net;
    input [6:0]staging_rd_addr_6__N_914;
    input spi1_mosi_c_0;
    input \spi_rx_shift[0] ;
    input \spi_rx_shift[1] ;
    input \spi_rx_shift[2] ;
    input \spi_rx_shift[3] ;
    input \spi_rx_shift[4] ;
    input \spi_rx_shift[5] ;
    input \spi_rx_shift[6] ;
    input [7:0]spi_phase_pending;
    output n12896;
    output n12898;
    output n12900;
    output n12902;
    output n12904;
    output n12906;
    output n12908;
    output n12910;
    output n12912;
    output n12914;
    output n12916;
    output n12918;
    output n12920;
    output n12922;
    output n12924;
    output n12926;
    output n12883;
    output n12885;
    
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[10:17])
    
    wire n14, n10;
    
    FD1S3AX mem_1524 (.D(spi_channel_index[4]), .CK(spi1_sck_c), .Q(n12887));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1524.GSR = "DISABLED";
    FD1S3AX rd_data_i0 (.D(rd_data_15__N_2873[0]), .CK(pll_clk), .Q(staging_q[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i0.GSR = "DISABLED";
    FD1S3AX mem_1518 (.D(spi_channel_index[1]), .CK(spi1_sck_c), .Q(n12881));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1518.GSR = "DISABLED";
    FD1S3AX mem_1516 (.D(spi_channel_index[0]), .CK(spi1_sck_c), .Q(n12879));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1516.GSR = "DISABLED";
    FD1S3AX mem_1526 (.D(spi_channel_index[5]), .CK(spi1_sck_c), .Q(n12889));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1526.GSR = "DISABLED";
    FD1S3AX mem_1528 (.D(spi_channel_index[6]), .CK(spi1_sck_c), .Q(n12891));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1528.GSR = "DISABLED";
    LUT4 i7_4_lut (.A(staging_q[3]), .B(n14), .C(n10), .D(staging_q[7]), 
         .Z(n13)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i7_4_lut.init = 16'hfffe;
    LUT4 i6_4_lut (.A(staging_q[4]), .B(staging_q[0]), .C(staging_q[1]), 
         .D(staging_q[5]), .Z(n14)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i6_4_lut.init = 16'hfffe;
    LUT4 i2_2_lut (.A(staging_q[2]), .B(staging_q[6]), .Z(n10)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i2_2_lut.init = 16'heeee;
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
            .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), .ADR4(staging_rd_addr_6__N_914[0]), 
            .ADR5(staging_rd_addr_6__N_914[1]), .ADR6(staging_rd_addr_6__N_914[2]), 
            .ADR7(staging_rd_addr_6__N_914[3]), .ADR8(staging_rd_addr_6__N_914[4]), 
            .ADR9(staging_rd_addr_6__N_914[5]), .ADR10(staging_rd_addr_6__N_914[6]), 
            .ADR11(GND_net), .ADR12(GND_net), .CER(VCC_net), .OCER(VCC_net), 
            .CLKR(pll_clk), .CSR0(GND_net), .CSR1(GND_net), .CSR2(GND_net), 
            .RST(GND_net), .DO0(n12914), .DO1(n12916), .DO2(n12918), 
            .DO3(n12920), .DO4(n12922), .DO5(n12924), .DO6(n12926), 
            .DO9(n12896), .DO10(n12898), .DO11(n12900), .DO12(n12902), 
            .DO13(n12904), .DO14(n12906), .DO15(n12908), .DO16(n12910), 
            .DO17(n12912));
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
    FD1S3AX mem_1520 (.D(spi_channel_index[2]), .CK(spi1_sck_c), .Q(n12883));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1520.GSR = "DISABLED";
    FD1S3AX mem_1522 (.D(spi_channel_index[3]), .CK(spi1_sck_c), .Q(n12885));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1522.GSR = "DISABLED";
    FD1S3AX rd_data_i1 (.D(rd_data_15__N_2873[1]), .CK(pll_clk), .Q(staging_q[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i1.GSR = "DISABLED";
    FD1S3AX rd_data_i2 (.D(rd_data_15__N_2873[2]), .CK(pll_clk), .Q(staging_q[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i2.GSR = "DISABLED";
    FD1S3AX rd_data_i3 (.D(rd_data_15__N_2873[3]), .CK(pll_clk), .Q(staging_q[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i3.GSR = "DISABLED";
    FD1S3AX rd_data_i4 (.D(rd_data_15__N_2873[4]), .CK(pll_clk), .Q(staging_q[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i4.GSR = "DISABLED";
    FD1S3AX rd_data_i5 (.D(rd_data_15__N_2873[5]), .CK(pll_clk), .Q(staging_q[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i5.GSR = "DISABLED";
    FD1S3AX rd_data_i6 (.D(rd_data_15__N_2873[6]), .CK(pll_clk), .Q(staging_q[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i6.GSR = "DISABLED";
    FD1S3AX rd_data_i7 (.D(rd_data_15__N_2873[7]), .CK(pll_clk), .Q(staging_q[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i7.GSR = "DISABLED";
    FD1S3AX rd_data_i8 (.D(rd_data_15__N_2873[8]), .CK(pll_clk), .Q(staging_q[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i8.GSR = "DISABLED";
    FD1S3AX rd_data_i9 (.D(rd_data_15__N_2873[9]), .CK(pll_clk), .Q(staging_q[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i9.GSR = "DISABLED";
    FD1S3AX rd_data_i10 (.D(rd_data_15__N_2873[10]), .CK(pll_clk), .Q(staging_q[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i10.GSR = "DISABLED";
    FD1S3AX rd_data_i11 (.D(rd_data_15__N_2873[11]), .CK(pll_clk), .Q(staging_q[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i11.GSR = "DISABLED";
    FD1S3AX rd_data_i12 (.D(rd_data_15__N_2873[12]), .CK(pll_clk), .Q(staging_q[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i12.GSR = "DISABLED";
    FD1S3AX rd_data_i13 (.D(rd_data_15__N_2873[13]), .CK(pll_clk), .Q(staging_q[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i13.GSR = "DISABLED";
    FD1S3AX rd_data_i14 (.D(rd_data_15__N_2873[14]), .CK(pll_clk), .Q(staging_q[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i14.GSR = "DISABLED";
    FD1S3AX rd_data_i15 (.D(rd_data_15__N_2873[15]), .CK(pll_clk), .Q(staging_q[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=219, LSE_RLINE=222 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i15.GSR = "DISABLED";
    
endmodule
//
// Verilog Description of module ws2812_stream
//

module ws2812_stream (n15, state, pll_clk, n17658, \rgb_hold[5] , 
            \rgb_hold[6] , \rgb_hold[7] , ws2812_enable, \shift_register[1] , 
            \shift_register_23__N_3134[1] , \shift_register[2] , \shift_register_23__N_3134[2] , 
            \shift_register[3] , \shift_register_23__N_3134[3] , \shift_register[4] , 
            \shift_register_23__N_3134[4] , \shift_register[5] , \shift_register_23__N_3134[5] , 
            \shift_register[6] , \shift_register_23__N_3134[6] , \shift_register[7] , 
            \shift_register_23__N_3134[7] , \shift_register[8] , \shift_register_23__N_3134[8] , 
            \shift_register[9] , \shift_register_23__N_3134[9] , \shift_register[10] , 
            \shift_register_23__N_3134[10] , \shift_register[11] , \shift_register_23__N_3134[11] , 
            \shift_register[12] , \shift_register_23__N_3134[12] , \shift_register[13] , 
            \shift_register_23__N_3134[13] , \shift_register[14] , \shift_register_23__N_3134[14] , 
            \shift_register[15] , \shift_register_23__N_3134[15] , \shift_register[16] , 
            \shift_register_23__N_3134[16] , \shift_register[17] , \shift_register_23__N_3134[17] , 
            \shift_register[18] , \shift_register_23__N_3134[18] , \shift_register[19] , 
            \shift_register_23__N_3134[19] , \shift_register_23__N_3134[20] , 
            pll_clk_enable_633, GND_net, \shift_register[0] , \rgb_hold[16] , 
            rgb_data_c) /* synthesis syn_module_defined=1 */ ;
    output n15;
    output [1:0]state;
    input pll_clk;
    input n17658;
    input \rgb_hold[5] ;
    input \rgb_hold[6] ;
    input \rgb_hold[7] ;
    input ws2812_enable;
    output \shift_register[1] ;
    input \shift_register_23__N_3134[1] ;
    output \shift_register[2] ;
    input \shift_register_23__N_3134[2] ;
    output \shift_register[3] ;
    input \shift_register_23__N_3134[3] ;
    output \shift_register[4] ;
    input \shift_register_23__N_3134[4] ;
    output \shift_register[5] ;
    input \shift_register_23__N_3134[5] ;
    output \shift_register[6] ;
    input \shift_register_23__N_3134[6] ;
    output \shift_register[7] ;
    input \shift_register_23__N_3134[7] ;
    output \shift_register[8] ;
    input \shift_register_23__N_3134[8] ;
    output \shift_register[9] ;
    input \shift_register_23__N_3134[9] ;
    output \shift_register[10] ;
    input \shift_register_23__N_3134[10] ;
    output \shift_register[11] ;
    input \shift_register_23__N_3134[11] ;
    output \shift_register[12] ;
    input \shift_register_23__N_3134[12] ;
    output \shift_register[13] ;
    input \shift_register_23__N_3134[13] ;
    output \shift_register[14] ;
    input \shift_register_23__N_3134[14] ;
    output \shift_register[15] ;
    input \shift_register_23__N_3134[15] ;
    output \shift_register[16] ;
    input \shift_register_23__N_3134[16] ;
    output \shift_register[17] ;
    input \shift_register_23__N_3134[17] ;
    output \shift_register[18] ;
    input \shift_register_23__N_3134[18] ;
    output \shift_register[19] ;
    input \shift_register_23__N_3134[19] ;
    input \shift_register_23__N_3134[20] ;
    input pll_clk_enable_633;
    input GND_net;
    output \shift_register[0] ;
    input \rgb_hold[16] ;
    output rgb_data_c;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[10:17])
    
    wire data_out_N_3171, data_out_N_3165;
    wire [1:0]state_1__N_3109;
    wire [12:0]reset_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[16:27])
    
    wire pll_clk_enable_317, n25424;
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[16:30])
    
    wire n17;
    wire [23:0]shift_register_23__N_3134;
    
    wire n25394, n25396, n26;
    wire [4:0]bit_number;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(28[15:25])
    
    wire pll_clk_enable_778, n18602;
    wire [4:0]n136;
    
    wire n26052, pll_clk_enable_774, n25324, n26039, n26029, n17199;
    wire [7:0]bit_cell_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(27[15:29])
    
    wire n4, n26031, n25423;
    wire [12:0]n22;
    
    wire n25422, n25421, n25420, n25419, n25418, n25417, n25416, 
        n25415, n25414, n25413, n25412, pll_clk_enable_320, n1;
    wire [7:0]n70;
    
    wire n24803, n24802, n24801, n24800, n24799, n25393, n24798, 
        n21944, n25348, n21942, n24797, n24796, n24795, n6, n24794, 
        n25563, n27_adj_3385, pll_clk_enable_779, n14, n12603, n25501;
    
    LUT4 i10164_2_lut (.A(data_out_N_3171), .B(n15), .Z(data_out_N_3165)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i10164_2_lut.init = 16'h2222;
    FD1S3IX state__i0 (.D(state_1__N_3109[0]), .CK(pll_clk), .CD(n17658), 
            .Q(state[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam state__i0.GSR = "DISABLED";
    FD1P3AX reset_count__i0 (.D(n25424), .SP(pll_clk_enable_317), .CK(pll_clk), 
            .Q(reset_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i0.GSR = "DISABLED";
    LUT4 i10114_4_lut (.A(\rgb_hold[5] ), .B(shift_register[20]), .C(state[1]), 
         .D(n17), .Z(shift_register_23__N_3134[21])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    defparam i10114_4_lut.init = 16'h0aca;
    LUT4 i36_2_lut (.A(state[0]), .B(n15), .Z(n17)) /* synthesis lut_function=((B)+!A) */ ;
    defparam i36_2_lut.init = 16'hdddd;
    LUT4 i10124_4_lut (.A(\rgb_hold[6] ), .B(shift_register[21]), .C(state[1]), 
         .D(n17), .Z(shift_register_23__N_3134[22])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    defparam i10124_4_lut.init = 16'h0aca;
    LUT4 i10119_4_lut (.A(\rgb_hold[7] ), .B(shift_register[22]), .C(state[1]), 
         .D(n17), .Z(shift_register_23__N_3134[23])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    defparam i10119_4_lut.init = 16'h0aca;
    PFUMX i43 (.BLUT(n25394), .ALUT(n25396), .C0(shift_register[23]), 
          .Z(n26));
    FD1P3IX bit_number_i0_i1 (.D(n136[1]), .SP(pll_clk_enable_778), .CD(n18602), 
            .CK(pll_clk), .Q(bit_number[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i1.GSR = "DISABLED";
    LUT4 i5058_2_lut_rep_139 (.A(state[0]), .B(ws2812_enable), .Z(pll_clk_enable_778)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i5058_2_lut_rep_139.init = 16'h8888;
    LUT4 i7077_2_lut_3_lut (.A(state[0]), .B(ws2812_enable), .C(state[1]), 
         .Z(n18602)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i7077_2_lut_3_lut.init = 16'h0808;
    LUT4 i659_2_lut_rep_118 (.A(n15), .B(data_out_N_3171), .Z(n26052)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i659_2_lut_rep_118.init = 16'heeee;
    LUT4 i7092_2_lut_3_lut_4_lut (.A(state[0]), .B(ws2812_enable), .C(state[1]), 
         .D(n15), .Z(pll_clk_enable_774)) /* synthesis lut_function=(!(((C (D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i7092_2_lut_3_lut_4_lut.init = 16'h0888;
    LUT4 i10463_2_lut_3_lut_4_lut (.A(n15), .B(data_out_N_3171), .C(state[0]), 
         .D(state[1]), .Z(state_1__N_3109[1])) /* synthesis lut_function=(A (C)+!A (B (C)+!B !((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i10463_2_lut_3_lut_4_lut.init = 16'he0f0;
    LUT4 i2_3_lut_4_lut (.A(state[0]), .B(ws2812_enable), .C(state[1]), 
         .D(n15), .Z(n25324)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i2_3_lut_4_lut.init = 16'h0080;
    LUT4 i776_2_lut_rep_105 (.A(data_out_N_3171), .B(n15), .Z(n26039)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i776_2_lut_rep_105.init = 16'hdddd;
    LUT4 i1795_2_lut_rep_95_3_lut_4_lut (.A(data_out_N_3171), .B(n15), .C(bit_number[1]), 
         .D(bit_number[0]), .Z(n26029)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i1795_2_lut_rep_95_3_lut_4_lut.init = 16'h2000;
    LUT4 i1_2_lut_3_lut (.A(data_out_N_3171), .B(n15), .C(bit_number[0]), 
         .Z(n17199)) /* synthesis lut_function=(A (B (C)+!B !(C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i1_2_lut_3_lut.init = 16'hd2d2;
    LUT4 i1793_2_lut_3_lut_4_lut (.A(data_out_N_3171), .B(n15), .C(bit_number[1]), 
         .D(bit_number[0]), .Z(n136[1])) /* synthesis lut_function=(A (B (C)+!B !(C (D)+!C !(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i1793_2_lut_3_lut_4_lut.init = 16'hd2f0;
    LUT4 i2_4_lut_4_lut (.A(bit_cell_count[2]), .B(bit_cell_count[6]), .C(n4), 
         .D(bit_cell_count[5]), .Z(n25396)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(54[25:53])
    defparam i2_4_lut_4_lut.init = 16'h0040;
    LUT4 i1787_2_lut_rep_97_3_lut (.A(data_out_N_3171), .B(n15), .C(bit_number[0]), 
         .Z(n26031)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i1787_2_lut_rep_97_3_lut.init = 16'h2020;
    FD1S3IX state__i1 (.D(state_1__N_3109[1]), .CK(pll_clk), .CD(n17658), 
            .Q(state[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam state__i1.GSR = "DISABLED";
    FD1P3AX reset_count__i1 (.D(n25423), .SP(pll_clk_enable_317), .CK(pll_clk), 
            .Q(reset_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i1.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_adj_33 (.A(state[1]), .B(ws2812_enable), .C(n22[0]), 
         .Z(n25424)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_33.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_34 (.A(state[1]), .B(ws2812_enable), .C(n22[1]), 
         .Z(n25423)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_34.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_35 (.A(state[1]), .B(ws2812_enable), .C(n22[2]), 
         .Z(n25422)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_35.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_36 (.A(state[1]), .B(ws2812_enable), .C(n22[3]), 
         .Z(n25421)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_36.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_37 (.A(state[1]), .B(ws2812_enable), .C(n22[4]), 
         .Z(n25420)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_37.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_38 (.A(state[1]), .B(ws2812_enable), .C(n22[5]), 
         .Z(n25419)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_38.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_39 (.A(state[1]), .B(ws2812_enable), .C(n22[6]), 
         .Z(n25418)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_39.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_40 (.A(state[1]), .B(ws2812_enable), .C(n22[7]), 
         .Z(n25417)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_40.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_41 (.A(state[1]), .B(ws2812_enable), .C(n22[8]), 
         .Z(n25416)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_41.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_42 (.A(state[1]), .B(ws2812_enable), .C(n22[9]), 
         .Z(n25415)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_42.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_43 (.A(state[1]), .B(ws2812_enable), .C(n22[10]), 
         .Z(n25414)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_43.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_44 (.A(state[1]), .B(ws2812_enable), .C(n22[11]), 
         .Z(n25413)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_44.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_45 (.A(state[1]), .B(ws2812_enable), .C(n22[12]), 
         .Z(n25412)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_45.init = 16'h4040;
    FD1P3AX reset_count__i2 (.D(n25422), .SP(pll_clk_enable_317), .CK(pll_clk), 
            .Q(reset_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i2.GSR = "DISABLED";
    FD1P3AX reset_count__i3 (.D(n25421), .SP(pll_clk_enable_317), .CK(pll_clk), 
            .Q(reset_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i3.GSR = "DISABLED";
    FD1P3AX reset_count__i4 (.D(n25420), .SP(pll_clk_enable_317), .CK(pll_clk), 
            .Q(reset_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i4.GSR = "DISABLED";
    FD1P3AX reset_count__i5 (.D(n25419), .SP(pll_clk_enable_317), .CK(pll_clk), 
            .Q(reset_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i5.GSR = "DISABLED";
    FD1P3AX reset_count__i6 (.D(n25418), .SP(pll_clk_enable_317), .CK(pll_clk), 
            .Q(reset_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i6.GSR = "DISABLED";
    FD1P3AX reset_count__i7 (.D(n25417), .SP(pll_clk_enable_317), .CK(pll_clk), 
            .Q(reset_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i7.GSR = "DISABLED";
    FD1P3AX reset_count__i8 (.D(n25416), .SP(pll_clk_enable_317), .CK(pll_clk), 
            .Q(reset_count[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i8.GSR = "DISABLED";
    FD1P3AX reset_count__i9 (.D(n25415), .SP(pll_clk_enable_317), .CK(pll_clk), 
            .Q(reset_count[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i9.GSR = "DISABLED";
    FD1P3AX reset_count__i10 (.D(n25414), .SP(pll_clk_enable_320), .CK(pll_clk), 
            .Q(reset_count[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i10.GSR = "DISABLED";
    FD1P3AX reset_count__i11 (.D(n25413), .SP(pll_clk_enable_320), .CK(pll_clk), 
            .Q(reset_count[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i11.GSR = "DISABLED";
    FD1P3AX reset_count__i12 (.D(n25412), .SP(pll_clk_enable_320), .CK(pll_clk), 
            .Q(reset_count[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i12.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i1 (.D(\shift_register_23__N_3134[1] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[1] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i1.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i2 (.D(\shift_register_23__N_3134[2] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[2] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i2.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i3 (.D(\shift_register_23__N_3134[3] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[3] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i3.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i4 (.D(\shift_register_23__N_3134[4] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[4] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i4.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i5 (.D(\shift_register_23__N_3134[5] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[5] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i5.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i6 (.D(\shift_register_23__N_3134[6] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[6] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i6.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i7 (.D(\shift_register_23__N_3134[7] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[7] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i7.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i8 (.D(\shift_register_23__N_3134[8] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[8] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i8.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i9 (.D(\shift_register_23__N_3134[9] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[9] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i9.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i10 (.D(\shift_register_23__N_3134[10] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[10] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i10.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i11 (.D(\shift_register_23__N_3134[11] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[11] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i11.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i12 (.D(\shift_register_23__N_3134[12] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[12] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i12.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i13 (.D(\shift_register_23__N_3134[13] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[13] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i13.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i14 (.D(\shift_register_23__N_3134[14] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[14] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i14.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i15 (.D(\shift_register_23__N_3134[15] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[15] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i15.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i16 (.D(\shift_register_23__N_3134[16] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[16] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i16.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i17 (.D(\shift_register_23__N_3134[17] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[17] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i17.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i18 (.D(\shift_register_23__N_3134[18] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[18] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i18.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i19 (.D(\shift_register_23__N_3134[19] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(\shift_register[19] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i19.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i20 (.D(\shift_register_23__N_3134[20] ), .SP(pll_clk_enable_774), 
            .CK(pll_clk), .Q(shift_register[20])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i20.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i21 (.D(shift_register_23__N_3134[21]), .SP(pll_clk_enable_633), 
            .CK(pll_clk), .Q(shift_register[21])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i21.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i22 (.D(shift_register_23__N_3134[22]), .SP(pll_clk_enable_633), 
            .CK(pll_clk), .Q(shift_register[22])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i22.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i23 (.D(shift_register_23__N_3134[23]), .SP(pll_clk_enable_633), 
            .CK(pll_clk), .Q(shift_register[23])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i23.GSR = "DISABLED";
    LUT4 n8877_bdd_4_lut (.A(n26052), .B(state[0]), .C(n1), .D(state[1]), 
         .Z(state_1__N_3109[0])) /* synthesis lut_function=(A (B (C+(D))+!B !((D)+!C))+!A !((D)+!C)) */ ;
    defparam n8877_bdd_4_lut.init = 16'h88f0;
    FD1P3IX bit_cell_count_i0_i7 (.D(n70[7]), .SP(pll_clk_enable_778), .CD(pll_clk_enable_774), 
            .CK(pll_clk), .Q(bit_cell_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i7.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i6 (.D(n70[6]), .SP(pll_clk_enable_778), .CD(pll_clk_enable_774), 
            .CK(pll_clk), .Q(bit_cell_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i6.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i5 (.D(n70[5]), .SP(pll_clk_enable_778), .CD(pll_clk_enable_774), 
            .CK(pll_clk), .Q(bit_cell_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i5.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i4 (.D(n70[4]), .SP(pll_clk_enable_778), .CD(pll_clk_enable_774), 
            .CK(pll_clk), .Q(bit_cell_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i4.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i3 (.D(n70[3]), .SP(pll_clk_enable_778), .CD(pll_clk_enable_774), 
            .CK(pll_clk), .Q(bit_cell_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i3.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i2 (.D(n70[2]), .SP(pll_clk_enable_778), .CD(pll_clk_enable_774), 
            .CK(pll_clk), .Q(bit_cell_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i2.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i1 (.D(n70[1]), .SP(pll_clk_enable_778), .CD(pll_clk_enable_774), 
            .CK(pll_clk), .Q(bit_cell_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i1.GSR = "DISABLED";
    FD1P3IX bit_number_i0_i0 (.D(n17199), .SP(pll_clk_enable_778), .CD(n18602), 
            .CK(pll_clk), .Q(bit_number[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i0.GSR = "DISABLED";
    LUT4 i14349_4_lut (.A(ws2812_enable), .B(state[0]), .C(n26052), .D(state[1]), 
         .Z(pll_clk_enable_317)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (D)))) */ ;
    defparam i14349_4_lut.init = 16'h5d77;
    FD1P3IX bit_number_i0_i4 (.D(n136[4]), .SP(pll_clk_enable_778), .CD(n18602), 
            .CK(pll_clk), .Q(bit_number[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i4.GSR = "DISABLED";
    CCU2D add_1194_13 (.A0(reset_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24803), .S0(n22[11]), .S1(n22[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1194_13.INIT0 = 16'h5aaa;
    defparam add_1194_13.INIT1 = 16'h5aaa;
    defparam add_1194_13.INJECT1_0 = "NO";
    defparam add_1194_13.INJECT1_1 = "NO";
    CCU2D add_1194_11 (.A0(reset_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24802), .COUT(n24803), .S0(n22[9]), .S1(n22[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1194_11.INIT0 = 16'h5aaa;
    defparam add_1194_11.INIT1 = 16'h5aaa;
    defparam add_1194_11.INJECT1_0 = "NO";
    defparam add_1194_11.INJECT1_1 = "NO";
    CCU2D add_1194_9 (.A0(reset_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24801), .COUT(n24802), .S0(n22[7]), .S1(n22[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1194_9.INIT0 = 16'h5aaa;
    defparam add_1194_9.INIT1 = 16'h5aaa;
    defparam add_1194_9.INJECT1_0 = "NO";
    defparam add_1194_9.INJECT1_1 = "NO";
    CCU2D add_1194_7 (.A0(reset_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24800), .COUT(n24801), .S0(n22[5]), .S1(n22[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1194_7.INIT0 = 16'h5aaa;
    defparam add_1194_7.INIT1 = 16'h5aaa;
    defparam add_1194_7.INJECT1_0 = "NO";
    defparam add_1194_7.INJECT1_1 = "NO";
    LUT4 i1807_2_lut_3_lut_4_lut (.A(bit_number[1]), .B(n26031), .C(bit_number[3]), 
         .D(bit_number[2]), .Z(n136[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i1807_2_lut_3_lut_4_lut.init = 16'h78f0;
    CCU2D add_1194_5 (.A0(reset_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24799), .COUT(n24800), .S0(n22[3]), .S1(n22[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1194_5.INIT0 = 16'h5aaa;
    defparam add_1194_5.INIT1 = 16'h5aaa;
    defparam add_1194_5.INJECT1_0 = "NO";
    defparam add_1194_5.INJECT1_1 = "NO";
    LUT4 i1_2_lut (.A(bit_cell_count[6]), .B(bit_cell_count[2]), .Z(n25393)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut.init = 16'h4444;
    CCU2D add_1194_3 (.A0(reset_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24798), .COUT(n24799), .S0(n22[1]), .S1(n22[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1194_3.INIT0 = 16'h5aaa;
    defparam add_1194_3.INIT1 = 16'h5aaa;
    defparam add_1194_3.INJECT1_0 = "NO";
    defparam add_1194_3.INJECT1_1 = "NO";
    LUT4 i1814_3_lut_4_lut (.A(bit_number[2]), .B(n26029), .C(bit_number[3]), 
         .D(bit_number[4]), .Z(n136[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i1814_3_lut_4_lut.init = 16'h7f80;
    CCU2D add_1194_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count[0]), .B1(n21944), .C1(n25348), .D1(n21942), 
          .COUT(n24798), .S1(n22[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1194_1.INIT0 = 16'hF000;
    defparam add_1194_1.INIT1 = 16'h5955;
    defparam add_1194_1.INJECT1_0 = "NO";
    defparam add_1194_1.INJECT1_1 = "NO";
    FD1P3IX bit_number_i0_i3 (.D(n136[3]), .SP(pll_clk_enable_778), .CD(n18602), 
            .CK(pll_clk), .Q(bit_number[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i3.GSR = "DISABLED";
    LUT4 i1_4_lut (.A(reset_count[9]), .B(reset_count[3]), .C(reset_count[7]), 
         .D(reset_count[1]), .Z(n25348)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[25:52])
    defparam i1_4_lut.init = 16'hbfff;
    CCU2D add_15_9 (.A0(bit_cell_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24797), .S0(n70[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_9.INIT0 = 16'h5aaa;
    defparam add_15_9.INIT1 = 16'h0000;
    defparam add_15_9.INJECT1_0 = "NO";
    defparam add_15_9.INJECT1_1 = "NO";
    CCU2D add_15_7 (.A0(bit_cell_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24796), .COUT(n24797), .S0(n70[5]), .S1(n70[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_7.INIT0 = 16'h5aaa;
    defparam add_15_7.INIT1 = 16'h5aaa;
    defparam add_15_7.INJECT1_0 = "NO";
    defparam add_15_7.INJECT1_1 = "NO";
    CCU2D add_15_5 (.A0(bit_cell_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24795), .COUT(n24796), .S0(n70[3]), .S1(n70[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_5.INIT0 = 16'h5aaa;
    defparam add_15_5.INIT1 = 16'h5aaa;
    defparam add_15_5.INJECT1_0 = "NO";
    defparam add_15_5.INJECT1_1 = "NO";
    LUT4 i3_4_lut (.A(bit_cell_count[4]), .B(bit_cell_count[5]), .C(bit_cell_count[0]), 
         .D(n25393), .Z(n25394)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam i3_4_lut.init = 16'h0400;
    FD1P3IX bit_number_i0_i2 (.D(n136[2]), .SP(pll_clk_enable_778), .CD(n18602), 
            .CK(pll_clk), .Q(bit_number[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i2.GSR = "DISABLED";
    LUT4 i4_4_lut (.A(reset_count[5]), .B(reset_count[6]), .C(reset_count[0]), 
         .D(n6), .Z(n21942)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i4_4_lut.init = 16'h8000;
    CCU2D add_15_3 (.A0(bit_cell_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24794), .COUT(n24795), .S0(n70[1]), .S1(n70[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_3.INIT0 = 16'h5aaa;
    defparam add_15_3.INIT1 = 16'h5aaa;
    defparam add_15_3.INJECT1_0 = "NO";
    defparam add_15_3.INJECT1_1 = "NO";
    CCU2D add_15_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_cell_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24794), .S1(n70[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_1.INIT0 = 16'hF000;
    defparam add_15_1.INIT1 = 16'h5555;
    defparam add_15_1.INJECT1_0 = "NO";
    defparam add_15_1.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_46 (.A(reset_count[8]), .B(reset_count[12]), .Z(n6)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_46.init = 16'h8888;
    LUT4 i3_4_lut_adj_47 (.A(reset_count[4]), .B(reset_count[2]), .C(reset_count[11]), 
         .D(reset_count[10]), .Z(n21944)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_4_lut_adj_47.init = 16'h8000;
    LUT4 i10163_4_lut (.A(n25348), .B(state[0]), .C(n21942), .D(n21944), 
         .Z(n1)) /* synthesis lut_function=(A (B)+!A (B+(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam i10163_4_lut.init = 16'hdccc;
    LUT4 i1800_2_lut_3_lut_4_lut (.A(bit_number[0]), .B(n26039), .C(bit_number[2]), 
         .D(bit_number[1]), .Z(n136[2])) /* synthesis lut_function=(A (B (C)+!B !(C (D)+!C !(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i1800_2_lut_3_lut_4_lut.init = 16'hd2f0;
    LUT4 i2_4_lut (.A(state[0]), .B(n25563), .C(n15), .D(n27_adj_3385), 
         .Z(pll_clk_enable_779)) /* synthesis lut_function=(A (((D)+!C)+!B)+!A !(B)) */ ;
    defparam i2_4_lut.init = 16'hbb3b;
    LUT4 i14105_2_lut (.A(ws2812_enable), .B(state[1]), .Z(n25563)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14105_2_lut.init = 16'h8888;
    LUT4 i7_4_lut (.A(bit_cell_count[3]), .B(n14), .C(n4), .D(bit_cell_count[6]), 
         .Z(n15)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;
    defparam i7_4_lut.init = 16'hffdf;
    LUT4 i3_4_lut_adj_48 (.A(bit_cell_count[1]), .B(bit_cell_count[3]), 
         .C(bit_cell_count[7]), .D(n26), .Z(n27_adj_3385)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam i3_4_lut_adj_48.init = 16'h0400;
    LUT4 i6_4_lut (.A(bit_cell_count[5]), .B(bit_cell_count[1]), .C(bit_cell_count[2]), 
         .D(bit_cell_count[7]), .Z(n14)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;
    defparam i6_4_lut.init = 16'hbfff;
    LUT4 i10165_4_lut (.A(state[0]), .B(ws2812_enable), .C(data_out_N_3165), 
         .D(state[1]), .Z(n12603)) /* synthesis lut_function=(A (B (C+!(D)))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i10165_4_lut.init = 16'hc088;
    LUT4 i1_4_lut_adj_49 (.A(bit_number[3]), .B(bit_number[2]), .C(n25501), 
         .D(bit_number[4]), .Z(data_out_N_3171)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;
    defparam i1_4_lut_adj_49.init = 16'hbfff;
    LUT4 i14043_2_lut (.A(bit_number[1]), .B(bit_number[0]), .Z(n25501)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14043_2_lut.init = 16'h8888;
    LUT4 i14346_4_lut_4_lut (.A(state[1]), .B(n26052), .C(state[0]), .D(ws2812_enable), 
         .Z(pll_clk_enable_320)) /* synthesis lut_function=(!(A (B (D)+!B !(C+!(D)))+!A (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam i14346_4_lut_4_lut.init = 16'h25ff;
    LUT4 i1_2_lut_adj_50 (.A(bit_cell_count[4]), .B(bit_cell_count[0]), 
         .Z(n4)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_50.init = 16'h8888;
    FD1P3IX shift_register_i0_i0 (.D(\rgb_hold[16] ), .SP(pll_clk_enable_774), 
            .CD(n25324), .CK(pll_clk), .Q(\shift_register[0] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i0.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i0 (.D(n70[0]), .SP(pll_clk_enable_778), .CD(pll_clk_enable_774), 
            .CK(pll_clk), .Q(bit_cell_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i0.GSR = "DISABLED";
    FD1P3AX data_out_reg_46 (.D(n12603), .SP(pll_clk_enable_779), .CK(pll_clk), 
            .Q(rgb_data_c)) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=679, LSE_RLINE=691 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam data_out_reg_46.GSR = "DISABLED";
    
endmodule
