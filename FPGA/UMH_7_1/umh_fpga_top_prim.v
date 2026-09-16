// Verilog netlist produced by program LSE :  version Diamond (64-bit) 3.13.0.56.2
// Netlist written on Wed Sep 16 14:36:05 2026
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
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(95[10:17])
    wire spi1_sck_N_305 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(130[17:33])
    wire sck_N_3050 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    
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
        mic_data_1_c, spi_mic_cs_n_c, spi_mic_miso_c, pll_feedback;
    wire [7:0]spi_rx_shift;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(119[17:29])
    wire [7:0]spi_command;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(119[31:42])
    wire [7:0]spi_version;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(119[44:55])
    wire [7:0]spi_phase_pending;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(119[57:74])
    wire [2:0]spi_bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[17:30])
    wire [15:0]spi_byte_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[17:31])
    
    wire n25457;
    wire [15:0]spi_extension_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[51:71])
    
    wire spi1_sck_c_enable_112;
    wire [31:0]spi_frame_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(122[17:35])
    wire [31:0]spi_expected_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(122[37:56])
    wire [31:0]accepted_sequence_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(122[58:79])
    
    wire n24901, n134, n135, n136, n24900, n24912, n24899, n24898;
    wire [6:0]spi_channel_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(124[17:34])
    wire [1:0]spi_channel_field;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(125[17:34])
    wire [3:0]spi_rgb_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(126[17:30])
    
    wire frame_toggle_spi, stop_toggle_spi, invalid_frame_spi, ws2812_toggle_spi, 
        n24936, n13435, n13431, n13427, n13423, n13419, n13406, 
        n13402, n13389, n13385, n13381, n13377, n13373, n13361, 
        n13357;
    wire [95:0]rgb_values;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(129[17:27])
    wire [6:0]status_bit_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(130[17:33])
    
    wire spi_write, n26251;
    wire [23:0]phase_frac;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(163[17:27])
    
    wire phase_step_s1;
    wire [7:0]global_phase_s2;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:32])
    
    wire phase_step_s2, wrap_s2;
    wire [8:0]run_addr_s3;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(168[17:28])
    
    wire phase_step_s3, swap_now_s3, phase_step_s4, swap_now_s4;
    wire [83:0]ev_run_hold_s5;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(173[17:31])
    
    wire phase_step_s5, swap_now_s5;
    wire [24:0]phase_frac_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[17:31])
    wire [31:0]fpga_time;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(179[17:26])
    wire [6:0]time_divider;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(180[17:29])
    wire [6:0]mic_divider;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(183[17:28])
    
    wire mic_tick;
    wire [15:0]mic_shift_0_l;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(186[17:30])
    wire [15:0]mic_shift_0_r;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(186[32:45])
    wire [15:0]mic_shift_1_l;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(187[17:30])
    wire [15:0]mic_shift_1_r;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(187[32:45])
    wire [4:0]mic_sample_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(188[17:33])
    wire [63:0]mic_latest;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(189[17:27])
    wire [3:0]ev_state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(203[17:25])
    wire [7:0]ev_clear_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(204[17:30])
    wire [6:0]ev_ch;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[17:22])
    
    wire n24978, n24977, n24935, n24911, n24934, n24975, n24974, 
        n24933, n24910, n24897, n24973;
    wire [83:0]init_shadow;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[17:28])
    wire [7:0]build_phase;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[17:28])
    wire [8:0]build_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[17:26])
    wire [6:0]staging_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(214[17:32])
    
    wire frame_req, swap_pending, active_bank;
    wire [15:0]staging_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[17:26])
    wire [7:0]ev_rd_slot;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[17:27])
    wire [8:0]event_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(241[17:30])
    wire [8:0]ev_wr_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(242[17:27])
    
    wire ev_we;
    wire [83:0]ev_rd_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(247[17:27])
    wire [83:0]ev_rd_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(248[17:27])
    wire [83:0]ev_wr_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(249[17:27])
    
    wire frame_toggle_meta, frame_toggle_sync, frame_toggle_seen, stop_toggle_meta, 
        stop_toggle_sync, stop_toggle_seen, ws2812_toggle_meta, ws2812_toggle_sync, 
        ws2812_toggle_seen, invalid_frame_meta;
    wire [31:0]accepted_sequence_meta;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[17:39])
    wire [31:0]accepted_sequence_sync;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[41:63])
    wire [31:0]pending_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[17:33])
    wire [31:0]accepted_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[35:52])
    
    wire n24972, n24971;
    wire [3:0]frame_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(270[17:29])
    wire [3:0]ws2812_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(271[17:30])
    
    wire n137, n138, n139, n24908, n24895, n24896, n24890, n140, 
        n141, n142, n24888, n24907, n143, n144, n145, n146, 
        n147, n24, n148, n149, n150, n24891, n24753, n24892, 
        n24887, n24970, n151, n152, n153, n154;
    wire [95:0]rgb_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[17:25])
    
    wire n8;
    wire [127:0]status_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(279[18:29])
    
    wire n17865, n4;
    wire [15:0]fifo_credit_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[17:33])
    wire [15:0]expected_next;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[17:30])
    
    wire frame_end, n155, n156, n157, n158, n159, n160, n161, 
        n162, n163, n164, n165, n13343, n13339, n5, n25, n24_adj_3164, 
        n23, n17190, n26153;
    wire [15:0]spi_rgb_payload_byte_N_2547;
    
    wire n24969, n24906, n24968, n24755, n24967, n24894, n24754, 
        n24889, n24905, n24927, n24926, n24925, n24966;
    wire [15:0]status_flags_wire_15__N_1210;
    wire [15:0]status_flags_wire_15__N_1226;
    wire [6:0]spi1_miso_N_2492;
    
    wire spi1_miso_N_2491, fpga_cs_n_N_2500;
    wire [15:0]expected_next_15__N_1351;
    
    wire pll_clk_enable_203, frame_end_N_2645;
    wire [31:0]frame_end_N_2613;
    wire [15:0]rd_data_15__N_2646;
    
    wire frame_end_N_2612, frame_end_N_2610, n25477, n22, n16988, 
        n24904, n12991, n18, n19, n26250, n18_adj_3165, spi1_sck_c_enable_35, 
        n26152, n26249, n26151, n25548, n25766, n26, n27, n28, 
        n29, n30, n34, n35, n36, n37, n38, n39, n40, spi1_sck_c_enable_43, 
        n4_adj_3166, spi1_sck_c_enable_8;
    wire [15:0]spi_byte_count_15__N_1504;
    
    wire ws2812_toggle_spi_N_2535, n26127, spi1_sck_c_enable_36, n25512, 
        stop_toggle_spi_N_2515, n25487, n11301, n26150, frame_toggle_spi_N_2505, 
        n26149, invalid_frame_spi_N_2522, n26126, n17110, pll_clk_enable_642, 
        n26148, n16173, spi1_sck_c_enable_90, pll_clk_enable_211, n6, 
        n14, n25024, n11289, n26247, n10, n26246, n25474, n21554, 
        wrap_s2_N_2576, active_bank_N_731, n25685, n26245, n13369, 
        n25756, n25683, n1, n25755, n25679, n25544, n25754, n66, 
        n1_adj_3167, n26487, n39_adj_3168, n38_adj_3169, n37_adj_3170, 
        n36_adj_3171, n24965, n25753, pll_clk_enable_683, n13, n25752, 
        n12, pll_clk_enable_446, n34_adj_3172, n35_adj_3173, n36_adj_3174, 
        n37_adj_3175, n38_adj_3176, n39_adj_3177, n40_adj_3178, n13395, 
        n24608, n25669, pll_clk_enable_640, pll_clk_enable_383, n26244, 
        n26243, n24964, pll_clk_enable_652, n25667, n26242, pll_clk_enable_649, 
        n24_adj_3179, n26241, n25661, n21606, n6_adj_3180, n21531, 
        n26240, n25751, n65, n26239, n26238;
    wire [3:0]frame_settle_3__N_1982;
    
    wire n26147, n5_adj_3181, spi1_sck_c_enable_44, pll_clk_enable_542, 
        n26237, n21195, n25649, n40_adj_3182, n24903, n25750, n35_adj_3183, 
        n25639, n24963, n34_adj_3184, n26482, n21234, n26236, n26235, 
        n25637, spi1_sck_c_enable_173, n13000, n11, n26234, spi1_sck_N_305_enable_7, 
        n25749, n25748, n25621, n26233, spi1_sck_c_enable_135, pll_clk_enable_149, 
        n12996, n12990, n24752, n24902, n24924;
    wire [31:0]accepted_sequence_31__N_943;
    
    wire n24962, n25747, n25746, n25726, n11840, swap_pending_N_2588, 
        frame_req_N_2584, n25745, n11277;
    wire [7:0]ev_clear_addr_7__N_2215;
    wire [8:0]build_sum_8__N_2031;
    wire [3:0]ev_state_3__N_1990;
    
    wire pll_clk_enable_226;
    wire [6:0]ev_ch_6__N_2010;
    wire [3:0]ev_state_3__N_602;
    wire [6:0]ev_ch_6__N_614;
    
    wire n24961, n24960, n24959, n24958, n24957, n25744, n25743, 
        n21588, n24955, n24954, n24953, n6_adj_3185, n24952, n24951, 
        n24950, n24923, n24922, n14_adj_3186;
    wire [6:0]staging_rd_addr_6__N_722;
    
    wire n43, n34_adj_3187, n25549, mic_tick_N_2577, n26121, n86, 
        n25742, n25741, n25826, n25740, n25739, n25738, n25737, 
        n26230, n34_adj_3188, n35_adj_3189, n36_adj_3190, n37_adj_3191, 
        n38_adj_3192, n39_adj_3193, n40_adj_3194, mic_clk_N_2501, n38_adj_3195, 
        n39_adj_3196, n40_adj_3197, n41, n42, n24886, n24921, n24949, 
        n10_adj_3198, n43_adj_3199, n44, n13365, n24948, n24920, 
        n24919, n24947, n45, n4_adj_3200, n24918, n24946, n24945, 
        n24917, pll_clk_enable_432, n12210, n12209, n12208, n12207, 
        n12206, n12205, n12204, n12203, n12202, n12201, n12200, 
        n12199, n12198, n12197, n12196, n12195, n12194, n12193, 
        n25736, pll_clk_enable_587, n25735, n24916, n24915, n25734, 
        n24914, n25733, pll_clk_enable_290, n12_adj_3201, n24751, 
        n26146, n22_adj_3202, pll_clk_enable_650, n25732, n26280, 
        n26279, n26123, n26278, pll_clk_enable_647, n26277, n25121, 
        n12162, spi1_sck_c_enable_82, n25526, n6_adj_3203, n5_adj_3204, 
        n26276, pll_clk_enable_115, n1_adj_3205, n26120, n26144, spi1_sck_c_enable_170, 
        n26228, pll_clk_enable_324, n17880, n26227, n25731, n25730, 
        n26275, n17861, n26274, n26273, pll_clk_enable_8, n17798, 
        pll_clk_enable_66, n26272;
    wire [1:0]state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(28[15:20])
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(32[16:30])
    
    wire n26271, n26270, n25729, n25728, pll_clk_enable_24, pll_clk_enable_22, 
        n26267, n25727, n26266, n26265, spi1_sck_c_enable_104, n16929, 
        spi1_sck_c_enable_179, n26264, spi1_sck_c_enable_166, pll_clk_enable_23, 
        spi1_sck_c_enable_97, n14_adj_3206, n25725, n26262, n26261, 
        n26260, n26226, spi1_sck_c_enable_171, n26258, n12192, n12191, 
        n12190, n12189, n12188, n12187, n12186, n12185, n12184, 
        n12183, n12182, n12181, n12180, n12179, n12178, n12177, 
        pll_clk_enable_18, n12175, n12174, n12173, n12172, n12171, 
        n12170, n12169, n12168, n12167, n12166, n12165, n12164, 
        n12163, n13071, n3, n20, n25724, n26256, n26255, n10_adj_3207, 
        n25723, n26253, n25722, n26252, n17908, n25721, n25720, 
        n25719, n26225, n26224, n26223, n26222, pll_clk_enable_20, 
        n20_adj_3208, n18_adj_3209, n25718, n26220, n26219, pll_clk_enable_355, 
        n26142, n4_adj_3210, n25717, n25716, n26217, n26216, n26141, 
        spi1_sck_c_enable_169, n26215, pll_clk_enable_241, n25715, n26214, 
        spi1_sck_c_enable_32, n25714, n25713, n26140, pll_clk_enable_576, 
        n25712, n25711, n25471, n26213, n26212, n25710, n25709, 
        pll_clk_enable_643, n26211, n26210, n26209, n26208, n25708, 
        n26139, n25707, n26138, n26207, n26206, n25706, spi1_sck_c_enable_66, 
        n26137, n4_adj_3211, n25705, n25704, n25703, n26471, n26205, 
        n25702, n25701, n26134, n26204, n25064, pll_clk_enable_684, 
        n25700, n26203, n25699, n25698, n26202, n6_adj_3212, n25697, 
        n26201, pll_clk_enable_639, n26199, n26198, n26197, n26196, 
        n26195, n26083, n25696, n26082, n26080, n24985, n26079, 
        n27_adj_3213, n25768, n26194, n26193, n26_adj_3214, n25694, 
        n25_adj_3215, n24_adj_3216, n14850, n14844, n26192, n14838, 
        n14832, n26191, n14826, n14820, n26190, n14814, n14808, 
        n14802, n24984, n14796, n14790, n24983, n14784, n14778, 
        n14772, n14766, pll_clk_enable_9, n26189, n14760, n14754, 
        n14748, n26188, n14742, n14736, n24982, n14730, n14724, 
        n26187, n26186, n14718, n14712, n26185, n14706, n14700, 
        n14694, n14688, n14682, n26067, n24981, n14676, n14670, 
        n26184, n14664, n26183, n14658, n9, n14652, n14646, n14640, 
        n26182, n14634, n14628, n26181, n14622, n14616, n14610, 
        n14604, n14598, n14592, n14586, n14580, n26132, n14574, 
        n26059, n14568, n14562, n14556, n14550, n26058, n14544, 
        n14538, n14532, n14526, n14520, n14514, n14508, n14502, 
        n26057, n14496, n14490, n14484, n14478, n25695, n14472, 
        n14466, n14460, n14454, n14448, n14442, n14436, n14430, 
        n14424, n14418, n26180, n14412, n14406, n14400, n14394, 
        n14388, n14382, n14376, n14370, n14364, spi1_sck_c_enable_59, 
        n14358, n25693, n26179, n25692, n26311, n26178, n26310, 
        n26309, n26177, n26307, n26176, n26131, n26306, n26304, 
        n26175, n26303, n26301, n26300, n26298, n26128, n26174, 
        n14125, n26173, n26297, n26172, n26295, n26042, spi1_sck_c_enable_74, 
        n26294, n26171, n17186, n24980, n26170, n26169, n26168, 
        n26292, n25529, n26167, n26291, n24979, n13892, n13890, 
        n13888, n13886, n13884, n13882, n13880, n13878, n13876, 
        n13874, n13872, n13870, n13868, n13866, n13864, n13862, 
        n13860, n13858, n13856, n13854, n13852, n13850, n13848, 
        n13846, n13844, n13842, n13840, n13838, n13836, n13834, 
        n13832, n13830, n13828, n13826, n13824, n13822, n13820, 
        n13818, n13816, n13814, n13812, n13810, n13808, n13806, 
        n13804, n13802, n13800, n13798, n13796, n13794, n13792, 
        n13790, n13788, n13786, n13784, n13782, n13780, n13778, 
        n13776, n13774, n13772, n13770, n13768, n13766, n13764, 
        n13762, n13760, n13758, n13756, n13754, n13752, n13750, 
        n13748, n13746, n13744, n13742, n13740, n13738, n13736, 
        n13734, n13732, n13730, n13728, n26289, n13711, n26166, 
        n26165, n26288, n26164, n7, n26286, n26285, n26163, n26031, 
        n26283, n26162, spi1_sck_c_enable_120, n26161, n26160, n26159, 
        n26158, n24913, spi1_sck_c_enable_51, n26157, n26156, n26_adj_3217, 
        n26155, n26022, n26154, n26282, n25675, n7_adj_3218, n14_adj_3219;
    
    VHI i2 (.Z(VCC_net));
    INV i15393 (.A(spi_mic_sck_c), .Z(sck_N_3050));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(71[24:35])
    FD1S3AX mem_1714 (.D(staging_rd_addr_6__N_722[6]), .CK(pll_clk), .Q(n12175));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mem_1714.GSR = "DISABLED";
    L6MUX21 i15138 (.D0(n25713), .D1(n25714), .SD(n26246), .Z(n25720));
    FD1S3AX mem_1711 (.D(staging_rd_addr_6__N_722[5]), .CK(pll_clk), .Q(n12173));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mem_1711.GSR = "DISABLED";
    FD1S3IX time_divider_1597__i6 (.D(n34), .CK(pll_clk), .CD(pll_clk_enable_683), 
            .Q(time_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597__i6.GSR = "DISABLED";
    PFUMX i15161 (.BLUT(n25728), .ALUT(n25729), .C0(spi1_miso_N_2492[1]), 
          .Z(n25743));
    PFUMX i15162 (.BLUT(n25730), .ALUT(n25731), .C0(spi1_miso_N_2492[1]), 
          .Z(n25744));
    LUT4 i1_2_lut_rep_249_3_lut (.A(ev_ch[2]), .B(ev_ch[0]), .C(ev_ch[1]), 
         .Z(n26213)) /* synthesis lut_function=(A+((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_249_3_lut.init = 16'hfbfb;
    FD1S3IX time_divider_1597__i5 (.D(n35), .CK(pll_clk), .CD(pll_clk_enable_683), 
            .Q(time_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597__i5.GSR = "DISABLED";
    FD1S3IX time_divider_1597__i4 (.D(n36), .CK(pll_clk), .CD(pll_clk_enable_683), 
            .Q(time_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597__i4.GSR = "DISABLED";
    FD1S3IX time_divider_1597__i3 (.D(n37), .CK(pll_clk), .CD(pll_clk_enable_683), 
            .Q(time_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597__i3.GSR = "DISABLED";
    FD1S3IX time_divider_1597__i2 (.D(n38), .CK(pll_clk), .CD(pll_clk_enable_683), 
            .Q(time_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597__i2.GSR = "DISABLED";
    FD1S3IX time_divider_1597__i1 (.D(n39), .CK(pll_clk), .CD(pll_clk_enable_683), 
            .Q(time_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597__i1.GSR = "DISABLED";
    FD1P3AX spi_command_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_8), 
            .CK(spi1_sck_c), .Q(spi_command[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_command_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_8), 
            .CK(spi1_sck_c), .Q(spi_command[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_command_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_8), 
            .CK(spi1_sck_c), .Q(spi_command[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_command_i0_i5.GSR = "ENABLED";
    FD1S3IX frame_settle__i0 (.D(n17186), .CK(pll_clk), .CD(n14125), .Q(frame_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam frame_settle__i0.GSR = "DISABLED";
    FD1S3AX spi_rx_shift_i1 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(spi_rx_shift[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_rx_shift_i1.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_8), 
            .CK(spi1_sck_c), .Q(spi_command[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_command_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_8), 
            .CK(spi1_sck_c), .Q(spi_command[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_command_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_8), 
            .CK(spi1_sck_c), .Q(spi_command[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_command_i0_i2.GSR = "ENABLED";
    L6MUX21 i15139 (.D0(n25715), .D1(n25716), .SD(n26246), .Z(n25721));
    LUT4 i4_3_lut (.A(n24_adj_3179), .B(spi_byte_count[9]), .C(spi_byte_count[13]), 
         .Z(n11)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;
    defparam i4_3_lut.init = 16'h0202;
    FD1P3AX spi_command_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_8), 
            .CK(spi1_sck_c), .Q(spi_command[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_command_i0_i1.GSR = "ENABLED";
    FD1S3AX mem_1707 (.D(staging_rd_addr_6__N_722[3]), .CK(pll_clk), .Q(n12169));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mem_1707.GSR = "DISABLED";
    CCU2D fpga_time_1596_add_4_3 (.A0(fpga_time[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24957), .COUT(n24958), .S0(n164), .S1(n163));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_3.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_3.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_3.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_3.INJECT1_1 = "NO";
    L6MUX21 i15140 (.D0(n25717), .D1(n25718), .SD(n26246), .Z(n25722));
    FD1S3AX mem_1709 (.D(staging_rd_addr_6__N_722[4]), .CK(pll_clk), .Q(n12171));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mem_1709.GSR = "DISABLED";
    OB us_tx_pad_76 (.I(us_tx_c_76), .O(us_tx[76]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX spi_command_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_8), 
            .CK(spi1_sck_c), .Q(spi_command[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_command_i0_i0.GSR = "ENABLED";
    FD1S3IX time_divider_1597__i0 (.D(n40), .CK(pll_clk), .CD(pll_clk_enable_683), 
            .Q(time_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597__i0.GSR = "DISABLED";
    OB us_tx_pad_77 (.I(us_tx_c_77), .O(us_tx[77]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_78 (.I(us_tx_c_78), .O(us_tx[78]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_79 (.I(us_tx_c_79), .O(us_tx[79]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_80 (.I(us_tx_c_80), .O(us_tx[80]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX spi_version_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_version_i0_i0.GSR = "ENABLED";
    CCU2D fpga_time_1596_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24957), .S1(n165));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_1.INIT0 = 16'hF000;
    defparam fpga_time_1596_add_4_1.INIT1 = 16'h0555;
    defparam fpga_time_1596_add_4_1.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_1.INJECT1_1 = "NO";
    FD1P3AX spi_update_flags_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(spi_rgb_payload_byte_N_2547[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_update_flags_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(expected_next[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_phase_pending_i0_i0.GSR = "ENABLED";
    FD1P3AX rgb_values_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(rgb_values[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i0.GSR = "DISABLED";
    FD1S3JX ws2812_settle_i0_i0 (.D(n17190), .CK(pll_clk), .PD(pll_clk_enable_23), 
            .Q(ws2812_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ws2812_settle_i0_i0.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1598__i0 (.D(n30), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_sample_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam mic_sample_count_1598__i0.GSR = "DISABLED";
    FD1S3AX phase_frac_i0 (.D(phase_frac_sum[0]), .CK(pll_clk), .Q(phase_frac[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i0.GSR = "DISABLED";
    FD1P3AX rgb_hold__i1 (.D(rgb_values[0]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i1.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i0 (.D(spi_byte_count_15__N_1504[0]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i0.GSR = "ENABLED";
    FD1P3IX us_tx__i1 (.D(n12990), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_0)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i1.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i0 (.D(mic_data_0_c), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i0.GSR = "DISABLED";
    FD1P3AX status_hold__i1 (.D(accepted_sequence[24]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i1.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0 (.D(accepted_sequence_31__N_943[0]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i0.GSR = "DISABLED";
    PFUMX i15163 (.BLUT(n25732), .ALUT(n25733), .C0(spi1_miso_N_2492[1]), 
          .Z(n25745));
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
            .PLLADDR4(GND_net), .CLKOP(pll_clk), .CLKINTFB(pll_feedback)) /* synthesis syn_instantiated=1 */ ;
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
    FD1S3AX phase_step_s1_560 (.D(phase_frac_sum[24]), .CK(pll_clk), .Q(phase_step_s1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_step_s1_560.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut (.A(n26220), .B(n26235), .C(init_shadow[74]), 
         .D(n16988), .Z(n14412)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut.init = 16'hf1f0;
    LUT4 ev_state_3__I_0_709_i6_2_lut_rep_288 (.A(ev_state[2]), .B(ev_state[3]), 
         .Z(n26252)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam ev_state_3__I_0_709_i6_2_lut_rep_288.init = 16'hdddd;
    FD1S3AX phase_step_s2_561 (.D(phase_step_s1), .CK(pll_clk), .Q(phase_step_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_step_s2_561.GSR = "DISABLED";
    FD1S3AX phase_step_s3_564 (.D(phase_step_s2), .CK(pll_clk), .Q(phase_step_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_step_s3_564.GSR = "DISABLED";
    FD1S3AX swap_now_s3_565 (.D(pll_clk_enable_18), .CK(pll_clk), .Q(swap_now_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam swap_now_s3_565.GSR = "DISABLED";
    FD1S3AX active_bank_566 (.D(active_bank_N_731), .CK(pll_clk), .Q(active_bank)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam active_bank_566.GSR = "DISABLED";
    FD1S3JX swap_pending_567 (.D(swap_pending_N_2588), .CK(pll_clk), .PD(n25549), 
            .Q(swap_pending)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam swap_pending_567.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i0 (.D(global_phase_s2[0]), .SP(pll_clk_enable_211), 
            .CK(pll_clk), .Q(run_addr_s3[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam run_addr_s3_i0.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i0 (.D(accepted_sequence_spi[0]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i0.GSR = "DISABLED";
    FD1S3AX phase_step_s4_571 (.D(phase_step_s3), .CK(pll_clk), .Q(phase_step_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_step_s4_571.GSR = "DISABLED";
    FD1S3AX swap_now_s4_572 (.D(swap_now_s3), .CK(pll_clk), .Q(swap_now_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam swap_now_s4_572.GSR = "DISABLED";
    FD1S3AX phase_step_s5_573 (.D(phase_step_s4), .CK(pll_clk), .Q(phase_step_s5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_step_s5_573.GSR = "DISABLED";
    FD1S3AX swap_now_s5_574 (.D(swap_now_s4), .CK(pll_clk), .Q(swap_now_s5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam swap_now_s5_574.GSR = "DISABLED";
    FD1S3AX frame_toggle_meta_579 (.D(frame_toggle_spi), .CK(pll_clk), .Q(frame_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam frame_toggle_meta_579.GSR = "DISABLED";
    FD1S3AX frame_toggle_sync_580 (.D(frame_toggle_meta), .CK(pll_clk), 
            .Q(frame_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam frame_toggle_sync_580.GSR = "DISABLED";
    FD1S3AX stop_toggle_meta_581 (.D(stop_toggle_spi), .CK(pll_clk), .Q(stop_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam stop_toggle_meta_581.GSR = "DISABLED";
    FD1S3AX stop_toggle_sync_582 (.D(stop_toggle_meta), .CK(pll_clk), .Q(stop_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam stop_toggle_sync_582.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_meta_583 (.D(ws2812_toggle_spi), .CK(pll_clk), 
            .Q(ws2812_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ws2812_toggle_meta_583.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_sync_584 (.D(ws2812_toggle_meta), .CK(pll_clk), 
            .Q(ws2812_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ws2812_toggle_sync_584.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i0 (.D(accepted_sequence_meta[0]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i0.GSR = "DISABLED";
    FD1S3AX invalid_frame_meta_589 (.D(invalid_frame_spi), .CK(pll_clk), 
            .Q(invalid_frame_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam invalid_frame_meta_589.GSR = "DISABLED";
    FD1S3AX invalid_frame_sync_590 (.D(invalid_frame_meta), .CK(pll_clk), 
            .Q(status_flags_wire_15__N_1210[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam invalid_frame_sync_590.GSR = "DISABLED";
    FD1P3IX frame_req_593 (.D(n26471), .SP(pll_clk_enable_8), .CD(n16173), 
            .CK(pll_clk), .Q(frame_req)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam frame_req_593.GSR = "DISABLED";
    LUT4 i10658_4_lut_4_lut_3_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[1]), .D(ev_state[0]), .Z(n21195)) /* synthesis lut_function=((B+!(C (D)+!C !(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam i10658_4_lut_4_lut_3_lut_4_lut.init = 16'hdffd;
    FD1P3AX ev_ch_i0 (.D(ev_ch_6__N_614[0]), .SP(pll_clk_enable_9), .CK(pll_clk), 
            .Q(ev_ch[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_ch_i0.GSR = "DISABLED";
    FD1P3AX build_phase_i0 (.D(staging_q[8]), .SP(pll_clk_enable_241), .CK(pll_clk), 
            .Q(build_phase[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_phase_i0.GSR = "DISABLED";
    FD1P3AX build_sum_i0 (.D(build_sum_8__N_2031[0]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_sum[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_sum_i0.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i0 (.D(ev_rd_data[0]), .SP(pll_clk_enable_290), .CK(pll_clk), 
            .Q(ev_rd_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i0.GSR = "DISABLED";
    L6MUX21 i15168 (.D0(n25742), .D1(n25743), .SD(n26246), .Z(n25750));
    FD1S3AX staging_rd_addr_i0 (.D(staging_rd_addr_6__N_722[0]), .CK(pll_clk), 
            .Q(staging_rd_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam staging_rd_addr_i0.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0 (.D(accepted_sequence_sync[0]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i1 (.D(mic_data_0_c), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i1.GSR = "DISABLED";
    FD1S3AX mic_tick_610 (.D(mic_tick_N_2577), .CK(pll_clk), .Q(mic_tick)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_tick_610.GSR = "DISABLED";
    FD1S3AX mic_clock_reg_612 (.D(mic_clk_N_2501), .CK(pll_clk), .Q(mic_clk_c)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_clock_reg_612.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i1 (.D(mic_data_1_c), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i0.GSR = "DISABLED";
    CCU2D add_14302_1 (.A0(spi_byte_count[2]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[3]), .B1(spi_rgb_payload_byte_N_2547[6]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24922));
    defparam add_14302_1.INIT0 = 16'h5000;
    defparam add_14302_1.INIT1 = 16'h5999;
    defparam add_14302_1.INJECT1_0 = "NO";
    defparam add_14302_1.INJECT1_1 = "NO";
    LUT4 i16_4_lut (.A(ev_run_hold_s5[79]), .B(us_tx_c_79), .C(init_shadow[79]), 
         .D(swap_now_s5), .Z(n13884)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut.init = 16'h5a66;
    OB us_tx_pad_81 (.I(us_tx_c_81), .O(us_tx[81]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX accepted_sequence_spi_i0_i0 (.D(spi_frame_sequence[0]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i0.GSR = "DISABLED";
    FD1P3AX spi_expected_length_i0 (.D(expected_next[0]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i0.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i1 (.D(expected_next[1]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i1.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i2 (.D(expected_next[2]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[2])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i2.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i3 (.D(expected_next[3]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i3.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i4 (.D(expected_next[4]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i4.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i5 (.D(expected_next[5]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[5])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i5.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i6 (.D(expected_next[6]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i6.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i7 (.D(expected_next[7]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i7.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i8 (.D(expected_next[8]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i8.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i9 (.D(expected_next[9]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i9.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i10 (.D(expected_next[10]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i10.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i11 (.D(expected_next[11]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i11.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i12 (.D(expected_next[12]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i12.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i13 (.D(expected_next[13]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i13.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i14 (.D(expected_next[14]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i14.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i15 (.D(expected_next[15]), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .Q(spi_expected_length[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_expected_length_i15.GSR = "ENABLED";
    FD1S3IX mic_divider_1599__i0 (.D(n40_adj_3194), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599__i0.GSR = "DISABLED";
    L6MUX21 i15169 (.D0(n25744), .D1(n25745), .SD(n26246), .Z(n25751));
    L6MUX21 i15170 (.D0(n25746), .D1(n25747), .SD(n26246), .Z(n25752));
    L6MUX21 i15171 (.D0(n25748), .D1(n25749), .SD(n26246), .Z(n25753));
    PFUMX i15129 (.BLUT(n25695), .ALUT(n25696), .C0(spi1_miso_N_2492[1]), 
          .Z(n25711));
    CCU2D status_bit_index_1590_add_4_7 (.A0(status_bit_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24955), .S0(n35_adj_3173), 
          .S1(n34_adj_3172));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590_add_4_7.INIT0 = 16'hfaaa;
    defparam status_bit_index_1590_add_4_7.INIT1 = 16'hfaaa;
    defparam status_bit_index_1590_add_4_7.INJECT1_0 = "NO";
    defparam status_bit_index_1590_add_4_7.INJECT1_1 = "NO";
    CCU2D status_bit_index_1590_add_4_5 (.A0(status_bit_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24954), .COUT(n24955), .S0(n37_adj_3175), 
          .S1(n36_adj_3174));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590_add_4_5.INIT0 = 16'hfaaa;
    defparam status_bit_index_1590_add_4_5.INIT1 = 16'hfaaa;
    defparam status_bit_index_1590_add_4_5.INJECT1_0 = "NO";
    defparam status_bit_index_1590_add_4_5.INJECT1_1 = "NO";
    CCU2D add_244_5 (.A0(spi_byte_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24899), .COUT(n24900), .S0(spi_byte_count_15__N_1504[3]), 
          .S1(spi_byte_count_15__N_1504[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[35:57])
    defparam add_244_5.INIT0 = 16'h5aaa;
    defparam add_244_5.INIT1 = 16'h5aaa;
    defparam add_244_5.INJECT1_0 = "NO";
    defparam add_244_5.INJECT1_1 = "NO";
    PFUMX i15130 (.BLUT(n25697), .ALUT(n25698), .C0(spi1_miso_N_2492[1]), 
          .Z(n25712));
    CCU2D add_779_25 (.A0(phase_frac[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24921), .S0(phase_frac_sum[23]), .S1(phase_frac_sum[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_25.INIT0 = 16'h5aaa;
    defparam add_779_25.INIT1 = 16'h0000;
    defparam add_779_25.INJECT1_0 = "NO";
    defparam add_779_25.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_777_7 (.A0(spi_rgb_payload_byte_N_2547[6]), 
          .B0(spi_extension_length[7]), .C0(GND_net), .D0(GND_net), .A1(spi1_mosi_c_0), 
          .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24888), .COUT(n24889), 
          .S0(expected_next[7]), .S1(expected_next[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[33] 314[86])
    defparam expected_next_15__I_0_777_7.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_777_7.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_777_7.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_777_7.INJECT1_1 = "NO";
    CCU2D add_779_23 (.A0(phase_frac[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24920), .COUT(n24921), .S0(phase_frac_sum[21]), 
          .S1(phase_frac_sum[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_23.INIT0 = 16'h5555;
    defparam add_779_23.INIT1 = 16'h5aaa;
    defparam add_779_23.INJECT1_0 = "NO";
    defparam add_779_23.INJECT1_1 = "NO";
    CCU2D add_244_3 (.A0(spi_byte_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24898), .COUT(n24899), .S0(spi_byte_count_15__N_1504[1]), 
          .S1(spi_byte_count_15__N_1504[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[35:57])
    defparam add_244_3.INIT0 = 16'h5aaa;
    defparam add_244_3.INIT1 = 16'h5aaa;
    defparam add_244_3.INJECT1_0 = "NO";
    defparam add_244_3.INJECT1_1 = "NO";
    CCU2D add_779_21 (.A0(phase_frac[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24919), .COUT(n24920), .S0(phase_frac_sum[19]), 
          .S1(phase_frac_sum[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_21.INIT0 = 16'h5555;
    defparam add_779_21.INIT1 = 16'h5aaa;
    defparam add_779_21.INJECT1_0 = "NO";
    defparam add_779_21.INJECT1_1 = "NO";
    CCU2D status_bit_index_1590_add_4_3 (.A0(status_bit_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24953), .COUT(n24954), .S0(n39_adj_3177), 
          .S1(n38_adj_3176));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590_add_4_3.INIT0 = 16'hfaaa;
    defparam status_bit_index_1590_add_4_3.INIT1 = 16'hfaaa;
    defparam status_bit_index_1590_add_4_3.INJECT1_0 = "NO";
    defparam status_bit_index_1590_add_4_3.INJECT1_1 = "NO";
    CCU2D add_779_19 (.A0(phase_frac[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24918), .COUT(n24919), .S0(phase_frac_sum[17]), 
          .S1(phase_frac_sum[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_19.INIT0 = 16'h5aaa;
    defparam add_779_19.INIT1 = 16'h5aaa;
    defparam add_779_19.INJECT1_0 = "NO";
    defparam add_779_19.INJECT1_1 = "NO";
    CCU2D status_bit_index_1590_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24953), .S1(n40_adj_3178));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590_add_4_1.INIT0 = 16'hF000;
    defparam status_bit_index_1590_add_4_1.INIT1 = 16'h0555;
    defparam status_bit_index_1590_add_4_1.INJECT1_0 = "NO";
    defparam status_bit_index_1590_add_4_1.INJECT1_1 = "NO";
    PFUMX i15131 (.BLUT(n25699), .ALUT(n25700), .C0(spi1_miso_N_2492[1]), 
          .Z(n25713));
    CCU2D global_phase_s2_1595_add_4_9 (.A0(global_phase_s2[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24952), .S0(n38_adj_3195));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595_add_4_9.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1595_add_4_9.INIT1 = 16'h0000;
    defparam global_phase_s2_1595_add_4_9.INJECT1_0 = "NO";
    defparam global_phase_s2_1595_add_4_9.INJECT1_1 = "NO";
    CCU2D equal_2398_13 (.A0(spi_expected_length[11]), .B0(spi_byte_count_15__N_1504[11]), 
          .C0(spi_expected_length[10]), .D0(spi_byte_count_15__N_1504[10]), 
          .A1(spi_expected_length[9]), .B1(spi_byte_count_15__N_1504[9]), 
          .C1(spi_expected_length[8]), .D1(spi_byte_count_15__N_1504[8]), 
          .CIN(n24752), .COUT(n24753));
    defparam equal_2398_13.INIT0 = 16'h9009;
    defparam equal_2398_13.INIT1 = 16'h9009;
    defparam equal_2398_13.INJECT1_0 = "YES";
    defparam equal_2398_13.INJECT1_1 = "YES";
    CCU2D add_779_17 (.A0(phase_frac[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24917), .COUT(n24918), .S0(phase_frac_sum[15]), 
          .S1(phase_frac_sum[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_17.INIT0 = 16'h5555;
    defparam add_779_17.INIT1 = 16'h5aaa;
    defparam add_779_17.INJECT1_0 = "NO";
    defparam add_779_17.INJECT1_1 = "NO";
    CCU2D add_244_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(spi_byte_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24898), .S1(spi_byte_count_15__N_1504[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[35:57])
    defparam add_244_1.INIT0 = 16'hF000;
    defparam add_244_1.INIT1 = 16'h5555;
    defparam add_244_1.INJECT1_0 = "NO";
    defparam add_244_1.INJECT1_1 = "NO";
    CCU2D add_779_15 (.A0(phase_frac[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24916), .COUT(n24917), .S0(phase_frac_sum[13]), 
          .S1(phase_frac_sum[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_15.INIT0 = 16'h5555;
    defparam add_779_15.INIT1 = 16'h5555;
    defparam add_779_15.INJECT1_0 = "NO";
    defparam add_779_15.INJECT1_1 = "NO";
    CCU2D global_phase_s2_1595_add_4_7 (.A0(global_phase_s2[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24951), .COUT(n24952), .S0(n40_adj_3197), 
          .S1(n39_adj_3196));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595_add_4_7.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1595_add_4_7.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1595_add_4_7.INJECT1_0 = "NO";
    defparam global_phase_s2_1595_add_4_7.INJECT1_1 = "NO";
    FD1S3AX mem_1701 (.D(staging_rd_addr_6__N_722[0]), .CK(pll_clk), .Q(n12163));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mem_1701.GSR = "DISABLED";
    CCU2D global_phase_s2_1595_add_4_5 (.A0(global_phase_s2[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24950), .COUT(n24951), .S0(n42), 
          .S1(n41));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595_add_4_5.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1595_add_4_5.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1595_add_4_5.INJECT1_0 = "NO";
    defparam global_phase_s2_1595_add_4_5.INJECT1_1 = "NO";
    OB us_tx_pad_82 (.I(us_tx_c_82), .O(us_tx[82]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    CCU2D add_779_13 (.A0(phase_frac[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24915), .COUT(n24916), .S0(phase_frac_sum[11]), 
          .S1(phase_frac_sum[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_13.INIT0 = 16'h5aaa;
    defparam add_779_13.INIT1 = 16'h5555;
    defparam add_779_13.INJECT1_0 = "NO";
    defparam add_779_13.INJECT1_1 = "NO";
    CCU2D add_781_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24897), 
          .S0(build_sum_8__N_2031[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[32:80])
    defparam add_781_cout.INIT0 = 16'h0000;
    defparam add_781_cout.INIT1 = 16'h0000;
    defparam add_781_cout.INJECT1_0 = "NO";
    defparam add_781_cout.INJECT1_1 = "NO";
    FD1P3AX spi_rgb_index_1593__i0 (.D(n25), .SP(spi1_sck_c_enable_169), 
            .CK(spi1_sck_c), .Q(spi_rgb_index[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(404[46:66])
    defparam spi_rgb_index_1593__i0.GSR = "ENABLED";
    OB us_tx_pad_75 (.I(us_tx_c_75), .O(us_tx[75]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    PFUMX i15132 (.BLUT(n25701), .ALUT(n25702), .C0(spi1_miso_N_2492[1]), 
          .Z(n25714));
    CCU2D add_779_11 (.A0(phase_frac[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24914), .COUT(n24915), .S0(phase_frac_sum[9]), 
          .S1(phase_frac_sum[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_11.INIT0 = 16'h5aaa;
    defparam add_779_11.INIT1 = 16'h5555;
    defparam add_779_11.INJECT1_0 = "NO";
    defparam add_779_11.INJECT1_1 = "NO";
    OB us_tx_pad_83 (.I(us_tx_c_83), .O(us_tx[83]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1S3AX spi_bit_count_1594__i0 (.D(n20), .CK(spi1_sck_c), .Q(spi_bit_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam spi_bit_count_1594__i0.GSR = "ENABLED";
    OBZ spi1_miso_pad (.I(spi1_miso_N_2491), .T(fpga_cs_n_c), .O(spi1_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[12:21])
    CCU2D global_phase_s2_1595_add_4_3 (.A0(global_phase_s2[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24949), .COUT(n24950), .S0(n44), 
          .S1(n43_adj_3199));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595_add_4_3.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1595_add_4_3.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1595_add_4_3.INJECT1_0 = "NO";
    defparam global_phase_s2_1595_add_4_3.INJECT1_1 = "NO";
    FD1P3AX ev_run_hold_s5_i0_i0 (.D(ev_rd_data[0]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i0.GSR = "DISABLED";
    CCU2D expected_next_15__I_0_777_9 (.A0(spi_rx_shift[0]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[1]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24889), .COUT(n24890), .S0(expected_next[9]), 
          .S1(expected_next[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[33] 314[86])
    defparam expected_next_15__I_0_777_9.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_777_9.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_777_9.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_777_9.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_777_5 (.A0(spi_rgb_payload_byte_N_2547[6]), 
          .B0(spi_extension_length[5]), .C0(GND_net), .D0(GND_net), .A1(spi_rgb_payload_byte_N_2547[6]), 
          .B1(spi_extension_length[6]), .C1(GND_net), .D1(GND_net), .CIN(n24887), 
          .COUT(n24888), .S0(expected_next[5]), .S1(expected_next[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[33] 314[86])
    defparam expected_next_15__I_0_777_5.INIT0 = 16'ha999;
    defparam expected_next_15__I_0_777_5.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_777_5.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_777_5.INJECT1_1 = "NO";
    PFUMX i15133 (.BLUT(n25703), .ALUT(n25704), .C0(spi1_miso_N_2492[1]), 
          .Z(n25715));
    CCU2D global_phase_s2_1595_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(global_phase_s2[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24949), .S1(n45));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595_add_4_1.INIT0 = 16'hF000;
    defparam global_phase_s2_1595_add_4_1.INIT1 = 16'h0555;
    defparam global_phase_s2_1595_add_4_1.INJECT1_0 = "NO";
    defparam global_phase_s2_1595_add_4_1.INJECT1_1 = "NO";
    FD1S3AX phase_step_s4_571_rep_318 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(pll_clk_enable_576)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_step_s4_571_rep_318.GSR = "DISABLED";
    FD1P3IX running_568 (.D(n26471), .SP(pll_clk_enable_18), .CD(pll_clk_enable_22), 
            .CK(pll_clk), .Q(status_flags_wire_15__N_1226[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam running_568.GSR = "DISABLED";
    FD1S3AX mem (.D(spi_phase_pending[7]), .CK(spi1_sck_c), .Q(n12210));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem.GSR = "DISABLED";
    FD1S3AX mem_1731 (.D(spi_phase_pending[6]), .CK(spi1_sck_c), .Q(n12208));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1731.GSR = "DISABLED";
    FD1S3AX mem_1730 (.D(spi_phase_pending[5]), .CK(spi1_sck_c), .Q(n12206));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1730.GSR = "DISABLED";
    FD1S3AX mem_1729 (.D(spi_phase_pending[4]), .CK(spi1_sck_c), .Q(n12204));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1729.GSR = "DISABLED";
    FD1S3AX mem_1728 (.D(spi_phase_pending[3]), .CK(spi1_sck_c), .Q(n12202));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1728.GSR = "DISABLED";
    FD1S3AX mem_1727 (.D(spi_phase_pending[2]), .CK(spi1_sck_c), .Q(n12200));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1727.GSR = "DISABLED";
    FD1S3AX mem_1726 (.D(spi_phase_pending[1]), .CK(spi1_sck_c), .Q(n12198));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1726.GSR = "DISABLED";
    FD1S3AX mem_1725 (.D(spi_phase_pending[0]), .CK(spi1_sck_c), .Q(n12196));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1725.GSR = "DISABLED";
    FD1S3AX mem_1724 (.D(spi_rx_shift[6]), .CK(spi1_sck_c), .Q(n12194));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1724.GSR = "DISABLED";
    FD1S3AX mem_1723 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(n12192));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1723.GSR = "DISABLED";
    FD1S3AX mem_1722 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(n12190));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1722.GSR = "DISABLED";
    FD1S3AX mem_1721 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(n12188));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1721.GSR = "DISABLED";
    FD1S3AX mem_1720 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(n12186));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1720.GSR = "DISABLED";
    PFUMX i15134 (.BLUT(n25705), .ALUT(n25706), .C0(spi1_miso_N_2492[1]), 
          .Z(n25716));
    FD1S3AX mem_1719 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(n12184));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1719.GSR = "DISABLED";
    FD1S3AX mem_1718 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(n12182));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1718.GSR = "DISABLED";
    FD1S3AX mem_1717 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(n12180));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1717.GSR = "DISABLED";
    FD1S3AX mem_1716 (.D(spi_write), .CK(pll_clk), .Q(n12177));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mem_1716.GSR = "DISABLED";
    FD1S3AX mem_1703 (.D(staging_rd_addr_6__N_722[1]), .CK(pll_clk), .Q(n12165));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mem_1703.GSR = "DISABLED";
    FD1S3AX spi_rx_shift_i7 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(spi_rx_shift[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_rx_shift_i7.GSR = "ENABLED";
    PFUMX i15311 (.BLUT(n26083), .ALUT(n26082), .C0(ev_state[3]), .Z(ev_state_3__N_602[3]));
    CCU2D add_779_9 (.A0(phase_frac[7]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[8]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24913), .COUT(n24914), .S0(phase_frac_sum[7]), .S1(phase_frac_sum[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_9.INIT0 = 16'h5555;
    defparam add_779_9.INIT1 = 16'h5555;
    defparam add_779_9.INJECT1_0 = "NO";
    defparam add_779_9.INJECT1_1 = "NO";
    CCU2D add_781_8 (.A0(staging_q[14]), .B0(staging_q[6]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[15]), .B1(staging_q[7]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24896), .COUT(n24897), .S0(build_sum_8__N_2031[6]), 
          .S1(build_sum_8__N_2031[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[32:80])
    defparam add_781_8.INIT0 = 16'h5666;
    defparam add_781_8.INIT1 = 16'h5666;
    defparam add_781_8.INJECT1_0 = "NO";
    defparam add_781_8.INJECT1_1 = "NO";
    FD1S3AX spi_rx_shift_i6 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(spi_rx_shift[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_rx_shift_i6.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i5 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(spi_rx_shift[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_rx_shift_i5.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i4 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(spi_rx_shift[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_rx_shift_i4.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i3 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(spi_rx_shift[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_rx_shift_i3.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i2 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(spi_rx_shift[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_rx_shift_i2.GSR = "ENABLED";
    CCU2D add_781_6 (.A0(staging_q[12]), .B0(staging_q[4]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[13]), .B1(staging_q[5]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24895), .COUT(n24896), .S0(build_sum_8__N_2031[4]), 
          .S1(build_sum_8__N_2031[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[32:80])
    defparam add_781_6.INIT0 = 16'h5666;
    defparam add_781_6.INIT1 = 16'h5666;
    defparam add_781_6.INJECT1_0 = "NO";
    defparam add_781_6.INJECT1_1 = "NO";
    CCU2D add_1381_9 (.A0(ev_clear_addr[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24948), .S0(ev_clear_addr_7__N_2215[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(585[26] 587[20])
    defparam add_1381_9.INIT0 = 16'h5aaa;
    defparam add_1381_9.INIT1 = 16'h0000;
    defparam add_1381_9.INJECT1_0 = "NO";
    defparam add_1381_9.INJECT1_1 = "NO";
    PFUMX i15135 (.BLUT(n25707), .ALUT(n25708), .C0(spi1_miso_N_2492[1]), 
          .Z(n25717));
    LUT4 ev_state_2__bdd_4_lut_15357 (.A(ev_state[2]), .B(ev_state[0]), 
         .C(ev_state[1]), .D(ev_state[3]), .Z(ev_we)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A ((C)+!B))) */ ;
    defparam ev_state_2__bdd_4_lut_15357.init = 16'h0424;
    LUT4 mux_1445_i8_3_lut_4_lut_then_3_lut (.A(n26238), .B(build_sum[7]), 
         .C(ev_clear_addr[7]), .Z(n26283)) /* synthesis lut_function=(A (B)+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i8_3_lut_4_lut_then_3_lut.init = 16'hd8d8;
    LUT4 ws2812_settle_3__bdd_4_lut (.A(ws2812_settle[3]), .B(ws2812_settle[1]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(n26042)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;
    defparam ws2812_settle_3__bdd_4_lut.init = 16'haaa9;
    LUT4 i1_3_lut_4_lut_adj_37 (.A(n26210), .B(n26235), .C(init_shadow[77]), 
         .D(n16988), .Z(n14394)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_37.init = 16'hf1f0;
    LUT4 mux_1445_i8_3_lut_4_lut_else_3_lut (.A(build_sum[7]), .B(ev_state[1]), 
         .C(n26252), .D(build_phase[7]), .Z(n26282)) /* synthesis lut_function=(A ((C+(D))+!B)+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i8_3_lut_4_lut_else_3_lut.init = 16'haea2;
    LUT4 i1_3_lut_4_lut_adj_38 (.A(n26211), .B(n26235), .C(init_shadow[78]), 
         .D(n16988), .Z(n14388)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_38.init = 16'hf1f0;
    PFUMX i15164 (.BLUT(n25734), .ALUT(n25735), .C0(spi1_miso_N_2492[1]), 
          .Z(n25746));
    LUT4 mux_1445_i7_3_lut_4_lut_then_3_lut (.A(n26238), .B(build_sum[6]), 
         .C(ev_clear_addr[6]), .Z(n26286)) /* synthesis lut_function=(A (B)+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i7_3_lut_4_lut_then_3_lut.init = 16'hd8d8;
    LUT4 mux_1445_i7_3_lut_4_lut_else_3_lut (.A(build_sum[6]), .B(ev_state[1]), 
         .C(n26252), .D(build_phase[6]), .Z(n26285)) /* synthesis lut_function=(A ((C+(D))+!B)+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i7_3_lut_4_lut_else_3_lut.init = 16'haea2;
    LUT4 i11009_2_lut_rep_251_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(n26215)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i11009_2_lut_rep_251_3_lut.init = 16'h8080;
    LUT4 i14350_2_lut_3_lut_4_lut (.A(n26140), .B(n26126), .C(spi_rgb_index[1]), 
         .D(spi_rgb_index[0]), .Z(n24_adj_3164)) /* synthesis lut_function=(A (C)+!A !(B (C (D)+!C !(D))+!B !(C))) */ ;
    defparam i14350_2_lut_3_lut_4_lut.init = 16'hb4f0;
    LUT4 equal_853_i10_2_lut_rep_292 (.A(ev_ch[3]), .B(ev_ch[4]), .Z(n26256)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam equal_853_i10_2_lut_rep_292.init = 16'heeee;
    LUT4 i15098_4_lut (.A(spi_byte_count[14]), .B(spi_byte_count[11]), .C(spi_byte_count[12]), 
         .D(spi_byte_count[10]), .Z(n25679)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i15098_4_lut.init = 16'hfffe;
    LUT4 mux_1445_i6_3_lut_4_lut_then_3_lut (.A(n26238), .B(build_sum[5]), 
         .C(ev_clear_addr[5]), .Z(n26289)) /* synthesis lut_function=(A (B)+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i6_3_lut_4_lut_then_3_lut.init = 16'hd8d8;
    LUT4 mux_1445_i6_3_lut_4_lut_else_3_lut (.A(build_sum[5]), .B(ev_state[1]), 
         .C(n26252), .D(build_phase[5]), .Z(n26288)) /* synthesis lut_function=(A ((C+(D))+!B)+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i6_3_lut_4_lut_else_3_lut.init = 16'haea2;
    LUT4 equal_846_i12_2_lut_rep_270_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), 
         .C(ev_ch[6]), .D(ev_ch[5]), .Z(n26234)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam equal_846_i12_2_lut_rep_270_3_lut_4_lut.init = 16'hffef;
    FD1P3IX frame_settle__i2 (.D(n26067), .SP(pll_clk_enable_20), .CD(n14125), 
            .CK(pll_clk), .Q(frame_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam frame_settle__i2.GSR = "DISABLED";
    FD1P3IX frame_settle__i1 (.D(n25768), .SP(pll_clk_enable_20), .CD(n14125), 
            .CK(pll_clk), .Q(frame_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam frame_settle__i1.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1595__i0 (.D(n45), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595__i0.GSR = "DISABLED";
    LUT4 i1_4_lut_rep_322 (.A(ev_state[1]), .B(ev_state[3]), .C(ev_state[2]), 
         .D(ev_state[0]), .Z(pll_clk_enable_290)) /* synthesis lut_function=(!(A+(B (C+(D))+!B !(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i1_4_lut_rep_322.init = 16'h1004;
    LUT4 i1_3_lut_4_lut_adj_39 (.A(n26215), .B(n26235), .C(init_shadow[79]), 
         .D(n16988), .Z(n14382)) /* synthesis lut_function=(A (B (C)+!B (C+(D)))+!A (C)) */ ;
    defparam i1_3_lut_4_lut_adj_39.init = 16'hf2f0;
    PFUMX i15136 (.BLUT(n25709), .ALUT(n25710), .C0(spi1_miso_N_2492[1]), 
          .Z(n25718));
    LUT4 run_addr_s3_8__I_0_i1_3_lut (.A(run_addr_s3[0]), .B(ev_rd_slot[0]), 
         .C(n21588), .Z(event_rd_addr[0])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(241[33:74])
    defparam run_addr_s3_8__I_0_i1_3_lut.init = 16'hacac;
    LUT4 mux_1445_i5_3_lut_4_lut_then_3_lut (.A(n26238), .B(build_sum[4]), 
         .C(ev_clear_addr[4]), .Z(n26292)) /* synthesis lut_function=(A (B)+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i5_3_lut_4_lut_then_3_lut.init = 16'hd8d8;
    LUT4 i1_3_lut_4_lut_adj_40 (.A(n26217), .B(n26212), .C(init_shadow[80]), 
         .D(n16988), .Z(n14376)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_40.init = 16'hf1f0;
    LUT4 mux_1445_i5_3_lut_4_lut_else_3_lut (.A(build_sum[4]), .B(ev_state[1]), 
         .C(n26252), .D(build_phase[4]), .Z(n26291)) /* synthesis lut_function=(A ((C+(D))+!B)+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i5_3_lut_4_lut_else_3_lut.init = 16'haea2;
    LUT4 i1_2_lut_rep_289 (.A(ev_state[3]), .B(ev_state[2]), .Z(n26253)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_rep_289.init = 16'h2222;
    LUT4 i6_4_lut_rep_324 (.A(mic_sample_count[3]), .B(n12_adj_3201), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_432)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_rep_324.init = 16'h0800;
    LUT4 i1_4_lut (.A(n17110), .B(n66), .C(fpga_cs_n_c), .D(n21531), 
         .Z(n24_adj_3179)) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A ((D)+!B))) */ ;
    defparam i1_4_lut.init = 16'h0ace;
    LUT4 mux_1445_i4_3_lut_4_lut_then_3_lut (.A(n26238), .B(build_sum[3]), 
         .C(ev_clear_addr[3]), .Z(n26295)) /* synthesis lut_function=(A (B)+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i4_3_lut_4_lut_then_3_lut.init = 16'hd8d8;
    LUT4 mux_1445_i4_3_lut_4_lut_else_3_lut (.A(build_sum[3]), .B(ev_state[1]), 
         .C(n26252), .D(build_phase[3]), .Z(n26294)) /* synthesis lut_function=(A ((C+(D))+!B)+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i4_3_lut_4_lut_else_3_lut.init = 16'haea2;
    FD1P3AX status_bit_index_1590__i0 (.D(n40_adj_3178), .SP(spi1_sck_N_305_enable_7), 
            .CK(spi1_sck_N_305), .Q(status_bit_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590__i0.GSR = "ENABLED";
    LUT4 mux_383_i2_3_lut (.A(pending_sequence[1]), .B(accepted_sequence_sync[1]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i2_3_lut.init = 16'hcaca;
    FD1P3AX fpga_time_1596__i0 (.D(n165), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i0.GSR = "DISABLED";
    PFUMX i15160 (.BLUT(n25726), .ALUT(n25727), .C0(spi1_miso_N_2492[1]), 
          .Z(n25742));
    LUT4 mux_1445_i3_3_lut_4_lut_then_3_lut (.A(n26238), .B(build_sum[2]), 
         .C(ev_clear_addr[2]), .Z(n26298)) /* synthesis lut_function=(A (B)+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i3_3_lut_4_lut_then_3_lut.init = 16'hd8d8;
    FD1P3AX spi_channel_index_1591__i0 (.D(n40_adj_3182), .SP(spi1_sck_c_enable_179), 
            .CK(spi1_sck_c), .Q(spi_channel_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591__i0.GSR = "ENABLED";
    LUT4 mux_1445_i3_3_lut_4_lut_else_3_lut (.A(build_sum[2]), .B(ev_state[1]), 
         .C(n26252), .D(build_phase[2]), .Z(n26297)) /* synthesis lut_function=(A ((C+(D))+!B)+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i3_3_lut_4_lut_else_3_lut.init = 16'haea2;
    LUT4 mux_1445_i2_3_lut_4_lut_then_3_lut (.A(n26238), .B(build_sum[1]), 
         .C(ev_clear_addr[1]), .Z(n26301)) /* synthesis lut_function=(A (B)+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i2_3_lut_4_lut_then_3_lut.init = 16'hd8d8;
    LUT4 i14377_1_lut (.A(spi_bit_count[0]), .Z(n20)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i14377_1_lut.init = 16'h5555;
    FD1P3AX stop_toggle_seen_599 (.D(stop_toggle_sync), .SP(pll_clk_enable_22), 
            .CK(pll_clk), .Q(stop_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam stop_toggle_seen_599.GSR = "DISABLED";
    LUT4 i1_2_lut_4_lut (.A(n17110), .B(n26132), .C(n21554), .D(n3), 
         .Z(n25121)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i1_2_lut_4_lut.init = 16'h0008;
    LUT4 mux_1445_i2_3_lut_4_lut_else_3_lut (.A(build_sum[1]), .B(ev_state[1]), 
         .C(n26252), .D(build_phase[1]), .Z(n26300)) /* synthesis lut_function=(A ((C+(D))+!B)+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i2_3_lut_4_lut_else_3_lut.init = 16'haea2;
    CCU2D add_781_4 (.A0(staging_q[10]), .B0(staging_q[2]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[11]), .B1(staging_q[3]), .C1(GND_net), 
          .D1(GND_net), .CIN(n24894), .COUT(n24895), .S0(build_sum_8__N_2031[2]), 
          .S1(build_sum_8__N_2031[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[32:80])
    defparam add_781_4.INIT0 = 16'h5666;
    defparam add_781_4.INIT1 = 16'h5666;
    defparam add_781_4.INJECT1_0 = "NO";
    defparam add_781_4.INJECT1_1 = "NO";
    LUT4 i1_4_lut_rep_328 (.A(frame_req_N_2584), .B(ev_state[3]), .C(ev_state[2]), 
         .D(n26247), .Z(pll_clk_enable_652)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C)))) */ ;
    defparam i1_4_lut_rep_328.init = 16'h3032;
    LUT4 mux_1445_i1_3_lut_4_lut_then_3_lut (.A(n26238), .B(build_sum[0]), 
         .C(ev_clear_addr[0]), .Z(n26304)) /* synthesis lut_function=(A (B)+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i1_3_lut_4_lut_then_3_lut.init = 16'hd8d8;
    LUT4 mux_1445_i1_3_lut_4_lut_else_3_lut (.A(build_sum[0]), .B(ev_state[1]), 
         .C(n26252), .D(build_phase[0]), .Z(n26303)) /* synthesis lut_function=(A ((C+(D))+!B)+!A !((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam mux_1445_i1_3_lut_4_lut_else_3_lut.init = 16'haea2;
    LUT4 i7295_2_lut_4_lut_rep_330 (.A(frame_req_N_2584), .B(ev_state[3]), 
         .C(ev_state[2]), .D(n26247), .Z(n26487)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i7295_2_lut_4_lut_rep_330.init = 16'h0002;
    LUT4 i1_4_lut_then_4_lut (.A(spi_byte_count[4]), .B(spi_byte_count[5]), 
         .C(spi_byte_count[1]), .D(spi_byte_count[2]), .Z(n26307)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(342[17] 407[24])
    defparam i1_4_lut_then_4_lut.init = 16'h0001;
    LUT4 i1_4_lut_else_4_lut (.A(spi_byte_count[4]), .B(spi_byte_count[5]), 
         .C(spi_byte_count[1]), .D(spi_byte_count[2]), .Z(n26306)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B ((D)+!C)+!B !(C (D)+!C !(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(342[17] 407[24])
    defparam i1_4_lut_else_4_lut.init = 16'h1061;
    LUT4 i1_4_lut_then_4_lut_adj_41 (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_hold[76]), .D(status_bit_index[1]), .Z(n26310)) /* synthesis lut_function=(!((B (D)+!B !(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(130[17:33])
    defparam i1_4_lut_then_4_lut_adj_41.init = 16'h2088;
    LUT4 i1_4_lut_else_4_lut_adj_42 (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_hold[76]), .D(status_bit_index[1]), .Z(n26309)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(130[17:33])
    defparam i1_4_lut_else_4_lut_adj_42.init = 16'h2000;
    LUT4 i14151_4_lut (.A(status_bit_index[3]), .B(n24608), .C(status_bit_index[6]), 
         .D(n25694), .Z(spi1_miso_N_2491)) /* synthesis lut_function=(A (B (C))+!A (B (C+(D))+!B !(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(130[17:33])
    defparam i14151_4_lut.init = 16'hc5c0;
    LUT4 i1_3_lut_4_lut_adj_43 (.A(n26213), .B(n26212), .C(init_shadow[81]), 
         .D(n16988), .Z(n14370)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_43.init = 16'hf1f0;
    LUT4 i14364_3_lut_4_lut (.A(spi_rgb_index[1]), .B(n26121), .C(spi_rgb_index[2]), 
         .D(spi_rgb_index[3]), .Z(n22_adj_3202)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(404[46:66])
    defparam i14364_3_lut_4_lut.init = 16'h7f80;
    LUT4 i5_4_lut (.A(n9), .B(n7), .C(n3), .D(spi_byte_count[15]), .Z(spi_write)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i5_4_lut.init = 16'h0008;
    LUT4 i1_3_lut_4_lut_adj_44 (.A(n26220), .B(n26212), .C(init_shadow[82]), 
         .D(n16988), .Z(n14364)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_44.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_45 (.A(n26211), .B(n26236), .C(init_shadow[30]), 
         .D(n16988), .Z(n14676)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_45.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_46 (.A(n26217), .B(n26239), .C(init_shadow[32]), 
         .D(n16988), .Z(n14664)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_46.init = 16'hf1f0;
    LUT4 wrap_s2_I_0_2_lut_rep_293 (.A(wrap_s2), .B(swap_pending), .Z(pll_clk_enable_18)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(459[13:36])
    defparam wrap_s2_I_0_2_lut_rep_293.init = 16'h8888;
    LUT4 i3745_4_lut (.A(ev_ch[1]), .B(staging_rd_addr[1]), .C(n17798), 
         .D(n25512), .Z(staging_rd_addr_6__N_722[1])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i3745_4_lut.init = 16'hcac0;
    LUT4 i3743_4_lut (.A(ev_ch[2]), .B(staging_rd_addr[2]), .C(n17798), 
         .D(n25512), .Z(staging_rd_addr_6__N_722[2])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i3743_4_lut.init = 16'hcac0;
    LUT4 i1_3_lut_4_lut_adj_47 (.A(n26213), .B(n26239), .C(init_shadow[33]), 
         .D(n16988), .Z(n14658)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_47.init = 16'hf1f0;
    LUT4 spi_channel_field_1__I_0_i3_2_lut (.A(spi_channel_field[0]), .B(spi_channel_field[1]), 
         .Z(n3)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(139[52:79])
    defparam spi_channel_field_1__I_0_i3_2_lut.init = 16'hdddd;
    LUT4 i2949_2_lut_3_lut (.A(wrap_s2), .B(swap_pending), .C(phase_step_s2), 
         .Z(pll_clk_enable_211)) /* synthesis lut_function=(A (B+(C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(459[13:36])
    defparam i2949_2_lut_3_lut.init = 16'hf8f8;
    LUT4 i1_3_lut_4_lut_adj_48 (.A(n26220), .B(n26239), .C(init_shadow[34]), 
         .D(n16988), .Z(n14652)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_48.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_49 (.A(n26215), .B(n26236), .C(init_shadow[31]), 
         .D(n16988), .Z(n14670)) /* synthesis lut_function=(A (B (C)+!B (C+(D)))+!A (C)) */ ;
    defparam i1_3_lut_4_lut_adj_49.init = 16'hf2f0;
    LUT4 active_bank_I_0_708_2_lut_3_lut (.A(wrap_s2), .B(swap_pending), 
         .C(active_bank), .Z(active_bank_N_731)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(459[13:36])
    defparam active_bank_I_0_708_2_lut_3_lut.init = 16'h7878;
    LUT4 i1_2_lut_rep_294 (.A(ev_ch[0]), .B(ev_ch[2]), .Z(n26258)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_294.init = 16'heeee;
    LUT4 i1_2_lut_rep_168 (.A(fpga_cs_n_c), .B(n25529), .Z(n26132)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[17:31])
    defparam i1_2_lut_rep_168.init = 16'h4444;
    LUT4 i1_2_lut_rep_253_3_lut (.A(ev_ch[0]), .B(ev_ch[2]), .C(ev_ch[1]), 
         .Z(n26217)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_253_3_lut.init = 16'hfefe;
    LUT4 i1_2_lut_rep_256_3_lut (.A(ev_ch[0]), .B(ev_ch[2]), .C(ev_ch[1]), 
         .Z(n26220)) /* synthesis lut_function=(A+(B+!(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_256_3_lut.init = 16'hefef;
    LUT4 i2_3_lut_rep_160_4_lut (.A(fpga_cs_n_c), .B(n25529), .C(n21554), 
         .D(n17110), .Z(spi1_sck_c_enable_173)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[17:31])
    defparam i2_3_lut_rep_160_4_lut.init = 16'h0400;
    FD1P3AX ws2812_toggle_seen_597 (.D(ws2812_toggle_sync), .SP(pll_clk_enable_23), 
            .CK(pll_clk), .Q(ws2812_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ws2812_toggle_seen_597.GSR = "DISABLED";
    FD1P3AX frame_toggle_seen_591 (.D(frame_toggle_sync), .SP(pll_clk_enable_24), 
            .CK(pll_clk), .Q(frame_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam frame_toggle_seen_591.GSR = "DISABLED";
    CCU2D add_779_7 (.A0(phase_frac[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24912), .COUT(n24913), .S0(phase_frac_sum[5]), .S1(phase_frac_sum[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_7.INIT0 = 16'h5aaa;
    defparam add_779_7.INIT1 = 16'h5555;
    defparam add_779_7.INJECT1_0 = "NO";
    defparam add_779_7.INJECT1_1 = "NO";
    FD1P3AX invalid_frame_spi_556 (.D(invalid_frame_spi_N_2522), .SP(spi1_sck_c_enable_35), 
            .CK(spi1_sck_c), .Q(invalid_frame_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam invalid_frame_spi_556.GSR = "DISABLED";
    LUT4 i1770_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n26237), .C(ev_ch[5]), 
         .D(ev_ch[4]), .Z(ev_ch_6__N_2010[5])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(609[27:39])
    defparam i1770_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i1_3_lut_4_lut_adj_50 (.A(n26208), .B(n26239), .C(init_shadow[35]), 
         .D(n16988), .Z(n14646)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_50.init = 16'hf1f0;
    LUT4 i1_4_lut_4_lut (.A(ev_state[2]), .B(n21588), .C(frame_req_N_2584), 
         .D(ev_state[1]), .Z(n18_adj_3209)) /* synthesis lut_function=(!(A ((D)+!B)+!A (C+(D)))) */ ;
    defparam i1_4_lut_4_lut.init = 16'h008d;
    LUT4 i38_4_lut_4_lut (.A(ev_state[2]), .B(n21588), .C(ev_state[1]), 
         .D(n26207), .Z(n20_adj_3208)) /* synthesis lut_function=(A (B (C))+!A !(C+(D))) */ ;
    defparam i38_4_lut_4_lut.init = 16'h8085;
    LUT4 i1_2_lut_rep_296 (.A(ev_ch[5]), .B(ev_ch[6]), .Z(n26260)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_296.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_51 (.A(n26210), .B(n26239), .C(init_shadow[37]), 
         .D(n16988), .Z(n14634)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_51.init = 16'hf1f0;
    FD1P3AX frame_toggle_spi_553 (.D(frame_toggle_spi_N_2505), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(frame_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam frame_toggle_spi_553.GSR = "DISABLED";
    LUT4 mux_383_i3_3_lut (.A(pending_sequence[2]), .B(accepted_sequence_sync[2]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i3_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_rep_169 (.A(ev_state[2]), .B(n26134), .C(ev_state[0]), 
         .D(ev_state[3]), .Z(pll_clk_enable_684)) /* synthesis lut_function=(!(A+!(B (C+!(D))+!B (C (D))))) */ ;
    defparam i1_4_lut_rep_169.init = 16'h5044;
    LUT4 i1_3_lut_4_lut_adj_52 (.A(n26209), .B(n26239), .C(init_shadow[36]), 
         .D(n16988), .Z(n14640)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_52.init = 16'hf1f0;
    LUT4 i7251_2_lut_3_lut (.A(ev_state[2]), .B(n26134), .C(ev_state[3]), 
         .Z(n17880)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i7251_2_lut_3_lut.init = 16'h0404;
    LUT4 mux_383_i4_3_lut (.A(pending_sequence[3]), .B(accepted_sequence_sync[3]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i4_3_lut.init = 16'hcaca;
    LUT4 mux_383_i5_3_lut (.A(pending_sequence[4]), .B(accepted_sequence_sync[4]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i5_3_lut.init = 16'hcaca;
    LUT4 i3741_4_lut (.A(ev_ch[3]), .B(staging_rd_addr[3]), .C(n17798), 
         .D(n25512), .Z(staging_rd_addr_6__N_722[3])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i3741_4_lut.init = 16'hcac0;
    LUT4 mux_383_i6_3_lut (.A(pending_sequence[5]), .B(accepted_sequence_sync[5]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i6_3_lut.init = 16'hcaca;
    LUT4 mux_383_i7_3_lut (.A(pending_sequence[6]), .B(accepted_sequence_sync[6]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i7_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_53 (.A(ev_state[1]), .B(n26142), .C(ev_state[3]), 
         .D(ev_state[2]), .Z(pll_clk_enable_9)) /* synthesis lut_function=(!(A ((D)+!C)+!A (B (D)+!B ((D)+!C)))) */ ;
    defparam i1_3_lut_4_lut_adj_53.init = 16'h00f4;
    LUT4 i1_3_lut_4_lut_adj_54 (.A(n26215), .B(n26239), .C(init_shadow[39]), 
         .D(n16988), .Z(n14622)) /* synthesis lut_function=(A (B (C)+!B (C+(D)))+!A (C)) */ ;
    defparam i1_3_lut_4_lut_adj_54.init = 16'hf2f0;
    LUT4 i3739_4_lut (.A(ev_ch[4]), .B(staging_rd_addr[4]), .C(n17798), 
         .D(n25512), .Z(staging_rd_addr_6__N_722[4])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i3739_4_lut.init = 16'hcac0;
    LUT4 equal_789_i12_2_lut_rep_250_3_lut_4_lut (.A(ev_ch[5]), .B(ev_ch[6]), 
         .C(ev_ch[4]), .D(ev_ch[3]), .Z(n26214)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam equal_789_i12_2_lut_rep_250_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_55 (.A(n26217), .B(n26240), .C(init_shadow[40]), 
         .D(n16988), .Z(n14616)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_55.init = 16'hf1f0;
    CCU2D add_1381_7 (.A0(ev_clear_addr[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24947), .COUT(n24948), .S0(ev_clear_addr_7__N_2215[5]), 
          .S1(ev_clear_addr_7__N_2215[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(585[26] 587[20])
    defparam add_1381_7.INIT0 = 16'h5aaa;
    defparam add_1381_7.INIT1 = 16'h5aaa;
    defparam add_1381_7.INJECT1_0 = "NO";
    defparam add_1381_7.INJECT1_1 = "NO";
    CCU2D add_1381_5 (.A0(ev_clear_addr[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24946), .COUT(n24947), .S0(ev_clear_addr_7__N_2215[3]), 
          .S1(ev_clear_addr_7__N_2215[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(585[26] 587[20])
    defparam add_1381_5.INIT0 = 16'h5aaa;
    defparam add_1381_5.INIT1 = 16'h5aaa;
    defparam add_1381_5.INJECT1_0 = "NO";
    defparam add_1381_5.INJECT1_1 = "NO";
    LUT4 mux_383_i8_3_lut (.A(pending_sequence[7]), .B(accepted_sequence_sync[7]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i8_3_lut.init = 16'hcaca;
    LUT4 mux_383_i9_3_lut (.A(pending_sequence[8]), .B(accepted_sequence_sync[8]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i9_3_lut.init = 16'hcaca;
    LUT4 mux_383_i10_3_lut (.A(pending_sequence[9]), .B(accepted_sequence_sync[9]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i10_3_lut.init = 16'hcaca;
    FD1P3AX spi_version_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_version_i0_i1.GSR = "ENABLED";
    CCU2D add_1381_3 (.A0(ev_clear_addr[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24945), .COUT(n24946), .S0(ev_clear_addr_7__N_2215[1]), 
          .S1(ev_clear_addr_7__N_2215[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(585[26] 587[20])
    defparam add_1381_3.INIT0 = 16'h5aaa;
    defparam add_1381_3.INIT1 = 16'h5aaa;
    defparam add_1381_3.INJECT1_0 = "NO";
    defparam add_1381_3.INJECT1_1 = "NO";
    CCU2D add_1381_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n25477), .B1(n4_adj_3211), .C1(ev_clear_addr[0]), .D1(GND_net), 
          .COUT(n24945), .S1(ev_clear_addr_7__N_2215[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(585[26] 587[20])
    defparam add_1381_1.INIT0 = 16'hF000;
    defparam add_1381_1.INIT1 = 16'h8787;
    defparam add_1381_1.INJECT1_0 = "NO";
    defparam add_1381_1.INJECT1_1 = "NO";
    LUT4 mux_383_i11_3_lut (.A(pending_sequence[10]), .B(accepted_sequence_sync[10]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i11_3_lut.init = 16'hcaca;
    CCU2D add_14301_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24985), 
          .S0(n11277));
    defparam add_14301_cout.INIT0 = 16'h0000;
    defparam add_14301_cout.INIT1 = 16'h0000;
    defparam add_14301_cout.INJECT1_0 = "NO";
    defparam add_14301_cout.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_56 (.A(n26211), .B(n26239), .C(init_shadow[38]), 
         .D(n16988), .Z(n14628)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_56.init = 16'hf1f0;
    CCU2D add_14301_17 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24984), 
          .COUT(n24985));
    defparam add_14301_17.INIT0 = 16'hffff;
    defparam add_14301_17.INIT1 = 16'hffff;
    defparam add_14301_17.INJECT1_0 = "NO";
    defparam add_14301_17.INJECT1_1 = "NO";
    LUT4 i2_3_lut_4_lut_rep_320 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(status_flags_wire_15__N_1226[4]), .D(phase_step_s5), .Z(pll_clk_enable_115)) /* synthesis lut_function=(A (((D)+!C)+!B)+!A (B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(274[23:61])
    defparam i2_3_lut_4_lut_rep_320.init = 16'hff6f;
    LUT4 i1_3_lut_4_lut_adj_57 (.A(n26220), .B(n26240), .C(init_shadow[42]), 
         .D(n16988), .Z(n14604)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_57.init = 16'hf1f0;
    LUT4 equal_796_i10_2_lut_rep_297 (.A(ev_ch[3]), .B(ev_ch[4]), .Z(n26261)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam equal_796_i10_2_lut_rep_297.init = 16'hdddd;
    LUT4 i1_2_lut_rep_255_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(n26219)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_255_3_lut_4_lut.init = 16'hfffd;
    LUT4 i1_2_lut_rep_271_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(n26235)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_271_3_lut_4_lut.init = 16'hffdf;
    LUT4 i1_3_lut_4_lut_adj_58 (.A(n26210), .B(n26240), .C(init_shadow[45]), 
         .D(n16988), .Z(n14586)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_58.init = 16'hf1f0;
    LUT4 i10921_4_lut_4_lut (.A(ev_state[2]), .B(ev_state[0]), .C(n26144), 
         .D(n34_adj_3187), .Z(n14_adj_3219)) /* synthesis lut_function=(!(A+!(B (C)+!B (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam i10921_4_lut_4_lut.init = 16'h5140;
    LUT4 mux_383_i12_3_lut (.A(pending_sequence[11]), .B(accepted_sequence_sync[11]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i12_3_lut.init = 16'hcaca;
    LUT4 mux_383_i13_3_lut (.A(pending_sequence[12]), .B(accepted_sequence_sync[12]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i13_3_lut.init = 16'hcaca;
    LUT4 i2_3_lut_rep_298 (.A(ws2812_settle[1]), .B(ws2812_settle[3]), .C(ws2812_settle[2]), 
         .Z(n26262)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(539[22:43])
    defparam i2_3_lut_rep_298.init = 16'hfefe;
    LUT4 i1_2_lut_rep_258_4_lut (.A(ws2812_settle[1]), .B(ws2812_settle[3]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(n26222)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(539[22:43])
    defparam i1_2_lut_rep_258_4_lut.init = 16'hfeff;
    LUT4 mux_383_i14_3_lut (.A(pending_sequence[13]), .B(accepted_sequence_sync[13]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i14_3_lut.init = 16'hcaca;
    LUT4 mux_383_i15_3_lut (.A(pending_sequence[14]), .B(accepted_sequence_sync[14]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i15_3_lut.init = 16'hcaca;
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
    LUT4 mux_383_i16_3_lut (.A(pending_sequence[15]), .B(accepted_sequence_sync[15]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i16_3_lut.init = 16'hcaca;
    LUT4 mux_383_i17_3_lut (.A(pending_sequence[16]), .B(accepted_sequence_sync[16]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i17_3_lut.init = 16'hcaca;
    LUT4 mux_383_i18_3_lut (.A(pending_sequence[17]), .B(accepted_sequence_sync[17]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[17])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i18_3_lut.init = 16'hcaca;
    FD1P3AX spi_version_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_version_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_version_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_version_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_version_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_version_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_version_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_update_flags_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(expected_next_15__N_1351[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_update_flags_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(expected_next[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i1.GSR = "ENABLED";
    CCU2D add_14301_15 (.A0(spi_expected_length[14]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[15]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24983), .COUT(n24984));
    defparam add_14301_15.INIT0 = 16'hf555;
    defparam add_14301_15.INIT1 = 16'hf555;
    defparam add_14301_15.INJECT1_0 = "NO";
    defparam add_14301_15.INJECT1_1 = "NO";
    LUT4 mux_383_i19_3_lut (.A(pending_sequence[18]), .B(accepted_sequence_sync[18]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[18])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i19_3_lut.init = 16'hcaca;
    FD1P3AX spi_extension_length_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_extension_length[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_extension_length[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_extension_length[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_extension_length[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_extension_length[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_extension_length[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_extension_length[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_extension_length[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_extension_length_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i16.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i17.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i18.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i19.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i20.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i21.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i22.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i23.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i24 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i24.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i25 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i25.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i26 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i26.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i27 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i27.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i28 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i28.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i29 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i29.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i30 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i30.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i31 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_frame_sequence_i0_i31.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_phase_pending_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_phase_pending_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_phase_pending_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_phase_pending_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_phase_pending_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_phase_pending_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_phase_pending_i0_i7.GSR = "ENABLED";
    FD1P3AX rgb_values_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(rgb_values[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i1.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(rgb_values[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i2.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(rgb_values[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i3.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(rgb_values[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i4.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(rgb_values[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i5.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(rgb_values[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i6.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(rgb_values[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i7.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_112), 
            .CK(spi1_sck_c), .Q(rgb_values[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i8.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_112), 
            .CK(spi1_sck_c), .Q(rgb_values[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i9.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_112), 
            .CK(spi1_sck_c), .Q(rgb_values[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i10.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_112), 
            .CK(spi1_sck_c), .Q(rgb_values[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i11.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_112), 
            .CK(spi1_sck_c), .Q(rgb_values[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i12.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_112), 
            .CK(spi1_sck_c), .Q(rgb_values[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i13.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_112), 
            .CK(spi1_sck_c), .Q(rgb_values[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i14.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_112), 
            .CK(spi1_sck_c), .Q(rgb_values[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i15.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_120), 
            .CK(spi1_sck_c), .Q(rgb_values[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i16.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_120), 
            .CK(spi1_sck_c), .Q(rgb_values[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i17.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_120), 
            .CK(spi1_sck_c), .Q(rgb_values[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i18.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_120), 
            .CK(spi1_sck_c), .Q(rgb_values[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i19.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_120), 
            .CK(spi1_sck_c), .Q(rgb_values[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i20.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_120), 
            .CK(spi1_sck_c), .Q(rgb_values[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i21.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_120), 
            .CK(spi1_sck_c), .Q(rgb_values[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i22.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_120), 
            .CK(spi1_sck_c), .Q(rgb_values[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam rgb_values_i0_i23.GSR = "DISABLED";
    CCU2D add_14301_13 (.A0(spi_expected_length[12]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[13]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24982), .COUT(n24983));
    defparam add_14301_13.INIT0 = 16'hf555;
    defparam add_14301_13.INIT1 = 16'hf555;
    defparam add_14301_13.INJECT1_0 = "NO";
    defparam add_14301_13.INJECT1_1 = "NO";
    CCU2D add_14301_11 (.A0(spi_expected_length[10]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[11]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24981), .COUT(n24982));
    defparam add_14301_11.INIT0 = 16'hf555;
    defparam add_14301_11.INIT1 = 16'hf555;
    defparam add_14301_11.INJECT1_0 = "NO";
    defparam add_14301_11.INJECT1_1 = "NO";
    LUT4 mux_383_i20_3_lut (.A(pending_sequence[19]), .B(accepted_sequence_sync[19]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[19])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i20_3_lut.init = 16'hcaca;
    CCU2D add_14301_9 (.A0(spi_expected_length[8]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[9]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24980), .COUT(n24981));
    defparam add_14301_9.INIT0 = 16'hf555;
    defparam add_14301_9.INIT1 = 16'hf555;
    defparam add_14301_9.INJECT1_0 = "NO";
    defparam add_14301_9.INJECT1_1 = "NO";
    LUT4 i5_3_lut_rep_164_4_lut (.A(n26150), .B(n26272), .C(spi_byte_count[0]), 
         .D(n10_adj_3207), .Z(n26128)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;
    defparam i5_3_lut_rep_164_4_lut.init = 16'hffef;
    LUT4 spi_byte_count_7__bdd_4_lut (.A(spi_byte_count[7]), .B(n26227), 
         .C(n86), .D(spi_byte_count[6]), .Z(n26120)) /* synthesis lut_function=(A (C+!(D))+!A (B (C+!(D))+!B (C (D)))) */ ;
    defparam spi_byte_count_7__bdd_4_lut.init = 16'hf0ee;
    LUT4 mux_383_i21_3_lut (.A(pending_sequence[20]), .B(accepted_sequence_sync[20]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[20])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i21_3_lut.init = 16'hcaca;
    LUT4 mux_383_i22_3_lut (.A(pending_sequence[21]), .B(accepted_sequence_sync[21]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[21])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i22_3_lut.init = 16'hcaca;
    LUT4 mux_383_i23_3_lut (.A(pending_sequence[22]), .B(accepted_sequence_sync[22]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[22])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i23_3_lut.init = 16'hcaca;
    LUT4 mux_383_i24_3_lut (.A(pending_sequence[23]), .B(accepted_sequence_sync[23]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[23])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i24_3_lut.init = 16'hcaca;
    LUT4 i6564_2_lut_4_lut (.A(ws2812_settle[1]), .B(ws2812_settle[3]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(n17190)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(539[22:43])
    defparam i6564_2_lut_4_lut.init = 16'h00fe;
    FD1P3AX mic_sample_count_1598__i1 (.D(n29), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_sample_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam mic_sample_count_1598__i1.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1598__i2 (.D(n28), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_sample_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam mic_sample_count_1598__i2.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1598__i3 (.D(n27), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_sample_count[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam mic_sample_count_1598__i3.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1598__i4 (.D(n26), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_sample_count[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam mic_sample_count_1598__i4.GSR = "DISABLED";
    FD1S3AX phase_frac_i1 (.D(phase_frac_sum[1]), .CK(pll_clk), .Q(phase_frac[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i1.GSR = "DISABLED";
    CCU2D add_14301_7 (.A0(spi_expected_length[6]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[7]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24979), .COUT(n24980));
    defparam add_14301_7.INIT0 = 16'hf555;
    defparam add_14301_7.INIT1 = 16'hf555;
    defparam add_14301_7.INJECT1_0 = "NO";
    defparam add_14301_7.INJECT1_1 = "NO";
    CCU2D add_779_5 (.A0(phase_frac[3]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24911), .COUT(n24912), .S0(phase_frac_sum[3]), .S1(phase_frac_sum[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_5.INIT0 = 16'h5aaa;
    defparam add_779_5.INIT1 = 16'h5aaa;
    defparam add_779_5.INJECT1_0 = "NO";
    defparam add_779_5.INJECT1_1 = "NO";
    FD1S3AX phase_frac_i2 (.D(phase_frac_sum[2]), .CK(pll_clk), .Q(phase_frac[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i2.GSR = "DISABLED";
    FD1S3AX phase_frac_i3 (.D(phase_frac_sum[3]), .CK(pll_clk), .Q(phase_frac[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i3.GSR = "DISABLED";
    FD1S3AX phase_frac_i4 (.D(phase_frac_sum[4]), .CK(pll_clk), .Q(phase_frac[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i4.GSR = "DISABLED";
    FD1S3AX phase_frac_i5 (.D(phase_frac_sum[5]), .CK(pll_clk), .Q(phase_frac[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i5.GSR = "DISABLED";
    FD1S3AX phase_frac_i6 (.D(phase_frac_sum[6]), .CK(pll_clk), .Q(phase_frac[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i6.GSR = "DISABLED";
    FD1S3AX phase_frac_i7 (.D(phase_frac_sum[7]), .CK(pll_clk), .Q(phase_frac[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i7.GSR = "DISABLED";
    FD1S3AX phase_frac_i8 (.D(phase_frac_sum[8]), .CK(pll_clk), .Q(phase_frac[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i8.GSR = "DISABLED";
    FD1S3AX phase_frac_i9 (.D(phase_frac_sum[9]), .CK(pll_clk), .Q(phase_frac[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i9.GSR = "DISABLED";
    FD1S3AX phase_frac_i10 (.D(phase_frac_sum[10]), .CK(pll_clk), .Q(phase_frac[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i10.GSR = "DISABLED";
    FD1S3AX phase_frac_i11 (.D(phase_frac_sum[11]), .CK(pll_clk), .Q(phase_frac[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i11.GSR = "DISABLED";
    FD1S3AX phase_frac_i12 (.D(phase_frac_sum[12]), .CK(pll_clk), .Q(phase_frac[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i12.GSR = "DISABLED";
    FD1S3AX phase_frac_i13 (.D(phase_frac_sum[13]), .CK(pll_clk), .Q(phase_frac[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i13.GSR = "DISABLED";
    FD1S3AX phase_frac_i14 (.D(phase_frac_sum[14]), .CK(pll_clk), .Q(phase_frac[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i14.GSR = "DISABLED";
    FD1S3AX phase_frac_i15 (.D(phase_frac_sum[15]), .CK(pll_clk), .Q(phase_frac[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i15.GSR = "DISABLED";
    FD1S3AX phase_frac_i16 (.D(phase_frac_sum[16]), .CK(pll_clk), .Q(phase_frac[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i16.GSR = "DISABLED";
    FD1S3AX phase_frac_i17 (.D(phase_frac_sum[17]), .CK(pll_clk), .Q(phase_frac[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i17.GSR = "DISABLED";
    FD1S3AX phase_frac_i18 (.D(phase_frac_sum[18]), .CK(pll_clk), .Q(phase_frac[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i18.GSR = "DISABLED";
    FD1S3AX phase_frac_i19 (.D(phase_frac_sum[19]), .CK(pll_clk), .Q(phase_frac[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i19.GSR = "DISABLED";
    FD1S3AX phase_frac_i20 (.D(phase_frac_sum[20]), .CK(pll_clk), .Q(phase_frac[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i20.GSR = "DISABLED";
    FD1S3AX phase_frac_i21 (.D(phase_frac_sum[21]), .CK(pll_clk), .Q(phase_frac[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i21.GSR = "DISABLED";
    FD1S3AX phase_frac_i22 (.D(phase_frac_sum[22]), .CK(pll_clk), .Q(phase_frac[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i22.GSR = "DISABLED";
    FD1S3AX phase_frac_i23 (.D(phase_frac_sum[23]), .CK(pll_clk), .Q(phase_frac[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_frac_i23.GSR = "DISABLED";
    FD1P3AX rgb_hold__i2 (.D(rgb_values[1]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i2.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_4_lut (.A(n26150), .B(n26272), .C(n26265), .D(n26227), 
         .Z(n7)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C (D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut.init = 16'hf0e0;
    LUT4 mux_383_i25_3_lut (.A(pending_sequence[24]), .B(accepted_sequence_sync[24]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[24])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i25_3_lut.init = 16'hcaca;
    FD1P3AX rgb_hold__i3 (.D(rgb_values[2]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i3.GSR = "DISABLED";
    FD1P3AX rgb_hold__i4 (.D(rgb_values[3]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i4.GSR = "DISABLED";
    FD1P3AX rgb_hold__i5 (.D(rgb_values[4]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i5.GSR = "DISABLED";
    FD1P3AX rgb_hold__i6 (.D(rgb_values[5]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i6.GSR = "DISABLED";
    FD1P3AX rgb_hold__i7 (.D(rgb_values[6]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i7.GSR = "DISABLED";
    FD1P3AX rgb_hold__i8 (.D(rgb_values[7]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i8.GSR = "DISABLED";
    FD1P3AX rgb_hold__i9 (.D(rgb_values[8]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i9.GSR = "DISABLED";
    FD1P3AX rgb_hold__i10 (.D(rgb_values[9]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i10.GSR = "DISABLED";
    FD1P3AX rgb_hold__i11 (.D(rgb_values[10]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i11.GSR = "DISABLED";
    FD1P3AX rgb_hold__i12 (.D(rgb_values[11]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i12.GSR = "DISABLED";
    FD1P3AX rgb_hold__i13 (.D(rgb_values[12]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i13.GSR = "DISABLED";
    FD1P3AX rgb_hold__i14 (.D(rgb_values[13]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i14.GSR = "DISABLED";
    FD1P3AX rgb_hold__i15 (.D(rgb_values[14]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i15.GSR = "DISABLED";
    FD1P3AX rgb_hold__i16 (.D(rgb_values[15]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i16.GSR = "DISABLED";
    FD1P3AX rgb_hold__i17 (.D(rgb_values[16]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i17.GSR = "DISABLED";
    FD1P3AX rgb_hold__i18 (.D(rgb_values[17]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i18.GSR = "DISABLED";
    FD1P3AX rgb_hold__i19 (.D(rgb_values[18]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i19.GSR = "DISABLED";
    FD1P3AX rgb_hold__i20 (.D(rgb_values[19]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i20.GSR = "DISABLED";
    FD1P3AX rgb_hold__i21 (.D(rgb_values[20]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i21.GSR = "DISABLED";
    FD1P3AX rgb_hold__i22 (.D(rgb_values[21]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i22.GSR = "DISABLED";
    FD1P3AX rgb_hold__i23 (.D(rgb_values[22]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i23.GSR = "DISABLED";
    FD1P3AX rgb_hold__i24 (.D(rgb_values[23]), .SP(pll_clk_enable_66), .CK(pll_clk), 
            .Q(rgb_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam rgb_hold__i24.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i1 (.D(spi_byte_count_15__N_1504[1]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i2 (.D(spi_byte_count_15__N_1504[2]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i3 (.D(spi_byte_count_15__N_1504[3]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i4 (.D(spi_byte_count_15__N_1504[4]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i5 (.D(spi_byte_count_15__N_1504[5]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i6 (.D(spi_byte_count_15__N_1504[6]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i7 (.D(spi_byte_count_15__N_1504[7]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i8 (.D(spi_byte_count_15__N_1504[8]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i9 (.D(spi_byte_count_15__N_1504[9]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i10 (.D(spi_byte_count_15__N_1504[10]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i11 (.D(spi_byte_count_15__N_1504[11]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i12 (.D(spi_byte_count_15__N_1504[12]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i13 (.D(spi_byte_count_15__N_1504[13]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i14 (.D(spi_byte_count_15__N_1504[14]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i15 (.D(spi_byte_count_15__N_1504[15]), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .Q(spi_byte_count[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam spi_byte_count_i0_i15.GSR = "ENABLED";
    FD1P3IX us_tx__i2 (.D(n13728), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i2.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut (.A(spi_byte_count[8]), .B(n26120), .C(spi_rgb_payload_byte_N_2547[6]), 
         .Z(n17110)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut.init = 16'h4040;
    LUT4 i1_2_lut_rep_259_4_lut (.A(ws2812_settle[1]), .B(ws2812_settle[3]), 
         .C(ws2812_settle[2]), .D(ws2812_settle[0]), .Z(n26223)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(539[22:43])
    defparam i1_2_lut_rep_259_4_lut.init = 16'hfffe;
    FD1P3IX us_tx__i3 (.D(n13730), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i3.GSR = "DISABLED";
    FD1P3IX us_tx__i4 (.D(n13732), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i4.GSR = "DISABLED";
    FD1P3IX us_tx__i5 (.D(n13734), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i5.GSR = "DISABLED";
    FD1P3IX us_tx__i6 (.D(n13736), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i6.GSR = "DISABLED";
    FD1P3IX us_tx__i7 (.D(n13738), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_6)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i7.GSR = "DISABLED";
    FD1P3IX us_tx__i8 (.D(n13740), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_7)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i8.GSR = "DISABLED";
    FD1P3IX us_tx__i9 (.D(n13742), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_8)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i9.GSR = "DISABLED";
    FD1P3IX us_tx__i10 (.D(n13744), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_9)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i10.GSR = "DISABLED";
    FD1P3IX us_tx__i11 (.D(n13746), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_10)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i11.GSR = "DISABLED";
    FD1P3IX us_tx__i12 (.D(n13748), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_11)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i12.GSR = "DISABLED";
    FD1P3IX us_tx__i13 (.D(n13750), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_12)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i13.GSR = "DISABLED";
    FD1P3IX us_tx__i14 (.D(n13752), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_13)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i14.GSR = "DISABLED";
    FD1P3IX us_tx__i15 (.D(n13754), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_14)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i15.GSR = "DISABLED";
    FD1P3IX us_tx__i16 (.D(n13756), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_15)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i16.GSR = "DISABLED";
    FD1P3IX us_tx__i17 (.D(n13758), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_16)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i17.GSR = "DISABLED";
    FD1P3IX us_tx__i18 (.D(n13760), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_17)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i18.GSR = "DISABLED";
    FD1P3IX us_tx__i19 (.D(n13762), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_18)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i19.GSR = "DISABLED";
    FD1P3IX us_tx__i20 (.D(n13764), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_19)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i20.GSR = "DISABLED";
    FD1P3IX us_tx__i21 (.D(n13766), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_20)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i21.GSR = "DISABLED";
    FD1P3IX us_tx__i22 (.D(n13768), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_21)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i22.GSR = "DISABLED";
    FD1P3IX us_tx__i23 (.D(n13770), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_22)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i23.GSR = "DISABLED";
    FD1P3IX us_tx__i24 (.D(n13772), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_23)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i24.GSR = "DISABLED";
    FD1P3IX us_tx__i25 (.D(n13774), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_24)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i25.GSR = "DISABLED";
    FD1P3IX us_tx__i26 (.D(n13776), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_25)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i26.GSR = "DISABLED";
    FD1P3IX us_tx__i27 (.D(n13778), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_26)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i27.GSR = "DISABLED";
    FD1P3IX us_tx__i28 (.D(n13780), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_27)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i28.GSR = "DISABLED";
    FD1P3IX us_tx__i29 (.D(n13782), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_28)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i29.GSR = "DISABLED";
    FD1P3IX us_tx__i30 (.D(n13784), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_29)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i30.GSR = "DISABLED";
    FD1P3IX us_tx__i31 (.D(n13786), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_30)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i31.GSR = "DISABLED";
    FD1P3IX us_tx__i32 (.D(n13788), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_31)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i32.GSR = "DISABLED";
    FD1P3IX us_tx__i33 (.D(n13790), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_32)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i33.GSR = "DISABLED";
    FD1P3IX us_tx__i34 (.D(n13792), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_33)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i34.GSR = "DISABLED";
    FD1P3IX us_tx__i35 (.D(n13794), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_34)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i35.GSR = "DISABLED";
    FD1P3IX us_tx__i36 (.D(n13796), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_35)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i36.GSR = "DISABLED";
    FD1P3IX us_tx__i37 (.D(n13798), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_36)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i37.GSR = "DISABLED";
    FD1P3IX us_tx__i38 (.D(n13800), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_37)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i38.GSR = "DISABLED";
    FD1P3IX us_tx__i39 (.D(n13802), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_38)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i39.GSR = "DISABLED";
    FD1P3IX us_tx__i40 (.D(n13804), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_39)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i40.GSR = "DISABLED";
    FD1P3IX us_tx__i41 (.D(n13806), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_40)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i41.GSR = "DISABLED";
    FD1P3IX us_tx__i42 (.D(n13808), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_41)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i42.GSR = "DISABLED";
    FD1P3IX us_tx__i43 (.D(n13810), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_42)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i43.GSR = "DISABLED";
    FD1P3IX us_tx__i44 (.D(n13812), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_43)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i44.GSR = "DISABLED";
    FD1P3IX us_tx__i45 (.D(n13814), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_44)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i45.GSR = "DISABLED";
    FD1P3IX us_tx__i46 (.D(n13816), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_45)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i46.GSR = "DISABLED";
    FD1P3IX us_tx__i47 (.D(n13818), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_46)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i47.GSR = "DISABLED";
    FD1P3IX us_tx__i48 (.D(n13820), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_47)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i48.GSR = "DISABLED";
    FD1P3IX us_tx__i49 (.D(n13822), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_48)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i49.GSR = "DISABLED";
    FD1P3IX us_tx__i50 (.D(n13824), .SP(pll_clk_enable_115), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_49)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i50.GSR = "DISABLED";
    FD1P3IX us_tx__i51 (.D(n13826), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_50)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i51.GSR = "DISABLED";
    FD1P3IX us_tx__i52 (.D(n13828), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_51)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i52.GSR = "DISABLED";
    FD1P3IX us_tx__i53 (.D(n13830), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_52)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i53.GSR = "DISABLED";
    FD1P3IX us_tx__i54 (.D(n13832), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_53)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i54.GSR = "DISABLED";
    FD1P3IX us_tx__i55 (.D(n13834), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_54)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i55.GSR = "DISABLED";
    FD1P3IX us_tx__i56 (.D(n13836), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_55)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i56.GSR = "DISABLED";
    FD1P3IX us_tx__i57 (.D(n13838), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_56)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i57.GSR = "DISABLED";
    FD1P3IX us_tx__i58 (.D(n13840), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_57)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i58.GSR = "DISABLED";
    FD1P3IX us_tx__i59 (.D(n13842), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_58)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i59.GSR = "DISABLED";
    FD1P3IX us_tx__i60 (.D(n13844), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_59)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i60.GSR = "DISABLED";
    FD1P3IX us_tx__i61 (.D(n13846), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_60)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i61.GSR = "DISABLED";
    FD1P3IX us_tx__i62 (.D(n13848), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_61)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i62.GSR = "DISABLED";
    FD1P3IX us_tx__i63 (.D(n13850), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_62)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i63.GSR = "DISABLED";
    FD1P3IX us_tx__i64 (.D(n13852), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_63)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i64.GSR = "DISABLED";
    FD1P3IX us_tx__i65 (.D(n13854), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_64)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i65.GSR = "DISABLED";
    FD1P3IX us_tx__i66 (.D(n13856), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_65)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i66.GSR = "DISABLED";
    FD1P3IX us_tx__i67 (.D(n13858), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_66)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i67.GSR = "DISABLED";
    FD1P3IX us_tx__i68 (.D(n13860), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_67)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i68.GSR = "DISABLED";
    FD1P3IX us_tx__i69 (.D(n13862), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_68)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i69.GSR = "DISABLED";
    FD1P3IX us_tx__i70 (.D(n13864), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_69)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i70.GSR = "DISABLED";
    FD1P3IX us_tx__i71 (.D(n13866), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_70)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i71.GSR = "DISABLED";
    FD1P3IX us_tx__i72 (.D(n13868), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_71)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i72.GSR = "DISABLED";
    FD1P3IX us_tx__i73 (.D(n13870), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_72)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i73.GSR = "DISABLED";
    FD1P3IX us_tx__i74 (.D(n13872), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_73)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i74.GSR = "DISABLED";
    FD1P3IX us_tx__i75 (.D(n13874), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_74)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i75.GSR = "DISABLED";
    FD1P3IX us_tx__i76 (.D(n13876), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_75)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i76.GSR = "DISABLED";
    FD1P3IX us_tx__i77 (.D(n13878), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_76)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i77.GSR = "DISABLED";
    FD1P3IX us_tx__i78 (.D(n13880), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_77)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i78.GSR = "DISABLED";
    FD1P3IX us_tx__i79 (.D(n13882), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_78)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i79.GSR = "DISABLED";
    FD1P3IX us_tx__i80 (.D(n13884), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_79)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i80.GSR = "DISABLED";
    FD1P3IX us_tx__i81 (.D(n13886), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_80)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i81.GSR = "DISABLED";
    FD1P3IX us_tx__i82 (.D(n13888), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_81)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i82.GSR = "DISABLED";
    FD1P3IX us_tx__i83 (.D(n13890), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_82)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i83.GSR = "DISABLED";
    FD1P3IX us_tx__i84 (.D(n13892), .SP(pll_clk_enable_149), .CD(n11840), 
            .CK(pll_clk), .Q(us_tx_c_83)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(686[12:52])
    defparam us_tx__i84.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i1 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i1.GSR = "DISABLED";
    LUT4 ws2812_toggle_sync_I_0_2_lut_rep_299 (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .Z(pll_clk_enable_23)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[13:53])
    defparam ws2812_toggle_sync_I_0_2_lut_rep_299.init = 16'h6666;
    FD1P3AX mic_shift_0_l_i0_i2 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i3 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i4 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i5 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i6 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i7 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i8 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i9 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i10 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i11 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i12 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i13 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i14 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i15 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_0_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_l_i0_i15.GSR = "DISABLED";
    FD1P3AX status_hold__i2 (.D(accepted_sequence[25]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i2.GSR = "DISABLED";
    FD1P3AX status_hold__i3 (.D(accepted_sequence[26]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i3.GSR = "DISABLED";
    FD1P3AX status_hold__i4 (.D(accepted_sequence[27]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i4.GSR = "DISABLED";
    FD1P3AX status_hold__i5 (.D(accepted_sequence[28]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i5.GSR = "DISABLED";
    FD1P3AX status_hold__i6 (.D(accepted_sequence[29]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i6.GSR = "DISABLED";
    FD1P3AX status_hold__i7 (.D(accepted_sequence[30]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i7.GSR = "DISABLED";
    FD1P3AX status_hold__i8 (.D(accepted_sequence[31]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i8.GSR = "DISABLED";
    FD1P3AX status_hold__i9 (.D(accepted_sequence[16]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i9.GSR = "DISABLED";
    FD1P3AX status_hold__i10 (.D(accepted_sequence[17]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i10.GSR = "DISABLED";
    FD1P3AX status_hold__i11 (.D(accepted_sequence[18]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i11.GSR = "DISABLED";
    FD1P3AX status_hold__i12 (.D(accepted_sequence[19]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i12.GSR = "DISABLED";
    FD1P3AX status_hold__i13 (.D(accepted_sequence[20]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i13.GSR = "DISABLED";
    FD1P3AX status_hold__i14 (.D(accepted_sequence[21]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i14.GSR = "DISABLED";
    FD1P3AX status_hold__i15 (.D(accepted_sequence[22]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i15.GSR = "DISABLED";
    FD1P3AX status_hold__i16 (.D(accepted_sequence[23]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i16.GSR = "DISABLED";
    FD1P3AX status_hold__i17 (.D(accepted_sequence[8]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i17.GSR = "DISABLED";
    FD1P3AX status_hold__i18 (.D(accepted_sequence[9]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i18.GSR = "DISABLED";
    FD1P3AX status_hold__i19 (.D(accepted_sequence[10]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i19.GSR = "DISABLED";
    FD1P3AX status_hold__i20 (.D(accepted_sequence[11]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i20.GSR = "DISABLED";
    FD1P3AX status_hold__i21 (.D(accepted_sequence[12]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i21.GSR = "DISABLED";
    FD1P3AX status_hold__i22 (.D(accepted_sequence[13]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i22.GSR = "DISABLED";
    FD1P3AX status_hold__i23 (.D(accepted_sequence[14]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i23.GSR = "DISABLED";
    FD1P3AX status_hold__i24 (.D(accepted_sequence[15]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i24.GSR = "DISABLED";
    FD1P3AX status_hold__i25 (.D(accepted_sequence[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i25.GSR = "DISABLED";
    FD1P3AX status_hold__i26 (.D(accepted_sequence[1]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i26.GSR = "DISABLED";
    FD1P3AX status_hold__i27 (.D(accepted_sequence[2]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i27.GSR = "DISABLED";
    FD1P3AX status_hold__i28 (.D(accepted_sequence[3]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i28.GSR = "DISABLED";
    FD1P3AX status_hold__i29 (.D(accepted_sequence[4]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i29.GSR = "DISABLED";
    FD1P3AX status_hold__i30 (.D(accepted_sequence[5]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i30.GSR = "DISABLED";
    FD1P3AX status_hold__i31 (.D(accepted_sequence[6]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i31.GSR = "DISABLED";
    FD1P3AX status_hold__i32 (.D(accepted_sequence[7]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i32.GSR = "DISABLED";
    FD1P3AX status_hold__i33 (.D(fpga_time[24]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i33.GSR = "DISABLED";
    FD1P3AX status_hold__i34 (.D(fpga_time[25]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i34.GSR = "DISABLED";
    FD1P3AX status_hold__i35 (.D(fpga_time[26]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i35.GSR = "DISABLED";
    FD1P3AX status_hold__i36 (.D(fpga_time[27]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i36.GSR = "DISABLED";
    FD1P3AX status_hold__i37 (.D(fpga_time[28]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i37.GSR = "DISABLED";
    FD1P3AX status_hold__i38 (.D(fpga_time[29]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i38.GSR = "DISABLED";
    FD1P3AX status_hold__i39 (.D(fpga_time[30]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i39.GSR = "DISABLED";
    FD1P3AX status_hold__i40 (.D(fpga_time[31]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i40.GSR = "DISABLED";
    FD1P3AX status_hold__i41 (.D(fpga_time[16]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i41.GSR = "DISABLED";
    FD1P3AX status_hold__i42 (.D(fpga_time[17]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i42.GSR = "DISABLED";
    FD1P3AX status_hold__i43 (.D(fpga_time[18]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i43.GSR = "DISABLED";
    FD1P3AX status_hold__i44 (.D(fpga_time[19]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i44.GSR = "DISABLED";
    FD1P3AX status_hold__i45 (.D(fpga_time[20]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i45.GSR = "DISABLED";
    FD1P3AX status_hold__i46 (.D(fpga_time[21]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i46.GSR = "DISABLED";
    FD1P3AX status_hold__i47 (.D(fpga_time[22]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i47.GSR = "DISABLED";
    FD1P3AX status_hold__i48 (.D(fpga_time[23]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i48.GSR = "DISABLED";
    FD1P3AX status_hold__i49 (.D(fpga_time[8]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i49.GSR = "DISABLED";
    FD1P3AX status_hold__i50 (.D(fpga_time[9]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i50.GSR = "DISABLED";
    FD1P3AX status_hold__i51 (.D(fpga_time[10]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i51.GSR = "DISABLED";
    FD1P3AX status_hold__i52 (.D(fpga_time[11]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i52.GSR = "DISABLED";
    FD1P3AX status_hold__i53 (.D(fpga_time[12]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i53.GSR = "DISABLED";
    FD1P3AX status_hold__i54 (.D(fpga_time[13]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i54.GSR = "DISABLED";
    FD1P3AX status_hold__i55 (.D(fpga_time[14]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i55.GSR = "DISABLED";
    FD1P3AX status_hold__i56 (.D(fpga_time[15]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i56.GSR = "DISABLED";
    FD1P3AX status_hold__i57 (.D(fpga_time[0]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i57.GSR = "DISABLED";
    FD1P3AX status_hold__i58 (.D(fpga_time[1]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i58.GSR = "DISABLED";
    FD1P3AX status_hold__i59 (.D(fpga_time[2]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i59.GSR = "DISABLED";
    FD1P3AX status_hold__i60 (.D(fpga_time[3]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i60.GSR = "DISABLED";
    FD1P3AX status_hold__i61 (.D(fpga_time[4]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i61.GSR = "DISABLED";
    FD1P3AX status_hold__i62 (.D(fpga_time[5]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i62.GSR = "DISABLED";
    FD1P3AX status_hold__i63 (.D(fpga_time[6]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i63.GSR = "DISABLED";
    FD1P3AX status_hold__i64 (.D(fpga_time[7]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i64.GSR = "DISABLED";
    FD1P3AX status_hold__i65 (.D(status_flags_wire_15__N_1210[2]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i65.GSR = "DISABLED";
    FD1P3AX status_hold__i66 (.D(status_flags_wire_15__N_1226[4]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i66.GSR = "DISABLED";
    FD1P3AX status_hold__i67 (.D(n26206), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i67.GSR = "DISABLED";
    FD1P3AX status_hold__i68 (.D(fifo_credit_wire[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[104])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam status_hold__i68.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i1 (.D(accepted_sequence_31__N_943[1]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i1.GSR = "DISABLED";
    CCU2D add_779_3 (.A0(phase_frac[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24910), .COUT(n24911), .S0(phase_frac_sum[1]), .S1(phase_frac_sum[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_3.INIT0 = 16'h5555;
    defparam add_779_3.INIT1 = 16'h5aaa;
    defparam add_779_3.INJECT1_0 = "NO";
    defparam add_779_3.INJECT1_1 = "NO";
    CCU2D add_14301_5 (.A0(spi_expected_length[4]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[5]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24978), .COUT(n24979));
    defparam add_14301_5.INIT0 = 16'hf555;
    defparam add_14301_5.INIT1 = 16'h0aaa;
    defparam add_14301_5.INJECT1_0 = "NO";
    defparam add_14301_5.INJECT1_1 = "NO";
    LUT4 mux_383_i26_3_lut (.A(pending_sequence[25]), .B(accepted_sequence_sync[25]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[25])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i26_3_lut.init = 16'hcaca;
    CCU2D add_779_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24910), .S1(phase_frac_sum[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(177[34:66])
    defparam add_779_1.INIT0 = 16'hF000;
    defparam add_779_1.INIT1 = 16'h5555;
    defparam add_779_1.INJECT1_0 = "NO";
    defparam add_779_1.INJECT1_1 = "NO";
    LUT4 mux_383_i27_3_lut (.A(pending_sequence[26]), .B(accepted_sequence_sync[26]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[26])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i27_3_lut.init = 16'hcaca;
    LUT4 run_addr_s3_8__I_0_i2_3_lut (.A(run_addr_s3[1]), .B(ev_rd_slot[1]), 
         .C(n21588), .Z(event_rd_addr[1])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(241[33:74])
    defparam run_addr_s3_8__I_0_i2_3_lut.init = 16'hacac;
    LUT4 mux_383_i28_3_lut (.A(pending_sequence[27]), .B(accepted_sequence_sync[27]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[27])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i28_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut_4_lut_adj_59 (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .C(n26262), .D(ws2812_settle[0]), .Z(pll_clk_enable_647)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[13:53])
    defparam i1_2_lut_3_lut_4_lut_adj_59.init = 16'hfff6;
    LUT4 mux_383_i29_3_lut (.A(pending_sequence[28]), .B(accepted_sequence_sync[28]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[28])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i29_3_lut.init = 16'hcaca;
    CCU2D time_divider_1597_add_4_7 (.A0(time_divider[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24908), .S0(n35), .S1(n34));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597_add_4_7.INIT0 = 16'hfaaa;
    defparam time_divider_1597_add_4_7.INIT1 = 16'hfaaa;
    defparam time_divider_1597_add_4_7.INJECT1_0 = "NO";
    defparam time_divider_1597_add_4_7.INJECT1_1 = "NO";
    CCU2D sub_1435_add_2_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24936), .S0(n11301));
    defparam sub_1435_add_2_cout.INIT0 = 16'h0000;
    defparam sub_1435_add_2_cout.INIT1 = 16'h0000;
    defparam sub_1435_add_2_cout.INJECT1_0 = "NO";
    defparam sub_1435_add_2_cout.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_60 (.A(n26209), .B(n26240), .C(init_shadow[44]), 
         .D(n16988), .Z(n14592)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_60.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_61 (.A(n26213), .B(n26240), .C(init_shadow[41]), 
         .D(n16988), .Z(n14610)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_61.init = 16'hf1f0;
    CCU2D add_14301_3 (.A0(spi_expected_length[2]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[3]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24977), .COUT(n24978));
    defparam add_14301_3.INIT0 = 16'h0aaa;
    defparam add_14301_3.INIT1 = 16'hf555;
    defparam add_14301_3.INJECT1_0 = "NO";
    defparam add_14301_3.INJECT1_1 = "NO";
    CCU2D sub_1435_add_2_7 (.A0(spi_byte_count[8]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[9]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24935), .COUT(n24936));
    defparam sub_1435_add_2_7.INIT0 = 16'h5555;
    defparam sub_1435_add_2_7.INIT1 = 16'h5555;
    defparam sub_1435_add_2_7.INJECT1_0 = "NO";
    defparam sub_1435_add_2_7.INJECT1_1 = "NO";
    LUT4 mux_383_i30_3_lut (.A(pending_sequence[29]), .B(accepted_sequence_sync[29]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[29])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i30_3_lut.init = 16'hcaca;
    CCU2D time_divider_1597_add_4_5 (.A0(time_divider[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24907), .COUT(n24908), .S0(n37), 
          .S1(n36));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597_add_4_5.INIT0 = 16'hfaaa;
    defparam time_divider_1597_add_4_5.INIT1 = 16'hfaaa;
    defparam time_divider_1597_add_4_5.INJECT1_0 = "NO";
    defparam time_divider_1597_add_4_5.INJECT1_1 = "NO";
    CCU2D add_781_2 (.A0(staging_q[8]), .B0(staging_q[0]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[9]), .B1(staging_q[1]), .C1(GND_net), 
          .D1(GND_net), .COUT(n24894), .S1(build_sum_8__N_2031[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(598[32:80])
    defparam add_781_2.INIT0 = 16'h7000;
    defparam add_781_2.INIT1 = 16'h5666;
    defparam add_781_2.INJECT1_0 = "NO";
    defparam add_781_2.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_777_3 (.A0(spi_rgb_payload_byte_N_2547[6]), 
          .B0(spi_extension_length[3]), .C0(GND_net), .D0(GND_net), .A1(expected_next_15__N_1351[3]), 
          .B1(spi_extension_length[4]), .C1(GND_net), .D1(GND_net), .CIN(n24886), 
          .COUT(n24887), .S0(expected_next[3]), .S1(expected_next[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[33] 314[86])
    defparam expected_next_15__I_0_777_3.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_777_3.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_777_3.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_777_3.INJECT1_1 = "NO";
    CCU2D time_divider_1597_add_4_3 (.A0(time_divider[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24906), .COUT(n24907), .S0(n39), 
          .S1(n38));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597_add_4_3.INIT0 = 16'hfaaa;
    defparam time_divider_1597_add_4_3.INIT1 = 16'hfaaa;
    defparam time_divider_1597_add_4_3.INJECT1_0 = "NO";
    defparam time_divider_1597_add_4_3.INJECT1_1 = "NO";
    CCU2D sub_1435_add_2_5 (.A0(spi_byte_count[6]), .B0(spi_rgb_payload_byte_N_2547[6]), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[7]), .B1(spi_rgb_payload_byte_N_2547[6]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24934), .COUT(n24935));
    defparam sub_1435_add_2_5.INIT0 = 16'h5999;
    defparam sub_1435_add_2_5.INIT1 = 16'h5999;
    defparam sub_1435_add_2_5.INJECT1_0 = "NO";
    defparam sub_1435_add_2_5.INJECT1_1 = "NO";
    CCU2D add_14301_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(spi_expected_length[0]), .B1(spi_expected_length[1]), .C1(GND_net), 
          .D1(GND_net), .COUT(n24977));
    defparam add_14301_1.INIT0 = 16'hF000;
    defparam add_14301_1.INIT1 = 16'ha666;
    defparam add_14301_1.INJECT1_0 = "NO";
    defparam add_14301_1.INJECT1_1 = "NO";
    CCU2D sub_1435_add_2_3 (.A0(spi_byte_count[4]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[5]), .B1(spi_rgb_payload_byte_N_2547[6]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24933), .COUT(n24934));
    defparam sub_1435_add_2_3.INIT0 = 16'h5aaa;
    defparam sub_1435_add_2_3.INIT1 = 16'h5666;
    defparam sub_1435_add_2_3.INJECT1_0 = "NO";
    defparam sub_1435_add_2_3.INJECT1_1 = "NO";
    CCU2D time_divider_1597_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(time_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24906), .S1(n40));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(488[29:48])
    defparam time_divider_1597_add_4_1.INIT0 = 16'hF000;
    defparam time_divider_1597_add_4_1.INIT1 = 16'h0555;
    defparam time_divider_1597_add_4_1.INJECT1_0 = "NO";
    defparam time_divider_1597_add_4_1.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_777_13 (.A0(spi_rx_shift[4]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[5]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24891), .COUT(n24892), .S0(expected_next[13]), 
          .S1(expected_next[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[33] 314[86])
    defparam expected_next_15__I_0_777_13.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_777_13.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_777_13.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_777_13.INJECT1_1 = "NO";
    LUT4 i1_3_lut_rep_182_4_lut_3_lut_4_lut (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .C(n26262), .D(ws2812_settle[0]), .Z(n26146)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(530[13:53])
    defparam i1_3_lut_rep_182_4_lut_3_lut_4_lut.init = 16'h0900;
    LUT4 i1_3_lut_4_lut_adj_62 (.A(n26208), .B(n26240), .C(init_shadow[43]), 
         .D(n16988), .Z(n14598)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_62.init = 16'hf1f0;
    CCU2D equal_2398_17 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n24755), 
          .S0(frame_end_N_2612));
    defparam equal_2398_17.INIT0 = 16'hFFFF;
    defparam equal_2398_17.INIT1 = 16'h0000;
    defparam equal_2398_17.INJECT1_0 = "NO";
    defparam equal_2398_17.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_777_11 (.A0(spi_rx_shift[2]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[3]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24890), .COUT(n24891), .S0(expected_next[11]), 
          .S1(expected_next[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[33] 314[86])
    defparam expected_next_15__I_0_777_11.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_777_11.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_777_11.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_777_11.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_777_15 (.A0(spi_rx_shift[6]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24892), .S0(expected_next[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[33] 314[86])
    defparam expected_next_15__I_0_777_15.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_777_15.INIT1 = 16'h0000;
    defparam expected_next_15__I_0_777_15.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_777_15.INJECT1_1 = "NO";
    CCU2D equal_2398_17_14301 (.A0(spi_expected_length[3]), .B0(spi_byte_count_15__N_1504[3]), 
          .C0(spi_expected_length[2]), .D0(spi_byte_count_15__N_1504[2]), 
          .A1(spi_expected_length[1]), .B1(spi_byte_count_15__N_1504[1]), 
          .C1(spi_expected_length[0]), .D1(spi_byte_count_15__N_1504[0]), 
          .CIN(n24754), .COUT(n24755));
    defparam equal_2398_17_14301.INIT0 = 16'h9009;
    defparam equal_2398_17_14301.INIT1 = 16'h9009;
    defparam equal_2398_17_14301.INJECT1_0 = "YES";
    defparam equal_2398_17_14301.INJECT1_1 = "YES";
    CCU2D equal_2398_11 (.A0(spi_expected_length[15]), .B0(spi_byte_count_15__N_1504[15]), 
          .C0(spi_expected_length[14]), .D0(spi_byte_count_15__N_1504[14]), 
          .A1(spi_expected_length[13]), .B1(spi_byte_count_15__N_1504[13]), 
          .C1(spi_expected_length[12]), .D1(spi_byte_count_15__N_1504[12]), 
          .CIN(n24751), .COUT(n24752));
    defparam equal_2398_11.INIT0 = 16'h9009;
    defparam equal_2398_11.INIT1 = 16'h9009;
    defparam equal_2398_11.INJECT1_0 = "YES";
    defparam equal_2398_11.INJECT1_1 = "YES";
    LUT4 i2_3_lut_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), .C(spi_byte_count[4]), 
         .D(spi_byte_count[5]), .Z(n25064)) /* synthesis lut_function=(A (B+(C+(D)))+!A (C+(D))) */ ;
    defparam i2_3_lut_4_lut.init = 16'hfff8;
    CCU2D equal_2398_15 (.A0(spi_expected_length[7]), .B0(spi_byte_count_15__N_1504[7]), 
          .C0(spi_expected_length[6]), .D0(spi_byte_count_15__N_1504[6]), 
          .A1(spi_expected_length[5]), .B1(spi_byte_count_15__N_1504[5]), 
          .C1(spi_expected_length[4]), .D1(spi_byte_count_15__N_1504[4]), 
          .CIN(n24753), .COUT(n24754));
    defparam equal_2398_15.INIT0 = 16'h9009;
    defparam equal_2398_15.INIT1 = 16'h9009;
    defparam equal_2398_15.INJECT1_0 = "YES";
    defparam equal_2398_15.INJECT1_1 = "YES";
    LUT4 i1777_3_lut_4_lut (.A(ev_ch[4]), .B(n26191), .C(ev_ch[5]), .D(ev_ch[6]), 
         .Z(ev_ch_6__N_2010[6])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(609[27:39])
    defparam i1777_3_lut_4_lut.init = 16'h7f80;
    CCU2D add_244_17 (.A0(spi_byte_count[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24905), .S0(spi_byte_count_15__N_1504[15]), .S1(frame_end_N_2613[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[35:57])
    defparam add_244_17.INIT0 = 16'h5aaa;
    defparam add_244_17.INIT1 = 16'h0000;
    defparam add_244_17.INJECT1_0 = "NO";
    defparam add_244_17.INJECT1_1 = "NO";
    LUT4 i10953_3_lut_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(n26276), .D(spi_byte_count[7]), .Z(n86)) /* synthesis lut_function=(!(A (B (D)+!B (C (D)))+!A (C (D)))) */ ;
    defparam i10953_3_lut_4_lut.init = 16'h07ff;
    LUT4 i1_3_lut_4_lut_adj_63 (.A(n26215), .B(n26240), .C(init_shadow[47]), 
         .D(n16988), .Z(n14574)) /* synthesis lut_function=(A (B (C)+!B (C+(D)))+!A (C)) */ ;
    defparam i1_3_lut_4_lut_adj_63.init = 16'hf2f0;
    LUT4 mux_383_i31_3_lut (.A(pending_sequence[30]), .B(accepted_sequence_sync[30]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[30])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i31_3_lut.init = 16'hcaca;
    CCU2D sub_1435_add_2_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[3]), .B1(spi_rgb_payload_byte_N_2547[6]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24933));
    defparam sub_1435_add_2_1.INIT0 = 16'h0000;
    defparam sub_1435_add_2_1.INIT1 = 16'h5999;
    defparam sub_1435_add_2_1.INJECT1_0 = "NO";
    defparam sub_1435_add_2_1.INJECT1_1 = "NO";
    LUT4 i2_3_lut_rep_300 (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[4]), .Z(n26264)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_3_lut_rep_300.init = 16'hfefe;
    LUT4 i1_3_lut_4_lut_adj_64 (.A(n26211), .B(n26240), .C(init_shadow[46]), 
         .D(n16988), .Z(n14580)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_64.init = 16'hf1f0;
    LUT4 ws2812_settle_2__bdd_3_lut (.A(ws2812_settle[2]), .B(ws2812_settle[1]), 
         .C(ws2812_settle[0]), .Z(n26031)) /* synthesis lut_function=(A (B+(C))+!A !(B+(C))) */ ;
    defparam ws2812_settle_2__bdd_3_lut.init = 16'ha9a9;
    LUT4 i1_2_lut_rep_263_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[4]), .D(spi_byte_count[5]), .Z(n26227)) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C (D)))) */ ;
    defparam i1_2_lut_rep_263_4_lut.init = 16'hfe00;
    LUT4 i2_2_lut_rep_243 (.A(n25477), .B(n4_adj_3211), .Z(n26207)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut_rep_243.init = 16'h8888;
    FD1P3IX ev_clear_addr_i7 (.D(ev_clear_addr_7__N_2215[7]), .SP(pll_clk_enable_639), 
            .CD(n17861), .CK(pll_clk), .Q(ev_clear_addr[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_clear_addr_i7.GSR = "DISABLED";
    LUT4 i8_2_lut_rep_301 (.A(spi_rgb_payload_byte_N_2547[6]), .B(fpga_cs_n_c), 
         .Z(n26265)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i8_2_lut_rep_301.init = 16'h2222;
    LUT4 i1_2_lut_3_lut_4_lut_adj_65 (.A(spi_rgb_payload_byte_N_2547[6]), 
         .B(fpga_cs_n_c), .C(n26120), .D(spi_byte_count[8]), .Z(n5_adj_3181)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_65.init = 16'h0020;
    LUT4 i10986_2_lut_rep_302 (.A(spi_command[0]), .B(spi_command[1]), .Z(n26266)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i10986_2_lut_rep_302.init = 16'h6666;
    FD1P3IX ev_clear_addr_i6 (.D(ev_clear_addr_7__N_2215[6]), .SP(pll_clk_enable_639), 
            .CD(n17861), .CK(pll_clk), .Q(ev_clear_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_clear_addr_i6.GSR = "DISABLED";
    LUT4 i14342_2_lut_3_lut_4_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_135), 
         .C(spi_rgb_index[0]), .D(n26126), .Z(n25)) /* synthesis lut_function=(A (C)+!A !(B (C (D)+!C !(D))+!B !(C))) */ ;
    defparam i14342_2_lut_3_lut_4_lut.init = 16'hb4f0;
    FD1P3IX ev_clear_addr_i5 (.D(ev_clear_addr_7__N_2215[5]), .SP(pll_clk_enable_639), 
            .CD(n17861), .CK(pll_clk), .Q(ev_clear_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_clear_addr_i5.GSR = "DISABLED";
    LUT4 mux_383_i32_3_lut (.A(pending_sequence[31]), .B(accepted_sequence_sync[31]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[31])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i32_3_lut.init = 16'hcaca;
    LUT4 i3_4_lut (.A(spi_channel_index[0]), .B(spi_channel_index[1]), .C(spi_channel_index[4]), 
         .D(spi_channel_index[6]), .Z(n25024)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_4_lut.init = 16'h8000;
    FD1P3IX ev_clear_addr_i4 (.D(ev_clear_addr_7__N_2215[4]), .SP(pll_clk_enable_639), 
            .CD(n17861), .CK(pll_clk), .Q(ev_clear_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_clear_addr_i4.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i3 (.D(ev_clear_addr_7__N_2215[3]), .SP(pll_clk_enable_639), 
            .CD(n17861), .CK(pll_clk), .Q(ev_clear_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_clear_addr_i3.GSR = "DISABLED";
    LUT4 i1_2_lut (.A(spi_channel_index[3]), .B(spi_channel_index[5]), .Z(n5)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(379[43:69])
    defparam i1_2_lut.init = 16'heeee;
    LUT4 i14344_2_lut_rep_157_3_lut_4_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_135), 
         .C(spi_rgb_index[0]), .D(n26126), .Z(n26121)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;
    defparam i14344_2_lut_rep_157_3_lut_4_lut.init = 16'h4000;
    LUT4 mux_1731_i2_3_lut (.A(n12181), .B(n12182), .C(n12178), .Z(rd_data_15__N_2646[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i2_3_lut.init = 16'hcaca;
    LUT4 i14357_2_lut_3_lut_4_lut (.A(n26123), .B(spi_rgb_index[0]), .C(spi_rgb_index[2]), 
         .D(spi_rgb_index[1]), .Z(n23)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(404[46:66])
    defparam i14357_2_lut_3_lut_4_lut.init = 16'h78f0;
    FD1P3IX ev_ch_i5 (.D(ev_ch_6__N_2010[5]), .SP(pll_clk_enable_684), .CD(n17880), 
            .CK(pll_clk), .Q(ev_ch[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_ch_i5.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i2 (.D(ev_clear_addr_7__N_2215[2]), .SP(pll_clk_enable_639), 
            .CD(n17861), .CK(pll_clk), .Q(ev_clear_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_clear_addr_i2.GSR = "DISABLED";
    LUT4 ev_state_3__I_0_717_Mux_1_i7_4_lut_4_lut (.A(n26207), .B(ev_state[1]), 
         .C(ev_state[0]), .D(ev_state[2]), .Z(n7_adj_3218)) /* synthesis lut_function=(!(A (B (C (D))+!B !(C))+!A (B (C (D))+!B !(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam ev_state_3__I_0_717_Mux_1_i7_4_lut_4_lut.init = 16'h3cec;
    LUT4 i1_2_lut_rep_303 (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .Z(n26267)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(342[17] 407[24])
    defparam i1_2_lut_rep_303.init = 16'h4444;
    LUT4 i2_2_lut_rep_266_3_lut (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .C(spi_byte_count[2]), .Z(n26230)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(342[17] 407[24])
    defparam i2_2_lut_rep_266_3_lut.init = 16'h0404;
    LUT4 mux_1731_i3_3_lut (.A(n12183), .B(n12184), .C(n12178), .Z(rd_data_15__N_2646[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i3_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_66 (.A(n26217), .B(n26214), .C(ev_rd_hold[0]), 
         .D(n26147), .Z(ev_wr_data[0])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_66.init = 16'hf100;
    LUT4 mux_1731_i4_3_lut (.A(n12185), .B(n12186), .C(n12178), .Z(rd_data_15__N_2646[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i4_3_lut.init = 16'hcaca;
    LUT4 mux_1731_i5_3_lut (.A(n12187), .B(n12188), .C(n12178), .Z(rd_data_15__N_2646[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i5_3_lut.init = 16'hcaca;
    LUT4 mux_1731_i6_3_lut (.A(n12189), .B(n12190), .C(n12178), .Z(rd_data_15__N_2646[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i6_3_lut.init = 16'hcaca;
    FD1P3AX accepted_sequence_i2 (.D(accepted_sequence_31__N_943[2]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i3 (.D(accepted_sequence_31__N_943[3]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i4 (.D(accepted_sequence_31__N_943[4]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i5 (.D(accepted_sequence_31__N_943[5]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i6 (.D(accepted_sequence_31__N_943[6]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i7 (.D(accepted_sequence_31__N_943[7]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i8 (.D(accepted_sequence_31__N_943[8]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i9 (.D(accepted_sequence_31__N_943[9]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i10 (.D(accepted_sequence_31__N_943[10]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i11 (.D(accepted_sequence_31__N_943[11]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i12 (.D(accepted_sequence_31__N_943[12]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i13 (.D(accepted_sequence_31__N_943[13]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i14 (.D(accepted_sequence_31__N_943[14]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i15 (.D(accepted_sequence_31__N_943[15]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i16 (.D(accepted_sequence_31__N_943[16]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i17 (.D(accepted_sequence_31__N_943[17]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i18 (.D(accepted_sequence_31__N_943[18]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i19 (.D(accepted_sequence_31__N_943[19]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i20 (.D(accepted_sequence_31__N_943[20]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i21 (.D(accepted_sequence_31__N_943[21]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i22 (.D(accepted_sequence_31__N_943[22]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i23 (.D(accepted_sequence_31__N_943[23]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i24 (.D(accepted_sequence_31__N_943[24]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i25 (.D(accepted_sequence_31__N_943[25]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i26 (.D(accepted_sequence_31__N_943[26]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i27 (.D(accepted_sequence_31__N_943[27]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i28 (.D(accepted_sequence_31__N_943[28]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i29 (.D(accepted_sequence_31__N_943[29]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i30 (.D(accepted_sequence_31__N_943[30]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i31 (.D(accepted_sequence_31__N_943[31]), .SP(pll_clk_enable_203), 
            .CK(pll_clk), .Q(accepted_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_i31.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i1 (.D(global_phase_s2[1]), .SP(pll_clk_enable_211), 
            .CK(pll_clk), .Q(run_addr_s3[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam run_addr_s3_i1.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i2 (.D(global_phase_s2[2]), .SP(pll_clk_enable_211), 
            .CK(pll_clk), .Q(run_addr_s3[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam run_addr_s3_i2.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i3 (.D(global_phase_s2[3]), .SP(pll_clk_enable_211), 
            .CK(pll_clk), .Q(run_addr_s3[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam run_addr_s3_i3.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i4 (.D(global_phase_s2[4]), .SP(pll_clk_enable_211), 
            .CK(pll_clk), .Q(run_addr_s3[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam run_addr_s3_i4.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i5 (.D(global_phase_s2[5]), .SP(pll_clk_enable_211), 
            .CK(pll_clk), .Q(run_addr_s3[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam run_addr_s3_i5.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i6 (.D(global_phase_s2[6]), .SP(pll_clk_enable_211), 
            .CK(pll_clk), .Q(run_addr_s3[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam run_addr_s3_i6.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i7 (.D(global_phase_s2[7]), .SP(pll_clk_enable_211), 
            .CK(pll_clk), .Q(run_addr_s3[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam run_addr_s3_i7.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i8 (.D(active_bank_N_731), .SP(pll_clk_enable_211), 
            .CK(pll_clk), .Q(run_addr_s3[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam run_addr_s3_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i1 (.D(accepted_sequence_spi[1]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i2 (.D(accepted_sequence_spi[2]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i3 (.D(accepted_sequence_spi[3]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i4 (.D(accepted_sequence_spi[4]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i5 (.D(accepted_sequence_spi[5]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i6 (.D(accepted_sequence_spi[6]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i7 (.D(accepted_sequence_spi[7]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i8 (.D(accepted_sequence_spi[8]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i9 (.D(accepted_sequence_spi[9]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i10 (.D(accepted_sequence_spi[10]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i11 (.D(accepted_sequence_spi[11]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i12 (.D(accepted_sequence_spi[12]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i13 (.D(accepted_sequence_spi[13]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i14 (.D(accepted_sequence_spi[14]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i15 (.D(accepted_sequence_spi[15]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i16 (.D(accepted_sequence_spi[16]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i17 (.D(accepted_sequence_spi[17]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i18 (.D(accepted_sequence_spi[18]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i19 (.D(accepted_sequence_spi[19]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i20 (.D(accepted_sequence_spi[20]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i21 (.D(accepted_sequence_spi[21]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i22 (.D(accepted_sequence_spi[22]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i23 (.D(accepted_sequence_spi[23]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i24 (.D(accepted_sequence_spi[24]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i25 (.D(accepted_sequence_spi[25]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i26 (.D(accepted_sequence_spi[26]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i27 (.D(accepted_sequence_spi[27]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i28 (.D(accepted_sequence_spi[28]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i29 (.D(accepted_sequence_spi[29]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i30 (.D(accepted_sequence_spi[30]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i31 (.D(accepted_sequence_spi[31]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_meta_i31.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i1 (.D(accepted_sequence_meta[1]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i2 (.D(accepted_sequence_meta[2]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i3 (.D(accepted_sequence_meta[3]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i4 (.D(accepted_sequence_meta[4]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i5 (.D(accepted_sequence_meta[5]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i6 (.D(accepted_sequence_meta[6]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i7 (.D(accepted_sequence_meta[7]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i8 (.D(accepted_sequence_meta[8]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i9 (.D(accepted_sequence_meta[9]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i10 (.D(accepted_sequence_meta[10]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i11 (.D(accepted_sequence_meta[11]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i12 (.D(accepted_sequence_meta[12]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i13 (.D(accepted_sequence_meta[13]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i14 (.D(accepted_sequence_meta[14]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i15 (.D(accepted_sequence_meta[15]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i16 (.D(accepted_sequence_meta[16]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i17 (.D(accepted_sequence_meta[17]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i18 (.D(accepted_sequence_meta[18]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i19 (.D(accepted_sequence_meta[19]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i20 (.D(accepted_sequence_meta[20]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i21 (.D(accepted_sequence_meta[21]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i22 (.D(accepted_sequence_meta[22]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i23 (.D(accepted_sequence_meta[23]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i24 (.D(accepted_sequence_meta[24]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i25 (.D(accepted_sequence_meta[25]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i26 (.D(accepted_sequence_meta[26]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i27 (.D(accepted_sequence_meta[27]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i28 (.D(accepted_sequence_meta[28]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i29 (.D(accepted_sequence_meta[29]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i30 (.D(accepted_sequence_meta[30]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i31 (.D(accepted_sequence_meta[31]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam accepted_sequence_sync_i31.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i1 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i2 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i3 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i4 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i5 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i6 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i7 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i8 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i9 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i10 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i11 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i12 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i13 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i14 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i15 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_226), 
            .CK(pll_clk), .Q(mic_shift_1_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_l_i0_i15.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_67 (.A(n26217), .B(n26214), .C(init_shadow[0]), 
         .D(n16988), .Z(n13711)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_67.init = 16'hf1f0;
    PFUMX i15165 (.BLUT(n25736), .ALUT(n25737), .C0(spi1_miso_N_2492[1]), 
          .Z(n25747));
    LUT4 mux_1731_i7_3_lut (.A(n12191), .B(n12192), .C(n12178), .Z(rd_data_15__N_2646[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i7_3_lut.init = 16'hcaca;
    LUT4 i2823_3_lut_rep_178_4_lut (.A(n25477), .B(n4_adj_3211), .C(ev_state[0]), 
         .D(frame_req_N_2584), .Z(n26142)) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A !(C+!(D))) */ ;
    defparam i2823_3_lut_rep_178_4_lut.init = 16'h8f80;
    LUT4 frame_end_I_27_2_lut (.A(n11277), .B(frame_end_N_2612), .Z(frame_end_N_2610)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(321[48] 322[97])
    defparam frame_end_I_27_2_lut.init = 16'h4444;
    FD1P3AX build_phase_i1 (.D(staging_q[9]), .SP(pll_clk_enable_241), .CK(pll_clk), 
            .Q(build_phase[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_phase_i1.GSR = "DISABLED";
    FD1P3AX build_phase_i2 (.D(staging_q[10]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_phase[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_phase_i2.GSR = "DISABLED";
    FD1P3AX build_phase_i3 (.D(staging_q[11]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_phase[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_phase_i3.GSR = "DISABLED";
    FD1P3AX build_phase_i4 (.D(staging_q[12]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_phase[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_phase_i4.GSR = "DISABLED";
    FD1P3AX build_phase_i5 (.D(staging_q[13]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_phase[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_phase_i5.GSR = "DISABLED";
    FD1P3AX build_phase_i6 (.D(staging_q[14]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_phase[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_phase_i6.GSR = "DISABLED";
    FD1P3AX build_phase_i7 (.D(staging_q[15]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_phase[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_phase_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i1 (.D(build_sum_8__N_2031[1]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_sum[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_sum_i1.GSR = "DISABLED";
    FD1P3AX build_sum_i2 (.D(build_sum_8__N_2031[2]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_sum[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_sum_i2.GSR = "DISABLED";
    FD1P3AX build_sum_i3 (.D(build_sum_8__N_2031[3]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_sum[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_sum_i3.GSR = "DISABLED";
    FD1P3AX build_sum_i4 (.D(build_sum_8__N_2031[4]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_sum[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_sum_i4.GSR = "DISABLED";
    FD1P3AX build_sum_i5 (.D(build_sum_8__N_2031[5]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_sum[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_sum_i5.GSR = "DISABLED";
    FD1P3AX build_sum_i6 (.D(build_sum_8__N_2031[6]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_sum[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_sum_i6.GSR = "DISABLED";
    FD1P3AX build_sum_i7 (.D(build_sum_8__N_2031[7]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_sum[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_sum_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i8 (.D(build_sum_8__N_2031[8]), .SP(pll_clk_enable_241), 
            .CK(pll_clk), .Q(build_sum[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam build_sum_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i1 (.D(ev_rd_data[1]), .SP(pll_clk_enable_290), .CK(pll_clk), 
            .Q(ev_rd_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i1.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i2 (.D(ev_rd_data[2]), .SP(pll_clk_enable_290), .CK(pll_clk), 
            .Q(ev_rd_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i2.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i3 (.D(ev_rd_data[3]), .SP(pll_clk_enable_290), .CK(pll_clk), 
            .Q(ev_rd_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i3.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i4 (.D(ev_rd_data[4]), .SP(pll_clk_enable_290), .CK(pll_clk), 
            .Q(ev_rd_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i4.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i5 (.D(ev_rd_data[5]), .SP(pll_clk_enable_290), .CK(pll_clk), 
            .Q(ev_rd_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i5.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i6 (.D(ev_rd_data[6]), .SP(pll_clk_enable_290), .CK(pll_clk), 
            .Q(ev_rd_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i6.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i7 (.D(ev_rd_data[7]), .SP(pll_clk_enable_290), .CK(pll_clk), 
            .Q(ev_rd_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i7.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i8 (.D(ev_rd_data[8]), .SP(pll_clk_enable_290), .CK(pll_clk), 
            .Q(ev_rd_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i9 (.D(ev_rd_data[9]), .SP(pll_clk_enable_290), .CK(pll_clk), 
            .Q(ev_rd_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i9.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i10 (.D(ev_rd_data[10]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i10.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i11 (.D(ev_rd_data[11]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i11.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i12 (.D(ev_rd_data[12]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i12.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i13 (.D(ev_rd_data[13]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i13.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i14 (.D(ev_rd_data[14]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i14.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i15 (.D(ev_rd_data[15]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i15.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i16 (.D(ev_rd_data[16]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i16.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i17 (.D(ev_rd_data[17]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i17.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i18 (.D(ev_rd_data[18]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i18.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i19 (.D(ev_rd_data[19]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i19.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i20 (.D(ev_rd_data[20]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i20.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i21 (.D(ev_rd_data[21]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i21.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i22 (.D(ev_rd_data[22]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i22.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i23 (.D(ev_rd_data[23]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i23.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i24 (.D(ev_rd_data[24]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i24.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i25 (.D(ev_rd_data[25]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i25.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i26 (.D(ev_rd_data[26]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i26.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i27 (.D(ev_rd_data[27]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i27.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i28 (.D(ev_rd_data[28]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i28.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i29 (.D(ev_rd_data[29]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i29.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i30 (.D(ev_rd_data[30]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i30.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i31 (.D(ev_rd_data[31]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i31.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i32 (.D(ev_rd_data[32]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i32.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i33 (.D(ev_rd_data[33]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i33.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i34 (.D(ev_rd_data[34]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i34.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i35 (.D(ev_rd_data[35]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i35.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i36 (.D(ev_rd_data[36]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i36.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i37 (.D(ev_rd_data[37]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i37.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i38 (.D(ev_rd_data[38]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i38.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i39 (.D(ev_rd_data[39]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i39.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i40 (.D(ev_rd_data[40]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i40.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i41 (.D(ev_rd_data[41]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i41.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i42 (.D(ev_rd_data[42]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i42.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i43 (.D(ev_rd_data[43]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i43.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i44 (.D(ev_rd_data[44]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i44.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i45 (.D(ev_rd_data[45]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i45.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i46 (.D(ev_rd_data[46]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i46.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i47 (.D(ev_rd_data[47]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i47.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i48 (.D(ev_rd_data[48]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i48.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i49 (.D(ev_rd_data[49]), .SP(pll_clk_enable_290), 
            .CK(pll_clk), .Q(ev_rd_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i49.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i50 (.D(ev_rd_data[50]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i50.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i51 (.D(ev_rd_data[51]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i51.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i52 (.D(ev_rd_data[52]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i52.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i53 (.D(ev_rd_data[53]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i53.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i54 (.D(ev_rd_data[54]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i54.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i55 (.D(ev_rd_data[55]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i55.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i56 (.D(ev_rd_data[56]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i56.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i57 (.D(ev_rd_data[57]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i57.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i58 (.D(ev_rd_data[58]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i58.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i59 (.D(ev_rd_data[59]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i59.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i60 (.D(ev_rd_data[60]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i60.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i61 (.D(ev_rd_data[61]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i61.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i62 (.D(ev_rd_data[62]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i62.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i63 (.D(ev_rd_data[63]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i63.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i64 (.D(ev_rd_data[64]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i64.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i65 (.D(ev_rd_data[65]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i65.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i66 (.D(ev_rd_data[66]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i66.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i67 (.D(ev_rd_data[67]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i67.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i68 (.D(ev_rd_data[68]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i68.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i69 (.D(ev_rd_data[69]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i69.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i70 (.D(ev_rd_data[70]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i70.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i71 (.D(ev_rd_data[71]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i71.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i72 (.D(ev_rd_data[72]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i72.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i73 (.D(ev_rd_data[73]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i73.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i74 (.D(ev_rd_data[74]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i74.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i75 (.D(ev_rd_data[75]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i75.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i76 (.D(ev_rd_data[76]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i76.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i77 (.D(ev_rd_data[77]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i77.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i78 (.D(ev_rd_data[78]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i78.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i79 (.D(ev_rd_data[79]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i79.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i80 (.D(ev_rd_data[80]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i80.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i81 (.D(ev_rd_data[81]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i81.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i82 (.D(ev_rd_data[82]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i82.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i83 (.D(ev_rd_data[83]), .SP(pll_clk_enable_324), 
            .CK(pll_clk), .Q(ev_rd_hold[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_rd_hold_i83.GSR = "DISABLED";
    LUT4 mux_1731_i8_3_lut (.A(n12193), .B(n12194), .C(n12178), .Z(rd_data_15__N_2646[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i8_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_68 (.A(n26213), .B(n26214), .C(ev_rd_hold[1]), 
         .D(n26147), .Z(ev_wr_data[1])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_68.init = 16'hf100;
    LUT4 mux_1731_i9_3_lut (.A(n12195), .B(n12196), .C(n12178), .Z(rd_data_15__N_2646[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i9_3_lut.init = 16'hcaca;
    LUT4 expected_next_15__I_0_i32_4_lut (.A(n25639), .B(n26_adj_3217), 
         .C(n22), .D(n26241), .Z(frame_end_N_2645)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(320[48:79])
    defparam expected_next_15__I_0_i32_4_lut.init = 16'h0002;
    LUT4 i1_3_lut_4_lut_adj_69 (.A(n26213), .B(n26214), .C(init_shadow[1]), 
         .D(n16988), .Z(n14850)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_69.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_70 (.A(n26220), .B(n26214), .C(ev_rd_hold[2]), 
         .D(n26147), .Z(ev_wr_data[2])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_70.init = 16'hf100;
    LUT4 mux_1731_i10_3_lut (.A(n12197), .B(n12198), .C(n12178), .Z(rd_data_15__N_2646[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i10_3_lut.init = 16'hcaca;
    LUT4 stop_toggle_sync_I_0_2_lut_rep_304 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .Z(pll_clk_enable_22)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(274[23:61])
    defparam stop_toggle_sync_I_0_2_lut_rep_304.init = 16'h6666;
    LUT4 i10920_3_lut_rep_177_4_lut (.A(n25477), .B(n4_adj_3211), .C(ev_state[0]), 
         .D(ev_state[1]), .Z(n26141)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;
    defparam i10920_3_lut_rep_177_4_lut.init = 16'hff80;
    LUT4 i2_3_lut_4_lut_adj_71 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(wrap_s2), .D(swap_pending), .Z(swap_pending_N_2588)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(274[23:61])
    defparam i2_3_lut_4_lut_adj_71.init = 16'h0900;
    LUT4 i1_3_lut_4_lut_adj_72 (.A(n26220), .B(n26214), .C(init_shadow[2]), 
         .D(n16988), .Z(n14844)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_72.init = 16'hf1f0;
    LUT4 i2_3_lut_4_lut_adj_73 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(status_flags_wire_15__N_1226[4]), .D(phase_step_s5), .Z(pll_clk_enable_149)) /* synthesis lut_function=(A (((D)+!C)+!B)+!A (B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(274[23:61])
    defparam i2_3_lut_4_lut_adj_73.init = 16'hff6f;
    LUT4 i15012_2_lut_rep_264_3_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(ev_state[3]), .Z(n26228)) /* synthesis lut_function=(A ((C)+!B)+!A (B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(274[23:61])
    defparam i15012_2_lut_rep_264_3_lut.init = 16'hf6f6;
    LUT4 i15255_2_lut_3_lut_4_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(n26059), .D(ev_state[3]), .Z(pll_clk_enable_642)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(274[23:61])
    defparam i15255_2_lut_3_lut_4_lut.init = 16'hfff6;
    LUT4 i1_2_lut_rep_170_4_lut (.A(frame_req_N_2584), .B(n26207), .C(ev_state[0]), 
         .D(ev_state[1]), .Z(n26134)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i1_2_lut_rep_170_4_lut.init = 16'h00ca;
    LUT4 i996_2_lut_3_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), .C(status_flags_wire_15__N_1226[4]), 
         .Z(n11840)) /* synthesis lut_function=(!(A (B (C))+!A !(B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(274[23:61])
    defparam i996_2_lut_3_lut.init = 16'h6f6f;
    LUT4 frame_toggle_sync_I_0_2_lut_rep_305 (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .Z(pll_clk_enable_24)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[13:51])
    defparam frame_toggle_sync_I_0_2_lut_rep_305.init = 16'h6666;
    LUT4 mux_1731_i11_3_lut (.A(n12199), .B(n12200), .C(n12178), .Z(rd_data_15__N_2646[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i11_3_lut.init = 16'hcaca;
    LUT4 n26022_bdd_2_lut_3_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n26022), .Z(frame_settle_3__N_1982[3])) /* synthesis lut_function=(A ((C)+!B)+!A (B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[13:51])
    defparam n26022_bdd_2_lut_3_lut.init = 16'hf6f6;
    LUT4 i3518_2_lut_3_lut_4_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(stop_toggle_seen), .D(stop_toggle_sync), .Z(n14125)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D)))+!A !(B+!(C (D)+!C !(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[13:51])
    defparam i3518_2_lut_3_lut_4_lut.init = 16'h6ff6;
    LUT4 i1_3_lut_4_lut_3_lut_4_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n26270), .D(frame_settle[0]), .Z(pll_clk_enable_8)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[13:51])
    defparam i1_3_lut_4_lut_3_lut_4_lut.init = 16'h0900;
    PFUMX i15308 (.BLUT(n26080), .ALUT(n26079), .C0(ev_state[3]), .Z(ev_state_3__N_602[2]));
    LUT4 mux_1731_i12_3_lut (.A(n12201), .B(n12202), .C(n12178), .Z(rd_data_15__N_2646[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i12_3_lut.init = 16'hcaca;
    LUT4 i2_3_lut_rep_306 (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .Z(n26270)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(512[22:42])
    defparam i2_3_lut_rep_306.init = 16'hfefe;
    LUT4 mux_1731_i13_3_lut (.A(n12203), .B(n12204), .C(n12178), .Z(rd_data_15__N_2646[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i13_3_lut.init = 16'hcaca;
    LUT4 frame_settle_3__bdd_4_lut (.A(frame_settle[3]), .B(frame_settle[1]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(n26022)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;
    defparam frame_settle_3__bdd_4_lut.init = 16'haaa9;
    LUT4 mux_1731_i14_3_lut (.A(n12205), .B(n12206), .C(n12178), .Z(rd_data_15__N_2646[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i14_3_lut.init = 16'hcaca;
    LUT4 mux_1731_i15_3_lut (.A(n12207), .B(n12208), .C(n12178), .Z(rd_data_15__N_2646[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i15_3_lut.init = 16'hcaca;
    FD1S3AX staging_rd_addr_i1 (.D(staging_rd_addr_6__N_722[1]), .CK(pll_clk), 
            .Q(staging_rd_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam staging_rd_addr_i1.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i2 (.D(staging_rd_addr_6__N_722[2]), .CK(pll_clk), 
            .Q(staging_rd_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam staging_rd_addr_i2.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i3 (.D(staging_rd_addr_6__N_722[3]), .CK(pll_clk), 
            .Q(staging_rd_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam staging_rd_addr_i3.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i4 (.D(staging_rd_addr_6__N_722[4]), .CK(pll_clk), 
            .Q(staging_rd_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam staging_rd_addr_i4.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i5 (.D(staging_rd_addr_6__N_722[5]), .CK(pll_clk), 
            .Q(staging_rd_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam staging_rd_addr_i5.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i6 (.D(staging_rd_addr_6__N_722[6]), .CK(pll_clk), 
            .Q(staging_rd_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam staging_rd_addr_i6.GSR = "DISABLED";
    LUT4 mux_1731_i16_3_lut (.A(n12209), .B(n12210), .C(n12178), .Z(rd_data_15__N_2646[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i16_3_lut.init = 16'hcaca;
    LUT4 i2469_3_lut (.A(rgb_hold[17]), .B(shift_register[0]), .C(state[1]), 
         .Z(n12991)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2469_3_lut.init = 16'hcaca;
    FD1P3AX pending_sequence_i1 (.D(accepted_sequence_sync[1]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i1.GSR = "DISABLED";
    FD1P3AX pending_sequence_i2 (.D(accepted_sequence_sync[2]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i2.GSR = "DISABLED";
    FD1P3AX pending_sequence_i3 (.D(accepted_sequence_sync[3]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i3.GSR = "DISABLED";
    FD1P3AX pending_sequence_i4 (.D(accepted_sequence_sync[4]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i4.GSR = "DISABLED";
    FD1P3AX pending_sequence_i5 (.D(accepted_sequence_sync[5]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i5.GSR = "DISABLED";
    FD1P3AX pending_sequence_i6 (.D(accepted_sequence_sync[6]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i6.GSR = "DISABLED";
    FD1P3AX pending_sequence_i7 (.D(accepted_sequence_sync[7]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i7.GSR = "DISABLED";
    FD1P3AX pending_sequence_i8 (.D(accepted_sequence_sync[8]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i8.GSR = "DISABLED";
    FD1P3AX pending_sequence_i9 (.D(accepted_sequence_sync[9]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i9.GSR = "DISABLED";
    FD1P3AX pending_sequence_i10 (.D(accepted_sequence_sync[10]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i10.GSR = "DISABLED";
    FD1P3AX pending_sequence_i11 (.D(accepted_sequence_sync[11]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i11.GSR = "DISABLED";
    FD1P3AX pending_sequence_i12 (.D(accepted_sequence_sync[12]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i12.GSR = "DISABLED";
    FD1P3AX pending_sequence_i13 (.D(accepted_sequence_sync[13]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i13.GSR = "DISABLED";
    FD1P3AX pending_sequence_i14 (.D(accepted_sequence_sync[14]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i14.GSR = "DISABLED";
    FD1P3AX pending_sequence_i15 (.D(accepted_sequence_sync[15]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i15.GSR = "DISABLED";
    FD1P3AX pending_sequence_i16 (.D(accepted_sequence_sync[16]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i16.GSR = "DISABLED";
    FD1P3AX pending_sequence_i17 (.D(accepted_sequence_sync[17]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i17.GSR = "DISABLED";
    FD1P3AX pending_sequence_i18 (.D(accepted_sequence_sync[18]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i18.GSR = "DISABLED";
    FD1P3AX pending_sequence_i19 (.D(accepted_sequence_sync[19]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i19.GSR = "DISABLED";
    FD1P3AX pending_sequence_i20 (.D(accepted_sequence_sync[20]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i20.GSR = "DISABLED";
    FD1P3AX pending_sequence_i21 (.D(accepted_sequence_sync[21]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i21.GSR = "DISABLED";
    FD1P3AX pending_sequence_i22 (.D(accepted_sequence_sync[22]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i22.GSR = "DISABLED";
    FD1P3AX pending_sequence_i23 (.D(accepted_sequence_sync[23]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i23.GSR = "DISABLED";
    FD1P3AX pending_sequence_i24 (.D(accepted_sequence_sync[24]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i24.GSR = "DISABLED";
    FD1P3AX pending_sequence_i25 (.D(accepted_sequence_sync[25]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i25.GSR = "DISABLED";
    FD1P3AX pending_sequence_i26 (.D(accepted_sequence_sync[26]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i26.GSR = "DISABLED";
    FD1P3AX pending_sequence_i27 (.D(accepted_sequence_sync[27]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i27.GSR = "DISABLED";
    FD1P3AX pending_sequence_i28 (.D(accepted_sequence_sync[28]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i28.GSR = "DISABLED";
    FD1P3AX pending_sequence_i29 (.D(accepted_sequence_sync[29]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i29.GSR = "DISABLED";
    FD1P3AX pending_sequence_i30 (.D(accepted_sequence_sync[30]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i30.GSR = "DISABLED";
    FD1P3AX pending_sequence_i31 (.D(accepted_sequence_sync[31]), .SP(pll_clk_enable_355), 
            .CK(pll_clk), .Q(pending_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam pending_sequence_i31.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i2 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i3 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i4 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i5 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i6 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i7 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i8 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i9 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i10 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i11 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i12 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i13 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i14 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i15 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_0_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_0_r__i15.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i2 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i3 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i4 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i5 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i6 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i7 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i8 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i9 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i10 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i11 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i12 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i13 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i14 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i15 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_383), 
            .CK(pll_clk), .Q(mic_shift_1_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_shift_1_r__i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i1 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i2 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i3 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i4 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i5 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i6 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i7 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i8 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i9 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i10 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i11 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i12 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i13 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i14 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i15 (.D(mic_shift_1_r[14]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i16 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i16.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i17 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i17.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i18 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i18.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i19 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i19.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i20 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i20.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i21 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i21.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i22 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i22.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i23 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i23.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i24 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i24.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i25 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i26 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i26.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i27 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i27.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i28 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i28.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i29 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i29.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i30 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i30.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i31 (.D(mic_shift_1_l[15]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i31.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i32 (.D(mic_data_0_c), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i32.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i33 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i33.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i34 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i34.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i35 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i35.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i36 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i36.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i37 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i37.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i38 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i38.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i39 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i39.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i40 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i40.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i41 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i41.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i42 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i42.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i43 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i43.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i44 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i44.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i45 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i45.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i46 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i46.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i47 (.D(mic_shift_0_r[14]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i47.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i48 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i48.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i49 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_432), 
            .CK(pll_clk), .Q(mic_latest[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i49.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i50 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i50.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i51 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i51.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i52 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i52.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i53 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i53.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i54 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i54.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i55 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i55.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i56 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i56.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i57 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i57.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i58 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i58.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i59 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i59.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i60 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i60.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i61 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i61.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i62 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i62.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i63 (.D(mic_shift_0_l[15]), .SP(pll_clk_enable_446), 
            .CK(pll_clk), .Q(mic_latest[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mic_latest_i0_i63.GSR = "DISABLED";
    LUT4 n21588_bdd_2_lut_15304_4_lut (.A(frame_req_N_2584), .B(n26207), 
         .C(ev_state[0]), .D(ev_state[1]), .Z(n26058)) /* synthesis lut_function=(A (B+((D)+!C))+!A (B (C+(D))+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam n21588_bdd_2_lut_15304_4_lut.init = 16'hffca;
    LUT4 i2474_3_lut (.A(rgb_hold[18]), .B(shift_register[1]), .C(state[1]), 
         .Z(n12996)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2474_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_rep_257_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(pll_clk_enable_20)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(512[22:42])
    defparam i1_2_lut_rep_257_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_adj_74 (.A(spi_byte_count[0]), .B(n25544), .Z(spi1_sck_c_enable_8)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_74.init = 16'h4444;
    LUT4 i6561_2_lut_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .D(frame_settle[0]), .Z(n17186)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(512[22:42])
    defparam i6561_2_lut_4_lut.init = 16'h00fe;
    LUT4 i4_4_lut (.A(spi_byte_count[3]), .B(n8), .C(n25457), .D(n21531), 
         .Z(n25544)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i4_4_lut.init = 16'h0004;
    CCU2D spi_channel_index_1591_add_4_7 (.A0(spi_channel_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24975), .S0(n35_adj_3183), 
          .S1(n34_adj_3184));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591_add_4_7.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1591_add_4_7.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1591_add_4_7.INJECT1_0 = "NO";
    defparam spi_channel_index_1591_add_4_7.INJECT1_1 = "NO";
    LUT4 i3_4_lut_adj_75 (.A(n26274), .B(spi1_sck_c_enable_135), .C(n21234), 
         .D(n4_adj_3166), .Z(n8)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i3_4_lut_adj_75.init = 16'h0004;
    LUT4 i2478_3_lut (.A(rgb_hold[19]), .B(shift_register[2]), .C(state[1]), 
         .Z(n13000)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2478_3_lut.init = 16'hcaca;
    CCU2D add_244_15 (.A0(spi_byte_count[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24904), .COUT(n24905), .S0(spi_byte_count_15__N_1504[13]), 
          .S1(spi_byte_count_15__N_1504[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[35:57])
    defparam add_244_15.INIT0 = 16'h5aaa;
    defparam add_244_15.INIT1 = 16'h5aaa;
    defparam add_244_15.INJECT1_0 = "NO";
    defparam add_244_15.INJECT1_1 = "NO";
    LUT4 i2547_3_lut (.A(rgb_hold[20]), .B(shift_register[3]), .C(state[1]), 
         .Z(n13071)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2547_3_lut.init = 16'hcaca;
    LUT4 i2815_3_lut (.A(rgb_hold[21]), .B(shift_register[4]), .C(state[1]), 
         .Z(n13339)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2815_3_lut.init = 16'hcaca;
    CCU2D spi_channel_index_1591_add_4_5 (.A0(spi_channel_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24974), .COUT(n24975), .S0(n37_adj_3170), 
          .S1(n36_adj_3171));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591_add_4_5.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1591_add_4_5.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1591_add_4_5.INJECT1_0 = "NO";
    defparam spi_channel_index_1591_add_4_5.INJECT1_1 = "NO";
    LUT4 i2819_3_lut (.A(rgb_hold[22]), .B(shift_register[5]), .C(state[1]), 
         .Z(n13343)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2819_3_lut.init = 16'hcaca;
    CCU2D spi_channel_index_1591_add_4_3 (.A0(spi_channel_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n24973), .COUT(n24974), .S0(n39_adj_3168), 
          .S1(n38_adj_3169));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591_add_4_3.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1591_add_4_3.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1591_add_4_3.INJECT1_0 = "NO";
    defparam spi_channel_index_1591_add_4_3.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_76 (.A(n26208), .B(n26214), .C(ev_rd_hold[3]), 
         .D(n26147), .Z(ev_wr_data[3])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_76.init = 16'hf100;
    LUT4 i2_3_lut_rep_307 (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_bit_index[1]), .Z(n26271)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam i2_3_lut_rep_307.init = 16'h8080;
    LUT4 i2833_3_lut (.A(rgb_hold[23]), .B(shift_register[6]), .C(state[1]), 
         .Z(n13357)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2833_3_lut.init = 16'hcaca;
    CCU2D add_244_13 (.A0(spi_byte_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24903), .COUT(n24904), .S0(spi_byte_count_15__N_1504[11]), 
          .S1(spi_byte_count_15__N_1504[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[35:57])
    defparam add_244_13.INIT0 = 16'h5aaa;
    defparam add_244_13.INIT1 = 16'h5aaa;
    defparam add_244_13.INJECT1_0 = "NO";
    defparam add_244_13.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_308 (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .Z(n26272)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_308.init = 16'heeee;
    CCU2D mic_divider_1599_add_4_7 (.A0(mic_divider[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24927), .S0(n35_adj_3189), .S1(n34_adj_3188));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_divider_1599_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_divider_1599_add_4_7.INJECT1_0 = "NO";
    defparam mic_divider_1599_add_4_7.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_adj_77 (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .C(spi_byte_count[8]), .Z(n21531)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_3_lut_adj_77.init = 16'hfefe;
    CCU2D add_244_11 (.A0(spi_byte_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24902), .COUT(n24903), .S0(spi_byte_count_15__N_1504[9]), 
          .S1(spi_byte_count_15__N_1504[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[35:57])
    defparam add_244_11.INIT0 = 16'h5aaa;
    defparam add_244_11.INIT1 = 16'h5aaa;
    defparam add_244_11.INJECT1_0 = "NO";
    defparam add_244_11.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_777_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(expected_next_15__N_1351[3]), .B1(spi_extension_length[2]), 
          .C1(GND_net), .D1(GND_net), .COUT(n24886), .S1(expected_next[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[33] 314[86])
    defparam expected_next_15__I_0_777_1.INIT0 = 16'hF000;
    defparam expected_next_15__I_0_777_1.INIT1 = 16'ha999;
    defparam expected_next_15__I_0_777_1.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_777_1.INJECT1_1 = "NO";
    CCU2D spi_channel_index_1591_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_channel_index[0]), .B1(n25024), .C1(spi_channel_index[2]), 
          .D1(n5), .COUT(n24973), .S1(n40_adj_3182));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591_add_4_1.INIT0 = 16'hF000;
    defparam spi_channel_index_1591_add_4_1.INIT1 = 16'h5559;
    defparam spi_channel_index_1591_add_4_1.INJECT1_0 = "NO";
    defparam spi_channel_index_1591_add_4_1.INJECT1_1 = "NO";
    LUT4 i14382_2_lut_rep_309 (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .Z(n26273)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i14382_2_lut_rep_309.init = 16'h8888;
    LUT4 i2837_3_lut (.A(rgb_hold[8]), .B(shift_register[7]), .C(state[1]), 
         .Z(n13361)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2837_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_rep_175_3_lut_4_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(n65), .D(spi_bit_count[2]), .Z(n26139)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i1_2_lut_rep_175_3_lut_4_lut.init = 16'h0800;
    LUT4 i1_3_lut_4_lut_adj_78 (.A(n26208), .B(n26214), .C(init_shadow[3]), 
         .D(n16988), .Z(n14838)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_78.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_268_3_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .Z(spi1_sck_c_enable_135)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i1_2_lut_rep_268_3_lut.init = 16'h8080;
    LUT4 i2841_3_lut (.A(rgb_hold[9]), .B(shift_register[8]), .C(state[1]), 
         .Z(n13365)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2841_3_lut.init = 16'hcaca;
    LUT4 i14386_2_lut_3_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .Z(n18)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i14386_2_lut_3_lut.init = 16'h7878;
    LUT4 i2845_3_lut (.A(rgb_hold[10]), .B(shift_register[9]), .C(state[1]), 
         .Z(n13369)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2845_3_lut.init = 16'hcaca;
    LUT4 i10821_2_lut_rep_176_3_lut_4_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(fpga_cs_n_c), .D(spi_bit_count[2]), .Z(n26140)) /* synthesis lut_function=(((C+!(D))+!B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i10821_2_lut_rep_176_3_lut_4_lut.init = 16'hf7ff;
    FD1P3AX accepted_sequence_spi_i0_i1 (.D(spi_frame_sequence[1]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i1.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i2 (.D(spi_frame_sequence[2]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i3 (.D(spi_frame_sequence[3]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i4 (.D(spi_frame_sequence[4]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i5 (.D(spi_frame_sequence[5]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i6 (.D(spi_frame_sequence[6]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i7 (.D(spi_frame_sequence[7]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i8 (.D(spi_frame_sequence[8]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i9 (.D(spi_frame_sequence[9]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i10 (.D(spi_frame_sequence[10]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i11 (.D(spi_frame_sequence[11]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i12 (.D(spi_frame_sequence[12]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i13 (.D(spi_frame_sequence[13]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i14 (.D(spi_frame_sequence[14]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i15 (.D(spi_frame_sequence[15]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i16 (.D(spi_frame_sequence[16]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i17 (.D(spi_frame_sequence[17]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[17]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i18 (.D(spi_frame_sequence[18]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i19 (.D(spi_frame_sequence[19]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[19]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i20 (.D(spi_frame_sequence[20]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i21 (.D(spi_frame_sequence[21]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[21]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i22 (.D(spi_frame_sequence[22]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i23 (.D(spi_frame_sequence[23]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[23]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i24 (.D(spi_frame_sequence[24]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i25 (.D(spi_frame_sequence[25]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[25]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i26 (.D(spi_frame_sequence[26]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[26]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i27 (.D(spi_frame_sequence[27]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[27]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i28 (.D(spi_frame_sequence[28]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[28]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i29 (.D(spi_frame_sequence[29]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[29]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i30 (.D(spi_frame_sequence[30]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[30]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i31 (.D(spi_frame_sequence[31]), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[31]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam accepted_sequence_spi_i0_i31.GSR = "DISABLED";
    LUT4 i2849_3_lut (.A(rgb_hold[11]), .B(shift_register[10]), .C(state[1]), 
         .Z(n13373)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2849_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_79 (.A(n26208), .B(n26235), .C(init_shadow[75]), 
         .D(n16988), .Z(n14406)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_79.init = 16'hf1f0;
    spi_mic_stream mic_stream_i (.sck_N_3050(sck_N_3050), .spi_mic_cs_n_c(spi_mic_cs_n_c), 
            .mic_latest({mic_latest}), .spi_mic_miso_c(spi_mic_miso_c)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(703[20] 706[6])
    FD1S3IX mic_divider_1599__i1 (.D(n39_adj_3193), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599__i1.GSR = "DISABLED";
    FD1S3IX mic_divider_1599__i2 (.D(n38_adj_3192), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599__i2.GSR = "DISABLED";
    FD1S3IX mic_divider_1599__i3 (.D(n37_adj_3191), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599__i3.GSR = "DISABLED";
    FD1S3IX mic_divider_1599__i4 (.D(n36_adj_3190), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599__i4.GSR = "DISABLED";
    FD1S3IX mic_divider_1599__i5 (.D(n35_adj_3189), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599__i5.GSR = "DISABLED";
    FD1S3IX mic_divider_1599__i6 (.D(n34_adj_3188), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599__i6.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_310 (.A(spi_byte_count[11]), .B(spi_byte_count[12]), 
         .Z(n26274)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_310.init = 16'heeee;
    LUT4 i1_2_lut_3_lut_4_lut_adj_80 (.A(ev_state[3]), .B(ev_state[2]), 
         .C(ev_state_3__N_1990[2]), .D(ev_state[1]), .Z(n25548)) /* synthesis lut_function=(!((B+(C (D)))+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_80.init = 16'h0222;
    LUT4 i1_2_lut_rep_262_3_lut (.A(spi_byte_count[11]), .B(spi_byte_count[12]), 
         .C(spi_byte_count[10]), .Z(n26226)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_rep_262_3_lut.init = 16'hfefe;
    FD1P3AX spi_rgb_index_1593__i1 (.D(n24_adj_3164), .SP(spi1_sck_c_enable_169), 
            .CK(spi1_sck_c), .Q(spi_rgb_index[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(404[46:66])
    defparam spi_rgb_index_1593__i1.GSR = "ENABLED";
    LUT4 i2_3_lut_4_lut_adj_81 (.A(spi_byte_count[11]), .B(spi_byte_count[12]), 
         .C(n25457), .D(n26275), .Z(n21554)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_81.init = 16'hfffe;
    FD1P3AX spi_rgb_index_1593__i2 (.D(n23), .SP(spi1_sck_c_enable_169), 
            .CK(spi1_sck_c), .Q(spi_rgb_index[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(404[46:66])
    defparam spi_rgb_index_1593__i2.GSR = "ENABLED";
    FD1P3AX spi_rgb_index_1593__i3 (.D(n22_adj_3202), .SP(spi1_sck_c_enable_169), 
            .CK(spi1_sck_c), .Q(spi_rgb_index[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(404[46:66])
    defparam spi_rgb_index_1593__i3.GSR = "ENABLED";
    FD1S3AX spi_bit_count_1594__i1 (.D(n19), .CK(spi1_sck_c), .Q(spi_bit_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam spi_bit_count_1594__i1.GSR = "ENABLED";
    LUT4 i3285_4_lut (.A(ev_run_hold_s5[83]), .B(us_tx_c_83), .C(init_shadow[83]), 
         .D(swap_now_s5), .Z(n13892)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(680[26] 681[64])
    defparam i3285_4_lut.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i1 (.D(ev_clear_addr_7__N_2215[1]), .SP(pll_clk_enable_639), 
            .CD(n17861), .CK(pll_clk), .Q(ev_clear_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_clear_addr_i1.GSR = "DISABLED";
    FD1P3IX init_shadow_i83 (.D(n14358), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i83.GSR = "DISABLED";
    FD1P3IX init_shadow_i82 (.D(n14364), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i82.GSR = "DISABLED";
    FD1P3IX init_shadow_i81 (.D(n14370), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i81.GSR = "DISABLED";
    FD1P3IX init_shadow_i80 (.D(n14376), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i80.GSR = "DISABLED";
    FD1P3IX init_shadow_i79 (.D(n14382), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i79.GSR = "DISABLED";
    FD1P3IX init_shadow_i78 (.D(n14388), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i78.GSR = "DISABLED";
    FD1P3IX init_shadow_i77 (.D(n14394), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i77.GSR = "DISABLED";
    FD1P3IX init_shadow_i76 (.D(n14400), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i76.GSR = "DISABLED";
    FD1P3IX init_shadow_i75 (.D(n14406), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i75.GSR = "DISABLED";
    FD1P3IX init_shadow_i74 (.D(n14412), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i74.GSR = "DISABLED";
    FD1P3IX init_shadow_i73 (.D(n14418), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i73.GSR = "DISABLED";
    FD1P3IX init_shadow_i72 (.D(n14424), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i72.GSR = "DISABLED";
    FD1P3IX init_shadow_i71 (.D(n14430), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i71.GSR = "DISABLED";
    FD1P3IX init_shadow_i70 (.D(n14436), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i70.GSR = "DISABLED";
    FD1P3IX init_shadow_i69 (.D(n14442), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i69.GSR = "DISABLED";
    FD1P3IX init_shadow_i68 (.D(n14448), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i68.GSR = "DISABLED";
    FD1P3IX init_shadow_i67 (.D(n14454), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i67.GSR = "DISABLED";
    FD1P3IX init_shadow_i66 (.D(n14460), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i66.GSR = "DISABLED";
    FD1P3IX init_shadow_i65 (.D(n14466), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i65.GSR = "DISABLED";
    FD1P3IX init_shadow_i64 (.D(n14472), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i64.GSR = "DISABLED";
    FD1P3IX init_shadow_i63 (.D(n14478), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i63.GSR = "DISABLED";
    FD1P3IX init_shadow_i62 (.D(n14484), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i62.GSR = "DISABLED";
    LUT4 i15272_4_lut (.A(spi1_sck_c_enable_135), .B(spi_byte_count[15]), 
         .C(n11), .D(n25679), .Z(spi1_sck_c_enable_169)) /* synthesis lut_function=(A (B+((D)+!C))) */ ;
    defparam i15272_4_lut.init = 16'haa8a;
    LUT4 i15269_2_lut_2_lut_3_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[1]), .D(ev_state[0]), .Z(pll_clk_enable_241)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i15269_2_lut_2_lut_3_lut_4_lut.init = 16'h0040;
    LUT4 i1_3_lut_4_lut_adj_82 (.A(n26209), .B(n26214), .C(ev_rd_hold[4]), 
         .D(n26147), .Z(ev_wr_data[4])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_82.init = 16'hf100;
    FD1P3IX init_shadow_i61 (.D(n14490), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i61.GSR = "DISABLED";
    LUT4 i14135_1_lut (.A(status_bit_index[1]), .Z(spi1_miso_N_2492[1])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam i14135_1_lut.init = 16'h5555;
    FD1S3AX spi_bit_count_1594__i2 (.D(n18), .CK(spi1_sck_c), .Q(spi_bit_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam spi_bit_count_1594__i2.GSR = "ENABLED";
    INV i15394 (.A(spi1_sck_c), .Z(spi1_sck_N_305));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    FD1P3AX ev_run_hold_s5_i0_i1 (.D(ev_rd_data[1]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i1.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i2 (.D(ev_rd_data[2]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i2.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i3 (.D(ev_rd_data[3]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i3.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i4 (.D(ev_rd_data[4]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i4.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i5 (.D(ev_rd_data[5]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i5.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i6 (.D(ev_rd_data[6]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i6.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i7 (.D(ev_rd_data[7]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i7.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i8 (.D(ev_rd_data[8]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i8.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i9 (.D(ev_rd_data[9]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i9.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i10 (.D(ev_rd_data[10]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i10.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i11 (.D(ev_rd_data[11]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i11.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i12 (.D(ev_rd_data[12]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i12.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i13 (.D(ev_rd_data[13]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i13.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i14 (.D(ev_rd_data[14]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i14.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i15 (.D(ev_rd_data[15]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i15.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i16 (.D(ev_rd_data[16]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i16.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i17 (.D(ev_rd_data[17]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i17.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i18 (.D(ev_rd_data[18]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i18.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i19 (.D(ev_rd_data[19]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i19.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i20 (.D(ev_rd_data[20]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i20.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i21 (.D(ev_rd_data[21]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i21.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i22 (.D(ev_rd_data[22]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i22.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i23 (.D(ev_rd_data[23]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i23.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i24 (.D(ev_rd_data[24]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i24.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i25 (.D(ev_rd_data[25]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i25.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i26 (.D(ev_rd_data[26]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i26.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i27 (.D(ev_rd_data[27]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i27.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i28 (.D(ev_rd_data[28]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i28.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i29 (.D(ev_rd_data[29]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i29.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i30 (.D(ev_rd_data[30]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i30.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i31 (.D(ev_rd_data[31]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i31.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i32 (.D(ev_rd_data[32]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i32.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i33 (.D(ev_rd_data[33]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i33.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i34 (.D(ev_rd_data[34]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i34.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i35 (.D(ev_rd_data[35]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i35.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i36 (.D(ev_rd_data[36]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i36.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i37 (.D(ev_rd_data[37]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i37.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i38 (.D(ev_rd_data[38]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i38.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i39 (.D(ev_rd_data[39]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i39.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i40 (.D(ev_rd_data[40]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i40.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i41 (.D(ev_rd_data[41]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i41.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i42 (.D(ev_rd_data[42]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i42.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i43 (.D(ev_rd_data[43]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i43.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i44 (.D(ev_rd_data[44]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i44.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i45 (.D(ev_rd_data[45]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i45.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i46 (.D(ev_rd_data[46]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i46.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i47 (.D(ev_rd_data[47]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i47.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i48 (.D(ev_rd_data[48]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i48.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i49 (.D(ev_rd_data[49]), .SP(pll_clk_enable_542), 
            .CK(pll_clk), .Q(ev_run_hold_s5[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i49.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i50 (.D(ev_rd_data[50]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i50.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i51 (.D(ev_rd_data[51]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i51.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i52 (.D(ev_rd_data[52]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i52.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i53 (.D(ev_rd_data[53]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i53.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i54 (.D(ev_rd_data[54]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i54.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i55 (.D(ev_rd_data[55]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i55.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i56 (.D(ev_rd_data[56]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i56.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i57 (.D(ev_rd_data[57]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i57.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i58 (.D(ev_rd_data[58]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i58.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i59 (.D(ev_rd_data[59]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i59.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i60 (.D(ev_rd_data[60]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i60.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i61 (.D(ev_rd_data[61]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i61.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i62 (.D(ev_rd_data[62]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i62.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i63 (.D(ev_rd_data[63]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i63.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i64 (.D(ev_rd_data[64]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i64.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i65 (.D(ev_rd_data[65]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i65.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i66 (.D(ev_rd_data[66]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i66.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i67 (.D(ev_rd_data[67]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i67.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i68 (.D(ev_rd_data[68]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i68.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i69 (.D(ev_rd_data[69]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i69.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i70 (.D(ev_rd_data[70]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i70.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i71 (.D(ev_rd_data[71]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i71.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i72 (.D(ev_rd_data[72]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i72.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i73 (.D(ev_rd_data[73]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i73.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i74 (.D(ev_rd_data[74]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i74.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i75 (.D(ev_rd_data[75]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i75.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i76 (.D(ev_rd_data[76]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i76.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i77 (.D(ev_rd_data[77]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i77.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i78 (.D(ev_rd_data[78]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i78.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i79 (.D(ev_rd_data[79]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i79.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i80 (.D(ev_rd_data[80]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i80.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i81 (.D(ev_rd_data[81]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i81.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i82 (.D(ev_rd_data[82]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i82.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i83 (.D(ev_rd_data[83]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_run_hold_s5_i0_i83.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_83 (.A(n26209), .B(n26214), .C(init_shadow[4]), 
         .D(n16988), .Z(n14832)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_83.init = 16'hf1f0;
    FD1P3IX init_shadow_i60 (.D(n14496), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i60.GSR = "DISABLED";
    FD1P3IX init_shadow_i59 (.D(n14502), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i59.GSR = "DISABLED";
    FD1P3IX init_shadow_i58 (.D(n14508), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i58.GSR = "DISABLED";
    FD1P3IX init_shadow_i57 (.D(n14514), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i57.GSR = "DISABLED";
    FD1P3IX init_shadow_i56 (.D(n14520), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i56.GSR = "DISABLED";
    FD1P3IX init_shadow_i55 (.D(n14526), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i55.GSR = "DISABLED";
    FD1P3IX init_shadow_i54 (.D(n14532), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i54.GSR = "DISABLED";
    FD1P3IX init_shadow_i53 (.D(n14538), .SP(pll_clk_enable_587), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i53.GSR = "DISABLED";
    FD1P3IX init_shadow_i52 (.D(n14544), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i52.GSR = "DISABLED";
    FD1P3IX init_shadow_i51 (.D(n14550), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i51.GSR = "DISABLED";
    FD1P3IX init_shadow_i50 (.D(n14556), .SP(pll_clk_enable_587), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i50.GSR = "DISABLED";
    FD1P3IX init_shadow_i49 (.D(n14562), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i49.GSR = "DISABLED";
    FD1P3IX init_shadow_i48 (.D(n14568), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i48.GSR = "DISABLED";
    FD1P3IX init_shadow_i47 (.D(n14574), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i47.GSR = "DISABLED";
    FD1P3IX init_shadow_i46 (.D(n14580), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i46.GSR = "DISABLED";
    FD1P3IX init_shadow_i45 (.D(n14586), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i45.GSR = "DISABLED";
    FD1P3IX init_shadow_i44 (.D(n14592), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i44.GSR = "DISABLED";
    FD1P3IX init_shadow_i43 (.D(n14598), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i43.GSR = "DISABLED";
    VLO i1 (.Z(GND_net));
    FD1P3IX init_shadow_i42 (.D(n14604), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i42.GSR = "DISABLED";
    FD1P3IX init_shadow_i41 (.D(n14610), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i41.GSR = "DISABLED";
    FD1P3IX init_shadow_i40 (.D(n14616), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i40.GSR = "DISABLED";
    FD1P3IX init_shadow_i39 (.D(n14622), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i39.GSR = "DISABLED";
    FD1P3IX init_shadow_i38 (.D(n14628), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i38.GSR = "DISABLED";
    FD1P3IX init_shadow_i37 (.D(n14634), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i37.GSR = "DISABLED";
    FD1P3IX init_shadow_i36 (.D(n14640), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i36.GSR = "DISABLED";
    FD1P3IX init_shadow_i35 (.D(n14646), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i35.GSR = "DISABLED";
    FD1P3IX init_shadow_i34 (.D(n14652), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i34.GSR = "DISABLED";
    FD1P3IX init_shadow_i33 (.D(n14658), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i33.GSR = "DISABLED";
    FD1P3IX init_shadow_i32 (.D(n14664), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i32.GSR = "DISABLED";
    FD1P3IX init_shadow_i31 (.D(n14670), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i31.GSR = "DISABLED";
    FD1P3IX init_shadow_i30 (.D(n14676), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i30.GSR = "DISABLED";
    FD1P3IX init_shadow_i29 (.D(n14682), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i29.GSR = "DISABLED";
    FD1P3IX init_shadow_i28 (.D(n14688), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i28.GSR = "DISABLED";
    FD1P3IX init_shadow_i27 (.D(n14694), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i27.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_311 (.A(spi_byte_count[13]), .B(spi_byte_count[14]), 
         .Z(n26275)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_311.init = 16'heeee;
    FD1P3IX init_shadow_i26 (.D(n14700), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i26.GSR = "DISABLED";
    LUT4 i1_3_lut_rep_162 (.A(expected_next_15__N_1351[3]), .B(n11301), 
         .C(n4), .Z(n26126)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_3_lut_rep_162.init = 16'h2020;
    FD1P3IX init_shadow_i25 (.D(n14706), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i25.GSR = "DISABLED";
    FD1P3IX init_shadow_i24 (.D(n14712), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i24.GSR = "DISABLED";
    FD1P3IX init_shadow_i23 (.D(n14718), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i23.GSR = "DISABLED";
    FD1P3IX init_shadow_i22 (.D(n14724), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i22.GSR = "DISABLED";
    FD1P3IX init_shadow_i21 (.D(n14730), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i21.GSR = "DISABLED";
    FD1P3IX init_shadow_i20 (.D(n14736), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i20.GSR = "DISABLED";
    FD1P3IX init_shadow_i19 (.D(n14742), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i19.GSR = "DISABLED";
    FD1P3IX init_shadow_i18 (.D(n14748), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i18.GSR = "DISABLED";
    FD1P3IX init_shadow_i17 (.D(n14754), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i17.GSR = "DISABLED";
    FD1P3IX init_shadow_i16 (.D(n14760), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i16.GSR = "DISABLED";
    FD1P3IX init_shadow_i15 (.D(n14766), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i15.GSR = "DISABLED";
    FD1P3IX init_shadow_i14 (.D(n14772), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i14.GSR = "DISABLED";
    FD1P3IX init_shadow_i13 (.D(n14778), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i13.GSR = "DISABLED";
    FD1P3IX init_shadow_i12 (.D(n14784), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i12.GSR = "DISABLED";
    FD1P3IX init_shadow_i11 (.D(n14790), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i11.GSR = "DISABLED";
    FD1P3IX init_shadow_i10 (.D(n14796), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i10.GSR = "DISABLED";
    FD1P3IX init_shadow_i9 (.D(n14802), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i9.GSR = "DISABLED";
    FD1P3IX init_shadow_i8 (.D(n14808), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i8.GSR = "DISABLED";
    FD1P3IX init_shadow_i7 (.D(n14814), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i7.GSR = "DISABLED";
    FD1P3IX init_shadow_i6 (.D(n14820), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i6.GSR = "DISABLED";
    FD1P3IX init_shadow_i5 (.D(n14826), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i5.GSR = "DISABLED";
    FD1P3IX init_shadow_i4 (.D(n14832), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i4.GSR = "DISABLED";
    LUT4 i2853_3_lut (.A(rgb_hold[12]), .B(shift_register[11]), .C(state[1]), 
         .Z(n13377)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2853_3_lut.init = 16'hcaca;
    LUT4 run_addr_s3_8__I_0_i3_3_lut (.A(run_addr_s3[2]), .B(ev_rd_slot[2]), 
         .C(n21588), .Z(event_rd_addr[2])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(241[33:74])
    defparam run_addr_s3_8__I_0_i3_3_lut.init = 16'hacac;
    LUT4 i15223_2_lut (.A(frame_settle[0]), .B(frame_settle[1]), .Z(n25768)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(516[13] 526[16])
    defparam i15223_2_lut.init = 16'h9999;
    LUT4 i1_2_lut_3_lut_4_lut_adj_84 (.A(spi_byte_count[13]), .B(spi_byte_count[14]), 
         .C(n21606), .D(spi_byte_count[15]), .Z(n4_adj_3200)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_84.init = 16'hfffe;
    LUT4 i1_2_lut_rep_261_3_lut (.A(spi_byte_count[13]), .B(spi_byte_count[14]), 
         .C(spi_byte_count[15]), .Z(n26225)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_rep_261_3_lut.init = 16'hfefe;
    LUT4 i10696_2_lut (.A(spi_byte_count[2]), .B(spi_byte_count[1]), .Z(n21234)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10696_2_lut.init = 16'heeee;
    LUT4 i2857_3_lut (.A(rgb_hold[13]), .B(shift_register[12]), .C(state[1]), 
         .Z(n13381)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2857_3_lut.init = 16'hcaca;
    LUT4 i2861_3_lut (.A(rgb_hold[14]), .B(shift_register[13]), .C(state[1]), 
         .Z(n13385)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2861_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_85 (.A(n26217), .B(n26216), .C(ev_rd_hold[48]), 
         .D(n26147), .Z(ev_wr_data[48])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_85.init = 16'hf100;
    LUT4 i2_3_lut (.A(spi_byte_count[10]), .B(spi_byte_count[9]), .C(spi_byte_count[15]), 
         .Z(n25457)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_3_lut.init = 16'hfefe;
    LUT4 i10667_2_lut_rep_312 (.A(spi_byte_count[5]), .B(spi_byte_count[4]), 
         .Z(n26276)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10667_2_lut_rep_312.init = 16'heeee;
    LUT4 i1_2_lut_3_lut_4_lut_adj_86 (.A(spi_byte_count[5]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[14]), .D(spi_byte_count[13]), .Z(n4_adj_3166)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_86.init = 16'hfffe;
    LUT4 i1_2_lut_rep_159_4_lut (.A(expected_next_15__N_1351[3]), .B(n11301), 
         .C(n4), .D(n26140), .Z(n26123)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i1_2_lut_rep_159_4_lut.init = 16'h0020;
    LUT4 i15241_4_lut (.A(status_bit_index[3]), .B(status_bit_index[6]), 
         .C(n26271), .D(n6_adj_3185), .Z(spi1_sck_N_305_enable_7)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i15241_4_lut.init = 16'h7fff;
    LUT4 equal_798_i10_2_lut_rep_313 (.A(ev_ch[3]), .B(ev_ch[4]), .Z(n26277)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam equal_798_i10_2_lut_rep_313.init = 16'hbbbb;
    LUT4 i1_3_lut_4_lut_adj_87 (.A(n26217), .B(n26216), .C(init_shadow[48]), 
         .D(n16988), .Z(n14568)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_87.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_269_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(n26233)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_269_3_lut_4_lut.init = 16'hfffb;
    LUT4 i1_3_lut_4_lut_adj_88 (.A(n26210), .B(n26214), .C(ev_rd_hold[5]), 
         .D(n26147), .Z(ev_wr_data[5])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_88.init = 16'hf100;
    LUT4 i2865_3_lut (.A(rgb_hold[15]), .B(shift_register[14]), .C(state[1]), 
         .Z(n13389)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2865_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_89 (.A(n26210), .B(n26214), .C(init_shadow[5]), 
         .D(n16988), .Z(n14826)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_89.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_90 (.A(n26211), .B(n26214), .C(ev_rd_hold[6]), 
         .D(n26147), .Z(ev_wr_data[6])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_90.init = 16'hf100;
    LUT4 i2871_3_lut (.A(rgb_hold[0]), .B(shift_register[15]), .C(state[1]), 
         .Z(n13395)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2871_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_adj_91 (.A(status_bit_index[5]), .B(status_bit_index[4]), 
         .Z(n6_adj_3185)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_91.init = 16'h8888;
    LUT4 i2878_3_lut (.A(rgb_hold[1]), .B(shift_register[16]), .C(state[1]), 
         .Z(n13402)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2878_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_rep_248_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(n26212)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_248_3_lut_4_lut.init = 16'hffbf;
    LUT4 i1_3_lut_4_lut_adj_92 (.A(n26211), .B(n26214), .C(init_shadow[6]), 
         .D(n16988), .Z(n14820)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_92.init = 16'hf1f0;
    LUT4 i1_2_lut_adj_93 (.A(spi_channel_field[0]), .B(n25471), .Z(spi1_sck_c_enable_179)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i1_2_lut_adj_93.init = 16'h8888;
    LUT4 i1_4_lut_adj_94 (.A(spi_command[0]), .B(spi1_sck_c_enable_36), 
         .C(n25685), .D(spi_command[4]), .Z(spi1_sck_c_enable_35)) /* synthesis lut_function=(A (B (C+(D)))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i1_4_lut_adj_94.init = 16'hccc4;
    LUT4 i15104_2_lut (.A(spi_command[1]), .B(n25683), .Z(n25685)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i15104_2_lut.init = 16'heeee;
    LUT4 i5556_2_lut_3_lut_4_lut (.A(n26247), .B(n26242), .C(pll_clk_enable_22), 
         .D(frame_req_N_2584), .Z(n16173)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i5556_2_lut_3_lut_4_lut.init = 16'hf1f0;
    LUT4 i1744_2_lut_rep_314 (.A(ev_ch[1]), .B(ev_ch[0]), .Z(n26278)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(609[27:39])
    defparam i1744_2_lut_rep_314.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_95 (.A(n26213), .B(n26216), .C(ev_rd_hold[49]), 
         .D(n26147), .Z(ev_wr_data[49])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_95.init = 16'hf100;
    LUT4 i1_3_lut_4_lut_adj_96 (.A(n26213), .B(n26216), .C(init_shadow[49]), 
         .D(n16988), .Z(n14562)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_96.init = 16'hf1f0;
    LUT4 i2882_3_lut (.A(rgb_hold[2]), .B(shift_register[17]), .C(state[1]), 
         .Z(n13406)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2882_3_lut.init = 16'hcaca;
    LUT4 i2_3_lut_rep_242_4_lut (.A(n26247), .B(n26242), .C(swap_pending), 
         .D(frame_req), .Z(n26206)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_3_lut_rep_242_4_lut.init = 16'hfffe;
    LUT4 i2895_3_lut (.A(rgb_hold[3]), .B(shift_register[18]), .C(state[1]), 
         .Z(n13419)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2895_3_lut.init = 16'hcaca;
    FD1P3AX global_phase_s2_1595__i1 (.D(n44), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595__i1.GSR = "DISABLED";
    LUT4 i2899_3_lut (.A(rgb_hold[4]), .B(shift_register[19]), .C(state[1]), 
         .Z(n13423)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2899_3_lut.init = 16'hcaca;
    LUT4 i2903_3_lut (.A(rgb_hold[5]), .B(shift_register[20]), .C(state[1]), 
         .Z(n13427)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2903_3_lut.init = 16'hcaca;
    LUT4 i1751_2_lut_rep_273_3_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[2]), 
         .Z(n26237)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(609[27:39])
    defparam i1751_2_lut_rep_273_3_lut.init = 16'h8080;
    LUT4 i11058_1_lut_3_lut_4_lut (.A(n26247), .B(n26242), .C(swap_pending), 
         .D(frame_req), .Z(fifo_credit_wire[0])) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;
    defparam i11058_1_lut_3_lut_4_lut.init = 16'h0001;
    LUT4 i1749_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[2]), 
         .Z(ev_ch_6__N_2010[2])) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(609[27:39])
    defparam i1749_2_lut_3_lut.init = 16'h7878;
    LUT4 i2907_3_lut (.A(rgb_hold[6]), .B(shift_register[21]), .C(state[1]), 
         .Z(n13431)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2907_3_lut.init = 16'hcaca;
    LUT4 i1758_2_lut_rep_227_3_lut_4_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(n26191)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(609[27:39])
    defparam i1758_2_lut_rep_227_3_lut_4_lut.init = 16'h8000;
    LUT4 i10811_2_lut_rep_291 (.A(ev_ch[1]), .B(ev_ch[2]), .Z(n26255)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10811_2_lut_rep_291.init = 16'h8888;
    LUT4 i1_2_lut_3_lut_adj_97 (.A(spi_rgb_index[0]), .B(n25526), .C(spi_rgb_index[1]), 
         .Z(spi1_sck_c_enable_112)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i1_2_lut_3_lut_adj_97.init = 16'h0404;
    LUT4 i2_3_lut_4_lut_adj_98 (.A(n26208), .B(n26212), .C(ev_state[0]), 
         .D(n26253), .Z(n25549)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i2_3_lut_4_lut_adj_98.init = 16'h1000;
    LUT4 i1756_2_lut_3_lut_4_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(ev_ch_6__N_2010[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(609[27:39])
    defparam i1756_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i10813_2_lut_rep_315 (.A(ev_ch[3]), .B(ev_ch[4]), .Z(n26279)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10813_2_lut_rep_315.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_99 (.A(n26208), .B(n26212), .C(n26147), .D(ev_rd_hold[83]), 
         .Z(ev_wr_data[83])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (C))) */ ;
    defparam i1_3_lut_4_lut_adj_99.init = 16'hf010;
    LUT4 i1_2_lut_rep_272_3_lut_4_lut (.A(ev_ch[3]), .B(ev_ch[4]), .C(ev_ch[6]), 
         .D(ev_ch[5]), .Z(n26236)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i1_2_lut_rep_272_3_lut_4_lut.init = 16'hfff7;
    LUT4 i2911_3_lut (.A(rgb_hold[7]), .B(shift_register[22]), .C(state[1]), 
         .Z(n13435)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i2911_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_100 (.A(n26208), .B(n26212), .C(init_shadow[83]), 
         .D(n16988), .Z(n14358)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_100.init = 16'hf1f0;
    LUT4 n21588_bdd_3_lut_15303 (.A(n21588), .B(ev_state[1]), .C(ev_state[0]), 
         .Z(n26057)) /* synthesis lut_function=(!(A (B (C)+!B !(C)))) */ ;
    defparam n21588_bdd_3_lut_15303.init = 16'h7d7d;
    LUT4 i1_3_lut_4_lut_adj_101 (.A(n26220), .B(n26216), .C(ev_rd_hold[50]), 
         .D(n26147), .Z(ev_wr_data[50])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_101.init = 16'hf100;
    LUT4 i1_3_lut_4_lut_adj_102 (.A(n26220), .B(n26216), .C(init_shadow[50]), 
         .D(n16988), .Z(n14556)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_102.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_103 (.A(n26208), .B(n26216), .C(ev_rd_hold[51]), 
         .D(n26147), .Z(ev_wr_data[51])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_103.init = 16'hf100;
    LUT4 i1_3_lut_4_lut_adj_104 (.A(n26208), .B(n26216), .C(init_shadow[51]), 
         .D(n16988), .Z(n14550)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_104.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_316 (.A(ev_ch[5]), .B(ev_ch[6]), .Z(n26280)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_316.init = 16'hdddd;
    LUT4 i1_2_lut_adj_105 (.A(spi_byte_count[0]), .B(n25544), .Z(spi1_sck_c_enable_43)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_105.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_106 (.A(n26209), .B(n26216), .C(ev_rd_hold[52]), 
         .D(n26147), .Z(ev_wr_data[52])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_106.init = 16'hf100;
    LUT4 i1_3_lut_4_lut_adj_107 (.A(n26209), .B(n26216), .C(init_shadow[52]), 
         .D(n16988), .Z(n14544)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_107.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_276_3_lut_4_lut (.A(ev_ch[5]), .B(ev_ch[6]), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(n26240)) /* synthesis lut_function=((B+(C+!(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_276_3_lut_4_lut.init = 16'hfdff;
    LUT4 i2_3_lut_rep_163 (.A(spi_command[1]), .B(n16929), .C(spi_command[0]), 
         .Z(n26127)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i2_3_lut_rep_163.init = 16'h0404;
    LUT4 equal_814_i12_2_lut_rep_275_3_lut_4_lut (.A(ev_ch[5]), .B(ev_ch[6]), 
         .C(ev_ch[4]), .D(ev_ch[3]), .Z(n26239)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam equal_814_i12_2_lut_rep_275_3_lut_4_lut.init = 16'hfffd;
    LUT4 i15280_4_lut (.A(spi_byte_count[0]), .B(spi_byte_count[5]), .C(n65), 
         .D(spi_byte_count[4]), .Z(n25826)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i15280_4_lut.init = 16'h0100;
    LUT4 i3_4_lut_adj_108 (.A(n21531), .B(n26275), .C(n25457), .D(n26274), 
         .Z(n65)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut_adj_108.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_109 (.A(n26213), .B(n26219), .C(ev_rd_hold[9]), 
         .D(n26147), .Z(ev_wr_data[9])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_109.init = 16'hf100;
    LUT4 n25474_bdd_4_lut (.A(n25474), .B(spi_byte_count[1]), .C(spi_byte_count[3]), 
         .D(spi_byte_count[2]), .Z(n1_adj_3167)) /* synthesis lut_function=(!((B (C+!(D))+!B ((D)+!C))+!A)) */ ;
    defparam n25474_bdd_4_lut.init = 16'h0820;
    LUT4 i1_2_lut_rep_246_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(n26210)) /* synthesis lut_function=(A+!(B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_246_3_lut.init = 16'hbfbf;
    LUT4 i1_3_lut_4_lut_adj_110 (.A(n26213), .B(n26219), .C(init_shadow[9]), 
         .D(n16988), .Z(n14802)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_110.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_111 (.A(n26210), .B(n26216), .C(ev_rd_hold[53]), 
         .D(n26147), .Z(ev_wr_data[53])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_111.init = 16'hf100;
    LUT4 equal_846_i11_2_lut_rep_286 (.A(ev_ch[5]), .B(ev_ch[6]), .Z(n26250)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam equal_846_i11_2_lut_rep_286.init = 16'hbbbb;
    LUT4 i25_4_lut (.A(ev_run_hold_s5[80]), .B(us_tx_c_80), .C(init_shadow[80]), 
         .D(swap_now_s5), .Z(n13886)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i25_4_lut.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_112 (.A(n26210), .B(n26216), .C(init_shadow[53]), 
         .D(n16988), .Z(n14538)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_112.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_260_3_lut_4_lut (.A(ev_ch[5]), .B(ev_ch[6]), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(n26224)) /* synthesis lut_function=((B+!(C (D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_260_3_lut_4_lut.init = 16'hdfff;
    LUT4 i1_2_lut_rep_252_3_lut_4_lut (.A(ev_ch[5]), .B(ev_ch[6]), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(n26216)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_252_3_lut_4_lut.init = 16'hffdf;
    CCU2D fpga_time_1596_add_4_33 (.A0(fpga_time[31]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24972), .S0(n134));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_33.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_33.INIT1 = 16'h0000;
    defparam fpga_time_1596_add_4_33.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_33.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_113 (.A(n26211), .B(n26216), .C(ev_rd_hold[54]), 
         .D(n26147), .Z(ev_wr_data[54])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_113.init = 16'hf100;
    LUT4 i15276_2_lut (.A(spi_byte_count[0]), .B(n1), .Z(spi1_sck_c_enable_51)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i15276_2_lut.init = 16'h4444;
    LUT4 frame_toggle_spi_I_0_2_lut_4_lut (.A(spi_command[1]), .B(n16929), 
         .C(spi_command[0]), .D(frame_toggle_spi), .Z(frame_toggle_spi_N_2505)) /* synthesis lut_function=(A (D)+!A (B (C (D)+!C !(D))+!B (D))) */ ;
    defparam frame_toggle_spi_I_0_2_lut_4_lut.init = 16'hfb04;
    LUT4 i1_3_lut_4_lut_adj_114 (.A(n26211), .B(n26216), .C(init_shadow[54]), 
         .D(n16988), .Z(n14532)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_114.init = 16'hf1f0;
    LUT4 i3283_4_lut (.A(ev_run_hold_s5[81]), .B(us_tx_c_81), .C(init_shadow[81]), 
         .D(swap_now_s5), .Z(n13888)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(680[26] 681[64])
    defparam i3283_4_lut.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_115 (.A(n26215), .B(n26214), .C(ev_rd_hold[7]), 
         .D(n26147), .Z(ev_wr_data[7])) /* synthesis lut_function=(A (B (C (D))+!B (D))+!A (C (D))) */ ;
    defparam i1_3_lut_4_lut_adj_115.init = 16'hf200;
    LUT4 run_addr_s3_8__I_0_i4_3_lut (.A(run_addr_s3[3]), .B(ev_rd_slot[3]), 
         .C(n21588), .Z(event_rd_addr[3])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(241[33:74])
    defparam run_addr_s3_8__I_0_i4_3_lut.init = 16'hacac;
    LUT4 i14379_2_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), .Z(n19)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i14379_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_116 (.A(n26215), .B(n26214), .C(init_shadow[7]), 
         .D(n16988), .Z(n14814)) /* synthesis lut_function=(A (B (C)+!B (C+(D)))+!A (C)) */ ;
    defparam i1_3_lut_4_lut_adj_116.init = 16'hf2f0;
    LUT4 i3_4_lut_adj_117 (.A(spi_byte_count[5]), .B(n26230), .C(spi_byte_count[4]), 
         .D(n26139), .Z(n1)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(342[17] 407[24])
    defparam i3_4_lut_adj_117.init = 16'h0800;
    LUT4 i1_3_lut_4_lut_adj_118 (.A(n26215), .B(n26216), .C(ev_rd_hold[55]), 
         .D(n26147), .Z(ev_wr_data[55])) /* synthesis lut_function=(A (B (C (D))+!B (D))+!A (C (D))) */ ;
    defparam i1_3_lut_4_lut_adj_118.init = 16'hf200;
    LUT4 i15102_4_lut (.A(spi_command[7]), .B(spi_command[2]), .C(spi_command[3]), 
         .D(n25649), .Z(n25683)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i15102_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_119 (.A(n26215), .B(n26216), .C(init_shadow[55]), 
         .D(n16988), .Z(n14526)) /* synthesis lut_function=(A (B (C)+!B (C+(D)))+!A (C)) */ ;
    defparam i1_3_lut_4_lut_adj_119.init = 16'hf2f0;
    FD1P3IX ev_ch_i4 (.D(ev_ch_6__N_2010[4]), .SP(pll_clk_enable_684), .CD(n17880), 
            .CK(pll_clk), .Q(ev_ch[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_ch_i4.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_120 (.A(n26217), .B(n26219), .C(ev_rd_hold[8]), 
         .D(n26147), .Z(ev_wr_data[8])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_120.init = 16'hf100;
    LUT4 i1_3_lut_4_lut_adj_121 (.A(n26217), .B(n26235), .C(init_shadow[72]), 
         .D(n16988), .Z(n14424)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_121.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_122 (.A(n26217), .B(n26219), .C(init_shadow[8]), 
         .D(n16988), .Z(n14808)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_122.init = 16'hf1f0;
    LUT4 i15088_4_lut (.A(time_divider[4]), .B(time_divider[2]), .C(time_divider[0]), 
         .D(time_divider[1]), .Z(n25669)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i15088_4_lut.init = 16'h8000;
    LUT4 i15068_2_lut (.A(spi_command[6]), .B(spi_command[5]), .Z(n25649)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i15068_2_lut.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_123 (.A(n26217), .B(n26224), .C(ev_rd_hold[56]), 
         .D(n26147), .Z(ev_wr_data[56])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_123.init = 16'hf100;
    LUT4 i7_4_lut (.A(ev_run_hold_s5[82]), .B(us_tx_c_82), .C(init_shadow[82]), 
         .D(swap_now_s5), .Z(n13890)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i7_4_lut.init = 16'h5a66;
    LUT4 i1_2_lut_rep_244_3_lut (.A(ev_ch[2]), .B(ev_ch[0]), .C(ev_ch[1]), 
         .Z(n26208)) /* synthesis lut_function=(A+!(B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_244_3_lut.init = 16'hbfbf;
    LUT4 i1_2_lut_rep_180_3_lut_4_lut (.A(n26277), .B(n26250), .C(n26251), 
         .D(ev_ch[1]), .Z(n26144)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i1_2_lut_rep_180_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_3_lut_4_lut_adj_124 (.A(n26217), .B(n26224), .C(init_shadow[56]), 
         .D(n16988), .Z(n14520)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_124.init = 16'hf1f0;
    LUT4 i2_3_lut_adj_125 (.A(ev_state[1]), .B(build_sum[8]), .C(ev_state[0]), 
         .Z(n16988)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i2_3_lut_adj_125.init = 16'h4040;
    LUT4 i1_3_lut_4_lut_adj_126 (.A(n26213), .B(n26224), .C(ev_rd_hold[57]), 
         .D(n26147), .Z(ev_wr_data[57])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_126.init = 16'hf100;
    LUT4 i1_3_lut_4_lut_adj_127 (.A(n26213), .B(n26224), .C(init_shadow[57]), 
         .D(n16988), .Z(n14514)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_127.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_128 (.A(n26220), .B(n26224), .C(ev_rd_hold[58]), 
         .D(n26147), .Z(ev_wr_data[58])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_128.init = 16'hf100;
    LUT4 i1_3_lut_4_lut_adj_129 (.A(n26220), .B(n26224), .C(init_shadow[58]), 
         .D(n16988), .Z(n14508)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_129.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_220_3_lut_4_lut (.A(ev_ch[1]), .B(n26251), .C(n26250), 
         .D(n26277), .Z(n26184)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_220_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_130 (.A(n26208), .B(n26224), .C(ev_rd_hold[59]), 
         .D(n26147), .Z(ev_wr_data[59])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_130.init = 16'hf100;
    LUT4 i5501_2_lut (.A(spi_byte_count[0]), .B(n1), .Z(spi1_sck_c_enable_59)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i5501_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_131 (.A(n26208), .B(n26224), .C(init_shadow[59]), 
         .D(n16988), .Z(n14502)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_131.init = 16'hf1f0;
    LUT4 i15266_2_lut_2_lut_4_lut (.A(n26138), .B(n10_adj_3207), .C(spi_byte_count[0]), 
         .D(spi1_sck_c_enable_135), .Z(spi1_sck_c_enable_32)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i15266_2_lut_2_lut_4_lut.init = 16'h1000;
    LUT4 i1_2_lut_rep_287 (.A(ev_ch[2]), .B(ev_ch[0]), .Z(n26251)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_287.init = 16'hbbbb;
    LUT4 i1_2_lut_adj_132 (.A(spi_channel_field[0]), .B(n25471), .Z(spi1_sck_c_enable_97)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i1_2_lut_adj_132.init = 16'h4444;
    FD1P3IX ev_ch_i3 (.D(ev_ch_6__N_2010[3]), .SP(pll_clk_enable_684), .CD(n17880), 
            .CK(pll_clk), .Q(ev_ch[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_ch_i3.GSR = "DISABLED";
    FD1P3IX ev_ch_i2 (.D(ev_ch_6__N_2010[2]), .SP(pll_clk_enable_684), .CD(n17880), 
            .CK(pll_clk), .Q(ev_ch[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_ch_i2.GSR = "DISABLED";
    LUT4 i2_3_lut_adj_133 (.A(n1_adj_3167), .B(spi_byte_count[1]), .C(spi_byte_count[0]), 
         .Z(spi1_sck_c_enable_74)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i2_3_lut_adj_133.init = 16'h8080;
    FD1P3IX ev_ch_i1 (.D(ev_ch_6__N_2010[1]), .SP(pll_clk_enable_684), .CD(n17880), 
            .CK(pll_clk), .Q(ev_ch[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_ch_i1.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_134 (.A(n26209), .B(n26224), .C(ev_rd_hold[60]), 
         .D(n26147), .Z(ev_wr_data[60])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_134.init = 16'hf100;
    FD1P3AX ev_state_i3 (.D(ev_state_3__N_602[3]), .SP(pll_clk_enable_642), 
            .CK(pll_clk), .Q(ev_state[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_state_i3.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i0 (.D(ev_clear_addr_7__N_2215[0]), .SP(pll_clk_enable_639), 
            .CD(n17861), .CK(pll_clk), .Q(ev_clear_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_clear_addr_i0.GSR = "DISABLED";
    FD1P3AX ev_state_i2 (.D(ev_state_3__N_602[2]), .SP(pll_clk_enable_640), 
            .CK(pll_clk), .Q(ev_state[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_state_i2.GSR = "DISABLED";
    LUT4 i3_4_lut_adj_135 (.A(n5_adj_3181), .B(spi_channel_field[1]), .C(n25529), 
         .D(n21554), .Z(n25471)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[17:31])
    defparam i3_4_lut_adj_135.init = 16'h0020;
    LUT4 i1_3_lut_4_lut_adj_136 (.A(n26209), .B(n26224), .C(init_shadow[60]), 
         .D(n16988), .Z(n14496)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_136.init = 16'hf1f0;
    FD1P3IX init_shadow_i0 (.D(n13711), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i0.GSR = "DISABLED";
    FD1P3AX ev_state_i1 (.D(ev_state_3__N_602[1]), .SP(pll_clk_enable_642), 
            .CK(pll_clk), .Q(ev_state[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_state_i1.GSR = "DISABLED";
    FD1P3AX ev_state_i0 (.D(ev_state_3__N_602[0]), .SP(pll_clk_enable_643), 
            .CK(pll_clk), .Q(ev_state[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_state_i0.GSR = "DISABLED";
    FD1P3JX ws2812_settle_i0_i3 (.D(n26042), .SP(pll_clk_enable_647), .PD(pll_clk_enable_23), 
            .CK(pll_clk), .Q(ws2812_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ws2812_settle_i0_i3.GSR = "DISABLED";
    LUT4 i978_2_lut_4_lut (.A(n26223), .B(n26222), .C(pll_clk_enable_23), 
         .D(pll_clk_enable_8), .Z(pll_clk_enable_66)) /* synthesis lut_function=(A (B (D)+!B ((D)+!C))+!A (D)) */ ;
    defparam i978_2_lut_4_lut.init = 16'hff02;
    FD1P3JX ws2812_settle_i0_i2 (.D(n26031), .SP(pll_clk_enable_647), .PD(pll_clk_enable_23), 
            .CK(pll_clk), .Q(ws2812_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ws2812_settle_i0_i2.GSR = "DISABLED";
    CCU2D fpga_time_1596_add_4_31 (.A0(fpga_time[29]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[30]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24971), .COUT(n24972), .S0(n136), .S1(n135));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_31.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_31.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_31.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_31.INJECT1_1 = "NO";
    LUT4 i4_4_lut_adj_137 (.A(spi_byte_count[1]), .B(spi_byte_count[2]), 
         .C(n25474), .D(n6_adj_3180), .Z(spi1_sck_c_enable_90)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i4_4_lut_adj_137.init = 16'h1000;
    LUT4 i1_2_lut_4_lut_adj_138 (.A(n26223), .B(n26222), .C(pll_clk_enable_23), 
         .D(pll_clk_enable_18), .Z(pll_clk_enable_203)) /* synthesis lut_function=(A (B (D)+!B ((D)+!C))+!A (D)) */ ;
    defparam i1_2_lut_4_lut_adj_138.init = 16'hff02;
    LUT4 i2_4_lut (.A(spi_bit_count[2]), .B(n26273), .C(n66), .D(n65), 
         .Z(n25529)) /* synthesis lut_function=(A (B ((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[17:31])
    defparam i2_4_lut.init = 16'h8808;
    LUT4 i1_2_lut_adj_139 (.A(spi_byte_count[0]), .B(spi_byte_count[3]), 
         .Z(n6_adj_3180)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i1_2_lut_adj_139.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_140 (.A(n26220), .B(n26219), .C(ev_rd_hold[10]), 
         .D(n26147), .Z(ev_wr_data[10])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_140.init = 16'hf100;
    LUT4 i1_3_lut_4_lut_adj_141 (.A(n26220), .B(n26219), .C(init_shadow[10]), 
         .D(n16988), .Z(n14796)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_141.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_142 (.A(n26208), .B(n26219), .C(ev_rd_hold[11]), 
         .D(n26147), .Z(ev_wr_data[11])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_142.init = 16'hf100;
    LUT4 i1_3_lut_4_lut_adj_143 (.A(n26208), .B(n26219), .C(init_shadow[11]), 
         .D(n16988), .Z(n14790)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_143.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_144 (.A(n26210), .B(n26224), .C(ev_rd_hold[61]), 
         .D(n26147), .Z(ev_wr_data[61])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_144.init = 16'hf100;
    LUT4 run_addr_s3_8__I_0_i5_3_lut (.A(run_addr_s3[4]), .B(ev_rd_slot[4]), 
         .C(n21588), .Z(event_rd_addr[4])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(241[33:74])
    defparam run_addr_s3_8__I_0_i5_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_4_lut_adj_145 (.A(n26210), .B(n26224), .C(init_shadow[61]), 
         .D(n16988), .Z(n14490)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_145.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_146 (.A(n26211), .B(n26224), .C(ev_rd_hold[62]), 
         .D(n26147), .Z(ev_wr_data[62])) /* synthesis lut_function=(A (C (D))+!A (B (C (D))+!B (D))) */ ;
    defparam i1_3_lut_4_lut_adj_146.init = 16'hf100;
    LUT4 i1_3_lut_4_lut_adj_147 (.A(n26211), .B(n26224), .C(init_shadow[62]), 
         .D(n16988), .Z(n14484)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_147.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_148 (.A(n26215), .B(n26224), .C(ev_rd_hold[63]), 
         .D(n26147), .Z(ev_wr_data[63])) /* synthesis lut_function=(A (B (C (D))+!B (D))+!A (C (D))) */ ;
    defparam i1_3_lut_4_lut_adj_148.init = 16'hf200;
    LUT4 i1_3_lut_4_lut_adj_149 (.A(n26215), .B(n26224), .C(init_shadow[63]), 
         .D(n16988), .Z(n14478)) /* synthesis lut_function=(A (B (C)+!B (C+(D)))+!A (C)) */ ;
    defparam i1_3_lut_4_lut_adj_149.init = 16'hf2f0;
    LUT4 i1_3_lut_4_lut_adj_150 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[66]), 
         .D(n26159), .Z(ev_wr_data[66])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_150.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_151 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[74]), 
         .D(n26177), .Z(ev_wr_data[74])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_151.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_152 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[39]), 
         .D(n26195), .Z(ev_wr_data[39])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_152.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_153 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[38]), 
         .D(n26197), .Z(ev_wr_data[38])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_153.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_154 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[36]), 
         .D(n26194), .Z(ev_wr_data[36])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_154.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_155 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[34]), 
         .D(n26189), .Z(ev_wr_data[34])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_155.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_219_3_lut_4_lut (.A(ev_ch[1]), .B(n26258), .C(n26250), 
         .D(n26277), .Z(n26183)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_219_3_lut_4_lut.init = 16'hfffe;
    LUT4 i14315_2_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .Z(n29)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam i14315_2_lut.init = 16'h6666;
    LUT4 i2_3_lut_adj_156 (.A(n25526), .B(spi_rgb_index[0]), .C(spi_rgb_index[1]), 
         .Z(spi1_sck_c_enable_104)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i2_3_lut_adj_156.init = 16'h0808;
    LUT4 run_addr_s3_8__I_0_i6_3_lut (.A(run_addr_s3[5]), .B(ev_rd_slot[5]), 
         .C(n21588), .Z(event_rd_addr[5])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(241[33:74])
    defparam run_addr_s3_8__I_0_i6_3_lut.init = 16'hacac;
    LUT4 i1742_2_lut (.A(ev_ch[1]), .B(ev_ch[0]), .Z(ev_ch_6__N_2010[1])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(609[27:39])
    defparam i1742_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_157 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[64]), 
         .D(n26148), .Z(ev_wr_data[64])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_157.init = 16'hd0dd;
    LUT4 i2_4_lut_adj_158 (.A(spi_rgb_index[2]), .B(n26126), .C(spi_rgb_index[3]), 
         .D(n4_adj_3210), .Z(n25526)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam i2_4_lut_adj_158.init = 16'h0400;
    LUT4 i1_3_lut_4_lut_adj_159 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[65]), 
         .D(n26149), .Z(ev_wr_data[65])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_159.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_188_3_lut_4_lut (.A(n26261), .B(n26260), .C(n26255), 
         .D(ev_ch[0]), .Z(n26152)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_188_3_lut_4_lut.init = 16'hffef;
    LUT4 i15257_4_lut (.A(n18_adj_3209), .B(n26228), .C(n20_adj_3208), 
         .D(ev_state[0]), .Z(pll_clk_enable_640)) /* synthesis lut_function=(A (B+!(C+!(D)))+!A (B+!(C (D)))) */ ;
    defparam i15257_4_lut.init = 16'hcfdd;
    LUT4 i1_3_lut_4_lut_adj_160 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[30]), 
         .D(n26186), .Z(ev_wr_data[30])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_160.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_190_3_lut_4_lut (.A(n26261), .B(n26260), .C(n26255), 
         .D(ev_ch[0]), .Z(n26154)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_190_3_lut_4_lut.init = 16'hefff;
    LUT4 i1_4_lut_adj_161 (.A(n26132), .B(spi_rgb_payload_byte_N_2547[6]), 
         .C(n4_adj_3200), .D(n26131), .Z(n4_adj_3210)) /* synthesis lut_function=(A ((C+!(D))+!B)) */ ;
    defparam i1_4_lut_adj_161.init = 16'ha2aa;
    LUT4 i1_2_lut_rep_189_3_lut_4_lut (.A(n26261), .B(n26260), .C(n26249), 
         .D(ev_ch[0]), .Z(n26153)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_189_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_3_lut_4_lut_adj_162 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[31]), 
         .D(n26190), .Z(ev_wr_data[31])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_162.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_187_3_lut_4_lut (.A(n26261), .B(n26260), .C(n26249), 
         .D(ev_ch[0]), .Z(n26151)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_187_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_163 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[33]), 
         .D(n26188), .Z(ev_wr_data[33])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_163.init = 16'hd0dd;
    LUT4 i3_4_lut_adj_164 (.A(n21195), .B(n26482), .C(phase_step_s3), 
         .D(phase_step_s2), .Z(n21588)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut_adj_164.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_165 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[32]), 
         .D(n26187), .Z(ev_wr_data[32])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_165.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_166 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[35]), 
         .D(n26192), .Z(ev_wr_data[35])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_166.init = 16'hd0dd;
    LUT4 i1_4_lut_adj_167 (.A(ev_state[0]), .B(n43), .C(n25548), .D(ev_state[3]), 
         .Z(ev_state_3__N_602[0])) /* synthesis lut_function=(!(A+!(B (C+!(D))+!B (C)))) */ ;
    defparam i1_4_lut_adj_167.init = 16'h5054;
    LUT4 i1_4_lut_adj_168 (.A(ev_state[1]), .B(frame_req_N_2584), .C(n21588), 
         .D(ev_state[2]), .Z(n43)) /* synthesis lut_function=(A+!(B (C (D))+!B (C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(203[17:25])
    defparam i1_4_lut_adj_168.init = 16'hafee;
    LUT4 i7_4_lut_adj_169 (.A(staging_q[0]), .B(n14), .C(n10), .D(staging_q[6]), 
         .Z(ev_state_3__N_1990[2])) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(600[32:56])
    defparam i7_4_lut_adj_169.init = 16'hfffe;
    LUT4 i6_4_lut (.A(staging_q[3]), .B(staging_q[1]), .C(staging_q[5]), 
         .D(staging_q[7]), .Z(n14)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(600[32:56])
    defparam i6_4_lut.init = 16'hfffe;
    LUT4 i2_2_lut (.A(staging_q[2]), .B(staging_q[4]), .Z(n10)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(600[32:56])
    defparam i2_2_lut.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_170 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[37]), 
         .D(n26193), .Z(ev_wr_data[37])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_170.init = 16'hd0dd;
    LUT4 frame_settle_2__bdd_3_lut (.A(frame_settle[2]), .B(frame_settle[1]), 
         .C(frame_settle[0]), .Z(n26067)) /* synthesis lut_function=(A (B+(C))+!A !(B+(C))) */ ;
    defparam frame_settle_2__bdd_3_lut.init = 16'ha9a9;
    LUT4 i1_3_lut_4_lut_adj_171 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[40]), 
         .D(n26196), .Z(ev_wr_data[40])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_171.init = 16'hd0dd;
    PFUMX i15166 (.BLUT(n25738), .ALUT(n25739), .C0(spi1_miso_N_2492[1]), 
          .Z(n25748));
    LUT4 i1_3_lut_4_lut_adj_172 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[41]), 
         .D(n26202), .Z(ev_wr_data[41])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_172.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_221_3_lut_4_lut (.A(ev_ch[1]), .B(n26258), .C(n26250), 
         .D(n26277), .Z(n26185)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_221_3_lut_4_lut.init = 16'hfffd;
    LUT4 i1_3_lut_4_lut_adj_173 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[42]), 
         .D(n26198), .Z(ev_wr_data[42])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_173.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_174 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[43]), 
         .D(n26203), .Z(ev_wr_data[43])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_174.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_175 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[44]), 
         .D(n26201), .Z(ev_wr_data[44])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_175.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_176 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[45]), 
         .D(n26199), .Z(ev_wr_data[45])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_176.init = 16'hd0dd;
    LUT4 i15234_2_lut_3_lut (.A(n26137), .B(state[1]), .C(state[0]), .Z(pll_clk_enable_650)) /* synthesis lut_function=(!(A (B+!(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i15234_2_lut_3_lut.init = 16'h7070;
    LUT4 i15221_2_lut (.A(ws2812_settle[0]), .B(ws2812_settle[1]), .Z(n25766)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam i15221_2_lut.init = 16'h9999;
    LUT4 i1_3_lut_4_lut_adj_177 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[46]), 
         .D(n26205), .Z(ev_wr_data[46])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_177.init = 16'hd0dd;
    LUT4 i2_3_lut_4_lut_adj_178 (.A(frame_settle[0]), .B(n26270), .C(pll_clk_enable_22), 
         .D(pll_clk_enable_24), .Z(pll_clk_enable_649)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(512[22:42])
    defparam i2_3_lut_4_lut_adj_178.init = 16'hfffe;
    LUT4 i11067_4_lut (.A(spi_byte_count[7]), .B(n26150), .C(n25064), 
         .D(spi_byte_count[6]), .Z(n21606)) /* synthesis lut_function=(A (B+(C (D)))+!A (B)) */ ;
    defparam i11067_4_lut.init = 16'heccc;
    LUT4 i1_3_lut_4_lut_adj_179 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[47]), 
         .D(n26204), .Z(ev_wr_data[47])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_179.init = 16'hd0dd;
    LUT4 ws2812_toggle_spi_I_0_4_lut (.A(ws2812_toggle_spi), .B(spi_command[1]), 
         .C(n16929), .D(spi_command[0]), .Z(ws2812_toggle_spi_N_2535)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(428[30] 430[24])
    defparam ws2812_toggle_spi_I_0_4_lut.init = 16'h6aaa;
    LUT4 i15248_4_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_135), .C(frame_end), 
         .D(n26127), .Z(spi1_sck_c_enable_171)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i15248_4_lut.init = 16'h0040;
    LUT4 i1_3_lut_4_lut_adj_180 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[12]), 
         .D(n26151), .Z(ev_wr_data[12])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_180.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_181 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[13]), 
         .D(n26153), .Z(ev_wr_data[13])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_181.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_182 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[14]), 
         .D(n26152), .Z(ev_wr_data[14])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_182.init = 16'hd0dd;
    LUT4 i1_4_lut_adj_183 (.A(n26225), .B(spi_byte_count[9]), .C(n26226), 
         .D(n11289), .Z(n4)) /* synthesis lut_function=(!(A+(B (C)+!B (C+!(D))))) */ ;
    defparam i1_4_lut_adj_183.init = 16'h0504;
    LUT4 i15058_2_lut (.A(expected_next[2]), .B(expected_next[5]), .Z(n25639)) /* synthesis lut_function=(A (B)) */ ;
    defparam i15058_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_184 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[15]), 
         .D(n26154), .Z(ev_wr_data[15])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_184.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_185 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[16]), 
         .D(n26156), .Z(ev_wr_data[16])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_185.init = 16'hd0dd;
    LUT4 i12_4_lut (.A(expected_next[6]), .B(n24), .C(n18_adj_3165), .D(expected_next[12]), 
         .Z(n26_adj_3217)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i12_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_186 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[17]), 
         .D(n26155), .Z(ev_wr_data[17])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_186.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_187 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[18]), 
         .D(n26158), .Z(ev_wr_data[18])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_187.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_188 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[19]), 
         .D(n26157), .Z(ev_wr_data[19])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_188.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_189 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[20]), 
         .D(n26174), .Z(ev_wr_data[20])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_189.init = 16'hd0dd;
    LUT4 i10665_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_383)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10665_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_rep_186_4_lut (.A(spi_byte_count[10]), .B(n26274), .C(spi_byte_count[8]), 
         .D(spi_byte_count[9]), .Z(n26150)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_3_lut_rep_186_4_lut.init = 16'hfffe;
    LUT4 i14313_1_lut (.A(mic_sample_count[0]), .Z(n30)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam i14313_1_lut.init = 16'h5555;
    LUT4 i10_4_lut (.A(expected_next[11]), .B(expected_next[10]), .C(expected_next[14]), 
         .D(expected_next[15]), .Z(n24)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i10_4_lut.init = 16'hfffe;
    LUT4 i4_2_lut (.A(expected_next[8]), .B(expected_next[7]), .Z(n18_adj_3165)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i4_2_lut.init = 16'heeee;
    LUT4 i8_4_lut (.A(expected_next[9]), .B(expected_next[13]), .C(expected_next[4]), 
         .D(expected_next[3]), .Z(n22)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i8_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_3_lut_adj_190 (.A(spi_rgb_index[0]), .B(n25526), .C(spi_rgb_index[1]), 
         .Z(spi1_sck_c_enable_120)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_190.init = 16'h4040;
    LUT4 i1_2_lut_rep_167_3_lut_4_lut (.A(spi_byte_count[5]), .B(n26264), 
         .C(n26272), .D(n26150), .Z(n26131)) /* synthesis lut_function=(A (B+(C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i1_2_lut_rep_167_3_lut_4_lut.init = 16'hfff8;
    LUT4 build_phase_7__I_0_i2_3_lut_4_lut (.A(n26252), .B(n26243), .C(build_phase[1]), 
         .D(build_sum[1]), .Z(ev_rd_slot[1])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam build_phase_7__I_0_i2_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i1_3_lut_4_lut (.A(n26252), .B(n26243), .C(build_phase[0]), 
         .D(build_sum[0]), .Z(ev_rd_slot[0])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam build_phase_7__I_0_i1_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i5_3_lut_4_lut (.A(n26252), .B(n26243), .C(build_phase[4]), 
         .D(build_sum[4]), .Z(ev_rd_slot[4])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam build_phase_7__I_0_i5_3_lut_4_lut.init = 16'hf4b0;
    LUT4 i1_3_lut_4_lut_adj_191 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[21]), 
         .D(n26160), .Z(ev_wr_data[21])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_191.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_192 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[22]), 
         .D(n26161), .Z(ev_wr_data[22])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_192.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_193 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[23]), 
         .D(n26162), .Z(ev_wr_data[23])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_193.init = 16'hd0dd;
    LUT4 ev_state_3__bdd_4_lut_15310 (.A(ev_state_3__N_1990[2]), .B(ev_state[0]), 
         .C(ev_state[2]), .D(ev_state[1]), .Z(n26082)) /* synthesis lut_function=(!(A (B+(C+(D)))+!A (B+(C)))) */ ;
    defparam ev_state_3__bdd_4_lut_15310.init = 16'h0103;
    LUT4 i16_4_lut_adj_194 (.A(ev_run_hold_s5[1]), .B(us_tx_c_1), .C(init_shadow[1]), 
         .D(swap_now_s5), .Z(n13728)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_194.init = 16'h5a66;
    LUT4 i16_4_lut_adj_195 (.A(ev_run_hold_s5[2]), .B(us_tx_c_2), .C(init_shadow[2]), 
         .D(swap_now_s5), .Z(n13730)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_195.init = 16'h5a66;
    LUT4 i16_4_lut_adj_196 (.A(ev_run_hold_s5[3]), .B(us_tx_c_3), .C(init_shadow[3]), 
         .D(swap_now_s5), .Z(n13732)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_196.init = 16'h5a66;
    LUT4 i16_4_lut_adj_197 (.A(ev_run_hold_s5[4]), .B(us_tx_c_4), .C(init_shadow[4]), 
         .D(swap_now_s5), .Z(n13734)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_197.init = 16'h5a66;
    LUT4 i16_4_lut_adj_198 (.A(ev_run_hold_s5[5]), .B(us_tx_c_5), .C(init_shadow[5]), 
         .D(swap_now_s5), .Z(n13736)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_198.init = 16'h5a66;
    LUT4 i16_4_lut_adj_199 (.A(ev_run_hold_s5[6]), .B(us_tx_c_6), .C(init_shadow[6]), 
         .D(swap_now_s5), .Z(n13738)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_199.init = 16'h5a66;
    LUT4 i16_4_lut_adj_200 (.A(ev_run_hold_s5[0]), .B(us_tx_c_0), .C(init_shadow[0]), 
         .D(swap_now_s5), .Z(n12990)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_200.init = 16'h5a66;
    LUT4 i16_4_lut_adj_201 (.A(ev_run_hold_s5[7]), .B(us_tx_c_7), .C(init_shadow[7]), 
         .D(swap_now_s5), .Z(n13740)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_201.init = 16'h5a66;
    LUT4 i16_4_lut_adj_202 (.A(ev_run_hold_s5[8]), .B(us_tx_c_8), .C(init_shadow[8]), 
         .D(swap_now_s5), .Z(n13742)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_202.init = 16'h5a66;
    LUT4 i16_4_lut_adj_203 (.A(ev_run_hold_s5[9]), .B(us_tx_c_9), .C(init_shadow[9]), 
         .D(swap_now_s5), .Z(n13744)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_203.init = 16'h5a66;
    LUT4 i16_4_lut_adj_204 (.A(ev_run_hold_s5[10]), .B(us_tx_c_10), .C(init_shadow[10]), 
         .D(swap_now_s5), .Z(n13746)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_204.init = 16'h5a66;
    LUT4 i16_4_lut_adj_205 (.A(ev_run_hold_s5[11]), .B(us_tx_c_11), .C(init_shadow[11]), 
         .D(swap_now_s5), .Z(n13748)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_205.init = 16'h5a66;
    LUT4 i16_4_lut_adj_206 (.A(ev_run_hold_s5[12]), .B(us_tx_c_12), .C(init_shadow[12]), 
         .D(swap_now_s5), .Z(n13750)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_206.init = 16'h5a66;
    LUT4 i16_4_lut_adj_207 (.A(ev_run_hold_s5[13]), .B(us_tx_c_13), .C(init_shadow[13]), 
         .D(swap_now_s5), .Z(n13752)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_207.init = 16'h5a66;
    LUT4 build_phase_7__I_0_i3_3_lut_4_lut (.A(n26252), .B(n26243), .C(build_phase[2]), 
         .D(build_sum[2]), .Z(ev_rd_slot[2])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam build_phase_7__I_0_i3_3_lut_4_lut.init = 16'hf4b0;
    LUT4 i988_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_226)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(666[18] 668[12])
    defparam i988_2_lut.init = 16'h4444;
    LUT4 build_phase_7__I_0_i4_3_lut_4_lut (.A(n26252), .B(n26243), .C(build_phase[3]), 
         .D(build_sum[3]), .Z(ev_rd_slot[3])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam build_phase_7__I_0_i4_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i6_3_lut_4_lut (.A(n26252), .B(n26243), .C(build_phase[5]), 
         .D(build_sum[5]), .Z(ev_rd_slot[5])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam build_phase_7__I_0_i6_3_lut_4_lut.init = 16'hf4b0;
    LUT4 i16_4_lut_adj_208 (.A(ev_run_hold_s5[14]), .B(us_tx_c_14), .C(init_shadow[14]), 
         .D(swap_now_s5), .Z(n13754)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_208.init = 16'h5a66;
    LUT4 build_phase_7__I_0_i7_3_lut_4_lut (.A(n26252), .B(n26243), .C(build_phase[6]), 
         .D(build_sum[6]), .Z(ev_rd_slot[6])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam build_phase_7__I_0_i7_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i8_3_lut_4_lut (.A(n26252), .B(n26243), .C(build_phase[7]), 
         .D(build_sum[7]), .Z(ev_rd_slot[7])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[33:53])
    defparam build_phase_7__I_0_i8_3_lut_4_lut.init = 16'hf4b0;
    LUT4 i16_4_lut_adj_209 (.A(ev_run_hold_s5[15]), .B(us_tx_c_15), .C(init_shadow[15]), 
         .D(swap_now_s5), .Z(n13756)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_209.init = 16'h5a66;
    LUT4 i16_4_lut_adj_210 (.A(ev_run_hold_s5[16]), .B(us_tx_c_16), .C(init_shadow[16]), 
         .D(swap_now_s5), .Z(n13758)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_210.init = 16'h5a66;
    LUT4 i16_4_lut_adj_211 (.A(ev_run_hold_s5[17]), .B(us_tx_c_17), .C(init_shadow[17]), 
         .D(swap_now_s5), .Z(n13760)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_211.init = 16'h5a66;
    LUT4 i16_4_lut_adj_212 (.A(ev_run_hold_s5[18]), .B(us_tx_c_18), .C(init_shadow[18]), 
         .D(swap_now_s5), .Z(n13762)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_212.init = 16'h5a66;
    LUT4 i16_4_lut_adj_213 (.A(ev_run_hold_s5[19]), .B(us_tx_c_19), .C(init_shadow[19]), 
         .D(swap_now_s5), .Z(n13764)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_213.init = 16'h5a66;
    LUT4 i16_4_lut_adj_214 (.A(ev_run_hold_s5[20]), .B(us_tx_c_20), .C(init_shadow[20]), 
         .D(swap_now_s5), .Z(n13766)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_214.init = 16'h5a66;
    LUT4 i16_4_lut_adj_215 (.A(ev_run_hold_s5[21]), .B(us_tx_c_21), .C(init_shadow[21]), 
         .D(swap_now_s5), .Z(n13768)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_215.init = 16'h5a66;
    LUT4 i16_4_lut_adj_216 (.A(ev_run_hold_s5[22]), .B(us_tx_c_22), .C(init_shadow[22]), 
         .D(swap_now_s5), .Z(n13770)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_216.init = 16'h5a66;
    LUT4 mux_383_i1_3_lut (.A(pending_sequence[0]), .B(accepted_sequence_sync[0]), 
         .C(n26146), .Z(accepted_sequence_31__N_943[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(545[13] 552[16])
    defparam mux_383_i1_3_lut.init = 16'hcaca;
    LUT4 run_addr_s3_8__I_0_i7_3_lut (.A(run_addr_s3[6]), .B(ev_rd_slot[6]), 
         .C(n21588), .Z(event_rd_addr[6])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(241[33:74])
    defparam run_addr_s3_8__I_0_i7_3_lut.init = 16'hacac;
    LUT4 i16_4_lut_adj_217 (.A(ev_run_hold_s5[23]), .B(us_tx_c_23), .C(init_shadow[23]), 
         .D(swap_now_s5), .Z(n13772)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_217.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_218 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[25]), 
         .D(n26168), .Z(ev_wr_data[25])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_218.init = 16'hd0dd;
    LUT4 i14336_3_lut_4_lut (.A(mic_sample_count[2]), .B(n26244), .C(mic_sample_count[3]), 
         .D(mic_sample_count[4]), .Z(n26)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam i14336_3_lut_4_lut.init = 16'h7f80;
    LUT4 ev_state_3__bdd_4_lut (.A(ev_state[0]), .B(ev_state[2]), .C(n21588), 
         .D(ev_state[1]), .Z(n26083)) /* synthesis lut_function=(!((B (C+!(D))+!B !(D))+!A)) */ ;
    defparam ev_state_3__bdd_4_lut.init = 16'h2a00;
    LUT4 i16_4_lut_adj_219 (.A(ev_run_hold_s5[24]), .B(us_tx_c_24), .C(init_shadow[24]), 
         .D(swap_now_s5), .Z(n13774)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_219.init = 16'h5a66;
    LUT4 i16_4_lut_adj_220 (.A(ev_run_hold_s5[25]), .B(us_tx_c_25), .C(init_shadow[25]), 
         .D(swap_now_s5), .Z(n13776)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_220.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_221 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[24]), 
         .D(n26166), .Z(ev_wr_data[24])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_221.init = 16'hd0dd;
    LUT4 i15232_3_lut_4_lut (.A(ev_state[3]), .B(pll_clk_enable_22), .C(n20_adj_3208), 
         .D(ev_state[0]), .Z(pll_clk_enable_643)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;
    defparam i15232_3_lut_4_lut.init = 16'hefff;
    LUT4 i1_4_lut_rep_265 (.A(frame_req_N_2584), .B(ev_state[3]), .C(ev_state[2]), 
         .D(n26247), .Z(pll_clk_enable_587)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C)))) */ ;
    defparam i1_4_lut_rep_265.init = 16'h3032;
    LUT4 i1_3_lut_4_lut_adj_222 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[26]), 
         .D(n26169), .Z(ev_wr_data[26])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_222.init = 16'hd0dd;
    LUT4 i7295_2_lut_4_lut (.A(frame_req_N_2584), .B(ev_state[3]), .C(ev_state[2]), 
         .D(n26247), .Z(n17908)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i7295_2_lut_4_lut.init = 16'h0002;
    LUT4 i1_3_lut_4_lut_adj_223 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[27]), 
         .D(n26170), .Z(ev_wr_data[27])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_223.init = 16'hd0dd;
    LUT4 i16_4_lut_adj_224 (.A(ev_run_hold_s5[26]), .B(us_tx_c_26), .C(init_shadow[26]), 
         .D(swap_now_s5), .Z(n13778)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_224.init = 16'h5a66;
    LUT4 i1326_1_lut (.A(fpga_cs_n_c), .Z(fpga_cs_n_N_2500)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:33])
    defparam i1326_1_lut.init = 16'h5555;
    LUT4 i15281_3_lut_4_lut (.A(spi_byte_count[2]), .B(n26267), .C(n25826), 
         .D(spi1_sck_c_enable_135), .Z(spi1_sck_c_enable_44)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;
    defparam i15281_3_lut_4_lut.init = 16'h4000;
    LUT4 i16_4_lut_adj_225 (.A(ev_run_hold_s5[27]), .B(us_tx_c_27), .C(init_shadow[27]), 
         .D(swap_now_s5), .Z(n13780)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_225.init = 16'h5a66;
    LUT4 i16_4_lut_adj_226 (.A(ev_run_hold_s5[28]), .B(us_tx_c_28), .C(init_shadow[28]), 
         .D(swap_now_s5), .Z(n13782)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_226.init = 16'h5a66;
    LUT4 i1_2_lut_3_lut_4_lut_adj_227 (.A(n26273), .B(spi_bit_count[2]), 
         .C(frame_end), .D(fpga_cs_n_c), .Z(spi1_sck_c_enable_36)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i1_2_lut_3_lut_4_lut_adj_227.init = 16'h0080;
    LUT4 i1_3_lut_4_lut_adj_228 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[28]), 
         .D(n26171), .Z(ev_wr_data[28])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_228.init = 16'hd0dd;
    LUT4 i1_2_lut_3_lut_4_lut_adj_229 (.A(n26273), .B(spi_bit_count[2]), 
         .C(n26276), .D(n65), .Z(n25474)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i1_2_lut_3_lut_4_lut_adj_229.init = 16'h0008;
    LUT4 i1_3_lut_4_lut_adj_230 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[29]), 
         .D(n26173), .Z(ev_wr_data[29])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_230.init = 16'hd0dd;
    PFUMX i15301 (.BLUT(n26058), .ALUT(n26057), .C0(ev_state[2]), .Z(n26059));
    LUT4 i1_3_lut_4_lut_adj_231 (.A(n26209), .B(n26235), .C(init_shadow[76]), 
         .D(n16988), .Z(n14400)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_231.init = 16'hf1f0;
    LUT4 i3_3_lut_4_lut (.A(n26273), .B(spi_bit_count[2]), .C(n26275), 
         .D(n21606), .Z(n9)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(434[34:54])
    defparam i3_3_lut_4_lut.init = 16'h0008;
    LUT4 i1_2_lut_rep_191_3_lut_4_lut (.A(n26277), .B(n26260), .C(n26251), 
         .D(ev_ch[1]), .Z(n26155)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_191_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_232 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[68]), 
         .D(n26164), .Z(ev_wr_data[68])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_232.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_194_3_lut_4_lut (.A(n26277), .B(n26260), .C(n26258), 
         .D(ev_ch[1]), .Z(n26158)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_194_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_2_lut_rep_193_3_lut_4_lut (.A(n26277), .B(n26260), .C(n26251), 
         .D(ev_ch[1]), .Z(n26157)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_193_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_2_lut_rep_196_3_lut_4_lut (.A(n26277), .B(n26260), .C(n26249), 
         .D(ev_ch[0]), .Z(n26160)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_196_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_3_lut_4_lut_adj_233 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[67]), 
         .D(n26163), .Z(ev_wr_data[67])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_233.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_192_3_lut_4_lut (.A(n26277), .B(n26260), .C(n26258), 
         .D(ev_ch[1]), .Z(n26156)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_192_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_rep_197_3_lut_4_lut (.A(n26277), .B(n26260), .C(n26255), 
         .D(ev_ch[0]), .Z(n26161)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_197_3_lut_4_lut.init = 16'hffef;
    LUT4 i1_2_lut_rep_198_3_lut_4_lut (.A(n26277), .B(n26260), .C(n26255), 
         .D(ev_ch[0]), .Z(n26162)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_198_3_lut_4_lut.init = 16'hefff;
    LUT4 i1_3_lut_4_lut_adj_234 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[69]), 
         .D(n26165), .Z(ev_wr_data[69])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_234.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_210_3_lut_4_lut (.A(n26277), .B(n26260), .C(n26249), 
         .D(ev_ch[0]), .Z(n26174)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_rep_210_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_235 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[70]), 
         .D(n26167), .Z(ev_wr_data[70])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_235.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_201_3_lut_4_lut (.A(n26256), .B(n26250), .C(n26249), 
         .D(ev_ch[0]), .Z(n26165)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_201_3_lut_4_lut.init = 16'hfeff;
    LUT4 i16_4_lut_adj_236 (.A(ev_run_hold_s5[29]), .B(us_tx_c_29), .C(init_shadow[29]), 
         .D(swap_now_s5), .Z(n13784)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_236.init = 16'h5a66;
    LUT4 i16_4_lut_adj_237 (.A(ev_run_hold_s5[30]), .B(us_tx_c_30), .C(init_shadow[30]), 
         .D(swap_now_s5), .Z(n13786)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_237.init = 16'h5a66;
    LUT4 i1_2_lut_rep_199_3_lut_4_lut (.A(n26256), .B(n26250), .C(n26251), 
         .D(ev_ch[1]), .Z(n26163)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_199_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_3_lut_4_lut_adj_238 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[71]), 
         .D(n26172), .Z(ev_wr_data[71])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_238.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_200_3_lut_4_lut (.A(n26256), .B(n26250), .C(n26249), 
         .D(ev_ch[0]), .Z(n26164)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_200_3_lut_4_lut.init = 16'hfffe;
    FD1P3JX ws2812_settle_i0_i1 (.D(n25766), .SP(pll_clk_enable_647), .PD(pll_clk_enable_23), 
            .CK(pll_clk), .Q(ws2812_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ws2812_settle_i0_i1.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_203_3_lut_4_lut (.A(n26256), .B(n26250), .C(n26255), 
         .D(ev_ch[0]), .Z(n26167)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_203_3_lut_4_lut.init = 16'hffef;
    LUT4 i1_3_lut_4_lut_adj_239 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[72]), 
         .D(n26175), .Z(ev_wr_data[72])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_239.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_240 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[73]), 
         .D(n26176), .Z(ev_wr_data[73])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_240.init = 16'hd0dd;
    LUT4 i16_4_lut_adj_241 (.A(ev_run_hold_s5[31]), .B(us_tx_c_31), .C(init_shadow[31]), 
         .D(swap_now_s5), .Z(n13788)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_241.init = 16'h5a66;
    LUT4 i1_2_lut_rep_195_3_lut_4_lut (.A(n26256), .B(n26250), .C(n26258), 
         .D(ev_ch[1]), .Z(n26159)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_195_3_lut_4_lut.init = 16'hfeff;
    LUT4 i16_4_lut_adj_242 (.A(ev_run_hold_s5[32]), .B(us_tx_c_32), .C(init_shadow[32]), 
         .D(swap_now_s5), .Z(n13790)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_242.init = 16'h5a66;
    LUT4 i16_4_lut_adj_243 (.A(ev_run_hold_s5[33]), .B(us_tx_c_33), .C(init_shadow[33]), 
         .D(swap_now_s5), .Z(n13792)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_243.init = 16'h5a66;
    LUT4 i1_2_lut_rep_208_3_lut_4_lut (.A(n26256), .B(n26250), .C(n26255), 
         .D(ev_ch[0]), .Z(n26172)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_208_3_lut_4_lut.init = 16'hefff;
    LUT4 i1_2_lut_rep_185_3_lut_4_lut (.A(n26256), .B(n26250), .C(n26251), 
         .D(ev_ch[1]), .Z(n26149)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_185_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_244 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[75]), 
         .D(n26178), .Z(ev_wr_data[75])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_244.init = 16'hd0dd;
    LUT4 i16_4_lut_adj_245 (.A(ev_run_hold_s5[34]), .B(us_tx_c_34), .C(init_shadow[34]), 
         .D(swap_now_s5), .Z(n13794)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_245.init = 16'h5a66;
    LUT4 i1_2_lut_rep_184_3_lut_4_lut (.A(n26256), .B(n26250), .C(n26258), 
         .D(ev_ch[1]), .Z(n26148)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_184_3_lut_4_lut.init = 16'hfffe;
    LUT4 i16_4_lut_adj_246 (.A(ev_run_hold_s5[35]), .B(us_tx_c_35), .C(init_shadow[35]), 
         .D(swap_now_s5), .Z(n13796)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_246.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_247 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[76]), 
         .D(n26179), .Z(ev_wr_data[76])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_247.init = 16'hd0dd;
    LUT4 i16_4_lut_adj_248 (.A(ev_run_hold_s5[36]), .B(us_tx_c_36), .C(init_shadow[36]), 
         .D(swap_now_s5), .Z(n13798)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_248.init = 16'h5a66;
    LUT4 frame_req_I_0_771_2_lut (.A(frame_req), .B(swap_pending), .Z(frame_req_N_2584)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(571[21:47])
    defparam frame_req_I_0_771_2_lut.init = 16'h2222;
    LUT4 i1_3_lut_4_lut_adj_249 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[77]), 
         .D(n26180), .Z(ev_wr_data[77])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_249.init = 16'hd0dd;
    LUT4 i16_4_lut_adj_250 (.A(ev_run_hold_s5[37]), .B(us_tx_c_37), .C(init_shadow[37]), 
         .D(swap_now_s5), .Z(n13800)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_250.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_251 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[78]), 
         .D(n26181), .Z(ev_wr_data[78])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_251.init = 16'hd0dd;
    LUT4 i3735_4_lut (.A(ev_ch[6]), .B(staging_rd_addr[6]), .C(n17798), 
         .D(n25512), .Z(staging_rd_addr_6__N_722[6])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i3735_4_lut.init = 16'hcac0;
    LUT4 i10727_3_lut (.A(ev_ch[0]), .B(ev_state[3]), .C(ev_state[0]), 
         .Z(ev_ch_6__N_614[0])) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i10727_3_lut.init = 16'h4848;
    LUT4 i16_4_lut_adj_252 (.A(ev_run_hold_s5[38]), .B(us_tx_c_38), .C(init_shadow[38]), 
         .D(swap_now_s5), .Z(n13802)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_252.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_253 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[79]), 
         .D(n26182), .Z(ev_wr_data[79])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_253.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_254 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[80]), 
         .D(n26183), .Z(ev_wr_data[80])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_254.init = 16'hd0dd;
    LUT4 i1_2_lut_rep_214_3_lut_4_lut (.A(n26250), .B(n26261), .C(n26251), 
         .D(ev_ch[1]), .Z(n26178)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_214_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_2_lut_rep_215_3_lut_4_lut (.A(n26250), .B(n26261), .C(n26249), 
         .D(ev_ch[0]), .Z(n26179)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_215_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_rep_217_3_lut_4_lut (.A(n26250), .B(n26261), .C(n26255), 
         .D(ev_ch[0]), .Z(n26181)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_217_3_lut_4_lut.init = 16'hffef;
    LUT4 i1_2_lut_rep_213_3_lut_4_lut (.A(n26250), .B(n26261), .C(n26258), 
         .D(ev_ch[1]), .Z(n26177)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_213_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_2_lut_rep_216_3_lut_4_lut (.A(n26250), .B(n26261), .C(n26249), 
         .D(ev_ch[0]), .Z(n26180)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_216_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_2_lut_rep_218_3_lut_4_lut (.A(n26250), .B(n26261), .C(n26255), 
         .D(ev_ch[0]), .Z(n26182)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_218_3_lut_4_lut.init = 16'hefff;
    LUT4 i1_2_lut_rep_212_3_lut_4_lut (.A(n26250), .B(n26261), .C(n26251), 
         .D(ev_ch[1]), .Z(n26176)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_212_3_lut_4_lut.init = 16'hfffe;
    FD1P3AX ws2812_toggle_spi_558 (.D(ws2812_toggle_spi_N_2535), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .Q(ws2812_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam ws2812_toggle_spi_558.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_211_3_lut_4_lut (.A(n26250), .B(n26261), .C(n26258), 
         .D(ev_ch[1]), .Z(n26175)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_211_3_lut_4_lut.init = 16'hfffe;
    PFUMX i15167 (.BLUT(n25740), .ALUT(n25741), .C0(spi1_miso_N_2492[1]), 
          .Z(n25749));
    LUT4 i16_4_lut_adj_255 (.A(ev_run_hold_s5[39]), .B(us_tx_c_39), .C(init_shadow[39]), 
         .D(swap_now_s5), .Z(n13804)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_255.init = 16'h5a66;
    LUT4 i14401_2_lut (.A(spi_channel_field[1]), .B(spi_channel_field[0]), 
         .Z(n14_adj_3206)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(384[78:102])
    defparam i14401_2_lut.init = 16'h6666;
    LUT4 i10709_3_lut_4_lut (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[81]), 
         .D(n26184), .Z(ev_wr_data[81])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i10709_3_lut_4_lut.init = 16'hd0dd;
    LUT4 i1_3_lut_4_lut_adj_256 (.A(ev_state[0]), .B(n26238), .C(ev_rd_hold[82]), 
         .D(n26185), .Z(ev_wr_data[82])) /* synthesis lut_function=(A (B (C+!(D)))+!A (C+!(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[29:51])
    defparam i1_3_lut_4_lut_adj_256.init = 16'hd0dd;
    LUT4 i16_4_lut_adj_257 (.A(ev_run_hold_s5[40]), .B(us_tx_c_40), .C(init_shadow[40]), 
         .D(swap_now_s5), .Z(n13806)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_257.init = 16'h5a66;
    LUT4 i16_4_lut_adj_258 (.A(ev_run_hold_s5[41]), .B(us_tx_c_41), .C(init_shadow[41]), 
         .D(swap_now_s5), .Z(n13808)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_258.init = 16'h5a66;
    LUT4 i1_2_lut_rep_206_3_lut_4_lut (.A(n26260), .B(n26279), .C(n26251), 
         .D(ev_ch[1]), .Z(n26170)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_206_3_lut_4_lut.init = 16'hfbff;
    LUT4 i1_2_lut_rep_222_3_lut_4_lut (.A(n26260), .B(n26279), .C(n26255), 
         .D(ev_ch[0]), .Z(n26186)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_222_3_lut_4_lut.init = 16'hffbf;
    FD1P3IX init_shadow_i3 (.D(n14838), .SP(pll_clk_enable_652), .CD(n17908), 
            .CK(pll_clk), .Q(init_shadow[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i3.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_259 (.A(n26217), .B(n26234), .C(init_shadow[64]), 
         .D(n16988), .Z(n14472)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_259.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_226_3_lut_4_lut (.A(n26260), .B(n26279), .C(n26255), 
         .D(ev_ch[0]), .Z(n26190)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_226_3_lut_4_lut.init = 16'hbfff;
    LUT4 i1_2_lut_rep_209_3_lut_4_lut (.A(n26260), .B(n26279), .C(n26249), 
         .D(ev_ch[0]), .Z(n26173)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_209_3_lut_4_lut.init = 16'hfbff;
    LUT4 i1_2_lut_rep_202_3_lut_4_lut (.A(n26260), .B(n26279), .C(n26258), 
         .D(ev_ch[1]), .Z(n26166)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_202_3_lut_4_lut.init = 16'hfffb;
    LUT4 i1_2_lut_rep_205_3_lut_4_lut (.A(n26260), .B(n26279), .C(n26258), 
         .D(ev_ch[1]), .Z(n26169)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_205_3_lut_4_lut.init = 16'hfbff;
    LUT4 i1_3_lut_4_lut_adj_260 (.A(n26213), .B(n26234), .C(init_shadow[65]), 
         .D(n16988), .Z(n14466)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_260.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_207_3_lut_4_lut (.A(n26260), .B(n26279), .C(n26249), 
         .D(ev_ch[0]), .Z(n26171)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_207_3_lut_4_lut.init = 16'hfffb;
    LUT4 i1_2_lut_rep_174_4_lut (.A(spi_byte_count[9]), .B(spi_byte_count[8]), 
         .C(n26226), .D(n26272), .Z(n26138)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_rep_174_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_rep_204_3_lut_4_lut (.A(n26260), .B(n26279), .C(n26251), 
         .D(ev_ch[1]), .Z(n26168)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_204_3_lut_4_lut.init = 16'hfffb;
    LUT4 run_addr_s3_8__I_0_i8_3_lut (.A(run_addr_s3[7]), .B(ev_rd_slot[7]), 
         .C(n21588), .Z(event_rd_addr[7])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(241[33:74])
    defparam run_addr_s3_8__I_0_i8_3_lut.init = 16'hacac;
    LUT4 i14407_2_lut (.A(staging_q[8]), .B(staging_q[0]), .Z(build_sum_8__N_2031[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i14407_2_lut.init = 16'h6666;
    LUT4 i2256_1_lut (.A(spi_channel_field[0]), .Z(n1_adj_3205)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(139[52:79])
    defparam i2256_1_lut.init = 16'h5555;
    LUT4 i1_2_lut_4_lut_adj_261 (.A(n26266), .B(spi_command[4]), .C(n25683), 
         .D(stop_toggle_spi), .Z(stop_toggle_spi_N_2515)) /* synthesis lut_function=(A (B (C (D)+!C !(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(418[34:54])
    defparam i1_2_lut_4_lut_adj_261.init = 16'hf708;
    LUT4 i3737_4_lut (.A(ev_ch[5]), .B(staging_rd_addr[5]), .C(n17798), 
         .D(n25512), .Z(staging_rd_addr_6__N_722[5])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i3737_4_lut.init = 16'hcac0;
    LUT4 i16_4_lut_adj_262 (.A(ev_run_hold_s5[42]), .B(us_tx_c_42), .C(init_shadow[42]), 
         .D(swap_now_s5), .Z(n13810)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_262.init = 16'h5a66;
    LUT4 i15274_2_lut_3_lut (.A(spi_byte_count[0]), .B(n1_adj_3167), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_66)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i15274_2_lut_3_lut.init = 16'h4040;
    LUT4 i1763_2_lut_3_lut_4_lut (.A(ev_ch[2]), .B(n26278), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(ev_ch_6__N_2010[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(609[27:39])
    defparam i1763_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i16_4_lut_adj_263 (.A(ev_run_hold_s5[43]), .B(us_tx_c_43), .C(init_shadow[43]), 
         .D(swap_now_s5), .Z(n13812)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_263.init = 16'h5a66;
    LUT4 i1_3_lut_rep_236_4_lut (.A(ev_state[1]), .B(n26242), .C(ev_state[0]), 
         .D(frame_req_N_2584), .Z(pll_clk_enable_639)) /* synthesis lut_function=(!(A+(B+!(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i1_3_lut_rep_236_4_lut.init = 16'h1110;
    FD1P3AX stop_toggle_spi_557 (.D(stop_toggle_spi_N_2515), .SP(spi1_sck_c_enable_171), 
            .CK(spi1_sck_c), .Q(stop_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam stop_toggle_spi_557.GSR = "DISABLED";
    LUT4 i16_4_lut_adj_264 (.A(ev_run_hold_s5[44]), .B(us_tx_c_44), .C(init_shadow[44]), 
         .D(swap_now_s5), .Z(n13814)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_264.init = 16'h5a66;
    LUT4 i7229_2_lut_3_lut_4_lut (.A(ev_state[1]), .B(n26242), .C(ev_state[0]), 
         .D(frame_req_N_2584), .Z(n17861)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i7229_2_lut_3_lut_4_lut.init = 16'h0100;
    LUT4 n21588_bdd_4_lut_15360 (.A(n21588), .B(ev_state[2]), .C(ev_state[1]), 
         .D(ev_state[0]), .Z(n26080)) /* synthesis lut_function=(!(A ((C (D)+!C !(D))+!B)+!A ((C (D))+!B))) */ ;
    defparam n21588_bdd_4_lut_15360.init = 16'h0cc4;
    LUT4 i1_2_lut_rep_225_3_lut_4_lut (.A(n26256), .B(n26280), .C(n26258), 
         .D(ev_ch[1]), .Z(n26189)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_225_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_2_lut_rep_230_3_lut_4_lut (.A(n26256), .B(n26280), .C(n26249), 
         .D(ev_ch[0]), .Z(n26194)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_230_3_lut_4_lut.init = 16'hfffe;
    LUT4 i16_4_lut_adj_265 (.A(ev_run_hold_s5[45]), .B(us_tx_c_45), .C(init_shadow[45]), 
         .D(swap_now_s5), .Z(n13816)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_265.init = 16'h5a66;
    LUT4 i1_2_lut_rep_228_3_lut_4_lut (.A(n26256), .B(n26280), .C(n26251), 
         .D(ev_ch[1]), .Z(n26192)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_228_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_2_lut_rep_233_3_lut_4_lut (.A(n26256), .B(n26280), .C(n26255), 
         .D(ev_ch[0]), .Z(n26197)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_233_3_lut_4_lut.init = 16'hffef;
    LUT4 i1_2_lut_rep_229_3_lut_4_lut (.A(n26256), .B(n26280), .C(n26249), 
         .D(ev_ch[0]), .Z(n26193)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_229_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_4_lut_adj_266 (.A(ev_state[1]), .B(ev_state[3]), .C(ev_state[2]), 
         .D(ev_state[0]), .Z(pll_clk_enable_324)) /* synthesis lut_function=(!(A+(B (C+(D))+!B !(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i1_4_lut_adj_266.init = 16'h1004;
    LUT4 i1_2_lut_adj_267 (.A(ev_state[1]), .B(ev_state[0]), .Z(n25512)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_adj_267.init = 16'h2222;
    LUT4 i16_4_lut_adj_268 (.A(ev_run_hold_s5[46]), .B(us_tx_c_46), .C(init_shadow[46]), 
         .D(swap_now_s5), .Z(n13818)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_268.init = 16'h5a66;
    LUT4 i16_4_lut_adj_269 (.A(ev_run_hold_s5[47]), .B(us_tx_c_47), .C(init_shadow[47]), 
         .D(swap_now_s5), .Z(n13820)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_269.init = 16'h5a66;
    LUT4 i3189_4_lut (.A(ev_ch[0]), .B(staging_rd_addr[0]), .C(n17798), 
         .D(n25512), .Z(staging_rd_addr_6__N_722[0])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i3189_4_lut.init = 16'hcac0;
    LUT4 i16_4_lut_adj_270 (.A(ev_run_hold_s5[48]), .B(us_tx_c_48), .C(init_shadow[48]), 
         .D(swap_now_s5), .Z(n13822)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_270.init = 16'h5a66;
    PFUMX frame_end_I_0 (.BLUT(frame_end_N_2645), .ALUT(frame_end_N_2610), 
          .C0(n26128), .Z(frame_end));
    LUT4 i15253_2_lut_2_lut_4_lut (.A(n26266), .B(spi_command[4]), .C(n25683), 
         .D(spi1_sck_c_enable_171), .Z(spi1_sck_c_enable_170)) /* synthesis lut_function=(A (B (C (D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(418[34:54])
    defparam i15253_2_lut_2_lut_4_lut.init = 16'hf700;
    LUT4 i1_2_lut_adj_271 (.A(ev_state[1]), .B(ev_state_3__N_1990[2]), .Z(n34_adj_3187)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i1_2_lut_adj_271.init = 16'h2222;
    LUT4 i15251_4_lut (.A(mic_divider[3]), .B(n25667), .C(mic_divider[5]), 
         .D(n6), .Z(mic_tick_N_2577)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(646[21:42])
    defparam i15251_4_lut.init = 16'h0004;
    LUT4 i16_4_lut_adj_272 (.A(ev_run_hold_s5[49]), .B(us_tx_c_49), .C(init_shadow[49]), 
         .D(swap_now_s5), .Z(n13824)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_272.init = 16'h5a66;
    LUT4 i15086_3_lut (.A(mic_divider[1]), .B(mic_divider[2]), .C(mic_divider[0]), 
         .Z(n25667)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i15086_3_lut.init = 16'h8080;
    LUT4 i1_2_lut_adj_273 (.A(mic_divider[4]), .B(mic_divider[6]), .Z(n6)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_273.init = 16'heeee;
    LUT4 i1_2_lut_rep_231_3_lut_4_lut (.A(n26256), .B(n26280), .C(n26255), 
         .D(ev_ch[0]), .Z(n26195)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_231_3_lut_4_lut.init = 16'hefff;
    LUT4 i1_2_lut_rep_224_3_lut_4_lut (.A(n26256), .B(n26280), .C(n26251), 
         .D(ev_ch[1]), .Z(n26188)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_224_3_lut_4_lut.init = 16'hfffe;
    TSALL TSALL_INST (.TSALL(GND_net));
    LUT4 sub_158_inv_0_i6_1_lut (.A(status_bit_index[5]), .Z(spi1_miso_N_2492[5])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[55:80])
    defparam sub_158_inv_0_i6_1_lut.init = 16'h5555;
    LUT4 mic_clk_I_0_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(mic_clk_N_2501)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(666[18] 668[12])
    defparam mic_clk_I_0_2_lut.init = 16'h6666;
    CCU2D fpga_time_1596_add_4_29 (.A0(fpga_time[27]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[28]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24970), .COUT(n24971), .S0(n138), .S1(n137));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_29.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_29.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_29.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_29.INJECT1_1 = "NO";
    LUT4 i15279_3_lut_3_lut_4_lut (.A(n26266), .B(spi_command[4]), .C(n25683), 
         .D(n16929), .Z(invalid_frame_spi_N_2522)) /* synthesis lut_function=(A ((C)+!B)+!A !(D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(418[34:54])
    defparam i15279_3_lut_3_lut_4_lut.init = 16'ha2f7;
    LUT4 i16_4_lut_adj_274 (.A(ev_run_hold_s5[50]), .B(us_tx_c_50), .C(init_shadow[50]), 
         .D(swap_now_s5), .Z(n13826)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_274.init = 16'h5a66;
    LUT4 i6_4_lut_adj_275 (.A(mic_sample_count[3]), .B(n12_adj_3201), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_446)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_adj_275.init = 16'h0800;
    CCU2D fpga_time_1596_add_4_27 (.A0(fpga_time[25]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[26]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24969), .COUT(n24970), .S0(n140), .S1(n139));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_27.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_27.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_27.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_27.INJECT1_1 = "NO";
    CCU2D fpga_time_1596_add_4_25 (.A0(fpga_time[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[24]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24968), .COUT(n24969), .S0(n142), .S1(n141));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_25.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_25.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_25.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_25.INJECT1_1 = "NO";
    LUT4 i14148_1_lut (.A(status_bit_index[3]), .Z(spi1_miso_N_2492[3])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam i14148_1_lut.init = 16'h5555;
    LUT4 i7233_1_lut (.A(phase_step_s1), .Z(n17865)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam i7233_1_lut.init = 16'h5555;
    LUT4 i7_4_lut_adj_276 (.A(global_phase_s2[0]), .B(n14_adj_3186), .C(n10_adj_3198), 
         .D(global_phase_s2[6]), .Z(wrap_s2_N_2576)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut_adj_276.init = 16'h8000;
    LUT4 i1_2_lut_rep_223_3_lut_4_lut (.A(n26256), .B(n26280), .C(n26258), 
         .D(ev_ch[1]), .Z(n26187)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_223_3_lut_4_lut.init = 16'hfffe;
    LUT4 i6_4_lut_adj_277 (.A(global_phase_s2[3]), .B(global_phase_s2[1]), 
         .C(global_phase_s2[5]), .D(global_phase_s2[7]), .Z(n14_adj_3186)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_277.init = 16'h8000;
    LUT4 i2_2_lut_adj_278 (.A(global_phase_s2[2]), .B(global_phase_s2[4]), 
         .Z(n10_adj_3198)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut_adj_278.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_279 (.A(n26209), .B(n26219), .C(init_shadow[12]), 
         .D(n16988), .Z(n14784)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_279.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_234_3_lut_4_lut (.A(n26261), .B(n26280), .C(n26258), 
         .D(ev_ch[1]), .Z(n26198)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i1_2_lut_rep_234_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_2_lut_rep_238_3_lut_4_lut (.A(n26261), .B(n26280), .C(n26251), 
         .D(ev_ch[1]), .Z(n26202)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_rep_238_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_280 (.A(n26211), .B(n26219), .C(init_shadow[14]), 
         .D(n16988), .Z(n14772)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_280.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_239_3_lut_4_lut (.A(n26261), .B(n26280), .C(n26251), 
         .D(ev_ch[1]), .Z(n26203)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i1_2_lut_rep_239_3_lut_4_lut.init = 16'hfeff;
    LUT4 i16_4_lut_adj_281 (.A(ev_run_hold_s5[51]), .B(us_tx_c_51), .C(init_shadow[51]), 
         .D(swap_now_s5), .Z(n13828)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_281.init = 16'h5a66;
    LUT4 i1_2_lut_rep_235_3_lut_4_lut (.A(n26261), .B(n26280), .C(n26249), 
         .D(ev_ch[0]), .Z(n26199)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i1_2_lut_rep_235_3_lut_4_lut.init = 16'hfeff;
    LUT4 i1_2_lut_rep_241_3_lut_4_lut (.A(n26261), .B(n26280), .C(n26255), 
         .D(ev_ch[0]), .Z(n26205)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;
    defparam i1_2_lut_rep_241_3_lut_4_lut.init = 16'hffef;
    GSR GSR_INST (.GSR(fpga_cs_n_N_2500));
    LUT4 i1_2_lut_rep_237_3_lut_4_lut (.A(n26261), .B(n26280), .C(n26249), 
         .D(ev_ch[0]), .Z(n26201)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_rep_237_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_282 (.A(n26210), .B(n26219), .C(init_shadow[13]), 
         .D(n16988), .Z(n14778)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_282.init = 16'hf1f0;
    FD1P3IX spi_channel_field_1592__i1 (.D(n14_adj_3206), .SP(spi1_sck_c_enable_173), 
            .CD(n25121), .CK(spi1_sck_c), .Q(spi_channel_field[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(384[78:102])
    defparam spi_channel_field_1592__i1.GSR = "ENABLED";
    LUT4 i1_2_lut_rep_240_3_lut_4_lut (.A(n26261), .B(n26280), .C(n26255), 
         .D(ev_ch[0]), .Z(n26204)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;
    defparam i1_2_lut_rep_240_3_lut_4_lut.init = 16'hefff;
    LUT4 i1_2_lut_rep_232_3_lut_4_lut (.A(n26261), .B(n26280), .C(n26258), 
         .D(ev_ch[1]), .Z(n26196)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_rep_232_3_lut_4_lut.init = 16'hfffe;
    LUT4 i1_4_lut_adj_283 (.A(spi_command[1]), .B(spi1_sck_c_enable_36), 
         .C(n16929), .D(spi_command[0]), .Z(spi1_sck_c_enable_166)) /* synthesis lut_function=(A (B (C (D)))+!A !(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i1_4_lut_adj_283.init = 16'h8040;
    LUT4 i16_4_lut_adj_284 (.A(ev_run_hold_s5[52]), .B(us_tx_c_52), .C(init_shadow[52]), 
         .D(swap_now_s5), .Z(n13830)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_284.init = 16'h5a66;
    LUT4 i16_4_lut_adj_285 (.A(ev_run_hold_s5[53]), .B(us_tx_c_53), .C(init_shadow[53]), 
         .D(swap_now_s5), .Z(n13832)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_285.init = 16'h5a66;
    LUT4 i16_4_lut_adj_286 (.A(ev_run_hold_s5[54]), .B(us_tx_c_54), .C(init_shadow[54]), 
         .D(swap_now_s5), .Z(n13834)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_286.init = 16'h5a66;
    LUT4 i16_4_lut_adj_287 (.A(ev_run_hold_s5[55]), .B(us_tx_c_55), .C(init_shadow[55]), 
         .D(swap_now_s5), .Z(n13836)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_287.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_288 (.A(n26215), .B(n26219), .C(init_shadow[15]), 
         .D(n16988), .Z(n14766)) /* synthesis lut_function=(A (B (C)+!B (C+(D)))+!A (C)) */ ;
    defparam i1_3_lut_4_lut_adj_288.init = 16'hf2f0;
    LUT4 i16_4_lut_adj_289 (.A(ev_run_hold_s5[56]), .B(us_tx_c_56), .C(init_shadow[56]), 
         .D(swap_now_s5), .Z(n13838)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_289.init = 16'h5a66;
    LUT4 n21588_bdd_4_lut_15307 (.A(ev_state_3__N_1990[2]), .B(ev_state[2]), 
         .C(ev_state[1]), .D(ev_state[0]), .Z(n26079)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam n21588_bdd_4_lut_15307.init = 16'h0020;
    LUT4 i16_4_lut_adj_290 (.A(ev_run_hold_s5[57]), .B(us_tx_c_57), .C(init_shadow[57]), 
         .D(swap_now_s5), .Z(n13840)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_290.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_291 (.A(n26213), .B(n26233), .C(init_shadow[17]), 
         .D(n16988), .Z(n14754)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_291.init = 16'hf1f0;
    LUT4 i15158_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[61]), 
         .C(status_hold[60]), .Z(n25740)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15158_3_lut_3_lut.init = 16'he4e4;
    LUT4 i3_4_lut_adj_292 (.A(spi_command[4]), .B(spi_version[0]), .C(n25661), 
         .D(n25487), .Z(n16929)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i3_4_lut_adj_292.init = 16'h0008;
    LUT4 i15159_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[63]), 
         .C(status_hold[62]), .Z(n25741)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15159_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15080_4_lut (.A(spi_extension_length[3]), .B(spi_extension_length[5]), 
         .C(n6_adj_3212), .D(spi_extension_length[4]), .Z(n25661)) /* synthesis lut_function=(A (B)+!A (B (C+(D)))) */ ;
    defparam i15080_4_lut.init = 16'hccc8;
    LUT4 i15156_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[57]), 
         .C(status_hold[56]), .Z(n25738)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15156_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15154_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[53]), 
         .C(status_hold[52]), .Z(n25736)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15154_3_lut_3_lut.init = 16'he4e4;
    LUT4 i16_4_lut_adj_293 (.A(ev_run_hold_s5[58]), .B(us_tx_c_58), .C(init_shadow[58]), 
         .D(swap_now_s5), .Z(n13842)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_293.init = 16'h5a66;
    LUT4 i15157_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[59]), 
         .C(status_hold[58]), .Z(n25739)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15157_3_lut_3_lut.init = 16'he4e4;
    LUT4 i16_4_lut_adj_294 (.A(ev_run_hold_s5[59]), .B(us_tx_c_59), .C(init_shadow[59]), 
         .D(swap_now_s5), .Z(n13844)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_294.init = 16'h5a66;
    LUT4 i16_4_lut_adj_295 (.A(ev_run_hold_s5[60]), .B(us_tx_c_60), .C(init_shadow[60]), 
         .D(swap_now_s5), .Z(n13846)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_295.init = 16'h5a66;
    LUT4 i15153_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[51]), 
         .C(status_hold[50]), .Z(n25735)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15153_3_lut_3_lut.init = 16'he4e4;
    LUT4 i16_4_lut_adj_296 (.A(ev_run_hold_s5[61]), .B(us_tx_c_61), .C(init_shadow[61]), 
         .D(swap_now_s5), .Z(n13848)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_296.init = 16'h5a66;
    LUT4 i7_4_lut_adj_297 (.A(n13), .B(spi_command[7]), .C(n12), .D(spi_extension_length[7]), 
         .Z(n25487)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i7_4_lut_adj_297.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_298 (.A(n26217), .B(n26233), .C(init_shadow[16]), 
         .D(n16988), .Z(n14760)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_298.init = 16'hf1f0;
    LUT4 i3_4_lut_adj_299 (.A(ev_clear_addr[0]), .B(ev_clear_addr[4]), .C(ev_clear_addr[2]), 
         .D(ev_clear_addr[1]), .Z(n25477)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_4_lut_adj_299.init = 16'h8000;
    LUT4 i15152_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[49]), 
         .C(status_hold[48]), .Z(n25734)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15152_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15155_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[55]), 
         .C(status_hold[54]), .Z(n25737)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15155_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15149_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[43]), 
         .C(status_hold[42]), .Z(n25731)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15149_3_lut_3_lut.init = 16'he4e4;
    FD1P3IX spi_channel_field_1592__i0 (.D(n1_adj_3205), .SP(spi1_sck_c_enable_173), 
            .CD(n25121), .CK(spi1_sck_c), .Q(spi_channel_field[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(384[78:102])
    defparam spi_channel_field_1592__i0.GSR = "ENABLED";
    LUT4 i15148_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[41]), 
         .C(status_hold[40]), .Z(n25730)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15148_3_lut_3_lut.init = 16'he4e4;
    LUT4 i16_4_lut_adj_300 (.A(ev_run_hold_s5[62]), .B(us_tx_c_62), .C(init_shadow[62]), 
         .D(swap_now_s5), .Z(n13850)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_300.init = 16'h5a66;
    CCU2D fpga_time_1596_add_4_23 (.A0(fpga_time[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24967), .COUT(n24968), .S0(n144), .S1(n143));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_23.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_23.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_23.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_23.INJECT1_1 = "NO";
    LUT4 i15147_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[39]), 
         .C(status_hold[38]), .Z(n25729)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15147_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15146_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[37]), 
         .C(status_hold[36]), .Z(n25728)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15146_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_301 (.A(n26208), .B(n26233), .C(init_shadow[19]), 
         .D(n16988), .Z(n14742)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_301.init = 16'hf1f0;
    LUT4 i15151_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[47]), 
         .C(status_hold[46]), .Z(n25733)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15151_3_lut_3_lut.init = 16'he4e4;
    FD1P3IX frame_settle__i3 (.D(frame_settle_3__N_1982[3]), .SP(pll_clk_enable_649), 
            .CD(pll_clk_enable_22), .CK(pll_clk), .Q(frame_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam frame_settle__i3.GSR = "DISABLED";
    LUT4 i15145_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[35]), 
         .C(status_hold[34]), .Z(n25727)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15145_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15144_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[33]), 
         .C(status_hold[32]), .Z(n25726)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15144_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15128_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[31]), 
         .C(status_hold[30]), .Z(n25710)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15128_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15127_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[29]), 
         .C(status_hold[28]), .Z(n25709)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15127_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_302 (.A(n26220), .B(n26233), .C(init_shadow[18]), 
         .D(n16988), .Z(n14748)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_302.init = 16'hf1f0;
    LUT4 i15126_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[27]), 
         .C(status_hold[26]), .Z(n25708)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15126_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15125_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[25]), 
         .C(status_hold[24]), .Z(n25707)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15125_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15124_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[23]), 
         .C(status_hold[22]), .Z(n25706)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15124_3_lut_3_lut.init = 16'he4e4;
    FD1S3AX mem_1705 (.D(staging_rd_addr_6__N_722[2]), .CK(pll_clk), .Q(n12167));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam mem_1705.GSR = "DISABLED";
    LUT4 i15150_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[45]), 
         .C(status_hold[44]), .Z(n25732)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15150_3_lut_3_lut.init = 16'he4e4;
    FD1P3IX init_shadow_i2 (.D(n14844), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i2.GSR = "DISABLED";
    LUT4 i1_4_lut_adj_303 (.A(ev_clear_addr[3]), .B(ev_clear_addr[6]), .C(ev_clear_addr[7]), 
         .D(ev_clear_addr[5]), .Z(n4_adj_3211)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i1_4_lut_adj_303.init = 16'h8000;
    LUT4 i1_2_lut_rep_247_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(n26211)) /* synthesis lut_function=(((C)+!B)+!A) */ ;
    defparam i1_2_lut_rep_247_3_lut.init = 16'hf7f7;
    LUT4 i15123_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[21]), 
         .C(status_hold[20]), .Z(n25705)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15123_3_lut_3_lut.init = 16'he4e4;
    LUT4 i5_4_lut_adj_304 (.A(spi_command[2]), .B(spi_extension_length[6]), 
         .C(spi_command[3]), .D(spi_command[6]), .Z(n13)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i5_4_lut_adj_304.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_305 (.A(n26220), .B(n26234), .C(init_shadow[66]), 
         .D(n16988), .Z(n14460)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_305.init = 16'hf1f0;
    FD1P3IX init_shadow_i1 (.D(n14850), .SP(pll_clk_enable_652), .CD(n26487), 
            .CK(pll_clk), .Q(init_shadow[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam init_shadow_i1.GSR = "DISABLED";
    LUT4 i15122_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[19]), 
         .C(status_hold[18]), .Z(n25704)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15122_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX global_phase_s2_1595__i2 (.D(n43_adj_3199), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595__i2.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1595__i3 (.D(n42), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595__i3.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1595__i4 (.D(n41), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595__i4.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1595__i5 (.D(n40_adj_3197), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595__i5.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1595__i6 (.D(n39_adj_3196), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595__i6.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1595__i7 (.D(n38_adj_3195), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(450[32:54])
    defparam global_phase_s2_1595__i7.GSR = "DISABLED";
    FD1P3AX status_bit_index_1590__i1 (.D(n39_adj_3177), .SP(spi1_sck_N_305_enable_7), 
            .CK(spi1_sck_N_305), .Q(status_bit_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590__i1.GSR = "ENABLED";
    CCU2D fpga_time_1596_add_4_21 (.A0(fpga_time[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24966), .COUT(n24967), .S0(n146), .S1(n145));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_21.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_21.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_21.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_21.INJECT1_1 = "NO";
    CCU2D mic_divider_1599_add_4_5 (.A0(mic_divider[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24926), .COUT(n24927), .S0(n37_adj_3191), 
          .S1(n36_adj_3190));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_divider_1599_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_divider_1599_add_4_5.INJECT1_0 = "NO";
    defparam mic_divider_1599_add_4_5.INJECT1_1 = "NO";
    CCU2D fpga_time_1596_add_4_19 (.A0(fpga_time[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24965), .COUT(n24966), .S0(n148), .S1(n147));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_19.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_19.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_19.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_19.INJECT1_1 = "NO";
    LUT4 i15121_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[17]), 
         .C(status_hold[16]), .Z(n25703)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15121_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX status_bit_index_1590__i2 (.D(n38_adj_3176), .SP(spi1_sck_N_305_enable_7), 
            .CK(spi1_sck_N_305), .Q(status_bit_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590__i2.GSR = "ENABLED";
    FD1P3AX status_bit_index_1590__i3 (.D(n37_adj_3175), .SP(spi1_sck_N_305_enable_7), 
            .CK(spi1_sck_N_305), .Q(status_bit_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590__i3.GSR = "ENABLED";
    FD1P3AX status_bit_index_1590__i4 (.D(n36_adj_3174), .SP(spi1_sck_N_305_enable_7), 
            .CK(spi1_sck_N_305), .Q(status_bit_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590__i4.GSR = "ENABLED";
    FD1P3AX status_bit_index_1590__i5 (.D(n35_adj_3173), .SP(spi1_sck_N_305_enable_7), 
            .CK(spi1_sck_N_305), .Q(status_bit_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590__i5.GSR = "ENABLED";
    FD1P3AX status_bit_index_1590__i6 (.D(n34_adj_3172), .SP(spi1_sck_N_305_enable_7), 
            .CK(spi1_sck_N_305), .Q(status_bit_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[66:89])
    defparam status_bit_index_1590__i6.GSR = "ENABLED";
    FD1P3AX fpga_time_1596__i1 (.D(n164), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i1.GSR = "DISABLED";
    LUT4 i15264_4_lut (.A(time_divider[6]), .B(time_divider[3]), .C(n25669), 
         .D(time_divider[5]), .Z(pll_clk_enable_683)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(484[13:34])
    defparam i15264_4_lut.init = 16'h4000;
    FD1P3AX fpga_time_1596__i2 (.D(n163), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i2.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i3 (.D(n162), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i3.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i4 (.D(n161), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i4.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i5 (.D(n160), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i5.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i6 (.D(n159), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i6.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i7 (.D(n158), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i7.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i8 (.D(n157), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i8.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i9 (.D(n156), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i9.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i10 (.D(n155), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[10])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i10.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i11 (.D(n154), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[11])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i11.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i12 (.D(n153), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[12])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i12.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i13 (.D(n152), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[13])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i13.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i14 (.D(n151), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[14])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i14.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i15 (.D(n150), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[15])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i15.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i16 (.D(n149), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[16])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i16.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i17 (.D(n148), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[17])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i17.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i18 (.D(n147), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[18])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i18.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i19 (.D(n146), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[19])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i19.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i20 (.D(n145), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[20])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i20.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i21 (.D(n144), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[21])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i21.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i22 (.D(n143), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[22])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i22.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i23 (.D(n142), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[23])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i23.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i24 (.D(n141), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[24])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i24.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i25 (.D(n140), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[25])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i25.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i26 (.D(n139), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[26])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i26.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i27 (.D(n138), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[27])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i27.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i28 (.D(n137), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[28])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i28.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i29 (.D(n136), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[29])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i29.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i30 (.D(n135), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[30])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i30.GSR = "DISABLED";
    FD1P3AX fpga_time_1596__i31 (.D(n134), .SP(pll_clk_enable_683), .CK(pll_clk), 
            .Q(fpga_time[31])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596__i31.GSR = "DISABLED";
    FD1P3AX spi_channel_index_1591__i1 (.D(n39_adj_3168), .SP(spi1_sck_c_enable_179), 
            .CK(spi1_sck_c), .Q(spi_channel_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591__i1.GSR = "ENABLED";
    PFUMX ev_state_3__I_0_717_Mux_1_i15 (.BLUT(n7_adj_3218), .ALUT(n14_adj_3219), 
          .C0(ev_state[3]), .Z(ev_state_3__N_602[1]));
    CCU2D fpga_time_1596_add_4_17 (.A0(fpga_time[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24964), .COUT(n24965), .S0(n150), .S1(n149));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_17.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_17.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_17.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_17.INJECT1_1 = "NO";
    LUT4 i15120_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[15]), 
         .C(status_hold[14]), .Z(n25702)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15120_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_306 (.A(n26210), .B(n26233), .C(init_shadow[21]), 
         .D(n16988), .Z(n14730)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_306.init = 16'hf1f0;
    LUT4 i16_4_lut_adj_307 (.A(ev_run_hold_s5[63]), .B(us_tx_c_63), .C(init_shadow[63]), 
         .D(swap_now_s5), .Z(n13852)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_307.init = 16'h5a66;
    LUT4 i15119_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[13]), 
         .C(status_hold[12]), .Z(n25701)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15119_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX spi_channel_index_1591__i2 (.D(n38_adj_3169), .SP(spi1_sck_c_enable_179), 
            .CK(spi1_sck_c), .Q(spi_channel_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591__i2.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1591__i3 (.D(n37_adj_3170), .SP(spi1_sck_c_enable_179), 
            .CK(spi1_sck_c), .Q(spi_channel_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591__i3.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1591__i4 (.D(n36_adj_3171), .SP(spi1_sck_c_enable_179), 
            .CK(spi1_sck_c), .Q(spi_channel_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591__i4.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1591__i5 (.D(n35_adj_3183), .SP(spi1_sck_c_enable_179), 
            .CK(spi1_sck_c), .Q(spi_channel_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591__i5.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1591__i6 (.D(n34_adj_3184), .SP(spi1_sck_c_enable_179), 
            .CK(spi1_sck_c), .Q(spi_channel_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(380[64:88])
    defparam spi_channel_index_1591__i6.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut_adj_308 (.A(n26211), .B(n26233), .C(init_shadow[22]), 
         .D(n16988), .Z(n14724)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_308.init = 16'hf1f0;
    LUT4 i15118_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[11]), 
         .C(status_hold[10]), .Z(n25700)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15118_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15117_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[9]), 
         .C(status_hold[8]), .Z(n25699)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15117_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15116_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[7]), 
         .C(status_hold[6]), .Z(n25698)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15116_3_lut_3_lut.init = 16'he4e4;
    LUT4 i15115_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[5]), 
         .C(status_hold[4]), .Z(n25697)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15115_3_lut_3_lut.init = 16'he4e4;
    LUT4 i16_4_lut_adj_309 (.A(ev_run_hold_s5[64]), .B(us_tx_c_64), .C(init_shadow[64]), 
         .D(swap_now_s5), .Z(n13854)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_309.init = 16'h5a66;
    LUT4 m1_lut (.Z(n26471)) /* synthesis lut_function=1, syn_instantiated=1 */ ;
    defparam m1_lut.init = 16'hffff;
    LUT4 i15114_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[3]), 
         .C(status_hold[2]), .Z(n25696)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15114_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_310 (.A(n26215), .B(n26233), .C(init_shadow[23]), 
         .D(n16988), .Z(n14718)) /* synthesis lut_function=(A (B (C)+!B (C+(D)))+!A (C)) */ ;
    defparam i1_3_lut_4_lut_adj_310.init = 16'hf2f0;
    CCU2D mic_divider_1599_add_4_3 (.A0(mic_divider[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24925), .COUT(n24926), .S0(n39_adj_3193), 
          .S1(n38_adj_3192));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_divider_1599_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_divider_1599_add_4_3.INJECT1_0 = "NO";
    defparam mic_divider_1599_add_4_3.INJECT1_1 = "NO";
    CCU2D mic_divider_1599_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n24925), .S1(n40_adj_3194));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(667[28:46])
    defparam mic_divider_1599_add_4_1.INIT0 = 16'hF000;
    defparam mic_divider_1599_add_4_1.INIT1 = 16'h0555;
    defparam mic_divider_1599_add_4_1.INJECT1_0 = "NO";
    defparam mic_divider_1599_add_4_1.INJECT1_1 = "NO";
    LUT4 i15113_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[1]), 
         .C(status_hold[0]), .Z(n25695)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(308[18:44])
    defparam i15113_3_lut_3_lut.init = 16'he4e4;
    LUT4 i16_4_lut_adj_311 (.A(ev_run_hold_s5[65]), .B(us_tx_c_65), .C(init_shadow[65]), 
         .D(swap_now_s5), .Z(n13856)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_311.init = 16'h5a66;
    LUT4 i1_2_lut_rep_277 (.A(expected_next[0]), .B(expected_next[1]), .Z(n26241)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_277.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_312 (.A(n26208), .B(n26234), .C(init_shadow[67]), 
         .D(n16988), .Z(n14454)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_312.init = 16'hf1f0;
    CCU2D fpga_time_1596_add_4_15 (.A0(fpga_time[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24963), .COUT(n24964), .S0(n152), .S1(n151));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_15.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_15.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_15.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_15.INJECT1_1 = "NO";
    CCU2D add_14302_7 (.A0(spi_byte_count[8]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24924), .S1(n11289));
    defparam add_14302_7.INIT0 = 16'h5555;
    defparam add_14302_7.INIT1 = 16'h0000;
    defparam add_14302_7.INJECT1_0 = "NO";
    defparam add_14302_7.INJECT1_1 = "NO";
    CCU2D add_14302_5 (.A0(spi_byte_count[6]), .B0(spi_rgb_payload_byte_N_2547[6]), 
          .C0(GND_net), .D0(GND_net), .A1(spi_byte_count[7]), .B1(spi_rgb_payload_byte_N_2547[6]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24923), .COUT(n24924));
    defparam add_14302_5.INIT0 = 16'h5999;
    defparam add_14302_5.INIT1 = 16'h5999;
    defparam add_14302_5.INJECT1_0 = "NO";
    defparam add_14302_5.INJECT1_1 = "NO";
    CCU2D add_244_9 (.A0(spi_byte_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24901), .COUT(n24902), .S0(spi_byte_count_15__N_1504[7]), 
          .S1(spi_byte_count_15__N_1504[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[35:57])
    defparam add_244_9.INIT0 = 16'h5aaa;
    defparam add_244_9.INIT1 = 16'h5aaa;
    defparam add_244_9.INJECT1_0 = "NO";
    defparam add_244_9.INJECT1_1 = "NO";
    CCU2D add_244_7 (.A0(spi_byte_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24900), .COUT(n24901), .S0(spi_byte_count_15__N_1504[5]), 
          .S1(spi_byte_count_15__N_1504[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[35:57])
    defparam add_244_7.INIT0 = 16'h5aaa;
    defparam add_244_7.INIT1 = 16'h5aaa;
    defparam add_244_7.INJECT1_0 = "NO";
    defparam add_244_7.INJECT1_1 = "NO";
    CCU2D add_14302_3 (.A0(spi_byte_count[4]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[5]), .B1(spi_rgb_payload_byte_N_2547[6]), 
          .C1(GND_net), .D1(GND_net), .CIN(n24922), .COUT(n24923));
    defparam add_14302_3.INIT0 = 16'h5555;
    defparam add_14302_3.INIT1 = 16'h5666;
    defparam add_14302_3.INJECT1_0 = "NO";
    defparam add_14302_3.INJECT1_1 = "NO";
    CCU2D fpga_time_1596_add_4_13 (.A0(fpga_time[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24962), .COUT(n24963), .S0(n154), .S1(n153));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_13.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_13.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_13.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_13.INJECT1_1 = "NO";
    CCU2D fpga_time_1596_add_4_11 (.A0(fpga_time[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24961), .COUT(n24962), .S0(n156), .S1(n155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_11.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_11.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_11.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_11.INJECT1_1 = "NO";
    LUT4 i2_2_lut_3_lut (.A(expected_next[0]), .B(expected_next[1]), .C(spi_extension_length[2]), 
         .Z(n6_adj_3212)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_2_lut_3_lut.init = 16'hfefe;
    LUT4 i4_4_lut_adj_313 (.A(spi_command[5]), .B(n27_adj_3213), .C(n25_adj_3215), 
         .D(n26_adj_3214), .Z(n12)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut_adj_313.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_314 (.A(n26209), .B(n26234), .C(init_shadow[68]), 
         .D(n16988), .Z(n14448)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_314.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_278 (.A(ev_state[2]), .B(ev_state[3]), .Z(n26242)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i1_2_lut_rep_278.init = 16'heeee;
    LUT4 i1_2_lut_rep_179_3_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(frame_req_N_2584), .D(n26247), .Z(pll_clk_enable_355)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i1_2_lut_rep_179_3_lut_4_lut.init = 16'h0010;
    PFUMX i14150 (.BLUT(n25756), .ALUT(n25725), .C0(status_bit_index[5]), 
          .Z(n24608));
    LUT4 i12_4_lut_adj_315 (.A(spi_extension_length[12]), .B(n24_adj_3216), 
         .C(spi_version[2]), .D(spi_version[4]), .Z(n27_adj_3213)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i12_4_lut_adj_315.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_316 (.A(ev_state[2]), .B(ev_state[3]), .C(n26141), 
         .D(n26243), .Z(n17798)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i1_3_lut_4_lut_adj_316.init = 16'hffef;
    LUT4 i1_3_lut_4_lut_adj_317 (.A(n26210), .B(n26234), .C(init_shadow[69]), 
         .D(n16988), .Z(n14442)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_317.init = 16'hf1f0;
    LUT4 i16_4_lut_adj_318 (.A(ev_run_hold_s5[66]), .B(us_tx_c_66), .C(init_shadow[66]), 
         .D(swap_now_s5), .Z(n13858)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_318.init = 16'h5a66;
    CCU2D equal_2398_0 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(frame_end_N_2613[16]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24751));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(322[49:96])
    defparam equal_2398_0.INIT0 = 16'hF000;
    defparam equal_2398_0.INIT1 = 16'h5555;
    defparam equal_2398_0.INJECT1_0 = "NO";
    defparam equal_2398_0.INJECT1_1 = "YES";
    LUT4 i1_2_lut_rep_274_3_lut (.A(ev_state[2]), .B(ev_state[3]), .C(ev_state[1]), 
         .Z(n26238)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i1_2_lut_rep_274_3_lut.init = 16'hfefe;
    LUT4 i1_3_lut_4_lut_adj_319 (.A(n26217), .B(n26236), .C(init_shadow[24]), 
         .D(n16988), .Z(n14712)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_319.init = 16'hf1f0;
    LUT4 i1_2_lut_rep_183_3_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[0]), .D(ev_state[1]), .Z(n26147)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(569[9] 643[16])
    defparam i1_2_lut_rep_183_3_lut_4_lut.init = 16'hffef;
    LUT4 i1_2_lut_rep_279 (.A(ev_state[0]), .B(ev_state[1]), .Z(n26243)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_rep_279.init = 16'h8888;
    LUT4 i16_4_lut_adj_320 (.A(ev_run_hold_s5[67]), .B(us_tx_c_67), .C(init_shadow[67]), 
         .D(swap_now_s5), .Z(n13860)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_320.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_321 (.A(n26211), .B(n26234), .C(init_shadow[70]), 
         .D(n16988), .Z(n14436)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_321.init = 16'hf1f0;
    umh_channel_ram18 staging_ram (.n12168(n12168), .spi1_sck_c(spi1_sck_c), 
            .spi_channel_index({spi_channel_index}), .n12170(n12170), .staging_q({staging_q}), 
            .pll_clk(pll_clk), .rd_data_15__N_2646({rd_data_15__N_2646}), 
            .n12164(n12164), .n12162(n12162), .n12174(n12174), .n12172(n12172), 
            .spi_write(spi_write), .VCC_net(VCC_net), .GND_net(GND_net), 
            .staging_rd_addr_6__N_722({staging_rd_addr_6__N_722}), .spi1_mosi_c_0(spi1_mosi_c_0), 
            .\spi_rx_shift[0] (spi_rx_shift[0]), .\spi_rx_shift[1] (spi_rx_shift[1]), 
            .\spi_rx_shift[2] (spi_rx_shift[2]), .\spi_rx_shift[3] (spi_rx_shift[3]), 
            .\spi_rx_shift[4] (spi_rx_shift[4]), .\spi_rx_shift[5] (spi_rx_shift[5]), 
            .\spi_rx_shift[6] (spi_rx_shift[6]), .spi_phase_pending({spi_phase_pending}), 
            .n12179(n12179), .n12181(n12181), .n12183(n12183), .n12185(n12185), 
            .n12187(n12187), .n12189(n12189), .n12191(n12191), .n12193(n12193), 
            .n12195(n12195), .n12197(n12197), .n12199(n12199), .n12201(n12201), 
            .n12203(n12203), .n12205(n12205), .n12207(n12207), .n12209(n12209), 
            .n12166(n12166)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(227[23] 230[6])
    LUT4 i16_4_lut_adj_322 (.A(ev_run_hold_s5[68]), .B(us_tx_c_68), .C(init_shadow[68]), 
         .D(swap_now_s5), .Z(n13862)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_322.init = 16'h5a66;
    LUT4 i10_4_lut_adj_323 (.A(spi_version[3]), .B(spi_extension_length[13]), 
         .C(spi_version[6]), .D(spi_version[7]), .Z(n25_adj_3215)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i10_4_lut_adj_323.init = 16'hfffe;
    LUT4 i14318_2_lut_rep_280 (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .Z(n26244)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam i14318_2_lut_rep_280.init = 16'h8888;
    LUT4 i14329_2_lut_3_lut_4_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .C(mic_sample_count[3]), .D(mic_sample_count[2]), .Z(n27)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam i14329_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i5_3_lut_4_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .C(mic_tick), .D(mic_sample_count[2]), .Z(n12_adj_3201)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam i5_3_lut_4_lut.init = 16'h8000;
    LUT4 i14322_2_lut_3_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .C(mic_sample_count[2]), .Z(n28)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(659[37:60])
    defparam i14322_2_lut_3_lut.init = 16'h7878;
    LUT4 i16_4_lut_adj_324 (.A(ev_run_hold_s5[69]), .B(us_tx_c_69), .C(init_shadow[69]), 
         .D(swap_now_s5), .Z(n13864)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_324.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_325 (.A(n26213), .B(n26236), .C(init_shadow[25]), 
         .D(n16988), .Z(n14706)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_325.init = 16'hf1f0;
    LUT4 i11_4_lut (.A(spi_extension_length[15]), .B(spi_extension_length[14]), 
         .C(spi_extension_length[9]), .D(spi_extension_length[10]), .Z(n26_adj_3214)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i11_4_lut.init = 16'hfffe;
    LUT4 i16_4_lut_adj_326 (.A(ev_run_hold_s5[70]), .B(us_tx_c_70), .C(init_shadow[70]), 
         .D(swap_now_s5), .Z(n13866)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_326.init = 16'h5a66;
    LUT4 i16_4_lut_adj_327 (.A(ev_run_hold_s5[71]), .B(us_tx_c_71), .C(init_shadow[71]), 
         .D(swap_now_s5), .Z(n13868)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_327.init = 16'h5a66;
    LUT4 i9_4_lut (.A(spi_version[5]), .B(spi_extension_length[11]), .C(spi_version[1]), 
         .D(spi_extension_length[8]), .Z(n24_adj_3216)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i9_4_lut.init = 16'hfffe;
    LUT4 i16_4_lut_adj_328 (.A(ev_run_hold_s5[72]), .B(us_tx_c_72), .C(init_shadow[72]), 
         .D(swap_now_s5), .Z(n13870)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_328.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_329 (.A(n26220), .B(n26236), .C(init_shadow[26]), 
         .D(n16988), .Z(n14700)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_329.init = 16'hf1f0;
    CCU2D fpga_time_1596_add_4_9 (.A0(fpga_time[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24960), .COUT(n24961), .S0(n158), .S1(n157));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_9.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_9.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_9.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_9.INJECT1_1 = "NO";
    LUT4 i15143_3_lut_3_lut (.A(status_bit_index[4]), .B(n25724), .C(n25723), 
         .Z(n25725)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[55:80])
    defparam i15143_3_lut_3_lut.init = 16'he4e4;
    CCU2D fpga_time_1596_add_4_7 (.A0(fpga_time[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24959), .COUT(n24960), .S0(n160), .S1(n159));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_7.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_7.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_7.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_7.INJECT1_1 = "NO";
    LUT4 i16_4_lut_adj_330 (.A(ev_run_hold_s5[73]), .B(us_tx_c_73), .C(init_shadow[73]), 
         .D(swap_now_s5), .Z(n13872)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_330.init = 16'h5a66;
    LUT4 i16_4_lut_adj_331 (.A(ev_run_hold_s5[74]), .B(us_tx_c_74), .C(init_shadow[74]), 
         .D(swap_now_s5), .Z(n13874)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_331.init = 16'h5a66;
    FD1P3IX ev_ch_i6 (.D(ev_ch_6__N_2010[6]), .SP(pll_clk_enable_684), .CD(n17880), 
            .CK(pll_clk), .Q(ev_ch[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam ev_ch_i6.GSR = "DISABLED";
    LUT4 mux_1731_i1_3_lut (.A(n12179), .B(n12180), .C(n12178), .Z(rd_data_15__N_2646[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1731_i1_3_lut.init = 16'hcaca;
    LUT4 i16_4_lut_adj_332 (.A(ev_run_hold_s5[75]), .B(us_tx_c_75), .C(init_shadow[75]), 
         .D(swap_now_s5), .Z(n13876)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_332.init = 16'h5a66;
    LUT4 i7_4_lut_adj_333 (.A(n12177), .B(n25675), .C(n25637), .D(n6_adj_3203), 
         .Z(n12178)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i7_4_lut_adj_333.init = 16'h0002;
    PFUMX i15112 (.BLUT(n25692), .ALUT(n25693), .C0(spi1_miso_N_2492[5]), 
          .Z(n25694));
    LUT4 i15094_4_lut (.A(n12166), .B(n25621), .C(n5_adj_3204), .D(n12167), 
         .Z(n25675)) /* synthesis lut_function=(A (B+(C+!(D)))+!A (B+(C+(D)))) */ ;
    defparam i15094_4_lut.init = 16'hfdfe;
    LUT4 i15056_4_lut (.A(n12168), .B(n12162), .C(n12169), .D(n12163), 
         .Z(n25637)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i15056_4_lut.init = 16'h7bde;
    LUT4 equal_1712_i6_2_lut (.A(n12172), .B(n12173), .Z(n6_adj_3203)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam equal_1712_i6_2_lut.init = 16'h6666;
    LUT4 i16_4_lut_adj_334 (.A(ev_run_hold_s5[76]), .B(us_tx_c_76), .C(init_shadow[76]), 
         .D(swap_now_s5), .Z(n13878)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_334.init = 16'h5a66;
    LUT4 i15040_4_lut (.A(n12174), .B(n12164), .C(n12175), .D(n12165), 
         .Z(n25621)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i15040_4_lut.init = 16'h7bde;
    LUT4 equal_1712_i5_2_lut (.A(n12170), .B(n12171), .Z(n5_adj_3204)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam equal_1712_i5_2_lut.init = 16'h6666;
    LUT4 i15111_3_lut_3_lut (.A(status_bit_index[4]), .B(n26271), .C(status_hold[104]), 
         .Z(n25693)) /* synthesis lut_function=(A (B (C))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[55:80])
    defparam i15111_3_lut_3_lut.init = 16'hc4c4;
    LUT4 i15210_3_lut_4_lut_4_lut (.A(status_bit_index[4]), .B(n26311), 
         .C(status_hold[88]), .D(n26271), .Z(n25692)) /* synthesis lut_function=(A (B)+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[55:80])
    defparam i15210_3_lut_4_lut_4_lut.init = 16'hd888;
    LUT4 i1_3_lut_4_lut_adj_335 (.A(n26208), .B(n26236), .C(init_shadow[27]), 
         .D(n16988), .Z(n14694)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_335.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_336 (.A(n26209), .B(n26236), .C(init_shadow[28]), 
         .D(n16988), .Z(n14688)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_336.init = 16'hf1f0;
    LUT4 i16_4_lut_adj_337 (.A(ev_run_hold_s5[77]), .B(us_tx_c_77), .C(init_shadow[77]), 
         .D(swap_now_s5), .Z(n13880)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_337.init = 16'h5a66;
    LUT4 i15174_3_lut_3_lut (.A(status_bit_index[4]), .B(n25755), .C(n25754), 
         .Z(n25756)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[55:80])
    defparam i15174_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_338 (.A(n26215), .B(n26234), .C(init_shadow[71]), 
         .D(n16988), .Z(n14430)) /* synthesis lut_function=(A (B (C)+!B (C+(D)))+!A (C)) */ ;
    defparam i1_3_lut_4_lut_adj_338.init = 16'hf2f0;
    LUT4 active_bank_I_0_1_lut_rep_281 (.A(active_bank), .Z(n26245)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[34:46])
    defparam active_bank_I_0_1_lut_rep_281.init = 16'h5555;
    LUT4 run_addr_s3_8__I_0_i9_3_lut_3_lut (.A(active_bank), .B(n21588), 
         .C(run_addr_s3[8]), .Z(event_rd_addr[8])) /* synthesis lut_function=(A (B (C))+!A ((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[34:46])
    defparam run_addr_s3_8__I_0_i9_3_lut_3_lut.init = 16'hd1d1;
    LUT4 i4_4_lut_adj_339 (.A(n26225), .B(spi_byte_count[5]), .C(spi_byte_count[1]), 
         .D(n26264), .Z(n10_adj_3207)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;
    defparam i4_4_lut_adj_339.init = 16'hffbf;
    LUT4 sub_158_inv_0_i3_1_lut_rep_282 (.A(status_bit_index[2]), .Z(n26246)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[55:80])
    defparam sub_158_inv_0_i3_1_lut_rep_282.init = 16'h5555;
    LUT4 i10837_2_lut_rep_283 (.A(ev_state[1]), .B(ev_state[0]), .Z(n26247)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10837_2_lut_rep_283.init = 16'heeee;
    LUT4 i1_2_lut_rep_285 (.A(ev_ch[1]), .B(ev_ch[2]), .Z(n26249)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_285.init = 16'hbbbb;
    LUT4 i1_2_lut_rep_245_3_lut (.A(ev_ch[1]), .B(ev_ch[2]), .C(ev_ch[0]), 
         .Z(n26209)) /* synthesis lut_function=(A+((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[46:69])
    defparam i1_2_lut_rep_245_3_lut.init = 16'hfbfb;
    LUT4 i1_3_lut_4_lut_adj_340 (.A(n26210), .B(n26236), .C(init_shadow[29]), 
         .D(n16988), .Z(n14682)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_340.init = 16'hf1f0;
    LUT4 i1_3_lut_4_lut_adj_341 (.A(n26209), .B(n26233), .C(init_shadow[20]), 
         .D(n16988), .Z(n14736)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_341.init = 16'hf1f0;
    LUT4 i15262_2_lut_3_lut (.A(spi_byte_count[0]), .B(n1_adj_3167), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_82)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(338[18] 436[12])
    defparam i15262_2_lut_3_lut.init = 16'h0404;
    LUT4 i16_4_lut_adj_342 (.A(ev_run_hold_s5[78]), .B(us_tx_c_78), .C(init_shadow[78]), 
         .D(swap_now_s5), .Z(n13882)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(175[17:28])
    defparam i16_4_lut_adj_342.init = 16'h5a66;
    PFUMX i15348 (.BLUT(n26309), .ALUT(n26310), .C0(status_hold[74]), 
          .Z(n26311));
    PUR PUR_INST (.PUR(VCC_net));
    defparam PUR_INST.RST_PULSE = 1;
    PFUMX i15346 (.BLUT(n26306), .ALUT(n26307), .C0(spi_byte_count[3]), 
          .Z(n66));
    LUT4 i1_3_lut_4_lut_adj_343 (.A(n26213), .B(n26235), .C(init_shadow[73]), 
         .D(n16988), .Z(n14418)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut_adj_343.init = 16'hf1f0;
    PFUMX i15344 (.BLUT(n26303), .ALUT(n26304), .C0(ev_state[0]), .Z(ev_wr_addr[0]));
    ws2812_stream ws2812_i (.state({state}), .pll_clk(pll_clk), .n26137(n26137), 
            .GND_net(GND_net), .pll_clk_enable_650(pll_clk_enable_650), 
            .\shift_register[1] (shift_register[1]), .n12991(n12991), .\shift_register[2] (shift_register[2]), 
            .n12996(n12996), .\shift_register[3] (shift_register[3]), .n13000(n13000), 
            .\shift_register[4] (shift_register[4]), .n13071(n13071), .\shift_register[5] (shift_register[5]), 
            .n13339(n13339), .\shift_register[6] (shift_register[6]), .n13343(n13343), 
            .\shift_register[7] (shift_register[7]), .n13357(n13357), .\shift_register[8] (shift_register[8]), 
            .n13361(n13361), .\shift_register[9] (shift_register[9]), .n13365(n13365), 
            .\shift_register[10] (shift_register[10]), .n13369(n13369), 
            .\shift_register[11] (shift_register[11]), .n13373(n13373), 
            .\shift_register[12] (shift_register[12]), .n13377(n13377), 
            .\shift_register[13] (shift_register[13]), .n13381(n13381), 
            .\shift_register[14] (shift_register[14]), .n13385(n13385), 
            .\shift_register[15] (shift_register[15]), .n13389(n13389), 
            .\shift_register[16] (shift_register[16]), .n13395(n13395), 
            .\shift_register[17] (shift_register[17]), .n13402(n13402), 
            .\shift_register[18] (shift_register[18]), .n13406(n13406), 
            .\shift_register[19] (shift_register[19]), .n13419(n13419), 
            .\shift_register[20] (shift_register[20]), .n13423(n13423), 
            .\shift_register[21] (shift_register[21]), .n13427(n13427), 
            .\shift_register[22] (shift_register[22]), .n13431(n13431), 
            .n13435(n13435), .rgb_data_c(rgb_data_c), .\rgb_hold[16] (rgb_hold[16]), 
            .\shift_register[0] (shift_register[0])) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(690[19] 702[6])
    PFUMX i15342 (.BLUT(n26300), .ALUT(n26301), .C0(ev_state[0]), .Z(ev_wr_addr[1]));
    L6MUX21 i15141 (.D0(n25719), .D1(n25720), .SD(spi1_miso_N_2492[3]), 
            .Z(n25723));
    PFUMX i15340 (.BLUT(n26297), .ALUT(n26298), .C0(ev_state[0]), .Z(ev_wr_addr[2]));
    FD1S3AX phase_step_s4_571_rep_326 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(pll_clk_enable_542)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_step_s4_571_rep_326.GSR = "DISABLED";
    L6MUX21 i15142 (.D0(n25721), .D1(n25722), .SD(spi1_miso_N_2492[3]), 
            .Z(n25724));
    PFUMX i15338 (.BLUT(n26294), .ALUT(n26295), .C0(ev_state[0]), .Z(ev_wr_addr[3]));
    FD1S3AX phase_step_s4_571_rep_325 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(n26482)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam phase_step_s4_571_rep_325.GSR = "DISABLED";
    PFUMX i15336 (.BLUT(n26291), .ALUT(n26292), .C0(ev_state[0]), .Z(ev_wr_addr[4]));
    L6MUX21 i15172 (.D0(n25750), .D1(n25751), .SD(spi1_miso_N_2492[3]), 
            .Z(n25754));
    CCU2D fpga_time_1596_add_4_5 (.A0(fpga_time[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24958), .COUT(n24959), .S0(n162), .S1(n161));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(486[29:46])
    defparam fpga_time_1596_add_4_5.INIT0 = 16'hfaaa;
    defparam fpga_time_1596_add_4_5.INIT1 = 16'hfaaa;
    defparam fpga_time_1596_add_4_5.INJECT1_0 = "NO";
    defparam fpga_time_1596_add_4_5.INJECT1_1 = "NO";
    PFUMX i15334 (.BLUT(n26288), .ALUT(n26289), .C0(ev_state[0]), .Z(ev_wr_addr[5]));
    L6MUX21 i15173 (.D0(n25752), .D1(n25753), .SD(spi1_miso_N_2492[3]), 
            .Z(n25755));
    umh_toggle_ram84 event_ram (.pll_clk(pll_clk), .ev_we(ev_we), .VCC_net(VCC_net), 
            .GND_net(GND_net), .\ev_wr_addr[0] (ev_wr_addr[0]), .event_rd_addr({event_rd_addr}), 
            .\ev_wr_addr[1] (ev_wr_addr[1]), .\ev_wr_addr[2] (ev_wr_addr[2]), 
            .\ev_wr_addr[3] (ev_wr_addr[3]), .\ev_wr_addr[4] (ev_wr_addr[4]), 
            .\ev_wr_addr[5] (ev_wr_addr[5]), .\ev_wr_addr[6] (ev_wr_addr[6]), 
            .\ev_wr_addr[7] (ev_wr_addr[7]), .n26245(n26245), .ev_wr_data({ev_wr_data}), 
            .ev_rd_data({ev_rd_data})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(250[22] 255[6])
    PFUMX i15332 (.BLUT(n26285), .ALUT(n26286), .C0(ev_state[0]), .Z(ev_wr_addr[6]));
    FD1S3IX wrap_s2_563 (.D(wrap_s2_N_2576), .CK(pll_clk), .CD(n17865), 
            .Q(wrap_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[12] 669[8])
    defparam wrap_s2_563.GSR = "DISABLED";
    PFUMX i15330 (.BLUT(n26282), .ALUT(n26283), .C0(ev_state[0]), .Z(ev_wr_addr[7]));
    L6MUX21 i15137 (.D0(n25711), .D1(n25712), .SD(n26246), .Z(n25719));
    
endmodule
//
// Verilog Description of module spi_mic_stream
//

module spi_mic_stream (sck_N_3050, spi_mic_cs_n_c, mic_latest, spi_mic_miso_c) /* synthesis syn_module_defined=1 */ ;
    input sck_N_3050;
    input spi_mic_cs_n_c;
    input [63:0]mic_latest;
    output spi_mic_miso_c;
    
    wire sck_N_3050 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    wire [95:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    
    wire sck_N_3050_enable_101, n18026, n18024, n18022, n18020, n18018, 
        n18016, n18014, n18012, n18062;
    wire [6:0]bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(12[11:20])
    
    wire n18010, n18060, n18008;
    wire [6:0]bit_count_6__N_3051;
    
    wire n13, n26259, n26218, n18002, n26145, n18058, n18006, 
        n18004, n18000, n12;
    wire [95:0]shift_register_95__N_2953;
    
    wire n18056, n18054, n18052, n18044, n26136, n18042, n18040, 
        n18038, n18050, n18036, n18034, n18032, n18030, n18028, 
        n18048, n18064, n18046;
    
    FD1P3DX shift_register_i43 (.D(n18026), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[43])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i43.GSR = "DISABLED";
    FD1P3DX shift_register_i41 (.D(n18024), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[41])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i41.GSR = "DISABLED";
    FD1P3DX shift_register_i24 (.D(n18022), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[24])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i24.GSR = "DISABLED";
    FD1P3DX shift_register_i23 (.D(n18020), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[23])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i23.GSR = "DISABLED";
    FD1P3DX shift_register_i22 (.D(n18018), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[22])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i22.GSR = "DISABLED";
    FD1P3DX shift_register_i21 (.D(n18016), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[21])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i21.GSR = "DISABLED";
    FD1P3DX shift_register_i20 (.D(n18014), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[20])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i20.GSR = "DISABLED";
    FD1P3DX shift_register_i19 (.D(n18012), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[19])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i19.GSR = "DISABLED";
    FD1P3DX shift_register_i94 (.D(n18062), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[94])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i94.GSR = "DISABLED";
    FD1P3DX bit_count_i6 (.D(n18010), .SP(sck_N_3050_enable_101), .CK(sck_N_3050), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i6.GSR = "DISABLED";
    FD1P3DX shift_register_i93 (.D(n18060), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[93])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i93.GSR = "DISABLED";
    FD1P3DX bit_count_i5 (.D(n18008), .SP(sck_N_3050_enable_101), .CK(sck_N_3050), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i5.GSR = "DISABLED";
    FD1S3DX bit_count_i0 (.D(bit_count_6__N_3051[0]), .CK(sck_N_3050), .CD(spi_mic_cs_n_c), 
            .Q(bit_count[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i0.GSR = "DISABLED";
    LUT4 i10776_2_lut (.A(shift_register[92]), .B(n13), .Z(n18060)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10776_2_lut.init = 16'h8888;
    LUT4 i1794_2_lut_rep_295 (.A(bit_count[1]), .B(bit_count[0]), .Z(n26259)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1794_2_lut_rep_295.init = 16'h8888;
    LUT4 i1801_2_lut_rep_254_3_lut (.A(bit_count[1]), .B(bit_count[0]), 
         .C(bit_count[2]), .Z(n26218)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1801_2_lut_rep_254_3_lut.init = 16'h8080;
    LUT4 i10747_3_lut_4_lut (.A(bit_count[1]), .B(bit_count[0]), .C(n13), 
         .D(bit_count[2]), .Z(n18002)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10747_3_lut_4_lut.init = 16'h7080;
    LUT4 i1808_2_lut_rep_181_3_lut_4_lut (.A(bit_count[1]), .B(bit_count[0]), 
         .C(bit_count[3]), .D(bit_count[2]), .Z(n26145)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1808_2_lut_rep_181_3_lut_4_lut.init = 16'h8000;
    LUT4 i10777_2_lut (.A(shift_register[93]), .B(n13), .Z(n18062)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10777_2_lut.init = 16'h8888;
    LUT4 i10750_3_lut_4_lut (.A(bit_count[4]), .B(n26145), .C(n13), .D(bit_count[5]), 
         .Z(n18008)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10750_3_lut_4_lut.init = 16'h7080;
    FD1P3DX shift_register_i92 (.D(n18058), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[92])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i92.GSR = "DISABLED";
    FD1P3DX bit_count_i4 (.D(n18006), .SP(sck_N_3050_enable_101), .CK(sck_N_3050), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i4.GSR = "DISABLED";
    FD1P3DX bit_count_i3 (.D(n18004), .SP(sck_N_3050_enable_101), .CK(sck_N_3050), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i3.GSR = "DISABLED";
    FD1P3DX bit_count_i2 (.D(n18002), .SP(sck_N_3050_enable_101), .CK(sck_N_3050), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i2.GSR = "DISABLED";
    FD1P3DX bit_count_i1 (.D(n18000), .SP(sck_N_3050_enable_101), .CK(sck_N_3050), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i1.GSR = "DISABLED";
    LUT4 i10759_2_lut (.A(shift_register[42]), .B(n13), .Z(n18026)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10759_2_lut.init = 16'h8888;
    LUT4 i6_4_lut (.A(bit_count[2]), .B(n12), .C(bit_count[6]), .D(bit_count[1]), 
         .Z(n13)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(26[16:33])
    defparam i6_4_lut.init = 16'hfffe;
    LUT4 i5_4_lut (.A(bit_count[0]), .B(bit_count[5]), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n12)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(26[16:33])
    defparam i5_4_lut.init = 16'hfffe;
    LUT4 i10758_2_lut (.A(shift_register[40]), .B(n13), .Z(n18024)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10758_2_lut.init = 16'h8888;
    LUT4 i10757_2_lut (.A(shift_register[23]), .B(n13), .Z(n18022)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10757_2_lut.init = 16'h8888;
    LUT4 i10756_2_lut (.A(shift_register[22]), .B(n13), .Z(n18020)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10756_2_lut.init = 16'h8888;
    LUT4 i10755_2_lut (.A(shift_register[21]), .B(n13), .Z(n18018)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10755_2_lut.init = 16'h8888;
    LUT4 i10754_2_lut (.A(shift_register[20]), .B(n13), .Z(n18016)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10754_2_lut.init = 16'h8888;
    LUT4 i10753_2_lut (.A(shift_register[19]), .B(n13), .Z(n18014)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10753_2_lut.init = 16'h8888;
    LUT4 i10752_2_lut (.A(shift_register[18]), .B(n13), .Z(n18012)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10752_2_lut.init = 16'h8888;
    LUT4 i10742_2_lut (.A(mic_latest[0]), .B(n13), .Z(shift_register_95__N_2953[1])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10742_2_lut.init = 16'h2222;
    LUT4 shift_register_95__I_0_19_i3_3_lut (.A(mic_latest[1]), .B(shift_register[1]), 
         .C(n13), .Z(shift_register_95__N_2953[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i3_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i4_3_lut (.A(mic_latest[2]), .B(shift_register[2]), 
         .C(n13), .Z(shift_register_95__N_2953[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i4_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i1 (.D(shift_register_95__N_2953[1]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i1.GSR = "DISABLED";
    FD1P3DX shift_register_i2 (.D(shift_register_95__N_2953[2]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i2.GSR = "DISABLED";
    FD1P3DX shift_register_i3 (.D(shift_register_95__N_2953[3]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i3.GSR = "DISABLED";
    FD1P3DX shift_register_i4 (.D(shift_register_95__N_2953[4]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i4.GSR = "DISABLED";
    FD1P3DX shift_register_i5 (.D(shift_register_95__N_2953[5]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i5.GSR = "DISABLED";
    FD1P3DX shift_register_i6 (.D(shift_register_95__N_2953[6]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i6.GSR = "DISABLED";
    FD1P3DX shift_register_i7 (.D(shift_register_95__N_2953[7]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i7.GSR = "DISABLED";
    FD1P3DX shift_register_i8 (.D(shift_register_95__N_2953[8]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i8.GSR = "DISABLED";
    FD1P3DX shift_register_i9 (.D(shift_register_95__N_2953[9]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i9.GSR = "DISABLED";
    FD1P3DX shift_register_i10 (.D(shift_register_95__N_2953[10]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i10.GSR = "DISABLED";
    FD1P3DX shift_register_i11 (.D(shift_register_95__N_2953[11]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i11.GSR = "DISABLED";
    FD1P3DX shift_register_i12 (.D(shift_register_95__N_2953[12]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i12.GSR = "DISABLED";
    FD1P3DX shift_register_i13 (.D(shift_register_95__N_2953[13]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i13.GSR = "DISABLED";
    FD1P3DX shift_register_i14 (.D(shift_register_95__N_2953[14]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i14.GSR = "DISABLED";
    FD1P3DX shift_register_i15 (.D(shift_register_95__N_2953[15]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i15.GSR = "DISABLED";
    FD1P3DX shift_register_i16 (.D(shift_register_95__N_2953[16]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[16])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i16.GSR = "DISABLED";
    FD1P3DX shift_register_i17 (.D(shift_register_95__N_2953[17]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[17])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i17.GSR = "DISABLED";
    FD1P3DX shift_register_i18 (.D(shift_register_95__N_2953[18]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[18])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i18.GSR = "DISABLED";
    FD1P3DX shift_register_i25 (.D(shift_register_95__N_2953[25]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[25])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i25.GSR = "DISABLED";
    FD1P3DX shift_register_i26 (.D(shift_register_95__N_2953[26]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[26])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i26.GSR = "DISABLED";
    FD1P3DX shift_register_i27 (.D(shift_register_95__N_2953[27]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[27])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i27.GSR = "DISABLED";
    FD1P3DX shift_register_i28 (.D(shift_register_95__N_2953[28]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[28])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i28.GSR = "DISABLED";
    FD1P3DX shift_register_i29 (.D(shift_register_95__N_2953[29]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[29])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i29.GSR = "DISABLED";
    FD1P3DX shift_register_i30 (.D(shift_register_95__N_2953[30]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[30])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i30.GSR = "DISABLED";
    FD1P3DX shift_register_i31 (.D(shift_register_95__N_2953[31]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[31])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i31.GSR = "DISABLED";
    FD1P3DX shift_register_i32 (.D(shift_register_95__N_2953[32]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[32])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i32.GSR = "DISABLED";
    FD1P3DX shift_register_i33 (.D(shift_register_95__N_2953[33]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[33])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i33.GSR = "DISABLED";
    FD1P3DX shift_register_i34 (.D(shift_register_95__N_2953[34]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[34])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i34.GSR = "DISABLED";
    FD1P3DX shift_register_i35 (.D(shift_register_95__N_2953[35]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[35])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i35.GSR = "DISABLED";
    FD1P3DX shift_register_i36 (.D(shift_register_95__N_2953[36]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[36])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i36.GSR = "DISABLED";
    FD1P3DX shift_register_i37 (.D(shift_register_95__N_2953[37]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[37])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i37.GSR = "DISABLED";
    FD1P3DX shift_register_i38 (.D(shift_register_95__N_2953[38]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[38])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i38.GSR = "DISABLED";
    FD1P3DX shift_register_i39 (.D(shift_register_95__N_2953[39]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[39])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i39.GSR = "DISABLED";
    FD1P3DX shift_register_i40 (.D(shift_register_95__N_2953[40]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[40])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i40.GSR = "DISABLED";
    FD1P3DX shift_register_i42 (.D(shift_register_95__N_2953[42]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[42])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i42.GSR = "DISABLED";
    FD1P3DX shift_register_i49 (.D(shift_register_95__N_2953[49]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[49])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i49.GSR = "DISABLED";
    FD1P3DX shift_register_i50 (.D(shift_register_95__N_2953[50]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[50])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i50.GSR = "DISABLED";
    FD1P3DX shift_register_i51 (.D(shift_register_95__N_2953[51]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[51])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i51.GSR = "DISABLED";
    FD1P3DX shift_register_i52 (.D(shift_register_95__N_2953[52]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[52])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i52.GSR = "DISABLED";
    FD1P3DX shift_register_i53 (.D(shift_register_95__N_2953[53]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[53])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i53.GSR = "DISABLED";
    FD1P3DX shift_register_i54 (.D(shift_register_95__N_2953[54]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[54])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i54.GSR = "DISABLED";
    FD1P3DX shift_register_i55 (.D(shift_register_95__N_2953[55]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[55])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i55.GSR = "DISABLED";
    FD1P3DX shift_register_i56 (.D(shift_register_95__N_2953[56]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[56])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i56.GSR = "DISABLED";
    FD1P3DX shift_register_i57 (.D(shift_register_95__N_2953[57]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[57])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i57.GSR = "DISABLED";
    FD1P3DX shift_register_i58 (.D(shift_register_95__N_2953[58]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[58])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i58.GSR = "DISABLED";
    FD1P3DX shift_register_i59 (.D(shift_register_95__N_2953[59]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[59])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i59.GSR = "DISABLED";
    FD1P3DX shift_register_i60 (.D(shift_register_95__N_2953[60]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[60])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i60.GSR = "DISABLED";
    FD1P3DX shift_register_i61 (.D(shift_register_95__N_2953[61]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[61])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i61.GSR = "DISABLED";
    FD1P3DX shift_register_i62 (.D(shift_register_95__N_2953[62]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[62])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i62.GSR = "DISABLED";
    FD1P3DX shift_register_i63 (.D(shift_register_95__N_2953[63]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[63])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i63.GSR = "DISABLED";
    FD1P3DX shift_register_i64 (.D(shift_register_95__N_2953[64]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[64])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i64.GSR = "DISABLED";
    FD1P3DX shift_register_i65 (.D(shift_register_95__N_2953[65]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[65])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i65.GSR = "DISABLED";
    FD1P3DX shift_register_i73 (.D(shift_register_95__N_2953[73]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[73])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i73.GSR = "DISABLED";
    FD1P3DX shift_register_i74 (.D(shift_register_95__N_2953[74]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[74])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i74.GSR = "DISABLED";
    FD1P3DX shift_register_i75 (.D(shift_register_95__N_2953[75]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[75])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i75.GSR = "DISABLED";
    FD1P3DX shift_register_i76 (.D(shift_register_95__N_2953[76]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[76])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i76.GSR = "DISABLED";
    FD1P3DX shift_register_i77 (.D(shift_register_95__N_2953[77]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[77])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i77.GSR = "DISABLED";
    FD1P3DX shift_register_i78 (.D(shift_register_95__N_2953[78]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[78])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i78.GSR = "DISABLED";
    FD1P3DX shift_register_i79 (.D(shift_register_95__N_2953[79]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[79])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i79.GSR = "DISABLED";
    FD1P3DX shift_register_i80 (.D(shift_register_95__N_2953[80]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[80])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i80.GSR = "DISABLED";
    FD1P3DX shift_register_i81 (.D(shift_register_95__N_2953[81]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[81])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i81.GSR = "DISABLED";
    FD1P3DX shift_register_i82 (.D(shift_register_95__N_2953[82]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[82])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i82.GSR = "DISABLED";
    FD1P3DX shift_register_i83 (.D(shift_register_95__N_2953[83]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[83])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i83.GSR = "DISABLED";
    FD1P3DX shift_register_i84 (.D(shift_register_95__N_2953[84]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[84])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i84.GSR = "DISABLED";
    FD1P3DX shift_register_i85 (.D(shift_register_95__N_2953[85]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[85])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i85.GSR = "DISABLED";
    FD1P3DX shift_register_i86 (.D(shift_register_95__N_2953[86]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[86])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i86.GSR = "DISABLED";
    FD1P3DX shift_register_i87 (.D(shift_register_95__N_2953[87]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[87])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i87.GSR = "DISABLED";
    FD1P3DX shift_register_i88 (.D(shift_register_95__N_2953[88]), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[88])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i88.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i5_3_lut (.A(mic_latest[3]), .B(shift_register[3]), 
         .C(n13), .Z(shift_register_95__N_2953[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i5_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i6_3_lut (.A(mic_latest[4]), .B(shift_register[4]), 
         .C(n13), .Z(shift_register_95__N_2953[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i6_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i7_3_lut (.A(mic_latest[5]), .B(shift_register[5]), 
         .C(n13), .Z(shift_register_95__N_2953[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i7_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i8_3_lut (.A(mic_latest[6]), .B(shift_register[6]), 
         .C(n13), .Z(shift_register_95__N_2953[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i8_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i9_3_lut (.A(mic_latest[7]), .B(shift_register[7]), 
         .C(n13), .Z(shift_register_95__N_2953[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i9_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i10_3_lut (.A(mic_latest[8]), .B(shift_register[8]), 
         .C(n13), .Z(shift_register_95__N_2953[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i10_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i11_3_lut (.A(mic_latest[9]), .B(shift_register[9]), 
         .C(n13), .Z(shift_register_95__N_2953[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i11_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i91 (.D(n18056), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[91])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i91.GSR = "DISABLED";
    FD1P3DX shift_register_i90 (.D(n18054), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[90])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i90.GSR = "DISABLED";
    FD1P3DX shift_register_i89 (.D(n18052), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[89])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i89.GSR = "DISABLED";
    LUT4 i15237_3_lut_4_lut (.A(bit_count[5]), .B(bit_count[6]), .C(n13), 
         .D(bit_count[0]), .Z(bit_count_6__N_3051[0])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D))) */ ;
    defparam i15237_3_lut_4_lut.init = 16'h8f7f;
    LUT4 i15230_2_lut_2_lut_3_lut (.A(bit_count[5]), .B(bit_count[6]), .C(n13), 
         .Z(sck_N_3050_enable_101)) /* synthesis lut_function=(!(A (B (C)))) */ ;
    defparam i15230_2_lut_2_lut_3_lut.init = 16'h7f7f;
    LUT4 shift_register_95__I_0_19_i12_3_lut (.A(mic_latest[10]), .B(shift_register[10]), 
         .C(n13), .Z(shift_register_95__N_2953[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i12_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i13_3_lut (.A(mic_latest[11]), .B(shift_register[11]), 
         .C(n13), .Z(shift_register_95__N_2953[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i13_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i14_3_lut (.A(mic_latest[12]), .B(shift_register[12]), 
         .C(n13), .Z(shift_register_95__N_2953[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i14_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i15_3_lut (.A(mic_latest[13]), .B(shift_register[13]), 
         .C(n13), .Z(shift_register_95__N_2953[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i15_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i16_3_lut (.A(mic_latest[14]), .B(shift_register[14]), 
         .C(n13), .Z(shift_register_95__N_2953[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i16_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i17_3_lut (.A(mic_latest[15]), .B(shift_register[15]), 
         .C(n13), .Z(shift_register_95__N_2953[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i17_3_lut.init = 16'hcaca;
    LUT4 i10743_2_lut (.A(shift_register[16]), .B(n13), .Z(shift_register_95__N_2953[17])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10743_2_lut.init = 16'hbbbb;
    LUT4 i10744_2_lut (.A(shift_register[17]), .B(n13), .Z(shift_register_95__N_2953[18])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10744_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i26_3_lut (.A(mic_latest[16]), .B(shift_register[24]), 
         .C(n13), .Z(shift_register_95__N_2953[25])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i26_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i27_3_lut (.A(mic_latest[17]), .B(shift_register[25]), 
         .C(n13), .Z(shift_register_95__N_2953[26])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i27_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i28_3_lut (.A(mic_latest[18]), .B(shift_register[26]), 
         .C(n13), .Z(shift_register_95__N_2953[27])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i28_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i29_3_lut (.A(mic_latest[19]), .B(shift_register[27]), 
         .C(n13), .Z(shift_register_95__N_2953[28])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i29_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i30_3_lut (.A(mic_latest[20]), .B(shift_register[28]), 
         .C(n13), .Z(shift_register_95__N_2953[29])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i30_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i31_3_lut (.A(mic_latest[21]), .B(shift_register[29]), 
         .C(n13), .Z(shift_register_95__N_2953[30])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i31_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i32_3_lut (.A(mic_latest[22]), .B(shift_register[30]), 
         .C(n13), .Z(shift_register_95__N_2953[31])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i32_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i33_3_lut (.A(mic_latest[23]), .B(shift_register[31]), 
         .C(n13), .Z(shift_register_95__N_2953[32])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i33_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i34_3_lut (.A(mic_latest[24]), .B(shift_register[32]), 
         .C(n13), .Z(shift_register_95__N_2953[33])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i34_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i35_3_lut (.A(mic_latest[25]), .B(shift_register[33]), 
         .C(n13), .Z(shift_register_95__N_2953[34])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i35_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i36_3_lut (.A(mic_latest[26]), .B(shift_register[34]), 
         .C(n13), .Z(shift_register_95__N_2953[35])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i36_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i37_3_lut (.A(mic_latest[27]), .B(shift_register[35]), 
         .C(n13), .Z(shift_register_95__N_2953[36])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i37_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i38_3_lut (.A(mic_latest[28]), .B(shift_register[36]), 
         .C(n13), .Z(shift_register_95__N_2953[37])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i38_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i39_3_lut (.A(mic_latest[29]), .B(shift_register[37]), 
         .C(n13), .Z(shift_register_95__N_2953[38])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i39_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i40_3_lut (.A(mic_latest[30]), .B(shift_register[38]), 
         .C(n13), .Z(shift_register_95__N_2953[39])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i40_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i41_3_lut (.A(mic_latest[31]), .B(shift_register[39]), 
         .C(n13), .Z(shift_register_95__N_2953[40])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i41_3_lut.init = 16'hcaca;
    LUT4 i10745_2_lut (.A(shift_register[41]), .B(n13), .Z(shift_register_95__N_2953[42])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10745_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i50_3_lut (.A(mic_latest[32]), .B(shift_register[48]), 
         .C(n13), .Z(shift_register_95__N_2953[49])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i50_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i69 (.D(n18044), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[69])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i69.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i51_3_lut (.A(mic_latest[33]), .B(shift_register[49]), 
         .C(n13), .Z(shift_register_95__N_2953[50])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i51_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i52_3_lut (.A(mic_latest[34]), .B(shift_register[50]), 
         .C(n13), .Z(shift_register_95__N_2953[51])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i52_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i53_3_lut (.A(mic_latest[35]), .B(shift_register[51]), 
         .C(n13), .Z(shift_register_95__N_2953[52])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i53_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i54_3_lut (.A(mic_latest[36]), .B(shift_register[52]), 
         .C(n13), .Z(shift_register_95__N_2953[53])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i54_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i55_3_lut (.A(mic_latest[37]), .B(shift_register[53]), 
         .C(n13), .Z(shift_register_95__N_2953[54])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i55_3_lut.init = 16'hcaca;
    LUT4 i10775_2_lut (.A(shift_register[91]), .B(n13), .Z(n18058)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10775_2_lut.init = 16'h8888;
    LUT4 i1_3_lut (.A(n13), .B(shift_register[95]), .C(spi_mic_cs_n_c), 
         .Z(spi_mic_miso_c)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[15] 26[68])
    defparam i1_3_lut.init = 16'h0808;
    LUT4 shift_register_95__I_0_19_i56_3_lut (.A(mic_latest[38]), .B(shift_register[54]), 
         .C(n13), .Z(shift_register_95__N_2953[55])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i56_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i57_3_lut (.A(mic_latest[39]), .B(shift_register[55]), 
         .C(n13), .Z(shift_register_95__N_2953[56])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i57_3_lut.init = 16'hcaca;
    LUT4 i10749_3_lut_4_lut (.A(bit_count[3]), .B(n26218), .C(n13), .D(bit_count[4]), 
         .Z(n18006)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10749_3_lut_4_lut.init = 16'h7080;
    LUT4 shift_register_95__I_0_19_i58_3_lut (.A(mic_latest[40]), .B(shift_register[56]), 
         .C(n13), .Z(shift_register_95__N_2953[57])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i58_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i59_3_lut (.A(mic_latest[41]), .B(shift_register[57]), 
         .C(n13), .Z(shift_register_95__N_2953[58])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i59_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i60_3_lut (.A(mic_latest[42]), .B(shift_register[58]), 
         .C(n13), .Z(shift_register_95__N_2953[59])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i60_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i61_3_lut (.A(mic_latest[43]), .B(shift_register[59]), 
         .C(n13), .Z(shift_register_95__N_2953[60])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i61_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i62_3_lut (.A(mic_latest[44]), .B(shift_register[60]), 
         .C(n13), .Z(shift_register_95__N_2953[61])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i62_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i63_3_lut (.A(mic_latest[45]), .B(shift_register[61]), 
         .C(n13), .Z(shift_register_95__N_2953[62])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i63_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i64_3_lut (.A(mic_latest[46]), .B(shift_register[62]), 
         .C(n13), .Z(shift_register_95__N_2953[63])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i64_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i65_3_lut (.A(mic_latest[47]), .B(shift_register[63]), 
         .C(n13), .Z(shift_register_95__N_2953[64])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i65_3_lut.init = 16'hcaca;
    LUT4 i10746_2_lut (.A(shift_register[64]), .B(n13), .Z(shift_register_95__N_2953[65])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10746_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i74_3_lut (.A(mic_latest[48]), .B(shift_register[72]), 
         .C(n13), .Z(shift_register_95__N_2953[73])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i74_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i75_3_lut (.A(mic_latest[49]), .B(shift_register[73]), 
         .C(n13), .Z(shift_register_95__N_2953[74])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i75_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i76_3_lut (.A(mic_latest[50]), .B(shift_register[74]), 
         .C(n13), .Z(shift_register_95__N_2953[75])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i76_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i77_3_lut (.A(mic_latest[51]), .B(shift_register[75]), 
         .C(n13), .Z(shift_register_95__N_2953[76])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i77_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i78_3_lut (.A(mic_latest[52]), .B(shift_register[76]), 
         .C(n13), .Z(shift_register_95__N_2953[77])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i78_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i79_3_lut (.A(mic_latest[53]), .B(shift_register[77]), 
         .C(n13), .Z(shift_register_95__N_2953[78])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i79_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i80_3_lut (.A(mic_latest[54]), .B(shift_register[78]), 
         .C(n13), .Z(shift_register_95__N_2953[79])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i80_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i81_3_lut (.A(mic_latest[55]), .B(shift_register[79]), 
         .C(n13), .Z(shift_register_95__N_2953[80])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i81_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i82_3_lut (.A(mic_latest[56]), .B(shift_register[80]), 
         .C(n13), .Z(shift_register_95__N_2953[81])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i82_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i83_3_lut (.A(mic_latest[57]), .B(shift_register[81]), 
         .C(n13), .Z(shift_register_95__N_2953[82])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i83_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i84_3_lut (.A(mic_latest[58]), .B(shift_register[82]), 
         .C(n13), .Z(shift_register_95__N_2953[83])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i84_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i85_3_lut (.A(mic_latest[59]), .B(shift_register[83]), 
         .C(n13), .Z(shift_register_95__N_2953[84])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i85_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i86_3_lut (.A(mic_latest[60]), .B(shift_register[84]), 
         .C(n13), .Z(shift_register_95__N_2953[85])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i86_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i87_3_lut (.A(mic_latest[61]), .B(shift_register[85]), 
         .C(n13), .Z(shift_register_95__N_2953[86])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i87_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i88_3_lut (.A(mic_latest[62]), .B(shift_register[86]), 
         .C(n13), .Z(shift_register_95__N_2953[87])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i88_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i89_3_lut (.A(mic_latest[63]), .B(shift_register[87]), 
         .C(n13), .Z(shift_register_95__N_2953[88])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i89_3_lut.init = 16'hcaca;
    LUT4 i10774_2_lut (.A(shift_register[90]), .B(n13), .Z(n18056)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10774_2_lut.init = 16'h8888;
    LUT4 i10773_2_lut (.A(shift_register[89]), .B(n13), .Z(n18054)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10773_2_lut.init = 16'h8888;
    LUT4 i10772_2_lut (.A(shift_register[88]), .B(n13), .Z(n18052)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10772_2_lut.init = 16'h8888;
    LUT4 i10768_2_lut (.A(shift_register[68]), .B(n13), .Z(n18044)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10768_2_lut.init = 16'h8888;
    LUT4 i10748_3_lut_4_lut (.A(bit_count[2]), .B(n26259), .C(n13), .D(bit_count[3]), 
         .Z(n18004)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10748_3_lut_4_lut.init = 16'h7080;
    LUT4 i1815_2_lut_rep_172_3_lut_4_lut (.A(bit_count[2]), .B(n26259), 
         .C(bit_count[4]), .D(bit_count[3]), .Z(n26136)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1815_2_lut_rep_172_3_lut_4_lut.init = 16'h8000;
    LUT4 i10767_2_lut (.A(shift_register[67]), .B(n13), .Z(n18042)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10767_2_lut.init = 16'h8888;
    LUT4 i10766_2_lut (.A(shift_register[66]), .B(n13), .Z(n18040)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10766_2_lut.init = 16'h8888;
    LUT4 i10765_2_lut (.A(shift_register[65]), .B(n13), .Z(n18038)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10765_2_lut.init = 16'h8888;
    LUT4 i10733_3_lut (.A(bit_count[1]), .B(n13), .C(bit_count[0]), .Z(n18000)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10733_3_lut.init = 16'h4848;
    LUT4 i10771_2_lut (.A(shift_register[71]), .B(n13), .Z(n18050)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10771_2_lut.init = 16'h8888;
    LUT4 i10764_2_lut (.A(shift_register[47]), .B(n13), .Z(n18036)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10764_2_lut.init = 16'h8888;
    LUT4 i10763_2_lut (.A(shift_register[46]), .B(n13), .Z(n18034)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10763_2_lut.init = 16'h8888;
    LUT4 i10762_2_lut (.A(shift_register[45]), .B(n13), .Z(n18032)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10762_2_lut.init = 16'h8888;
    LUT4 i10751_4_lut (.A(bit_count[6]), .B(n13), .C(bit_count[5]), .D(n26136), 
         .Z(n18010)) /* synthesis lut_function=(!(A ((C (D))+!B)+!A !(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10751_4_lut.init = 16'h4888;
    LUT4 i10761_2_lut (.A(shift_register[44]), .B(n13), .Z(n18030)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10761_2_lut.init = 16'h8888;
    LUT4 i10760_2_lut (.A(shift_register[43]), .B(n13), .Z(n18028)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10760_2_lut.init = 16'h8888;
    LUT4 i10770_2_lut (.A(shift_register[70]), .B(n13), .Z(n18048)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10770_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i68 (.D(n18042), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[68])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i68.GSR = "DISABLED";
    FD1P3DX shift_register_i67 (.D(n18040), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[67])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i67.GSR = "DISABLED";
    FD1P3DX shift_register_i66 (.D(n18038), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[66])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i66.GSR = "DISABLED";
    FD1P3DX shift_register_i72 (.D(n18050), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[72])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i72.GSR = "DISABLED";
    FD1P3DX shift_register_i48 (.D(n18036), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[48])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i48.GSR = "DISABLED";
    FD1P3DX shift_register_i47 (.D(n18034), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[47])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i47.GSR = "DISABLED";
    LUT4 i10778_2_lut (.A(shift_register[94]), .B(n13), .Z(n18064)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10778_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i46 (.D(n18032), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[46])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i46.GSR = "DISABLED";
    LUT4 i10769_2_lut (.A(shift_register[69]), .B(n13), .Z(n18046)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10769_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i45 (.D(n18030), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[45])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i45.GSR = "DISABLED";
    FD1P3DX shift_register_i44 (.D(n18028), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[44])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i44.GSR = "DISABLED";
    FD1P3DX shift_register_i71 (.D(n18048), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[71])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i71.GSR = "DISABLED";
    FD1P3DX shift_register_i95 (.D(n18064), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[95])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i95.GSR = "DISABLED";
    FD1P3DX shift_register_i70 (.D(n18046), .SP(sck_N_3050_enable_101), 
            .CK(sck_N_3050), .CD(spi_mic_cs_n_c), .Q(shift_register[70])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=703, LSE_RLINE=706 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i70.GSR = "DISABLED";
    
endmodule
//
// Verilog Description of module TSALL
// module not written out since it is a black-box. 
//

//
// Verilog Description of module umh_channel_ram18
//

module umh_channel_ram18 (n12168, spi1_sck_c, spi_channel_index, n12170, 
            staging_q, pll_clk, rd_data_15__N_2646, n12164, n12162, 
            n12174, n12172, spi_write, VCC_net, GND_net, staging_rd_addr_6__N_722, 
            spi1_mosi_c_0, \spi_rx_shift[0] , \spi_rx_shift[1] , \spi_rx_shift[2] , 
            \spi_rx_shift[3] , \spi_rx_shift[4] , \spi_rx_shift[5] , \spi_rx_shift[6] , 
            spi_phase_pending, n12179, n12181, n12183, n12185, n12187, 
            n12189, n12191, n12193, n12195, n12197, n12199, n12201, 
            n12203, n12205, n12207, n12209, n12166) /* synthesis syn_module_defined=1 */ ;
    output n12168;
    input spi1_sck_c;
    input [6:0]spi_channel_index;
    output n12170;
    output [15:0]staging_q;
    input pll_clk;
    input [15:0]rd_data_15__N_2646;
    output n12164;
    output n12162;
    output n12174;
    output n12172;
    input spi_write;
    input VCC_net;
    input GND_net;
    input [6:0]staging_rd_addr_6__N_722;
    input spi1_mosi_c_0;
    input \spi_rx_shift[0] ;
    input \spi_rx_shift[1] ;
    input \spi_rx_shift[2] ;
    input \spi_rx_shift[3] ;
    input \spi_rx_shift[4] ;
    input \spi_rx_shift[5] ;
    input \spi_rx_shift[6] ;
    input [7:0]spi_phase_pending;
    output n12179;
    output n12181;
    output n12183;
    output n12185;
    output n12187;
    output n12189;
    output n12191;
    output n12193;
    output n12195;
    output n12197;
    output n12199;
    output n12201;
    output n12203;
    output n12205;
    output n12207;
    output n12209;
    output n12166;
    
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(95[10:17])
    
    FD1S3AX mem_1706 (.D(spi_channel_index[3]), .CK(spi1_sck_c), .Q(n12168));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1706.GSR = "DISABLED";
    FD1S3AX mem_1708 (.D(spi_channel_index[4]), .CK(spi1_sck_c), .Q(n12170));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1708.GSR = "DISABLED";
    FD1S3AX rd_data_i0 (.D(rd_data_15__N_2646[0]), .CK(pll_clk), .Q(staging_q[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i0.GSR = "DISABLED";
    FD1S3AX mem_1702 (.D(spi_channel_index[1]), .CK(spi1_sck_c), .Q(n12164));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1702.GSR = "DISABLED";
    FD1S3AX mem_1700 (.D(spi_channel_index[0]), .CK(spi1_sck_c), .Q(n12162));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1700.GSR = "DISABLED";
    FD1S3AX mem_1712 (.D(spi_channel_index[6]), .CK(spi1_sck_c), .Q(n12174));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1712.GSR = "DISABLED";
    FD1S3AX mem_1710 (.D(spi_channel_index[5]), .CK(spi1_sck_c), .Q(n12172));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1710.GSR = "DISABLED";
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
            .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), .ADR4(staging_rd_addr_6__N_722[0]), 
            .ADR5(staging_rd_addr_6__N_722[1]), .ADR6(staging_rd_addr_6__N_722[2]), 
            .ADR7(staging_rd_addr_6__N_722[3]), .ADR8(staging_rd_addr_6__N_722[4]), 
            .ADR9(staging_rd_addr_6__N_722[5]), .ADR10(staging_rd_addr_6__N_722[6]), 
            .ADR11(GND_net), .ADR12(GND_net), .CER(VCC_net), .OCER(VCC_net), 
            .CLKR(pll_clk), .CSR0(GND_net), .CSR1(GND_net), .CSR2(GND_net), 
            .RST(GND_net), .DO0(n12197), .DO1(n12199), .DO2(n12201), 
            .DO3(n12203), .DO4(n12205), .DO5(n12207), .DO6(n12209), 
            .DO9(n12179), .DO10(n12181), .DO11(n12183), .DO12(n12185), 
            .DO13(n12187), .DO14(n12189), .DO15(n12191), .DO16(n12193), 
            .DO17(n12195));
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
    FD1S3AX mem_1704 (.D(spi_channel_index[2]), .CK(spi1_sck_c), .Q(n12166));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1704.GSR = "DISABLED";
    FD1S3AX rd_data_i1 (.D(rd_data_15__N_2646[1]), .CK(pll_clk), .Q(staging_q[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i1.GSR = "DISABLED";
    FD1S3AX rd_data_i2 (.D(rd_data_15__N_2646[2]), .CK(pll_clk), .Q(staging_q[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i2.GSR = "DISABLED";
    FD1S3AX rd_data_i3 (.D(rd_data_15__N_2646[3]), .CK(pll_clk), .Q(staging_q[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i3.GSR = "DISABLED";
    FD1S3AX rd_data_i4 (.D(rd_data_15__N_2646[4]), .CK(pll_clk), .Q(staging_q[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i4.GSR = "DISABLED";
    FD1S3AX rd_data_i5 (.D(rd_data_15__N_2646[5]), .CK(pll_clk), .Q(staging_q[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i5.GSR = "DISABLED";
    FD1S3AX rd_data_i6 (.D(rd_data_15__N_2646[6]), .CK(pll_clk), .Q(staging_q[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i6.GSR = "DISABLED";
    FD1S3AX rd_data_i7 (.D(rd_data_15__N_2646[7]), .CK(pll_clk), .Q(staging_q[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i7.GSR = "DISABLED";
    FD1S3AX rd_data_i8 (.D(rd_data_15__N_2646[8]), .CK(pll_clk), .Q(staging_q[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i8.GSR = "DISABLED";
    FD1S3AX rd_data_i9 (.D(rd_data_15__N_2646[9]), .CK(pll_clk), .Q(staging_q[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i9.GSR = "DISABLED";
    FD1S3AX rd_data_i10 (.D(rd_data_15__N_2646[10]), .CK(pll_clk), .Q(staging_q[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i10.GSR = "DISABLED";
    FD1S3AX rd_data_i11 (.D(rd_data_15__N_2646[11]), .CK(pll_clk), .Q(staging_q[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i11.GSR = "DISABLED";
    FD1S3AX rd_data_i12 (.D(rd_data_15__N_2646[12]), .CK(pll_clk), .Q(staging_q[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i12.GSR = "DISABLED";
    FD1S3AX rd_data_i13 (.D(rd_data_15__N_2646[13]), .CK(pll_clk), .Q(staging_q[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i13.GSR = "DISABLED";
    FD1S3AX rd_data_i14 (.D(rd_data_15__N_2646[14]), .CK(pll_clk), .Q(staging_q[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i14.GSR = "DISABLED";
    FD1S3AX rd_data_i15 (.D(rd_data_15__N_2646[15]), .CK(pll_clk), .Q(staging_q[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=227, LSE_RLINE=230 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i15.GSR = "DISABLED";
    
endmodule
//
// Verilog Description of module PUR
// module not written out since it is a black-box. 
//

//
// Verilog Description of module ws2812_stream
//

module ws2812_stream (state, pll_clk, n26137, GND_net, pll_clk_enable_650, 
            \shift_register[1] , n12991, \shift_register[2] , n12996, 
            \shift_register[3] , n13000, \shift_register[4] , n13071, 
            \shift_register[5] , n13339, \shift_register[6] , n13343, 
            \shift_register[7] , n13357, \shift_register[8] , n13361, 
            \shift_register[9] , n13365, \shift_register[10] , n13369, 
            \shift_register[11] , n13373, \shift_register[12] , n13377, 
            \shift_register[13] , n13381, \shift_register[14] , n13385, 
            \shift_register[15] , n13389, \shift_register[16] , n13395, 
            \shift_register[17] , n13402, \shift_register[18] , n13406, 
            \shift_register[19] , n13419, \shift_register[20] , n13423, 
            \shift_register[21] , n13427, \shift_register[22] , n13431, 
            n13435, rgb_data_c, \rgb_hold[16] , \shift_register[0] ) /* synthesis syn_module_defined=1 */ ;
    output [1:0]state;
    input pll_clk;
    output n26137;
    input GND_net;
    input pll_clk_enable_650;
    output \shift_register[1] ;
    input n12991;
    output \shift_register[2] ;
    input n12996;
    output \shift_register[3] ;
    input n13000;
    output \shift_register[4] ;
    input n13071;
    output \shift_register[5] ;
    input n13339;
    output \shift_register[6] ;
    input n13343;
    output \shift_register[7] ;
    input n13357;
    output \shift_register[8] ;
    input n13361;
    output \shift_register[9] ;
    input n13365;
    output \shift_register[10] ;
    input n13369;
    output \shift_register[11] ;
    input n13373;
    output \shift_register[12] ;
    input n13377;
    output \shift_register[13] ;
    input n13381;
    output \shift_register[14] ;
    input n13385;
    output \shift_register[15] ;
    input n13389;
    output \shift_register[16] ;
    input n13395;
    output \shift_register[17] ;
    input n13402;
    output \shift_register[18] ;
    input n13406;
    output \shift_register[19] ;
    input n13419;
    output \shift_register[20] ;
    input n13423;
    output \shift_register[21] ;
    input n13427;
    output \shift_register[22] ;
    input n13431;
    input n13435;
    output rgb_data_c;
    input \rgb_hold[16] ;
    output \shift_register[0] ;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(95[10:17])
    wire [1:0]state_1__N_2885;
    
    wire data_out_N_2950, n25677, n25454, n26469, n17864;
    wire [15:0]reset_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[16:27])
    
    wire n25005, n18, n17, n14, n10, n25452, n6;
    wire [4:0]bit_number;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(31[15:25])
    wire [4:0]n148;
    
    wire n26125, n2;
    wire [7:0]bit_cell_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(30[15:29])
    
    wire n24944;
    wire [15:0]n25;
    
    wire n24943, pll_clk_enable_171, n24942, n24941, n5, n6_adj_3163, 
        n1, n26130, n24940, n24939, n24938, n17867, n17994;
    wire [7:0]n79;
    
    wire n26122, n24937;
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(32[16:30])
    
    wire n25996, n24932, n26231, n24931, n24930, n21383, n25504, 
        n24929, pll_clk_enable_645, data_out_N_2943, n25501, n25500, 
        n25645, n25631;
    wire [23:0]shift_register_23__N_2913;
    
    wire n16743;
    
    FD1S3AX state__i0 (.D(state_1__N_2885[0]), .CK(pll_clk), .Q(state[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam state__i0.GSR = "DISABLED";
    LUT4 i10725_4_lut_rep_317 (.A(data_out_N_2950), .B(state[1]), .C(n25677), 
         .D(n25454), .Z(n26469)) /* synthesis lut_function=(A (B)+!A (B ((D)+!C))) */ ;
    defparam i10725_4_lut_rep_317.init = 16'hcc8c;
    LUT4 i7349_2_lut_3_lut_2_lut_4_lut (.A(data_out_N_2950), .B(state[1]), 
         .C(n25677), .D(n25454), .Z(n17864)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;
    defparam i7349_2_lut_3_lut_2_lut_4_lut.init = 16'h0040;
    LUT4 i6_2_lut (.A(reset_count[15]), .B(n25005), .Z(n18)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[25:52])
    defparam i6_2_lut.init = 16'heeee;
    LUT4 i5_2_lut (.A(reset_count[13]), .B(reset_count[7]), .Z(n17)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[25:52])
    defparam i5_2_lut.init = 16'heeee;
    LUT4 i7_4_lut (.A(reset_count[0]), .B(n14), .C(n10), .D(reset_count[1]), 
         .Z(n25452)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[25:52])
    defparam i7_4_lut.init = 16'hfffe;
    LUT4 i6_4_lut (.A(reset_count[6]), .B(reset_count[2]), .C(reset_count[12]), 
         .D(reset_count[3]), .Z(n14)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[25:52])
    defparam i6_4_lut.init = 16'hfffe;
    LUT4 i2_2_lut (.A(reset_count[4]), .B(reset_count[8]), .Z(n10)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[25:52])
    defparam i2_2_lut.init = 16'heeee;
    LUT4 i15244_4_lut (.A(reset_count[14]), .B(reset_count[9]), .C(reset_count[10]), 
         .D(n6), .Z(n25005)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i15244_4_lut.init = 16'h7fff;
    LUT4 i1_2_lut (.A(reset_count[5]), .B(reset_count[11]), .Z(n6)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut.init = 16'h8888;
    LUT4 i1977_2_lut_3_lut_4_lut (.A(data_out_N_2950), .B(n26137), .C(bit_number[1]), 
         .D(bit_number[0]), .Z(n148[1])) /* synthesis lut_function=(A (B (C)+!B !(C (D)+!C !(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(70[30] 82[24])
    defparam i1977_2_lut_3_lut_4_lut.init = 16'hd2f0;
    LUT4 i1971_2_lut_rep_161_3_lut (.A(data_out_N_2950), .B(n26137), .C(bit_number[0]), 
         .Z(n26125)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(70[30] 82[24])
    defparam i1971_2_lut_rep_161_3_lut.init = 16'h2020;
    LUT4 i10961_3_lut_4_lut (.A(n26137), .B(data_out_N_2950), .C(state[0]), 
         .D(state[1]), .Z(state_1__N_2885[1])) /* synthesis lut_function=(A (C)+!A (B (C)+!B !((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(66[34] 69[28])
    defparam i10961_3_lut_4_lut.init = 16'he0f0;
    LUT4 i10671_2_lut_3_lut (.A(n26137), .B(data_out_N_2950), .C(state[0]), 
         .Z(n2)) /* synthesis lut_function=(A (C)+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(66[34] 69[28])
    defparam i10671_2_lut_3_lut.init = 16'he0e0;
    LUT4 i1_3_lut_rep_173 (.A(bit_cell_count[0]), .B(n25454), .C(bit_cell_count[1]), 
         .Z(n26137)) /* synthesis lut_function=((B+!(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(75[33:61])
    defparam i1_3_lut_rep_173.init = 16'hdfdf;
    CCU2D add_1383_17 (.A0(reset_count[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24944), .S0(n25[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26:61])
    defparam add_1383_17.INIT0 = 16'h5aaa;
    defparam add_1383_17.INIT1 = 16'h0000;
    defparam add_1383_17.INJECT1_0 = "NO";
    defparam add_1383_17.INJECT1_1 = "NO";
    CCU2D add_1383_15 (.A0(reset_count[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24943), .COUT(n24944), .S0(n25[13]), .S1(n25[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26:61])
    defparam add_1383_15.INIT0 = 16'h5aaa;
    defparam add_1383_15.INIT1 = 16'h5aaa;
    defparam add_1383_15.INJECT1_0 = "NO";
    defparam add_1383_15.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_165_3_lut (.A(state[0]), .B(state[1]), .C(n26469), 
         .Z(pll_clk_enable_171)) /* synthesis lut_function=(!(A ((C)+!B)+!A (C))) */ ;
    defparam i1_2_lut_rep_165_3_lut.init = 16'h0d0d;
    CCU2D add_1383_13 (.A0(reset_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24942), .COUT(n24943), .S0(n25[11]), .S1(n25[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26:61])
    defparam add_1383_13.INIT0 = 16'h5aaa;
    defparam add_1383_13.INIT1 = 16'h5aaa;
    defparam add_1383_13.INJECT1_0 = "NO";
    defparam add_1383_13.INJECT1_1 = "NO";
    CCU2D add_1383_11 (.A0(reset_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24941), .COUT(n24942), .S0(n25[9]), .S1(n25[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26:61])
    defparam add_1383_11.INIT0 = 16'h5aaa;
    defparam add_1383_11.INIT1 = 16'h5aaa;
    defparam add_1383_11.INJECT1_0 = "NO";
    defparam add_1383_11.INJECT1_1 = "NO";
    LUT4 i10676_4_lut (.A(n5), .B(state[0]), .C(n25005), .D(n6_adj_3163), 
         .Z(n1)) /* synthesis lut_function=(A (B)+!A (B+!(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i10676_4_lut.init = 16'hcccd;
    LUT4 i1001_2_lut_rep_166_4_lut (.A(bit_cell_count[0]), .B(n25454), .C(bit_cell_count[1]), 
         .D(data_out_N_2950), .Z(n26130)) /* synthesis lut_function=((B+!(C (D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(75[33:61])
    defparam i1001_2_lut_rep_166_4_lut.init = 16'hdfff;
    CCU2D add_1383_9 (.A0(reset_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24940), .COUT(n24941), .S0(n25[7]), .S1(n25[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26:61])
    defparam add_1383_9.INIT0 = 16'h5aaa;
    defparam add_1383_9.INIT1 = 16'h5aaa;
    defparam add_1383_9.INJECT1_0 = "NO";
    defparam add_1383_9.INJECT1_1 = "NO";
    CCU2D add_1383_7 (.A0(reset_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24939), .COUT(n24940), .S0(n25[5]), .S1(n25[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26:61])
    defparam add_1383_7.INIT0 = 16'h5aaa;
    defparam add_1383_7.INIT1 = 16'h5aaa;
    defparam add_1383_7.INJECT1_0 = "NO";
    defparam add_1383_7.INJECT1_1 = "NO";
    CCU2D add_1383_5 (.A0(reset_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24938), .COUT(n24939), .S0(n25[3]), .S1(n25[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26:61])
    defparam add_1383_5.INIT0 = 16'h5aaa;
    defparam add_1383_5.INIT1 = 16'h5aaa;
    defparam add_1383_5.INJECT1_0 = "NO";
    defparam add_1383_5.INJECT1_1 = "NO";
    FD1P3IX bit_number_i0_i4 (.D(n148[4]), .SP(state[0]), .CD(n17867), 
            .CK(pll_clk), .Q(bit_number[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_number_i0_i4.GSR = "DISABLED";
    FD1P3IX bit_number_i0_i3 (.D(n148[3]), .SP(state[0]), .CD(n17867), 
            .CK(pll_clk), .Q(bit_number[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_number_i0_i3.GSR = "DISABLED";
    FD1P3IX bit_number_i0_i2 (.D(n148[2]), .SP(state[0]), .CD(n17867), 
            .CK(pll_clk), .Q(bit_number[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_number_i0_i2.GSR = "DISABLED";
    FD1P3IX bit_number_i0_i1 (.D(n148[1]), .SP(state[0]), .CD(n17867), 
            .CK(pll_clk), .Q(bit_number[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_number_i0_i1.GSR = "DISABLED";
    FD1P3IX reset_count__i15 (.D(n25[15]), .SP(pll_clk_enable_171), .CD(n17994), 
            .CK(pll_clk), .Q(reset_count[15])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i15.GSR = "DISABLED";
    FD1P3IX reset_count__i14 (.D(n25[14]), .SP(pll_clk_enable_171), .CD(n17994), 
            .CK(pll_clk), .Q(reset_count[14])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i14.GSR = "DISABLED";
    FD1P3IX reset_count__i13 (.D(n25[13]), .SP(pll_clk_enable_171), .CD(n17994), 
            .CK(pll_clk), .Q(reset_count[13])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i13.GSR = "DISABLED";
    FD1P3IX reset_count__i12 (.D(n25[12]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i12.GSR = "DISABLED";
    FD1P3IX reset_count__i11 (.D(n25[11]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i11.GSR = "DISABLED";
    FD1P3IX reset_count__i10 (.D(n25[10]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i10.GSR = "DISABLED";
    FD1P3IX reset_count__i9 (.D(n25[9]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i9.GSR = "DISABLED";
    FD1P3IX reset_count__i8 (.D(n25[8]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i8.GSR = "DISABLED";
    FD1P3IX reset_count__i7 (.D(n25[7]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i7.GSR = "DISABLED";
    FD1P3IX reset_count__i6 (.D(n25[6]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i6.GSR = "DISABLED";
    FD1P3IX reset_count__i5 (.D(n25[5]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i5.GSR = "DISABLED";
    FD1P3IX reset_count__i4 (.D(n25[4]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i4.GSR = "DISABLED";
    FD1P3IX reset_count__i3 (.D(n25[3]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i3.GSR = "DISABLED";
    FD1P3IX reset_count__i2 (.D(n25[2]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i2.GSR = "DISABLED";
    FD1P3IX reset_count__i1 (.D(n25[1]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i1.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i7 (.D(n79[7]), .SP(state[0]), .CD(pll_clk_enable_650), 
            .CK(pll_clk), .Q(bit_cell_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_cell_count_i0_i7.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i6 (.D(n79[6]), .SP(state[0]), .CD(pll_clk_enable_650), 
            .CK(pll_clk), .Q(bit_cell_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_cell_count_i0_i6.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i5 (.D(n79[5]), .SP(state[0]), .CD(pll_clk_enable_650), 
            .CK(pll_clk), .Q(bit_cell_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_cell_count_i0_i5.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i4 (.D(n79[4]), .SP(state[0]), .CD(pll_clk_enable_650), 
            .CK(pll_clk), .Q(bit_cell_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_cell_count_i0_i4.GSR = "DISABLED";
    LUT4 i1998_3_lut_4_lut (.A(bit_number[2]), .B(n26122), .C(bit_number[3]), 
         .D(bit_number[4]), .Z(n148[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(66[34] 69[28])
    defparam i1998_3_lut_4_lut.init = 16'h7f80;
    FD1P3IX bit_cell_count_i0_i3 (.D(n79[3]), .SP(state[0]), .CD(pll_clk_enable_650), 
            .CK(pll_clk), .Q(bit_cell_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_cell_count_i0_i3.GSR = "DISABLED";
    FD1S3AX state__i1 (.D(state_1__N_2885[1]), .CK(pll_clk), .Q(state[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam state__i1.GSR = "DISABLED";
    CCU2D add_1383_3 (.A0(reset_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24937), .COUT(n24938), .S0(n25[1]), .S1(n25[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26:61])
    defparam add_1383_3.INIT0 = 16'h5aaa;
    defparam add_1383_3.INIT1 = 16'h5aaa;
    defparam add_1383_3.INJECT1_0 = "NO";
    defparam add_1383_3.INJECT1_1 = "NO";
    CCU2D add_1383_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count[0]), .B1(n18), .C1(n17), .D1(n25452), .COUT(n24937), 
          .S1(n25[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26:61])
    defparam add_1383_1.INIT0 = 16'hF000;
    defparam add_1383_1.INIT1 = 16'h5556;
    defparam add_1383_1.INJECT1_0 = "NO";
    defparam add_1383_1.INJECT1_1 = "NO";
    FD1P3IX bit_cell_count_i0_i2 (.D(n79[2]), .SP(state[0]), .CD(pll_clk_enable_650), 
            .CK(pll_clk), .Q(bit_cell_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_cell_count_i0_i2.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i1 (.D(n79[1]), .SP(state[0]), .CD(pll_clk_enable_650), 
            .CK(pll_clk), .Q(bit_cell_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_cell_count_i0_i1.GSR = "DISABLED";
    FD1P3IX reset_count__i0 (.D(n25[0]), .SP(pll_clk_enable_171), .CD(n17864), 
            .CK(pll_clk), .Q(reset_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam reset_count__i0.GSR = "DISABLED";
    LUT4 bit_cell_count_2__bdd_4_lut_15292 (.A(bit_cell_count[2]), .B(shift_register[23]), 
         .C(bit_cell_count[5]), .D(bit_cell_count[4]), .Z(n25996)) /* synthesis lut_function=(!(A (((D)+!C)+!B)+!A (B+(C+!(D))))) */ ;
    defparam bit_cell_count_2__bdd_4_lut_15292.init = 16'h0180;
    CCU2D add_15_9 (.A0(bit_cell_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n24932), .S0(n79[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(72[43:64])
    defparam add_15_9.INIT0 = 16'h5aaa;
    defparam add_15_9.INIT1 = 16'h0000;
    defparam add_15_9.INJECT1_0 = "NO";
    defparam add_15_9.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_267_2_lut (.A(bit_cell_count[2]), .B(bit_cell_count[4]), 
         .Z(n26231)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(75[33:61])
    defparam i1_2_lut_rep_267_2_lut.init = 16'hdddd;
    CCU2D add_15_7 (.A0(bit_cell_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24931), .COUT(n24932), .S0(n79[5]), .S1(n79[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(72[43:64])
    defparam add_15_7.INIT0 = 16'h5aaa;
    defparam add_15_7.INIT1 = 16'h5aaa;
    defparam add_15_7.INJECT1_0 = "NO";
    defparam add_15_7.INJECT1_1 = "NO";
    CCU2D add_15_5 (.A0(bit_cell_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24930), .COUT(n24931), .S0(n79[3]), .S1(n79[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(72[43:64])
    defparam add_15_5.INIT0 = 16'h5aaa;
    defparam add_15_5.INIT1 = 16'h5aaa;
    defparam add_15_5.INJECT1_0 = "NO";
    defparam add_15_5.INJECT1_1 = "NO";
    LUT4 i4_3_lut_4_lut_4_lut (.A(bit_cell_count[2]), .B(n21383), .C(n25677), 
         .D(bit_cell_count[4]), .Z(n25504)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(75[33:61])
    defparam i4_3_lut_4_lut_4_lut.init = 16'h0080;
    CCU2D add_15_3 (.A0(bit_cell_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n24929), .COUT(n24930), .S0(n79[1]), .S1(n79[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(72[43:64])
    defparam add_15_3.INIT0 = 16'h5aaa;
    defparam add_15_3.INIT1 = 16'h5aaa;
    defparam add_15_3.INJECT1_0 = "NO";
    defparam add_15_3.INJECT1_1 = "NO";
    CCU2D add_15_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_cell_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n24929), .S1(n79[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(72[43:64])
    defparam add_15_1.INIT0 = 16'hF000;
    defparam add_15_1.INIT1 = 16'h5555;
    defparam add_15_1.INJECT1_0 = "NO";
    defparam add_15_1.INJECT1_1 = "NO";
    LUT4 i1984_2_lut_3_lut_4_lut (.A(bit_number[0]), .B(n26130), .C(bit_number[2]), 
         .D(bit_number[1]), .Z(n148[2])) /* synthesis lut_function=(A (B (C)+!B !(C (D)+!C !(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(66[34] 69[28])
    defparam i1984_2_lut_3_lut_4_lut.init = 16'hd2f0;
    FD1P3AX shift_register_i0_i1 (.D(n12991), .SP(pll_clk_enable_650), .CK(pll_clk), 
            .Q(\shift_register[1] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i1.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i2 (.D(n12996), .SP(pll_clk_enable_650), .CK(pll_clk), 
            .Q(\shift_register[2] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i2.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i3 (.D(n13000), .SP(pll_clk_enable_650), .CK(pll_clk), 
            .Q(\shift_register[3] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i3.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i4 (.D(n13071), .SP(pll_clk_enable_650), .CK(pll_clk), 
            .Q(\shift_register[4] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i4.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i5 (.D(n13339), .SP(pll_clk_enable_650), .CK(pll_clk), 
            .Q(\shift_register[5] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i5.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i6 (.D(n13343), .SP(pll_clk_enable_650), .CK(pll_clk), 
            .Q(\shift_register[6] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i6.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i7 (.D(n13357), .SP(pll_clk_enable_650), .CK(pll_clk), 
            .Q(\shift_register[7] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i7.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i8 (.D(n13361), .SP(pll_clk_enable_650), .CK(pll_clk), 
            .Q(\shift_register[8] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i8.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i9 (.D(n13365), .SP(pll_clk_enable_650), .CK(pll_clk), 
            .Q(\shift_register[9] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i9.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i10 (.D(n13369), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[10] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i10.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i11 (.D(n13373), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[11] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i11.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i12 (.D(n13377), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[12] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i12.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i13 (.D(n13381), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[13] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i13.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i14 (.D(n13385), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[14] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i14.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i15 (.D(n13389), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[15] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i15.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i16 (.D(n13395), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[16] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i16.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i17 (.D(n13402), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[17] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i17.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i18 (.D(n13406), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[18] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i18.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i19 (.D(n13419), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[19] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i19.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i20 (.D(n13423), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[20] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i20.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i21 (.D(n13427), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[21] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i21.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i22 (.D(n13431), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[22] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i22.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i23 (.D(n13435), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(shift_register[23])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i23.GSR = "DISABLED";
    LUT4 i1991_2_lut_3_lut_4_lut (.A(bit_number[1]), .B(n26125), .C(bit_number[3]), 
         .D(bit_number[2]), .Z(n148[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(66[34] 69[28])
    defparam i1991_2_lut_3_lut_4_lut.init = 16'h78f0;
    FD1P3IX bit_cell_count_i0_i0 (.D(n79[0]), .SP(state[0]), .CD(pll_clk_enable_650), 
            .CK(pll_clk), .Q(bit_cell_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_cell_count_i0_i0.GSR = "DISABLED";
    FD1P3AX data_out_reg_46 (.D(data_out_N_2943), .SP(pll_clk_enable_645), 
            .CK(pll_clk), .Q(rgb_data_c)) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam data_out_reg_46.GSR = "DISABLED";
    LUT4 i1_4_lut (.A(state[1]), .B(state[0]), .C(n26137), .D(n25501), 
         .Z(pll_clk_enable_645)) /* synthesis lut_function=((B ((D)+!C))+!A) */ ;
    defparam i1_4_lut.init = 16'hdd5d;
    LUT4 i3_4_lut (.A(bit_cell_count[7]), .B(bit_cell_count[1]), .C(bit_cell_count[0]), 
         .D(n25500), .Z(n25501)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;
    defparam i3_4_lut.init = 16'h4000;
    LUT4 i2_3_lut (.A(n25996), .B(bit_cell_count[3]), .C(bit_cell_count[6]), 
         .Z(n25500)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;
    defparam i2_3_lut.init = 16'h0202;
    LUT4 mux_893_Mux_0_i3_4_lut (.A(state[0]), .B(data_out_N_2950), .C(state[1]), 
         .D(n26137), .Z(data_out_N_2943)) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam mux_893_Mux_0_i3_4_lut.init = 16'h0aca;
    LUT4 i7235_2_lut (.A(state[0]), .B(state[1]), .Z(n17867)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam i7235_2_lut.init = 16'h2222;
    LUT4 i1_4_lut_adj_33 (.A(bit_number[3]), .B(bit_number[2]), .C(n25645), 
         .D(bit_number[4]), .Z(data_out_N_2950)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;
    defparam i1_4_lut_adj_33.init = 16'hbfff;
    LUT4 i15064_2_lut (.A(bit_number[1]), .B(bit_number[0]), .Z(n25645)) /* synthesis lut_function=(A (B)) */ ;
    defparam i15064_2_lut.init = 16'h8888;
    LUT4 i3_4_lut_adj_34 (.A(bit_cell_count[7]), .B(bit_cell_count[5]), 
         .C(n21383), .D(n26231), .Z(n25454)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;
    defparam i3_4_lut_adj_34.init = 16'hffef;
    LUT4 i10845_2_lut (.A(bit_cell_count[6]), .B(bit_cell_count[3]), .Z(n21383)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10845_2_lut.init = 16'h8888;
    LUT4 i4_4_lut (.A(n25631), .B(n25504), .C(state[1]), .D(bit_cell_count[7]), 
         .Z(n17994)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam i4_4_lut.init = 16'h0040;
    LUT4 i15050_2_lut (.A(data_out_N_2950), .B(bit_cell_count[5]), .Z(n25631)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i15050_2_lut.init = 16'heeee;
    LUT4 i15096_3_lut (.A(bit_cell_count[1]), .B(bit_cell_count[0]), .C(state[0]), 
         .Z(n25677)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i15096_3_lut.init = 16'h8080;
    LUT4 i1_2_lut_adj_35 (.A(reset_count[13]), .B(reset_count[15]), .Z(n5)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[25:52])
    defparam i1_2_lut_adj_35.init = 16'heeee;
    LUT4 i2_2_lut_adj_36 (.A(n25452), .B(reset_count[7]), .Z(n6_adj_3163)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[25:52])
    defparam i2_2_lut_adj_36.init = 16'heeee;
    LUT4 i10732_2_lut (.A(\rgb_hold[16] ), .B(state[1]), .Z(shift_register_23__N_2913[0])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[13] 85[20])
    defparam i10732_2_lut.init = 16'h2222;
    PFUMX mux_27_Mux_0_i3 (.BLUT(n1), .ALUT(n2), .C0(state[1]), .Z(state_1__N_2885[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;
    FD1P3IX bit_number_i0_i0 (.D(n16743), .SP(state[0]), .CD(n17867), 
            .CK(pll_clk), .Q(bit_number[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam bit_number_i0_i0.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i0 (.D(shift_register_23__N_2913[0]), .SP(pll_clk_enable_650), 
            .CK(pll_clk), .Q(\shift_register[0] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=690, LSE_RLINE=702 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(36[12] 87[8])
    defparam shift_register_i0_i0.GSR = "DISABLED";
    LUT4 i1979_2_lut_rep_158_3_lut_4_lut (.A(data_out_N_2950), .B(n26137), 
         .C(bit_number[1]), .D(bit_number[0]), .Z(n26122)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(70[30] 82[24])
    defparam i1979_2_lut_rep_158_3_lut_4_lut.init = 16'h2000;
    LUT4 i1_2_lut_3_lut (.A(data_out_N_2950), .B(n26137), .C(bit_number[0]), 
         .Z(n16743)) /* synthesis lut_function=(A (B (C)+!B !(C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(70[30] 82[24])
    defparam i1_2_lut_3_lut.init = 16'hd2d2;
    
endmodule
//
// Verilog Description of module umh_toggle_ram84
//

module umh_toggle_ram84 (pll_clk, ev_we, VCC_net, GND_net, \ev_wr_addr[0] , 
            event_rd_addr, \ev_wr_addr[1] , \ev_wr_addr[2] , \ev_wr_addr[3] , 
            \ev_wr_addr[4] , \ev_wr_addr[5] , \ev_wr_addr[6] , \ev_wr_addr[7] , 
            n26245, ev_wr_data, ev_rd_data) /* synthesis syn_module_defined=1 */ ;
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
    input n26245;
    input [83:0]ev_wr_data;
    output [83:0]ev_rd_data;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(95[10:17])
    
    PDPW8KC mem1 (.DI0(ev_wr_data[48]), .DI1(ev_wr_data[49]), .DI2(ev_wr_data[50]), 
            .DI3(ev_wr_data[51]), .DI4(ev_wr_data[52]), .DI5(ev_wr_data[53]), 
            .DI6(ev_wr_data[54]), .DI7(ev_wr_data[55]), .DI8(ev_wr_data[56]), 
            .DI9(ev_wr_data[57]), .DI10(ev_wr_data[58]), .DI11(ev_wr_data[59]), 
            .DI12(ev_wr_data[60]), .DI13(ev_wr_data[61]), .DI14(ev_wr_data[62]), 
            .DI15(ev_wr_data[63]), .DI16(ev_wr_data[64]), .DI17(ev_wr_data[65]), 
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n26245), 
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
            .ADW8(n26245), .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), 
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n26245), 
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n26245), 
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n26245), 
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
